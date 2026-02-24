// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/diffusion_planner/diffusion_planner_node.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/inference/tensorrt_inference.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/marker_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using diagnostic_msgs::msg::DiagnosticStatus;

DiffusionPlanner::DiffusionPlanner(const rclcpp::NodeOptions & options)
: Node("diffusion_planner", options), generator_uuid_(autoware_utils_uuid::generate_uuid())
{
  // Initialize the node
  pub_trajectory_ = this->create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_trajectories_ = this->create_publisher<CandidateTrajectories>("~/output/trajectories", 1);
  pub_objects_ =
    this->create_publisher<PredictedObjects>("~/output/predicted_objects", rclcpp::QoS(1));
  pub_route_marker_ = this->create_publisher<MarkerArray>("~/debug/route_marker", 10);
  pub_lane_marker_ = this->create_publisher<MarkerArray>("~/debug/lane_marker", 10);
  pub_turn_indicators_ =
    this->create_publisher<TurnIndicatorsCommand>("~/output/turn_indicators", 1);
  pub_traffic_signal_ = this->create_publisher<autoware_perception_msgs::msg::TrafficLightGroup>(
    "~/output/debug/traffic_signal", 1);
  debug_processing_time_detail_pub_ = this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);

  set_up_params();
  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  // Create core instance
  core_ = std::make_unique<DiffusionPlannerCore>(params_, vehicle_info_);

  turn_indicator_manager_.set_hold_duration(
    rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration));
  turn_indicator_manager_.set_keep_offset(params_.turn_indicator_keep_offset);

  diagnostics_inference_ = std::make_unique<DiagnosticsInterface>(this, "inference_status");
  try {
    load_model();
    if (params_.build_only) {
      RCLCPP_INFO(get_logger(), "Build only mode enabled. Exiting after loading model.");
      std::exit(EXIT_SUCCESS);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Inference will be disabled.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(get_clock()->now());
    if (params_.build_only) {
      RCLCPP_ERROR(get_logger(), "Build only mode: exiting due to model load failure.");
      std::exit(EXIT_FAILURE);
    }
  }

  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Rate(params_.planning_frequency_hz).period(),
    std::bind(&DiffusionPlanner::on_timer, this));

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&DiffusionPlanner::on_map, this, std::placeholders::_1));

  // Parameter Callback
  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&DiffusionPlanner::on_parameter, this, std::placeholders::_1));
}

DiffusionPlanner::~DiffusionPlanner()
{
}

void DiffusionPlanner::set_up_params()
{
  // node params
  params_.model_path = this->declare_parameter<std::string>("onnx_model_path", "");
  params_.args_path = this->declare_parameter<std::string>("args_path", "");
  params_.plugins_path = this->declare_parameter<std::string>("plugins_path", "");
  params_.build_only = this->declare_parameter<bool>("build_only", false);
  params_.planning_frequency_hz = this->declare_parameter<double>("planning_frequency_hz", 10.0);
  params_.ignore_neighbors = this->declare_parameter<bool>("ignore_neighbors", false);
  params_.ignore_unknown_neighbors =
    this->declare_parameter<bool>("ignore_unknown_neighbors", false);
  params_.predict_neighbor_trajectory =
    this->declare_parameter<bool>("predict_neighbor_trajectory", false);
  params_.traffic_light_group_msg_timeout_seconds =
    this->declare_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
  params_.batch_size = this->declare_parameter<int>("batch_size", 1);
  params_.temperature_list = this->declare_parameter<std::vector<double>>("temperature", {0.0});
  params_.velocity_smoothing_window =
    this->declare_parameter<int64_t>("velocity_smoothing_window", 8);
  params_.stopping_threshold = this->declare_parameter<double>("stopping_threshold", 0.3);
  params_.turn_indicator_keep_offset =
    this->declare_parameter<float>("turn_indicator_keep_offset", -1.25f);
  params_.turn_indicator_hold_duration =
    this->declare_parameter<double>("turn_indicator_hold_duration", 0.0);
  params_.shift_x = this->declare_parameter<bool>("shift_x", false);

  // debug params
  debug_params_.publish_debug_map =
    this->declare_parameter<bool>("debug_params.publish_debug_map", false);
  debug_params_.publish_debug_route =
    this->declare_parameter<bool>("debug_params.publish_debug_route", true);
}

void DiffusionPlanner::load_model()
{
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Loading model");
  diagnostics_inference_->publish(get_clock()->now());
  core_->load_model();
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::OK, "Model loaded");
  diagnostics_inference_->publish(get_clock()->now());
}

SetParametersResult DiffusionPlanner::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  {
    DiffusionPlannerParams temp_params = params_;
    const auto previous_args_path = params_.args_path;
    const auto previous_model_path = params_.model_path;
    const auto previous_batch_size = params_.batch_size;
    update_param<std::string>(parameters, "onnx_model_path", temp_params.model_path);
    update_param<std::string>(parameters, "args_path", temp_params.args_path);
    update_param<bool>(
      parameters, "ignore_unknown_neighbors", temp_params.ignore_unknown_neighbors);
    update_param<bool>(parameters, "ignore_neighbors", temp_params.ignore_neighbors);
    update_param<bool>(
      parameters, "predict_neighbor_trajectory", temp_params.predict_neighbor_trajectory);
    update_param<double>(
      parameters, "traffic_light_group_msg_timeout_seconds",
      temp_params.traffic_light_group_msg_timeout_seconds);
    update_param<int>(parameters, "batch_size", temp_params.batch_size);
    update_param<std::vector<double>>(parameters, "temperature", temp_params.temperature_list);
    update_param<int64_t>(
      parameters, "velocity_smoothing_window", temp_params.velocity_smoothing_window);
    update_param<double>(parameters, "stopping_threshold", temp_params.stopping_threshold);
    update_param<float>(
      parameters, "turn_indicator_keep_offset", temp_params.turn_indicator_keep_offset);
    update_param<double>(
      parameters, "turn_indicator_hold_duration", temp_params.turn_indicator_hold_duration);
    update_param<bool>(parameters, "shift_x", temp_params.shift_x);
    const bool args_path_changed = temp_params.args_path != previous_args_path;
    const bool model_path_changed = temp_params.model_path != previous_model_path;
    const bool batch_size_changed = temp_params.batch_size != previous_batch_size;
    params_ = temp_params;
    turn_indicator_manager_.set_hold_duration(
      rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration));
    turn_indicator_manager_.set_keep_offset(params_.turn_indicator_keep_offset);

    core_->update_params(params_);

    if (args_path_changed || model_path_changed || batch_size_changed) {
      try {
        load_model();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Failed to reload model.");
        SetParametersResult result;
        result.successful = false;
        result.reason = e.what();
        return result;
      }
    }
  }

  {
    DiffusionPlannerDebugParams temp_debug_params = debug_params_;
    update_param<bool>(
      parameters, "debug_params.publish_debug_map", temp_debug_params.publish_debug_map);
    update_param<bool>(
      parameters, "debug_params.publish_debug_route", temp_debug_params.publish_debug_route);
    debug_params_ = temp_debug_params;
  }

  SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void DiffusionPlanner::publish_first_traffic_light_on_route(
  const FrameContext & frame_context) const
{
  const auto msg = core_->get_first_traffic_light_on_route(frame_context);
  pub_traffic_signal_->publish(msg);
}

void DiffusionPlanner::publish_debug_markers(
  const InputDataMap & input_data_map, const Eigen::Matrix4d & ego_to_map_transform,
  const rclcpp::Time & timestamp) const
{
  if (debug_params_.publish_debug_route) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto route_markers = utils::create_lane_marker(
      ego_to_map_transform, input_data_map.at("route_lanes"),
      std::vector<int64_t>(ROUTE_LANES_SHAPE.begin(), ROUTE_LANES_SHAPE.end()), timestamp, lifetime,
      {0.8, 0.8, 0.8, 0.8}, "map", true);
    pub_route_marker_->publish(route_markers);
  }

  if (debug_params_.publish_debug_map) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto lane_markers = utils::create_lane_marker(
      ego_to_map_transform, input_data_map.at("lanes"),
      std::vector<int64_t>(LANES_SHAPE.begin(), LANES_SHAPE.end()), timestamp, lifetime,
      {0.1, 0.1, 0.7, 0.8}, "map", true);
    pub_lane_marker_->publish(lane_markers);
  }
}

void DiffusionPlanner::publish_predictions(
  const std::vector<float> & predictions, const FrameContext & frame_context,
  const rclcpp::Time & timestamp) const
{
  CandidateTrajectories candidate_trajectories;

  // when ego is moving, enable force stop
  const bool enable_force_stop =
    frame_context.ego_kinematic_state.twist.twist.linear.x > std::numeric_limits<double>::epsilon();

  // Parse predictions once: [batch][agent][timestep] -> pose
  const auto agent_poses =
    postprocess::parse_predictions(predictions, frame_context.ego_to_map_transform);

  for (int i = 0; i < params_.batch_size; i++) {
    Trajectory trajectory = postprocess::create_ego_trajectory(
      agent_poses, timestamp, frame_context.ego_kinematic_state.pose.pose.position, i,
      params_.velocity_smoothing_window, enable_force_stop, params_.stopping_threshold);
    if (params_.shift_x) {
      // center to base_link
      for (auto & point : trajectory.points) {
        point.pose = utils::shift_x(point.pose, -vehicle_info_.wheel_base_m / 2.0);
      }
    }
    if (i == 0) {
      pub_trajectory_->publish(trajectory);
    }

    const auto candidate_trajectory = autoware_internal_planning_msgs::build<
                                        autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                                        .header(trajectory.header)
                                        .generator_id(generator_uuid_)
                                        .points(trajectory.points);

    std_msgs::msg::String generator_name_msg;
    generator_name_msg.data = "DiffusionPlanner_batch_" + std::to_string(i);

    const auto generator_info =
      autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::GeneratorInfo>()
        .generator_id(generator_uuid_)
        .generator_name(generator_name_msg);

    candidate_trajectories.candidate_trajectories.push_back(candidate_trajectory);
    candidate_trajectories.generator_info.push_back(generator_info);
  }

  pub_trajectories_->publish(candidate_trajectories);

  // Other agents prediction
  if (params_.predict_neighbor_trajectory) {
    // Use batch 0 for neighbor predictions
    constexpr int64_t batch_idx = 0;
    auto predicted_objects = postprocess::create_predicted_objects(
      agent_poses, frame_context.ego_centric_neighbor_histories, timestamp, batch_idx);
    pub_objects_->publish(predicted_objects);
  }
}

void DiffusionPlanner::on_timer()
{
  // Timer callback function
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);

  diagnostics_inference_->clear();

  const rclcpp::Time current_time(get_clock()->now());
  if (!core_->is_model_loaded()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Model not loaded. Inference is disabled. Check onnx_model_path and args_path parameters.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, "Model not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  if (!core_->is_map_loaded()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Waiting for map data...");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Map data not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Take data from subscribers
  auto objects = sub_tracked_objects_.take_data();
  auto ego_kinematic_state = sub_current_odometry_.take_data();
  auto ego_acceleration = sub_current_acceleration_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto temp_route_ptr = route_subscriber_.take_data();
  auto turn_indicators_ptr = sub_turn_indicators_.take_data();

  // Prepare frame context using core
  const std::optional<FrameContext> frame_context = core_->create_frame_context(
    ego_kinematic_state, ego_acceleration, objects, traffic_signals, turn_indicators_ptr,
    temp_route_ptr, this->now());

  if (!frame_context) {
    // Log detailed information about missing inputs
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "There is no input data. objects: "
        << (objects ? "true" : "false")
        << ", ego_kinematic_state: " << (ego_kinematic_state ? "true" : "false")
        << ", ego_acceleration: " << (ego_acceleration ? "true" : "false")
        << ", route: " << (core_->get_route() ? "true" : "false")
        << ", turn_indicators: " << (turn_indicators_ptr ? "true" : "false"));
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "No input data available for inference");
    diagnostics_inference_->publish(current_time);
    return;
  }

  if (traffic_signals.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "no traffic signal received. traffic light info will not be updated");
  }

  const rclcpp::Time frame_time(frame_context->frame_time);
  InputDataMap input_data_map = core_->create_input_data(*frame_context);

  publish_debug_markers(input_data_map, frame_context->ego_to_map_transform, frame_time);

  publish_first_traffic_light_on_route(*frame_context);

  // Calculate and record metrics for diagnostics using core
  diagnostics_inference_->add_key_value(
    "valid_lane_count", core_->count_valid_elements(input_data_map, "lanes"));
  diagnostics_inference_->add_key_value(
    "valid_route_count", core_->count_valid_elements(input_data_map, "route_lanes"));
  diagnostics_inference_->add_key_value(
    "valid_polygon_count", core_->count_valid_elements(input_data_map, "polygons"));
  diagnostics_inference_->add_key_value(
    "valid_line_string_count", core_->count_valid_elements(input_data_map, "line_strings"));
  diagnostics_inference_->add_key_value(
    "valid_neighbor_count", core_->count_valid_elements(input_data_map, "neighbor_agents_past"));

  // normalization of data
  preprocess::normalize_input_data(input_data_map, core_->get_normalization_map());
  if (!utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "Input data contains invalid values");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run inference using core
  const auto inference_result = core_->run_inference(input_data_map);
  if (!inference_result.outputs) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Inference failed: " << inference_result.error_msg);
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::ERROR, inference_result.error_msg);
    diagnostics_inference_->publish(frame_time);
    return;
  }
  const auto & [predictions, turn_indicator_logit] = inference_result.outputs.value();

  publish_predictions(predictions, *frame_context, frame_time);

  // Get previous turn indicator report from core's history
  const int64_t prev_report = core_->get_previous_turn_indicator_report();
  const auto turn_indicator_command =
    turn_indicator_manager_.evaluate(turn_indicator_logit, frame_time, prev_report);
  pub_turn_indicators_->publish(turn_indicator_command);

  // Publish diagnostics
  diagnostics_inference_->publish(frame_time);
}

void DiffusionPlanner::on_map(const HADMapBin::ConstSharedPtr map_msg)
{
  std::shared_ptr<const lanelet::LaneletMap> lanelet_map_ptr =
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*map_msg);
  core_->set_map(lanelet_map_ptr);
}

}  // namespace autoware::diffusion_planner
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diffusion_planner::DiffusionPlanner)
