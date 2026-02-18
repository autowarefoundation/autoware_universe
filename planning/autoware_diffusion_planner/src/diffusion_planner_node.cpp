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
  debug_processing_time_detail_pub_ = this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);

  set_up_params();
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
    tensorrt_inference_.reset();
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, e.what());
    diagnostics_inference_->publish(get_clock()->now());
    if (params_.build_only) {
      RCLCPP_ERROR(get_logger(), "Build only mode: exiting due to model load failure.");
      std::exit(EXIT_FAILURE);
    }
  }

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

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
  utils::check_weight_version(params_.args_path);
  normalization_map_ = utils::load_normalization_stats(params_.args_path);
  diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Loading model");
  diagnostics_inference_->publish(get_clock()->now());
  tensorrt_inference_ = std::make_unique<TensorrtInference>(
    params_.model_path, params_.plugins_path, params_.batch_size);
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

    if (args_path_changed || model_path_changed || batch_size_changed) {
      try {
        load_model();
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(get_logger(), e.what() << ". Failed to reload model.");
        tensorrt_inference_.reset();
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

std::optional<FrameContext> DiffusionPlanner::create_frame_context()
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  auto objects = sub_tracked_objects_.take_data();
  auto ego_kinematic_state = sub_current_odometry_.take_data();
  auto ego_acceleration = sub_current_acceleration_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto temp_route_ptr = route_subscriber_.take_data();
  auto turn_indicators_ptr = sub_turn_indicators_.take_data();

  route_ptr_ = (!route_ptr_ || temp_route_ptr) ? temp_route_ptr : route_ptr_;

  TrackedObjects empty_object_list;

  if (params_.ignore_neighbors) {
    objects = std::make_shared<TrackedObjects>(empty_object_list);
  }

  if (
    !objects || !ego_kinematic_state || !ego_acceleration || !route_ptr_ || !turn_indicators_ptr) {
    RCLCPP_WARN_STREAM_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "There is no input data. objects: "
        << (objects ? "true" : "false")
        << ", ego_kinematic_state: " << (ego_kinematic_state ? "true" : "false")
        << ", ego_acceleration: " << (ego_acceleration ? "true" : "false")
        << ", route: " << (route_ptr_ ? "true" : "false")
        << ", turn_indicators: " << (turn_indicators_ptr ? "true" : "false"));
    return std::nullopt;
  }

  if (traffic_signals.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "no traffic signal received. traffic light info will not be updated");
  }

  Odometry kinematic_state = *ego_kinematic_state;
  if (params_.shift_x) {
    kinematic_state.pose.pose =
      utils::shift_x(kinematic_state.pose.pose, vehicle_info_.wheel_base_m / 2.0);
  }

  // Get transforms
  const geometry_msgs::msg::Pose & pose_base_link = kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_base_link);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);

  // Update ego history
  ego_history_.push_back(kinematic_state);
  if (ego_history_.size() > static_cast<size_t>(EGO_HISTORY_SHAPE[1])) {
    ego_history_.pop_front();
  }

  // Update turn indicators history
  turn_indicators_history_.push_back(*turn_indicators_ptr);
  if (turn_indicators_history_.size() > static_cast<size_t>(TURN_INDICATORS_SHAPE[1])) {
    turn_indicators_history_.pop_front();
  }

  // Update neighbor agent data
  agent_data_.update_histories(*objects, params_.ignore_unknown_neighbors);
  const auto processed_neighbor_histories =
    agent_data_.transformed_and_trimmed_histories(map_to_ego_transform, NEIGHBOR_SHAPE[1]);

  // Update traffic light map
  const auto & traffic_light_msg_timeout_s = params_.traffic_light_group_msg_timeout_seconds;
  preprocess::process_traffic_signals(
    traffic_signals, traffic_light_id_map_, this->now(), traffic_light_msg_timeout_s);

  // Create frame context
  const rclcpp::Time frame_time(ego_kinematic_state->header.stamp);
  const FrameContext frame_context{
    *ego_kinematic_state, *ego_acceleration, ego_to_map_transform, processed_neighbor_histories,
    frame_time};

  return frame_context;
}

InputDataMap DiffusionPlanner::create_input_data(const FrameContext & frame_context)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  InputDataMap input_data_map;

  // random sample trajectories
  {
    for (int64_t b = 0; b < params_.batch_size; b++) {
      const std::vector<float> sampled_trajectories =
        preprocess::create_sampled_trajectories(params_.temperature_list[b]);
      input_data_map["sampled_trajectories"].insert(
        input_data_map["sampled_trajectories"].end(), sampled_trajectories.begin(),
        sampled_trajectories.end());
    }
  }

  const geometry_msgs::msg::Pose & pose_center =
    params_.shift_x
      ? utils::shift_x(
          frame_context.ego_kinematic_state.pose.pose, vehicle_info_.wheel_base_m / 2.0)
      : frame_context.ego_kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_center);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);
  const auto & center_x = static_cast<float>(pose_center.position.x);
  const auto & center_y = static_cast<float>(pose_center.position.y);
  const auto & center_z = static_cast<float>(pose_center.position.z);

  // Ego history
  {
    const std::vector<float> single_ego_agent_past =
      preprocess::create_ego_agent_past(ego_history_, EGO_HISTORY_SHAPE[1], map_to_ego_transform);
    input_data_map["ego_agent_past"] =
      utils::replicate_for_batch(single_ego_agent_past, params_.batch_size);
  }
  // Ego state
  {
    const auto ego_current_state = preprocess::create_ego_current_state(
      frame_context.ego_kinematic_state, frame_context.ego_acceleration,
      static_cast<float>(vehicle_info_.wheel_base_m));
    input_data_map["ego_current_state"] =
      utils::replicate_for_batch(ego_current_state, params_.batch_size);
  }
  // Agent data on ego reference frame
  {
    const auto neighbor_agents_past = flatten_histories_to_vector(
      frame_context.ego_centric_neighbor_histories, MAX_NUM_NEIGHBORS, INPUT_T + 1);
    input_data_map["neighbor_agents_past"] =
      utils::replicate_for_batch(neighbor_agents_past, params_.batch_size);
  }
  // Static objects
  // TODO(Daniel): add static objects
  {
    std::vector<int64_t> single_batch_shape(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end());
    auto static_objects_data = utils::create_float_data(single_batch_shape, 0.0f);
    input_data_map["static_objects"] =
      utils::replicate_for_batch(static_objects_data, params_.batch_size);
  }

  // map data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context_->select_lane_segment_indices(
      map_to_ego_transform, center_x, center_y, NUM_SEGMENTS_IN_LANE);
    const auto [lanes, lanes_speed_limit] = lane_segment_context_->create_tensor_data_from_indices(
      map_to_ego_transform, traffic_light_id_map_, segment_indices, NUM_SEGMENTS_IN_LANE);
    input_data_map["lanes"] = utils::replicate_for_batch(lanes, params_.batch_size);
    input_data_map["lanes_speed_limit"] =
      utils::replicate_for_batch(lanes_speed_limit, params_.batch_size);
  }

  // route data on ego reference frame
  {
    const std::vector<int64_t> segment_indices =
      lane_segment_context_->select_route_segment_indices(
        *route_ptr_, center_x, center_y, center_z, NUM_SEGMENTS_IN_ROUTE);
    const auto [route_lanes, route_lanes_speed_limit] =
      lane_segment_context_->create_tensor_data_from_indices(
        map_to_ego_transform, traffic_light_id_map_, segment_indices, NUM_SEGMENTS_IN_ROUTE);
    input_data_map["route_lanes"] = utils::replicate_for_batch(route_lanes, params_.batch_size);
    input_data_map["route_lanes_speed_limit"] =
      utils::replicate_for_batch(route_lanes_speed_limit, params_.batch_size);
  }

  // polygons
  {
    const auto & polygons =
      lane_segment_context_->create_polygon_tensor(map_to_ego_transform, center_x, center_y);
    input_data_map["polygons"] = utils::replicate_for_batch(polygons, params_.batch_size);
  }

  // line strings
  {
    const auto & line_strings =
      lane_segment_context_->create_line_string_tensor(map_to_ego_transform, center_x, center_y);
    input_data_map["line_strings"] = utils::replicate_for_batch(line_strings, params_.batch_size);
  }

  // goal pose
  {
    const auto & goal_pose = route_ptr_->goal_pose;

    // Convert goal pose to 4x4 transformation matrix
    const Eigen::Matrix4d goal_pose_map_4x4 = utils::pose_to_matrix4d(goal_pose);

    // Transform to ego frame
    const Eigen::Matrix4d goal_pose_ego_4x4 = map_to_ego_transform * goal_pose_map_4x4;

    // Extract relative position
    const float x = goal_pose_ego_4x4(0, 3);
    const float y = goal_pose_ego_4x4(1, 3);

    // Extract heading as cos/sin from rotation matrix
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(goal_pose_ego_4x4.block<3, 3>(0, 0));

    std::vector<float> single_goal_pose = {x, y, cos_yaw, sin_yaw};
    input_data_map["goal_pose"] = utils::replicate_for_batch(single_goal_pose, params_.batch_size);
  }

  // ego shape
  {
    const float wheel_base = static_cast<float>(vehicle_info_.wheel_base_m);
    const float vehicle_length = static_cast<float>(
      vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m + vehicle_info_.rear_overhang_m);
    const float vehicle_width = static_cast<float>(
      vehicle_info_.left_overhang_m + vehicle_info_.wheel_tread_m + vehicle_info_.right_overhang_m);
    std::vector<float> single_ego_shape = {wheel_base, vehicle_length, vehicle_width};
    input_data_map["ego_shape"] = utils::replicate_for_batch(single_ego_shape, params_.batch_size);
  }

  // turn indicators
  {
    // copy from back to front, and use the front value for padding if not enough history
    std::vector<float> single_turn_indicators(INPUT_T + 1, 0.0f);
    for (int64_t t = 0; t < INPUT_T + 1; ++t) {
      const int64_t index = std::max(
        static_cast<int64_t>(turn_indicators_history_.size()) - 1 - t, static_cast<int64_t>(0));
      single_turn_indicators[INPUT_T - t] = turn_indicators_history_[index].report;
    }
    input_data_map["turn_indicators"] =
      utils::replicate_for_batch(single_turn_indicators, params_.batch_size);
  }

  return input_data_map;
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
  if (!tensorrt_inference_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Model not loaded. Inference is disabled. Check onnx_model_path and args_path parameters.");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::ERROR, "Model not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  if (!lane_segment_context_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Waiting for map data...");
    diagnostics_inference_->update_level_and_message(DiagnosticStatus::WARN, "Map data not loaded");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Prepare frame context and input data for the model
  const std::optional<FrameContext> frame_context = create_frame_context();
  if (!frame_context) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "No input data available for inference");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "No input data available for inference");
    diagnostics_inference_->publish(current_time);
    return;
  }

  const rclcpp::Time frame_time(frame_context->frame_time);
  InputDataMap input_data_map = create_input_data(*frame_context);

  publish_debug_markers(input_data_map, frame_context->ego_to_map_transform, frame_time);

  // Calculate and record metrics for diagnostics using the proper logic
  const int64_t batch_idx = 0;
  const int64_t valid_lane_count = postprocess::count_valid_elements(
    input_data_map["lanes"], LANES_SHAPE[1], LANES_SHAPE[2], LANES_SHAPE[3], batch_idx);
  diagnostics_inference_->add_key_value("valid_lane_count", valid_lane_count);

  const int64_t valid_route_count = postprocess::count_valid_elements(
    input_data_map["route_lanes"], ROUTE_LANES_SHAPE[1], ROUTE_LANES_SHAPE[2], ROUTE_LANES_SHAPE[3],
    batch_idx);
  diagnostics_inference_->add_key_value("valid_route_count", valid_route_count);

  const int64_t valid_polygon_count = postprocess::count_valid_elements(
    input_data_map["polygons"], POLYGONS_SHAPE[1], POLYGONS_SHAPE[2], POLYGONS_SHAPE[3], batch_idx);
  diagnostics_inference_->add_key_value("valid_polygon_count", valid_polygon_count);

  const int64_t valid_line_string_count = postprocess::count_valid_elements(
    input_data_map["line_strings"], LINE_STRINGS_SHAPE[1], LINE_STRINGS_SHAPE[2],
    LINE_STRINGS_SHAPE[3], batch_idx);
  diagnostics_inference_->add_key_value("valid_line_string_count", valid_line_string_count);

  const int64_t valid_neighbor_count = postprocess::count_valid_elements(
    input_data_map["neighbor_agents_past"], NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2], NEIGHBOR_SHAPE[3],
    batch_idx);
  diagnostics_inference_->add_key_value("valid_neighbor_count", valid_neighbor_count);

  // normalization of data
  preprocess::normalize_input_data(input_data_map, normalization_map_);
  if (!utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    diagnostics_inference_->update_level_and_message(
      DiagnosticStatus::WARN, "Input data contains invalid values");
    diagnostics_inference_->publish(current_time);
    return;
  }

  // Run inference
  const auto inference_result = tensorrt_inference_->infer(input_data_map);
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

  // Publish turn indicators
  const int64_t prev_report = turn_indicators_history_.empty()
                                ? TurnIndicatorsReport::DISABLE
                                : turn_indicators_history_.back().report;
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
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(lanelet_map_ptr);
}

}  // namespace autoware::diffusion_planner
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diffusion_planner::DiffusionPlanner)
