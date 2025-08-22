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
#include "autoware/diffusion_planner/conversion/ego.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/marker_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <Eigen/src/Core/Matrix.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
using autoware::tensorrt_common::NetworkIO;
using autoware::tensorrt_common::ProfileDims;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;

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
  debug_processing_time_detail_pub_ = this->create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);

  set_up_params();
  normalization_map_ = utils::load_normalization_stats(params_.args_path);

  init_pointers();
  load_engine(params_.model_path);
  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  if (params_.build_only) {
    RCLCPP_INFO(get_logger(), "Build only mode enabled. Exiting after loading model.");
    std::exit(EXIT_SUCCESS);
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
  // Clean up CUDA resources
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
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
  params_.update_traffic_light_group_info =
    this->declare_parameter<bool>("update_traffic_light_group_info", false);
  params_.keep_last_traffic_light_group_info =
    this->declare_parameter<bool>("keep_last_traffic_light_group_info", false);
  params_.traffic_light_group_msg_timeout_seconds =
    this->declare_parameter<double>("traffic_light_group_msg_timeout_seconds", 0.2);
  params_.batch_size = this->declare_parameter<int>("batch_size", 1);

  // debug params
  debug_params_.publish_debug_map =
    this->declare_parameter<bool>("debug_params.publish_debug_map", false);
  debug_params_.publish_debug_route =
    this->declare_parameter<bool>("debug_params.publish_debug_route", false);
}

SetParametersResult DiffusionPlanner::on_parameter(
  [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  {
    DiffusionPlannerParams temp_params = params_;
    update_param<bool>(
      parameters, "ignore_unknown_neighbors", temp_params.ignore_unknown_neighbors);
    update_param<bool>(parameters, "ignore_neighbors", temp_params.ignore_neighbors);
    update_param<bool>(
      parameters, "predict_neighbor_trajectory", temp_params.predict_neighbor_trajectory);
    update_param<bool>(
      parameters, "update_traffic_light_group_info", temp_params.update_traffic_light_group_info);
    update_param<bool>(
      parameters, "keep_last_traffic_light_group_info",
      temp_params.keep_last_traffic_light_group_info);
    update_param<double>(
      parameters, "traffic_light_group_msg_timeout_seconds",
      temp_params.traffic_light_group_msg_timeout_seconds);
    update_param<int>(parameters, "batch_size", temp_params.batch_size);
    params_ = temp_params;
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

void DiffusionPlanner::init_pointers()
{
  const int batch_size = params_.batch_size;

  // Calculate tensor sizes with batch support
  const size_t ego_history_size =
    batch_size * std::accumulate(
                   EGO_HISTORY_SHAPE.begin() + 1, EGO_HISTORY_SHAPE.end(), 1L, std::multiplies<>());
  const size_t ego_current_state_size =
    batch_size *
    std::accumulate(
      EGO_CURRENT_STATE_SHAPE.begin() + 1, EGO_CURRENT_STATE_SHAPE.end(), 1L, std::multiplies<>());
  const size_t neighbor_agents_past_size =
    batch_size *
    std::accumulate(NEIGHBOR_SHAPE.begin() + 1, NEIGHBOR_SHAPE.end(), 1L, std::multiplies<>());
  const size_t static_objects_size =
    batch_size *
    std::accumulate(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end(), 1L, std::multiplies<>());
  const size_t lanes_size =
    batch_size *
    std::accumulate(LANES_SHAPE.begin() + 1, LANES_SHAPE.end(), 1L, std::multiplies<>());
  const size_t lanes_has_speed_limit_size =
    batch_size * std::accumulate(
                   LANES_HAS_SPEED_LIMIT_SHAPE.begin() + 1, LANES_HAS_SPEED_LIMIT_SHAPE.end(), 1L,
                   std::multiplies<>());
  const size_t lanes_speed_limit_size =
    batch_size *
    std::accumulate(
      LANES_SPEED_LIMIT_SHAPE.begin() + 1, LANES_SPEED_LIMIT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t route_lanes_size =
    batch_size * std::accumulate(
                   ROUTE_LANES_SHAPE.begin() + 1, ROUTE_LANES_SHAPE.end(), 1L, std::multiplies<>());
  const size_t route_lanes_has_speed_limit_size =
    batch_size * std::accumulate(
                   ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.begin() + 1,
                   ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t route_lanes_speed_limit_size =
    batch_size * std::accumulate(
                   ROUTE_LANES_SPEED_LIMIT_SHAPE.begin() + 1, ROUTE_LANES_SPEED_LIMIT_SHAPE.end(),
                   1L, std::multiplies<>());
  const size_t goal_pose_size =
    batch_size *
    std::accumulate(GOAL_POSE_SHAPE.begin() + 1, GOAL_POSE_SHAPE.end(), 1L, std::multiplies<>());
  const size_t ego_shape_size =
    batch_size *
    std::accumulate(EGO_SHAPE_SHAPE.begin() + 1, EGO_SHAPE_SHAPE.end(), 1L, std::multiplies<>());
  const size_t output_size =
    batch_size *
    std::accumulate(OUTPUT_SHAPE.begin() + 1, OUTPUT_SHAPE.end(), 1L, std::multiplies<>());
  const size_t turn_indicator_logit_size =
    batch_size * std::accumulate(
                   TURN_INDICATOR_LOGIT_SHAPE.begin() + 1, TURN_INDICATOR_LOGIT_SHAPE.end(), 1L,
                   std::multiplies<>());

  ego_history_d_ = autoware::cuda_utils::make_unique<float[]>(ego_history_size);
  ego_current_state_d_ = autoware::cuda_utils::make_unique<float[]>(ego_current_state_size);
  neighbor_agents_past_d_ = autoware::cuda_utils::make_unique<float[]>(neighbor_agents_past_size);
  static_objects_d_ = autoware::cuda_utils::make_unique<float[]>(static_objects_size);
  lanes_d_ = autoware::cuda_utils::make_unique<float[]>(lanes_size);
  lanes_has_speed_limit_d_ = autoware::cuda_utils::make_unique<bool[]>(lanes_has_speed_limit_size);
  lanes_speed_limit_d_ = autoware::cuda_utils::make_unique<float[]>(lanes_speed_limit_size);
  route_lanes_d_ = autoware::cuda_utils::make_unique<float[]>(route_lanes_size);
  route_lanes_has_speed_limit_d_ =
    autoware::cuda_utils::make_unique<bool[]>(route_lanes_has_speed_limit_size);
  route_lanes_speed_limit_d_ =
    autoware::cuda_utils::make_unique<float[]>(route_lanes_speed_limit_size);
  goal_pose_d_ = autoware::cuda_utils::make_unique<float[]>(goal_pose_size);
  ego_shape_d_ = autoware::cuda_utils::make_unique<float[]>(ego_shape_size);

  // Output
  output_d_ = autoware::cuda_utils::make_unique<float[]>(output_size);
  turn_indicator_logit_d_ = autoware::cuda_utils::make_unique<float[]>(turn_indicator_logit_size);
}

void DiffusionPlanner::load_engine(const std::string & model_path)
{
  const int batch_size = params_.batch_size;

  // Convert std::array to nvinfer1::Dims with dynamic batch dimension
  auto to_dynamic_dims = [batch_size](auto const & arr) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(arr.size());
    // TensorRT requires the batch dimension to be explicitly set to 1 when batch_size == 1
    dims.d[0] = (batch_size == 1 ? 1 : -1);
    for (size_t i = 1; i < arr.size(); ++i) {
      dims.d[i] = static_cast<int>(arr[i]);
    }
    return dims;
  };

  auto make_dynamic_dims = [batch_size](const std::string & name, const nvinfer1::Dims & dims) {
    nvinfer1::Dims min_dims = dims, opt_dims = dims, max_dims = dims;
    min_dims.d[0] = 1;
    opt_dims.d[0] = batch_size;
    max_dims.d[0] = batch_size;
    return ProfileDims{name, min_dims, opt_dims, max_dims};
  };

  std::string precision = "fp32";  // Default precision

  // Create engine path with batch size suffix to avoid conflicts
  std::filesystem::path engine_path(model_path);
  std::string engine_file_path =
    (engine_path.parent_path() /
     (engine_path.stem().string() + "_batch" + std::to_string(batch_size) + ".engine"))
      .string();

  auto trt_config = tensorrt_common::TrtCommonConfig(model_path, precision, engine_file_path);
  trt_common_ = std::make_unique<TrtConvCalib>(trt_config);

  std::vector<ProfileDims> profile_dims;

  {
    profile_dims.emplace_back(
      make_dynamic_dims("ego_agent_past", to_dynamic_dims(EGO_HISTORY_SHAPE)));
    profile_dims.emplace_back(
      make_dynamic_dims("ego_current_state", to_dynamic_dims(EGO_CURRENT_STATE_SHAPE)));
    profile_dims.emplace_back(
      make_dynamic_dims("neighbor_agents_past", to_dynamic_dims(NEIGHBOR_SHAPE)));
    profile_dims.emplace_back(
      make_dynamic_dims("static_objects", to_dynamic_dims(STATIC_OBJECTS_SHAPE)));
    profile_dims.emplace_back(make_dynamic_dims("lanes", to_dynamic_dims(LANES_SHAPE)));
    profile_dims.emplace_back(
      make_dynamic_dims("lanes_speed_limit", to_dynamic_dims(LANES_SPEED_LIMIT_SHAPE)));
    profile_dims.emplace_back(
      make_dynamic_dims("lanes_has_speed_limit", to_dynamic_dims(LANES_HAS_SPEED_LIMIT_SHAPE)));
    profile_dims.emplace_back(make_dynamic_dims("route_lanes", to_dynamic_dims(ROUTE_LANES_SHAPE)));
    profile_dims.emplace_back(make_dynamic_dims(
      "route_lanes_has_speed_limit", to_dynamic_dims(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE)));
    profile_dims.emplace_back(
      make_dynamic_dims("route_lanes_speed_limit", to_dynamic_dims(ROUTE_LANES_SPEED_LIMIT_SHAPE)));
    profile_dims.emplace_back(make_dynamic_dims("goal_pose", to_dynamic_dims(GOAL_POSE_SHAPE)));
    profile_dims.emplace_back(make_dynamic_dims("ego_shape", to_dynamic_dims(EGO_SHAPE_SHAPE)));
  }

  std::vector<autoware::tensorrt_common::NetworkIO> network_io;
  {  // Inputs with dynamic batch dimension
    network_io.emplace_back("ego_agent_past", to_dynamic_dims(EGO_HISTORY_SHAPE));
    network_io.emplace_back("ego_current_state", to_dynamic_dims(EGO_CURRENT_STATE_SHAPE));
    network_io.emplace_back("neighbor_agents_past", to_dynamic_dims(NEIGHBOR_SHAPE));
    network_io.emplace_back("static_objects", to_dynamic_dims(STATIC_OBJECTS_SHAPE));
    network_io.emplace_back("lanes", to_dynamic_dims(LANES_SHAPE));
    network_io.emplace_back("lanes_has_speed_limit", to_dynamic_dims(LANES_HAS_SPEED_LIMIT_SHAPE));
    network_io.emplace_back("lanes_speed_limit", to_dynamic_dims(LANES_SPEED_LIMIT_SHAPE));
    network_io.emplace_back("route_lanes", to_dynamic_dims(ROUTE_LANES_SHAPE));
    network_io.emplace_back(
      "route_lanes_has_speed_limit", to_dynamic_dims(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
    network_io.emplace_back(
      "route_lanes_speed_limit", to_dynamic_dims(ROUTE_LANES_SPEED_LIMIT_SHAPE));
    network_io.emplace_back("goal_pose", to_dynamic_dims(GOAL_POSE_SHAPE));
    network_io.emplace_back("ego_shape", to_dynamic_dims(EGO_SHAPE_SHAPE));

    // Output with dynamic batch dimension
    network_io.emplace_back("prediction", to_dynamic_dims(OUTPUT_SHAPE));
    network_io.emplace_back("turn_indicator_logit", to_dynamic_dims(TURN_INDICATOR_LOGIT_SHAPE));
  }
  auto network_io_ptr = std::make_unique<std::vector<NetworkIO>>(network_io);
  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);

  network_trt_ptr_ = std::make_unique<TrtCommon>(
    trt_config, std::make_shared<Profiler>(), std::vector<std::string>{params_.plugins_path});

  if (!network_trt_ptr_->setup(std::move(profile_dims_ptr), std::move(network_io_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine." + params_.plugins_path);
  }

  // For dynamic batch size, we don't set input shapes here - they will be set at inference time
}

AgentData DiffusionPlanner::get_ego_centric_agent_data(
  const TrackedObjects & objects, const Eigen::Matrix4f & map_to_ego_transform)
{
  if (!agent_data_) {
    agent_data_ =
      AgentData(objects, NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2], params_.ignore_unknown_neighbors);
  } else {
    agent_data_->update_histories(objects, params_.ignore_unknown_neighbors);
  }

  auto ego_centric_agent_data = agent_data_.value();
  ego_centric_agent_data.apply_transform(map_to_ego_transform);
  ego_centric_agent_data.trim_to_k_closest_agents();
  return ego_centric_agent_data;
}

std::vector<float> DiffusionPlanner::create_ego_agent_past(
  const Eigen::Matrix4f & map_to_ego_transform)
{
  const size_t max_timesteps = EGO_HISTORY_SHAPE[1];
  const size_t features_per_timestep = EGO_HISTORY_SHAPE[2];  // 4 (x, y, cos, sin)
  const size_t single_size = max_timesteps * features_per_timestep;

  std::vector<float> single_ego_agent_past(single_size, 0.0f);

  // Fill ego history data
  const size_t history_size = ego_history_.size();
  const size_t start_idx = (history_size >= max_timesteps) ? history_size - max_timesteps : 0;

  for (size_t i = start_idx; i < history_size; ++i) {
    const auto & historical_pose = ego_history_[i].pose.pose;

    // Convert pose to 4x4 matrix
    const Eigen::Matrix4f pose_map_4x4 = utils::pose_to_matrix4f(historical_pose);

    // Transform to ego frame
    const Eigen::Matrix4f pose_ego_4x4 = map_to_ego_transform * pose_map_4x4;

    // Extract position
    const float x = pose_ego_4x4(0, 3);
    const float y = pose_ego_4x4(1, 3);

    // Extract heading as cos/sin
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(pose_ego_4x4.block<3, 3>(0, 0));

    // Store in flat array: [timestep, features]
    const size_t timestep_idx = i - start_idx;
    const size_t base_idx = timestep_idx * features_per_timestep;
    single_ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_X] = x;
    single_ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_Y] = y;
    single_ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_COS] = cos_yaw;
    single_ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_SIN] = sin_yaw;
  }

  return replicate_for_batch(single_ego_agent_past);
}

InputDataMap DiffusionPlanner::create_input_data()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  InputDataMap input_data_map;
  auto objects = sub_tracked_objects_.take_data();
  auto ego_kinematic_state = sub_current_odometry_.take_data();
  auto ego_acceleration = sub_current_acceleration_.take_data();
  auto traffic_signals = sub_traffic_signals_.take_data();
  auto temp_route_ptr = route_subscriber_.take_data();

  route_ptr_ = (!route_ptr_ || temp_route_ptr) ? temp_route_ptr : route_ptr_;

  TrackedObjects empty_object_list;

  if (params_.ignore_neighbors) {
    objects = std::make_shared<TrackedObjects>(empty_object_list);
  }

  if (!objects || !ego_kinematic_state || !ego_acceleration || !route_ptr_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "No tracked objects or ego kinematic state or route data received");
    return {};
  }

  route_handler_->setRoute(*route_ptr_);
  if (params_.update_traffic_light_group_info) {
    const auto & traffic_light_msg_timeout_s = params_.traffic_light_group_msg_timeout_seconds;
    preprocess::process_traffic_signals(
      traffic_signals, traffic_light_id_map_, this->now(), traffic_light_msg_timeout_s,
      params_.keep_last_traffic_light_group_info);
    if (!traffic_signals) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
        "no traffic signal received. traffic light info will not be updated/used");
    }
  }

  ego_kinematic_state_ = *ego_kinematic_state;

  // Add current state to ego history
  ego_history_.push_back(*ego_kinematic_state);
  if (ego_history_.size() > static_cast<size_t>(EGO_HISTORY_SHAPE[1])) {
    ego_history_.pop_front();
  }

  transforms_ = utils::get_transform_matrix(*ego_kinematic_state);
  const auto & map_to_ego_transform = transforms_.second;
  const auto & center_x = static_cast<float>(ego_kinematic_state->pose.pose.position.x);
  const auto & center_y = static_cast<float>(ego_kinematic_state->pose.pose.position.y);

  // Ego history
  {
    input_data_map["ego_agent_past"] = create_ego_agent_past(map_to_ego_transform);
  }
  // Ego state
  {
    EgoState ego_state(
      *ego_kinematic_state, *ego_acceleration, static_cast<float>(vehicle_info_.wheel_base_m));
    input_data_map["ego_current_state"] = replicate_for_batch(ego_state.as_array());
  }
  // Agent data on ego reference frame
  {
    auto neighbor_data = get_ego_centric_agent_data(*objects, map_to_ego_transform).as_vector();
    input_data_map["neighbor_agents_past"] = replicate_for_batch(neighbor_data);
  }
  // Static objects
  // TODO(Daniel): add static objects
  {
    std::vector<int64_t> single_batch_shape(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end());
    auto static_objects_data = utils::create_float_data(single_batch_shape, 0.0f);
    input_data_map["static_objects"] = replicate_for_batch(static_objects_data);
  }

  // map data on ego reference frame
  {
    const auto [lanes, lanes_speed_limit] = lane_segment_context_->get_lane_segments(
      map_to_ego_transform, traffic_light_id_map_, center_x, center_y, LANES_SHAPE[1]);
    input_data_map["lanes"] = replicate_for_batch(lanes);
    input_data_map["lanes_speed_limit"] = replicate_for_batch(lanes_speed_limit);
  }

  // route data on ego reference frame
  {
    const auto & current_pose = ego_kinematic_state->pose.pose;
    constexpr double backward_path_length{constants::BACKWARD_PATH_LENGTH_M};
    constexpr double forward_path_length{constants::FORWARD_PATH_LENGTH_M};
    lanelet::ConstLanelet current_preferred_lane;

    if (
      !route_handler_->isHandlerReady() || !route_handler_->getClosestPreferredLaneletWithinRoute(
                                             current_pose, &current_preferred_lane)) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
        "failed to find closest lanelet within route!!!");
      return {};
    }
    auto current_lanes = route_handler_->getLaneletSequence(
      current_preferred_lane, backward_path_length, forward_path_length);

    const auto [route_lanes, route_lanes_speed_limit] = lane_segment_context_->get_route_segments(
      map_to_ego_transform, traffic_light_id_map_, current_lanes);
    input_data_map["route_lanes"] = replicate_for_batch(route_lanes);
    input_data_map["route_lanes_speed_limit"] = replicate_for_batch(route_lanes_speed_limit);
  }

  // goal pose
  {
    const auto & goal_pose = route_handler_->getGoalPose();

    // Convert goal pose to 4x4 transformation matrix
    const Eigen::Matrix4f goal_pose_map_4x4 = utils::pose_to_matrix4f(goal_pose);

    // Transform to ego frame
    const Eigen::Matrix4f goal_pose_ego_4x4 = map_to_ego_transform * goal_pose_map_4x4;

    // Extract relative position
    const float x = goal_pose_ego_4x4(0, 3);
    const float y = goal_pose_ego_4x4(1, 3);

    // Extract heading as cos/sin from rotation matrix
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(goal_pose_ego_4x4.block<3, 3>(0, 0));

    std::vector<float> single_goal_pose = {x, y, cos_yaw, sin_yaw};
    input_data_map["goal_pose"] = replicate_for_batch(single_goal_pose);
  }

  // ego shape
  {
    const float wheel_base = static_cast<float>(vehicle_info_.wheel_base_m);
    const float vehicle_length = static_cast<float>(
      vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m + vehicle_info_.rear_overhang_m);
    const float vehicle_width = static_cast<float>(
      vehicle_info_.left_overhang_m + vehicle_info_.wheel_tread_m + vehicle_info_.right_overhang_m);
    std::vector<float> single_ego_shape = {wheel_base, vehicle_length, vehicle_width};
    input_data_map["ego_shape"] = replicate_for_batch(single_ego_shape);
  }

  return input_data_map;
}

std::vector<float> DiffusionPlanner::replicate_for_batch(const std::vector<float> & single_data)
{
  const int batch_size = params_.batch_size;
  const size_t single_size = single_data.size();
  const size_t total_size = static_cast<size_t>(batch_size) * single_size;

  std::vector<float> batch_data;
  batch_data.reserve(total_size);

  for (int i = 0; i < batch_size; ++i) {
    batch_data.insert(batch_data.end(), single_data.begin(), single_data.end());
  }

  return batch_data;
}

void DiffusionPlanner::publish_debug_markers(InputDataMap & input_data_map) const
{
  if (debug_params_.publish_debug_route) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto route_markers = utils::create_lane_marker(
      transforms_.first, input_data_map["route_lanes"],
      std::vector<int64_t>(ROUTE_LANES_SHAPE.begin(), ROUTE_LANES_SHAPE.end()), this->now(),
      lifetime, {0.8, 0.8, 0.8, 0.8}, "map", true);
    pub_route_marker_->publish(route_markers);
  }

  if (debug_params_.publish_debug_map) {
    auto lifetime = rclcpp::Duration::from_seconds(0.2);
    auto lane_markers = utils::create_lane_marker(
      transforms_.first, input_data_map["lanes"],
      std::vector<int64_t>(LANES_SHAPE.begin(), LANES_SHAPE.end()), this->now(), lifetime,
      {0.1, 0.1, 0.7, 0.8}, "map", true);
    pub_lane_marker_->publish(lane_markers);
  }
}

void DiffusionPlanner::publish_predictions(const std::vector<float> & predictions) const
{
  constexpr int64_t batch_idx = 0;
  constexpr int64_t ego_agent_idx = 0;

  const autoware_planning_msgs::msg::Trajectory output_trajectory = postprocess::create_trajectory(
    predictions, this->now(), transforms_.first, batch_idx, ego_agent_idx);
  pub_trajectory_->publish(output_trajectory);

  // Publish all batch results as candidate trajectories
  const int batch_size = params_.batch_size;

  // Start with first trajectory as main candidate
  CandidateTrajectories ego_trajectory_as_candidate_msg =
    postprocess::to_candidate_trajectories_msg(
      output_trajectory, generator_uuid_, "DiffusionPlanner");

  // Add additional batch results as more candidates
  for (int i = 1; i < batch_size; i++) {
    const Trajectory trajectory =
      postprocess::create_trajectory(predictions, this->now(), transforms_.first, i, ego_agent_idx);

    const CandidateTrajectories additional_candidate = postprocess::to_candidate_trajectories_msg(
      trajectory, generator_uuid_, "DiffusionPlanner_batch_" + std::to_string(i));

    ego_trajectory_as_candidate_msg.candidate_trajectories.push_back(
      additional_candidate.candidate_trajectories[0]);
    ego_trajectory_as_candidate_msg.generator_info.push_back(
      additional_candidate.generator_info[0]);
  }

  pub_trajectories_->publish(ego_trajectory_as_candidate_msg);

  // Other agents prediction
  if (params_.predict_neighbor_trajectory && agent_data_.has_value()) {
    auto reduced_agent_data = agent_data_.value();
    reduced_agent_data.trim_to_k_closest_agents(ego_kinematic_state_.pose.pose.position);
    const size_t single_batch_output_size =
      std::accumulate(OUTPUT_SHAPE.begin() + 1, OUTPUT_SHAPE.end(), 1UL, std::multiplies<>());
    const std::vector<float> single_batch_predictions(
      predictions.begin(), predictions.begin() + single_batch_output_size);
    auto predicted_objects = postprocess::create_predicted_objects(
      single_batch_predictions, reduced_agent_data, this->now(), transforms_.first);
    pub_objects_->publish(predicted_objects);
  }
}

std::vector<float> DiffusionPlanner::do_inference_trt(InputDataMap & input_data_map)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  auto ego_history = input_data_map["ego_agent_past"];
  auto ego_current_state = input_data_map["ego_current_state"];
  auto neighbor_agents_past = input_data_map["neighbor_agents_past"];
  auto static_objects = input_data_map["static_objects"];
  auto lanes = input_data_map["lanes"];
  auto lanes_speed_limit = input_data_map["lanes_speed_limit"];
  auto route_lanes = input_data_map["route_lanes"];
  auto route_lanes_speed_limit = input_data_map["route_lanes_speed_limit"];
  auto goal_pose = input_data_map["goal_pose"];
  auto ego_shape = input_data_map["ego_shape"];

  // Allocate bool array for lane speed limits
  // Note: Using std::vector<uint8_t> instead of std::vector<bool> to ensure contiguous memory
  // layout
  const int batch_size = params_.batch_size;
  size_t lane_speed_tensor_num_elements =
    batch_size *
    std::accumulate(
      LANES_SPEED_LIMIT_SHAPE.begin() + 1, LANES_SPEED_LIMIT_SHAPE.end(), 1, std::multiplies<>());
  std::vector<uint8_t> speed_bool_array(lane_speed_tensor_num_elements);

  for (size_t i = 0; i < lane_speed_tensor_num_elements; ++i) {
    speed_bool_array[i] = (lanes_speed_limit[i] > std::numeric_limits<float>::epsilon()) ? 1 : 0;
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    ego_history_d_.get(), ego_history.data(), ego_history.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    ego_current_state_d_.get(), ego_current_state.data(), ego_current_state.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    neighbor_agents_past_d_.get(), neighbor_agents_past.data(),
    neighbor_agents_past.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    static_objects_d_.get(), static_objects.data(), static_objects.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(
    cudaMemcpy(lanes_d_.get(), lanes.data(), lanes.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    lanes_speed_limit_d_.get(), lanes_speed_limit.data(), lanes_speed_limit.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    route_lanes_d_.get(), route_lanes.data(), route_lanes.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  // Copy uint8_t array to bool array on device
  // Note: sizeof(bool) might be implementation-specific, but CUDA typically uses 1 byte for bool
  CHECK_CUDA_ERROR(cudaMemcpy(
    lanes_has_speed_limit_d_.get(), speed_bool_array.data(),
    lane_speed_tensor_num_elements * sizeof(uint8_t), cudaMemcpyHostToDevice));

  // Allocate bool array for route lanes speed limits
  size_t route_lanes_has_speed_limit_tensor_num_elements =
    batch_size * std::accumulate(
                   ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.begin() + 1,
                   ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE.end(), 1, std::multiplies<>());
  std::vector<uint8_t> route_has_speed_bool_array(route_lanes_has_speed_limit_tensor_num_elements);
  for (size_t i = 0; i < route_lanes_has_speed_limit_tensor_num_elements; ++i) {
    route_has_speed_bool_array[i] =
      (route_lanes_speed_limit[i] > std::numeric_limits<float>::epsilon()) ? 1 : 0;
  }

  CHECK_CUDA_ERROR(cudaMemcpy(
    route_lanes_speed_limit_d_.get(), route_lanes_speed_limit.data(),
    route_lanes_speed_limit.size() * sizeof(float), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    route_lanes_has_speed_limit_d_.get(), route_has_speed_bool_array.data(),
    route_lanes_has_speed_limit_tensor_num_elements * sizeof(uint8_t), cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    goal_pose_d_.get(), goal_pose.data(), goal_pose.size() * sizeof(float),
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaMemcpy(
    ego_shape_d_.get(), ego_shape.data(), ego_shape.size() * sizeof(float),
    cudaMemcpyHostToDevice));

  // Set input shapes for current batch size
  auto to_dims_with_batch = [batch_size](auto const & arr) {
    nvinfer1::Dims dims;
    dims.nbDims = static_cast<int>(arr.size());
    dims.d[0] = batch_size;
    for (size_t i = 1; i < arr.size(); ++i) {
      dims.d[i] = static_cast<int>(arr[i]);
    }
    return dims;
  };

  bool set_input_shapes = true;
  set_input_shapes &=
    network_trt_ptr_->setInputShape("ego_agent_past", to_dims_with_batch(EGO_HISTORY_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "ego_current_state", to_dims_with_batch(EGO_CURRENT_STATE_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("neighbor_agents_past", to_dims_with_batch(NEIGHBOR_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("static_objects", to_dims_with_batch(STATIC_OBJECTS_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape("lanes", to_dims_with_batch(LANES_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "lanes_has_speed_limit", to_dims_with_batch(LANES_HAS_SPEED_LIMIT_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "lanes_speed_limit", to_dims_with_batch(LANES_SPEED_LIMIT_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("route_lanes", to_dims_with_batch(ROUTE_LANES_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "route_lanes_speed_limit", to_dims_with_batch(ROUTE_LANES_SPEED_LIMIT_SHAPE));
  set_input_shapes &= network_trt_ptr_->setInputShape(
    "route_lanes_has_speed_limit", to_dims_with_batch(ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("goal_pose", to_dims_with_batch(GOAL_POSE_SHAPE));
  set_input_shapes &=
    network_trt_ptr_->setInputShape("ego_shape", to_dims_with_batch(EGO_SHAPE_SHAPE));

  if (!set_input_shapes) {
    RCLCPP_ERROR(
      rclcpp::get_logger("diffusion_planner"), "Failed to set input shapes for inference.");
    return {};
  }

  network_trt_ptr_->setTensorAddress("ego_agent_past", ego_history_d_.get());
  network_trt_ptr_->setTensorAddress("ego_current_state", ego_current_state_d_.get());
  network_trt_ptr_->setTensorAddress("neighbor_agents_past", neighbor_agents_past_d_.get());
  network_trt_ptr_->setTensorAddress("static_objects", static_objects_d_.get());
  network_trt_ptr_->setTensorAddress("lanes", lanes_d_.get());
  network_trt_ptr_->setTensorAddress("lanes_has_speed_limit", lanes_has_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("lanes_speed_limit", lanes_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("route_lanes", route_lanes_d_.get());
  network_trt_ptr_->setTensorAddress("route_lanes_speed_limit", route_lanes_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress(
    "route_lanes_has_speed_limit", route_lanes_has_speed_limit_d_.get());
  network_trt_ptr_->setTensorAddress("goal_pose", goal_pose_d_.get());
  network_trt_ptr_->setTensorAddress("ego_shape", ego_shape_d_.get());

  // Output
  network_trt_ptr_->setTensorAddress("prediction", output_d_.get());
  network_trt_ptr_->setTensorAddress("turn_indicator_logit", turn_indicator_logit_d_.get());

  auto status = network_trt_ptr_->enqueueV3(stream_);
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger("diffusion_planner"), "Failed to enqueue and do inference.");
  }

  // Compute total number of elements in the output
  size_t output_num_elements =
    batch_size *
    std::accumulate(OUTPUT_SHAPE.begin() + 1, OUTPUT_SHAPE.end(), 1UL, std::multiplies<>());

  // Allocate host vector
  std::vector<float> output_host(output_num_elements);

  // Copy data from device to host
  cudaMemcpy(
    output_host.data(),  // destination (host)
    output_d_.get(),     // source (device)
    output_num_elements * sizeof(float), cudaMemcpyDeviceToHost);
  return output_host;
}

void DiffusionPlanner::on_timer()
{
  // Timer callback function
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  if (!is_map_loaded_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Waiting for map data...");
    return;
  }

  // Prepare input data for the model
  auto input_data_map = create_input_data();
  if (input_data_map.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "No input data available for inference");
    return;
  }

  publish_debug_markers(input_data_map);

  // normalization of data
  preprocess::normalize_input_data(input_data_map, normalization_map_);
  if (!utils::check_input_map(input_data_map)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *this->get_clock(), constants::LOG_THROTTLE_INTERVAL_MS,
      "Input data contains invalid values");
    return;
  }
  const auto predictions = do_inference_trt(input_data_map);
  publish_predictions(predictions);
}

void DiffusionPlanner::on_map(const HADMapBin::ConstSharedPtr map_msg)
{
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *map_msg, lanelet_map_ptr, &traffic_rules_ptr_, &routing_graph_ptr_);

  // Create LaneSegmentContext with the static data
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(lanelet_map_ptr);

  route_handler_->setMap(*map_msg);
  is_map_loaded_ = true;
}

}  // namespace autoware::diffusion_planner
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diffusion_planner::DiffusionPlanner)
