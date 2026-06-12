// Copyright 2026 TIER IV, Inc.
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

#include "autoware/tensorrt_oneplanner/oneplanner_core.hpp"

#include "autoware/tensorrt_oneplanner/dimensions.hpp"

#include <autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/diffusion_planner/utils/utils.hpp>
#include <nlohmann/json.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

namespace dp = autoware::diffusion_planner;
namespace preprocess = autoware::diffusion_planner::preprocess;
namespace postprocess = autoware::diffusion_planner::postprocess;
namespace utils = autoware::diffusion_planner::utils;

using dp::EGO_HISTORY_SHAPE;
using dp::INPUT_T;
using dp::MAX_NUM_AGENTS;
using dp::NEIGHBOR_SHAPE;
using dp::NUM_SEGMENTS_IN_LANE;
using dp::NUM_SEGMENTS_IN_ROUTE;
using dp::OUTPUT_T;
using dp::POSE_DIM;
using dp::STATIC_OBJECTS_SHAPE;
using dp::TURN_INDICATOR_OUTPUT_DIM;
using dp::TURN_INDICATORS_SHAPE;

namespace
{
/**
 * @brief Expand the ego-only prediction [1, 1, T, 4] to the diffusion-planner layout
 *        [1, MAX_NUM_AGENTS, T, 4] (neighbors zero-filled) so that the diffusion-planner
 *        postprocessing utilities can be reused unchanged.
 */
std::vector<float> expand_ego_prediction(const std::vector<float> & ego_prediction)
{
  const size_t agent_stride = OUTPUT_T * POSE_DIM;
  if (ego_prediction.size() != agent_stride) {
    throw std::runtime_error(
      "Unexpected ego prediction size: " + std::to_string(ego_prediction.size()) + " (expected " +
      std::to_string(agent_stride) + ")");
  }
  std::vector<float> expanded(MAX_NUM_AGENTS * agent_stride, 0.0f);
  std::copy(ego_prediction.begin(), ego_prediction.end(), expanded.begin());
  return expanded;
}
}  // namespace

OnePlannerCore::OnePlannerCore(const OnePlannerParams & params, const VehicleInfo & vehicle_info)
: params_(params),
  vehicle_spec_(vehicle_info),
  turn_indicator_manager_(
    rclcpp::Duration::from_seconds(params.turn_indicator_hold_duration),
    params.turn_indicator_keep_offset)
{
}

void OnePlannerCore::load_normalization()
{
  std::ifstream file(params_.args_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open the args file: " + params_.args_path);
  }
  nlohmann::json j;
  file >> j;
  if (!j.contains("major_version")) {
    throw std::runtime_error("Missing 'major_version' key in " + params_.args_path);
  }
  const int major_version = j["major_version"].get<int>();
  if (major_version != WEIGHT_MAJOR_VERSION) {
    throw std::runtime_error(
      "Unsupported OnePlanner weight major_version: " + std::to_string(major_version) +
      " (expected " + std::to_string(WEIGHT_MAJOR_VERSION) +
      "). Please update the model artifacts or this node.");
  }

  normalization_map_ = utils::load_normalization_stats(params_.args_path);
  last_agent_poses_map_.clear();
}

void OnePlannerCore::update_params(const OnePlannerParams & params)
{
  params_ = params;
  turn_indicator_manager_.set_hold_duration(
    rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration));
  turn_indicator_manager_.set_keep_offset(params_.turn_indicator_keep_offset);
}

void OnePlannerCore::set_map(const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(
    lanelet_map_ptr, params_.line_string_max_step_m);
}

std::optional<OnePlannerFrameContext> OnePlannerCore::create_frame_context(
  const std::shared_ptr<const Odometry> & ego_kinematic_state,
  const std::shared_ptr<const AccelWithCovarianceStamped> & ego_acceleration,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
  const LaneletRoute::ConstSharedPtr & route_ptr, const rclcpp::Time & current_time)
{
  route_ptr_ = (!route_ptr_ || route_ptr) ? route_ptr : route_ptr_;

  if (!ego_kinematic_state || !ego_acceleration || !turn_indicators) {
    return std::nullopt;
  }
  if (!route_ptr_) {
    return std::nullopt;
  }

  Odometry kinematic_state = *ego_kinematic_state;
  if (params_.shift_x) {
    kinematic_state.pose.pose =
      utils::shift_x(kinematic_state.pose.pose, vehicle_spec_.base_link_to_center);
  }

  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(kinematic_state.pose.pose);

  ego_history_.push_back(kinematic_state);
  if (ego_history_.size() > static_cast<size_t>(EGO_HISTORY_SHAPE[1])) {
    ego_history_.pop_front();
  }

  turn_indicators_history_.push_back(*turn_indicators);
  if (turn_indicators_history_.size() > static_cast<size_t>(TURN_INDICATORS_SHAPE[1])) {
    turn_indicators_history_.pop_front();
  }

  preprocess::process_traffic_signals(
    traffic_signals, traffic_light_id_map_, current_time,
    params_.traffic_light_group_msg_timeout_seconds);

  const rclcpp::Time frame_time(ego_kinematic_state->header.stamp);
  return OnePlannerFrameContext{
    *ego_kinematic_state, *ego_acceleration, ego_to_map_transform, frame_time};
}

InputDataMap OnePlannerCore::create_input_data(const OnePlannerFrameContext & frame_context)
{
  InputDataMap input_data_map;

  const geometry_msgs::msg::Pose & pose_center =
    params_.shift_x
      ? utils::shift_x(
          frame_context.ego_kinematic_state.pose.pose, vehicle_spec_.base_link_to_center)
      : frame_context.ego_kinematic_state.pose.pose;
  const Eigen::Matrix4d ego_to_map_transform = utils::pose_to_matrix4d(pose_center);
  const Eigen::Matrix4d map_to_ego_transform = utils::inverse(ego_to_map_transform);
  const auto center_x = static_cast<float>(pose_center.position.x);
  const auto center_y = static_cast<float>(pose_center.position.y);
  const auto center_z = static_cast<float>(pose_center.position.z);

  // sampled trajectories (with warm start copied from the previous output)
  int64_t delay_step = 0;
  {
    const int64_t copy_steps = std::clamp<int64_t>(params_.delay_step, 0, OUTPUT_T / 2);
    const bool has_previous_output = !last_agent_poses_map_.empty();

    std::vector<float> sampled_trajectories =
      preprocess::create_sampled_trajectories(params_.temperature);

    if (has_previous_output) {
      constexpr int64_t agent_idx = 0;
      delay_step = copy_steps;
      for (int64_t t = 0; t <= copy_steps; ++t) {
        const size_t dst_base = agent_idx * (OUTPUT_T + 1) * POSE_DIM + t * POSE_DIM;
        const Eigen::Matrix4d pose_ego =
          map_to_ego_transform * last_agent_poses_map_[0][agent_idx][t];
        const auto shifted_x = static_cast<float>(pose_ego(0, 3));
        const auto shifted_y = static_cast<float>(pose_ego(1, 3));
        const auto [shifted_cos, shifted_sin] =
          utils::rotation_matrix_to_cos_sin(pose_ego.block<3, 3>(0, 0));

        sampled_trajectories[dst_base + 0] = (shifted_x - 10.0f) / 20.0f;
        sampled_trajectories[dst_base + 1] = shifted_y / 20.0f;
        sampled_trajectories[dst_base + 2] = shifted_cos;
        sampled_trajectories[dst_base + 3] = shifted_sin;
      }
    }

    input_data_map["sampled_trajectories"] = std::move(sampled_trajectories);
  }

  // Ego history
  {
    const std::optional<rclcpp::Time> reference_time =
      params_.use_time_interpolation ? std::make_optional(frame_context.frame_time) : std::nullopt;
    input_data_map["ego_agent_past"] = preprocess::create_ego_agent_past(
      ego_history_, EGO_HISTORY_SHAPE[1], map_to_ego_transform, reference_time);
  }
  // Ego state
  {
    input_data_map["ego_current_state"] = preprocess::create_ego_current_state(
      frame_context.ego_kinematic_state, frame_context.ego_acceleration,
      static_cast<float>(vehicle_spec_.wheel_base));
  }
  // Neighbor agents: zeros. OnePlanner conditions on the LiDAR BEV feature map instead of
  // tracked objects; the tensor is kept only for graph compatibility.
  {
    std::vector<int64_t> single_batch_shape(NEIGHBOR_SHAPE.begin() + 1, NEIGHBOR_SHAPE.end());
    input_data_map["neighbor_agents_past"] = utils::create_float_data(single_batch_shape, 0.0f);
  }
  // Static objects: zeros (same as the diffusion planner)
  {
    std::vector<int64_t> single_batch_shape(
      STATIC_OBJECTS_SHAPE.begin() + 1, STATIC_OBJECTS_SHAPE.end());
    input_data_map["static_objects"] = utils::create_float_data(single_batch_shape, 0.0f);
  }

  // map data on ego reference frame
  {
    const std::vector<int64_t> segment_indices = lane_segment_context_->select_lane_segment_indices(
      map_to_ego_transform, center_x, center_y, NUM_SEGMENTS_IN_LANE);
    const auto [lanes, lanes_speed_limit] = lane_segment_context_->create_tensor_data_from_indices(
      map_to_ego_transform, traffic_light_id_map_, segment_indices, NUM_SEGMENTS_IN_LANE);
    input_data_map["lanes"] = lanes;
    input_data_map["lanes_speed_limit"] = lanes_speed_limit;
  }

  // route data on ego reference frame
  {
    const std::vector<int64_t> segment_indices =
      lane_segment_context_->select_route_segment_indices(
        *route_ptr_, center_x, center_y, center_z, NUM_SEGMENTS_IN_ROUTE);
    const auto [route_lanes, route_lanes_speed_limit] =
      lane_segment_context_->create_tensor_data_from_indices(
        map_to_ego_transform, traffic_light_id_map_, segment_indices, NUM_SEGMENTS_IN_ROUTE);
    input_data_map["route_lanes"] = route_lanes;
    input_data_map["route_lanes_speed_limit"] = route_lanes_speed_limit;
  }

  // polygons
  input_data_map["polygons"] =
    lane_segment_context_->create_polygon_tensor(map_to_ego_transform, center_x, center_y);

  // line strings
  input_data_map["line_strings"] =
    lane_segment_context_->create_line_string_tensor(map_to_ego_transform, center_x, center_y);

  // goal pose
  {
    const Eigen::Matrix4d goal_pose_map_4x4 = utils::pose_to_matrix4d(route_ptr_->goal_pose);
    const Eigen::Matrix4d goal_pose_ego_4x4 = map_to_ego_transform * goal_pose_map_4x4;
    const float x = goal_pose_ego_4x4(0, 3);
    const float y = goal_pose_ego_4x4(1, 3);
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(goal_pose_ego_4x4.block<3, 3>(0, 0));
    input_data_map["goal_pose"] = {x, y, cos_yaw, sin_yaw};
  }

  // ego shape
  input_data_map["ego_shape"] = {
    static_cast<float>(vehicle_spec_.wheel_base), static_cast<float>(vehicle_spec_.vehicle_length),
    static_cast<float>(vehicle_spec_.vehicle_width)};

  // turn indicators
  {
    std::vector<float> single_turn_indicators(INPUT_T + 1, 0.0f);
    for (int64_t t = 0; t < INPUT_T + 1; ++t) {
      const int64_t index = std::max(
        static_cast<int64_t>(turn_indicators_history_.size()) - 1 - t, static_cast<int64_t>(0));
      single_turn_indicators[INPUT_T - t] = turn_indicators_history_[index].report;
    }
    input_data_map["turn_indicators"] = std::move(single_turn_indicators);
  }

  // control delay
  input_data_map["delay"] = {static_cast<float>(delay_step)};

  return input_data_map;
}

OnePlannerOutput OnePlannerCore::create_planner_output(
  const std::vector<float> & ego_prediction, const std::vector<float> & turn_indicator_logit,
  const OnePlannerFrameContext & frame_context, const rclcpp::Time & timestamp,
  const UUID & generator_uuid)
{
  const auto predictions = expand_ego_prediction(ego_prediction);
  const auto agent_poses =
    postprocess::parse_predictions(predictions, frame_context.ego_to_map_transform);
  last_agent_poses_map_ = agent_poses;

  const bool enable_force_stop =
    frame_context.ego_kinematic_state.twist.twist.linear.x > std::numeric_limits<double>::epsilon();

  OnePlannerOutput output;

  constexpr int64_t batch_idx = 0;
  auto trajectory = postprocess::create_ego_trajectory(
    agent_poses, timestamp, frame_context.ego_kinematic_state.pose.pose.position, batch_idx,
    params_.velocity_smoothing_window, enable_force_stop, params_.stopping_threshold);

  if (params_.shift_x) {
    for (auto & point : trajectory.points) {
      point.pose = utils::shift_x(point.pose, -vehicle_spec_.base_link_to_center);
    }
  }

  output.trajectory = trajectory;

  const auto candidate_trajectory = autoware_internal_planning_msgs::build<
                                      autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                                      .header(trajectory.header)
                                      .generator_id(generator_uuid)
                                      .points(trajectory.points);

  std_msgs::msg::String generator_name_msg;
  generator_name_msg.data = "OnePlanner";

  const auto generator_info =
    autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::GeneratorInfo>()
      .generator_id(generator_uuid)
      .generator_name(generator_name_msg);

  output.candidate_trajectories.candidate_trajectories.push_back(candidate_trajectory);
  output.candidate_trajectories.generator_info.push_back(generator_info);

  // TurnIndicatorsCommand
  const std::vector<float> first_turn_indicator_logit(
    turn_indicator_logit.begin(), turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM);
  const int64_t prev_report = turn_indicators_history_.empty()
                                ? TurnIndicatorsReport::DISABLE
                                : turn_indicators_history_.back().report;
  output.turn_indicator_command =
    turn_indicator_manager_.evaluate(first_turn_indicator_logit, timestamp, prev_report);

  return output;
}

}  // namespace autoware::tensorrt_oneplanner
