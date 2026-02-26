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

#include "autoware/diffusion_planner/diffusion_planner_core.hpp"

#include "autoware/diffusion_planner/conversion/agent.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/postprocessing/postprocessing_utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_internal_planning_msgs/msg/generator_info.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

DiffusionPlannerCore::DiffusionPlannerCore(
  const DiffusionPlannerParams & params, const VehicleInfo & vehicle_info)
: params_(params),
  vehicle_spec_(vehicle_info),
  turn_indicator_manager_(
    rclcpp::Duration::from_seconds(params.turn_indicator_hold_duration),
    params.turn_indicator_keep_offset)
{
}

void DiffusionPlannerCore::load_model()
{
  tensorrt_inference_.reset();
  utils::check_weight_version(params_.args_path);
  normalization_map_ = utils::load_normalization_stats(params_.args_path);
  tensorrt_inference_ = std::make_unique<TensorrtInference>(
    params_.model_path, params_.plugins_path, params_.batch_size);
}

void DiffusionPlannerCore::update_params(const DiffusionPlannerParams & params)
{
  params_ = params;
  turn_indicator_manager_.set_hold_duration(
    rclcpp::Duration::from_seconds(params_.turn_indicator_hold_duration));
  turn_indicator_manager_.set_keep_offset(params_.turn_indicator_keep_offset);
}

void DiffusionPlannerCore::set_map(
  const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr)
{
  lane_segment_context_ = std::make_unique<preprocess::LaneSegmentContext>(lanelet_map_ptr);
}

std::optional<FrameContext> DiffusionPlannerCore::create_frame_context(
  const std::shared_ptr<const Odometry> & ego_kinematic_state,
  const std::shared_ptr<const AccelWithCovarianceStamped> & ego_acceleration,
  const std::shared_ptr<const TrackedObjects> & objects,
  const std::vector<std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
    traffic_signals,
  const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
  const LaneletRoute::ConstSharedPtr & route_ptr, const rclcpp::Time & current_time)
{
  route_ptr_ = (!route_ptr_ || route_ptr) ? route_ptr : route_ptr_;

  TrackedObjects empty_object_list;
  auto effective_objects = objects;

  if (params_.ignore_neighbors) {
    effective_objects = std::make_shared<TrackedObjects>(empty_object_list);
  }

  if (!effective_objects || !ego_kinematic_state || !ego_acceleration || !turn_indicators) {
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
  turn_indicators_history_.push_back(*turn_indicators);
  if (turn_indicators_history_.size() > static_cast<size_t>(TURN_INDICATORS_SHAPE[1])) {
    turn_indicators_history_.pop_front();
  }

  // Update neighbor agent data
  agent_data_.update_histories(*effective_objects, params_.ignore_unknown_neighbors);
  const auto processed_neighbor_histories =
    agent_data_.transformed_and_trimmed_histories(map_to_ego_transform, NEIGHBOR_SHAPE[1]);

  // Update traffic light map
  const auto & traffic_light_msg_timeout_s = params_.traffic_light_group_msg_timeout_seconds;
  preprocess::process_traffic_signals(
    traffic_signals, traffic_light_id_map_, current_time, traffic_light_msg_timeout_s);

  // Create frame context
  const rclcpp::Time frame_time(ego_kinematic_state->header.stamp);
  const FrameContext frame_context{
    *ego_kinematic_state, *ego_acceleration, ego_to_map_transform, processed_neighbor_histories,
    frame_time};

  return frame_context;
}

InputDataMap DiffusionPlannerCore::create_input_data(const FrameContext & frame_context)
{
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
          frame_context.ego_kinematic_state.pose.pose, vehicle_spec_.base_link_to_center)
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
      static_cast<float>(vehicle_spec_.wheel_base));
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
    const std::vector<float> single_ego_shape = {
      static_cast<float>(vehicle_spec_.wheel_base),
      static_cast<float>(vehicle_spec_.vehicle_length),
      static_cast<float>(vehicle_spec_.vehicle_width)};
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

TensorrtInference::InferenceResult DiffusionPlannerCore::run_inference(
  const InputDataMap & input_data_map)
{
  if (!tensorrt_inference_) {
    TensorrtInference::InferenceResult result;
    result.error_msg = "Model not loaded";
    return result;
  }
  return tensorrt_inference_->infer(input_data_map);
}

PlannerOutput DiffusionPlannerCore::create_planner_output(
  const std::vector<float> & predictions, const std::vector<float> & turn_indicator_logit,
  const FrameContext & frame_context, const rclcpp::Time & timestamp, const UUID & generator_uuid)
{
  const auto agent_poses =
    postprocess::parse_predictions(predictions, frame_context.ego_to_map_transform);

  const bool enable_force_stop =
    frame_context.ego_kinematic_state.twist.twist.linear.x > std::numeric_limits<double>::epsilon();

  PlannerOutput output;

  // Trajectory and CandidateTrajectories
  for (int i = 0; i < params_.batch_size; i++) {
    auto trajectory = postprocess::create_ego_trajectory(
      agent_poses, timestamp, frame_context.ego_kinematic_state.pose.pose.position, i,
      params_.velocity_smoothing_window, enable_force_stop, params_.stopping_threshold);

    if (params_.shift_x) {
      for (auto & point : trajectory.points) {
        point.pose = utils::shift_x(point.pose, -vehicle_spec_.base_link_to_center);
      }
    }

    if (i == 0) {
      // Use the first trajectory as the main output trajectory
      output.trajectory = trajectory;
    }

    const auto candidate_trajectory = autoware_internal_planning_msgs::build<
                                        autoware_internal_planning_msgs::msg::CandidateTrajectory>()
                                        .header(trajectory.header)
                                        .generator_id(generator_uuid)
                                        .points(trajectory.points);

    std_msgs::msg::String generator_name_msg;
    generator_name_msg.data = std::string("DiffusionPlanner_batch_") + std::to_string(i);

    const auto generator_info =
      autoware_internal_planning_msgs::build<autoware_internal_planning_msgs::msg::GeneratorInfo>()
        .generator_id(generator_uuid)
        .generator_name(generator_name_msg);

    output.candidate_trajectories.candidate_trajectories.push_back(candidate_trajectory);
    output.candidate_trajectories.generator_info.push_back(generator_info);
  }

  // PredictedObjects
  // Use the first prediction as the main predicted objects
  constexpr int64_t batch_idx = 0;
  output.predicted_objects = postprocess::create_predicted_objects(
    agent_poses, frame_context.ego_centric_neighbor_histories, timestamp, batch_idx);

  // TurnIndicatorsCommand
  // Use the first batch's logit as the main turn indicator command.
  constexpr int64_t turn_indicator_batch_idx = 0;
  const std::vector<float> first_turn_indicator_logit(
    turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * turn_indicator_batch_idx,
    turn_indicator_logit.begin() + TURN_INDICATOR_OUTPUT_DIM * (turn_indicator_batch_idx + 1));
  const int64_t prev_report = turn_indicators_history_.empty()
                                ? TurnIndicatorsReport::DISABLE
                                : turn_indicators_history_.back().report;
  output.turn_indicator_command =
    turn_indicator_manager_.evaluate(first_turn_indicator_logit, timestamp, prev_report);

  return output;
}

autoware_perception_msgs::msg::TrafficLightGroup
DiffusionPlannerCore::get_first_traffic_light_on_route(const FrameContext & frame_context) const
{
  if (!lane_segment_context_ || !route_ptr_) {
    return autoware_perception_msgs::msg::TrafficLightGroup{};
  }

  const geometry_msgs::msg::Pose & pose_center =
    params_.shift_x
      ? utils::shift_x(
          frame_context.ego_kinematic_state.pose.pose, vehicle_spec_.base_link_to_center)
      : frame_context.ego_kinematic_state.pose.pose;

  const double center_x = pose_center.position.x;
  const double center_y = pose_center.position.y;
  const double center_z = pose_center.position.z;

  return lane_segment_context_->get_first_traffic_light_on_route(
    *route_ptr_, center_x, center_y, center_z, traffic_light_id_map_);
}

int64_t DiffusionPlannerCore::count_valid_elements(
  const InputDataMap & input_data_map, const std::string & data_key) const
{
  const int64_t batch_idx = 0;

  if (data_key == "lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("lanes"), LANES_SHAPE[1], LANES_SHAPE[2], LANES_SHAPE[3], batch_idx);
  } else if (data_key == "route_lanes") {
    return postprocess::count_valid_elements(
      input_data_map.at("route_lanes"), ROUTE_LANES_SHAPE[1], ROUTE_LANES_SHAPE[2],
      ROUTE_LANES_SHAPE[3], batch_idx);
  } else if (data_key == "polygons") {
    return postprocess::count_valid_elements(
      input_data_map.at("polygons"), POLYGONS_SHAPE[1], POLYGONS_SHAPE[2], POLYGONS_SHAPE[3],
      batch_idx);
  } else if (data_key == "line_strings") {
    return postprocess::count_valid_elements(
      input_data_map.at("line_strings"), LINE_STRINGS_SHAPE[1], LINE_STRINGS_SHAPE[2],
      LINE_STRINGS_SHAPE[3], batch_idx);
  } else if (data_key == "neighbor_agents_past") {
    return postprocess::count_valid_elements(
      input_data_map.at("neighbor_agents_past"), NEIGHBOR_SHAPE[1], NEIGHBOR_SHAPE[2],
      NEIGHBOR_SHAPE[3], batch_idx);
  }

  throw std::invalid_argument("Unknown data_key '" + data_key + "' in count_valid_elements()");
}

}  // namespace autoware::diffusion_planner
