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

#ifndef AUTOWARE__TENSORRT_ONEPLANNER__ONEPLANNER_CORE_HPP_
#define AUTOWARE__TENSORRT_ONEPLANNER__ONEPLANNER_CORE_HPP_

#include "autoware/tensorrt_oneplanner/planner_inference.hpp"

#include <Eigen/Dense>
#include <autoware/diffusion_planner/postprocessing/turn_indicator_manager.hpp>
#include <autoware/diffusion_planner/preprocessing/lane_segments.hpp>
#include <autoware/diffusion_planner/preprocessing/traffic_signals.hpp>
#include <autoware/diffusion_planner/utils/arg_reader.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Trajectory;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;

using autoware::diffusion_planner::preprocess::TrafficSignalStamped;
using NormalizationMap = autoware::diffusion_planner::utils::NormalizationMap;

struct OnePlannerVehicleSpec
{
  double wheel_base;
  double vehicle_length;
  double vehicle_width;
  double base_link_to_center;

  explicit OnePlannerVehicleSpec(const VehicleInfo & info)
  : wheel_base(info.wheel_base_m),
    vehicle_length(info.front_overhang_m + info.wheel_base_m + info.rear_overhang_m),
    vehicle_width(info.left_overhang_m + info.wheel_tread_m + info.right_overhang_m),
    base_link_to_center((info.front_overhang_m + info.wheel_base_m - info.rear_overhang_m) / 2.0)
  {
  }
};

struct OnePlannerFrameContext
{
  Odometry ego_kinematic_state;
  AccelWithCovarianceStamped ego_acceleration;
  Eigen::Matrix4d ego_to_map_transform;
  rclcpp::Time frame_time;
};

struct OnePlannerOutput
{
  Trajectory trajectory;
  CandidateTrajectories candidate_trajectories;
  TurnIndicatorsCommand turn_indicator_command;
};

struct OnePlannerParams
{
  std::string args_path;
  double temperature;
  int64_t velocity_smoothing_window;
  double stopping_threshold;
  float turn_indicator_keep_offset;
  double turn_indicator_hold_duration;
  bool shift_x;
  int64_t delay_step;
  double line_string_max_step_m;
  bool use_time_interpolation;
  double traffic_light_group_msg_timeout_seconds;
};

/**
 * @class OnePlannerCore
 * @brief ROS-independent preprocessing/postprocessing for the OnePlanner node.
 *
 * Mirrors autoware::diffusion_planner::DiffusionPlannerCore with three differences:
 * - no tracked-objects input: OnePlanner is a one-stage model conditioned on the raw
 *   LiDAR BEV feature map, so `neighbor_agents_past` is fed as zeros (the model's
 *   neighbor encoder was removed at training time, the input is kept for graph
 *   compatibility),
 * - inference is owned by the node (dual TensorRT engines, see BevEncoder and
 *   PlannerInference),
 * - the prediction output is ego-only (P=1) and gets expanded to the diffusion-planner
 *   layout before reusing its postprocessing.
 */
class OnePlannerCore
{
public:
  OnePlannerCore(const OnePlannerParams & params, const VehicleInfo & vehicle_info);

  /// Load and validate normalization statistics from the args JSON.
  void load_normalization();

  void update_params(const OnePlannerParams & params);

  void set_map(const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr);
  bool is_map_loaded() const { return lane_segment_context_ != nullptr; }
  bool is_normalization_loaded() const { return !normalization_map_.empty(); }
  const NormalizationMap & get_normalization_map() const { return normalization_map_; }
  const LaneletRoute::ConstSharedPtr & get_route() const { return route_ptr_; }

  std::optional<OnePlannerFrameContext> create_frame_context(
    const std::shared_ptr<const Odometry> & ego_kinematic_state,
    const std::shared_ptr<const AccelWithCovarianceStamped> & ego_acceleration,
    const std::vector<
      std::shared_ptr<const autoware_perception_msgs::msg::TrafficLightGroupArray>> &
      traffic_signals,
    const std::shared_ptr<const TurnIndicatorsReport> & turn_indicators,
    const LaneletRoute::ConstSharedPtr & route_ptr, const rclcpp::Time & current_time);

  InputDataMap create_input_data(const OnePlannerFrameContext & frame_context);

  /**
   * @brief Create output messages from the raw ego-only prediction.
   *
   * @param ego_prediction Raw model output [1, 1, OUTPUT_T, POSE_DIM] (ego frame).
   * @param turn_indicator_logit Logits for turn indicator classes [1, 5].
   */
  OnePlannerOutput create_planner_output(
    const std::vector<float> & ego_prediction, const std::vector<float> & turn_indicator_logit,
    const OnePlannerFrameContext & frame_context, const rclcpp::Time & timestamp,
    const UUID & generator_uuid);

private:
  OnePlannerParams params_;
  OnePlannerVehicleSpec vehicle_spec_;
  NormalizationMap normalization_map_;

  autoware::diffusion_planner::postprocess::TurnIndicatorManager turn_indicator_manager_;

  std::deque<Odometry> ego_history_;
  std::deque<TurnIndicatorsReport> turn_indicators_history_;
  std::map<lanelet::Id, TrafficSignalStamped> traffic_light_id_map_;
  std::vector<std::vector<std::vector<Eigen::Matrix4d>>> last_agent_poses_map_;

  LaneletRoute::ConstSharedPtr route_ptr_;
  std::unique_ptr<autoware::diffusion_planner::preprocess::LaneSegmentContext>
    lane_segment_context_;
};

}  // namespace autoware::tensorrt_oneplanner

#endif  // AUTOWARE__TENSORRT_ONEPLANNER__ONEPLANNER_CORE_HPP_
