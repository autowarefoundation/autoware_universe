// Copyright 2025 Tier IV, Inc.
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

#ifndef EXPERIMENTAL__SCENE_HPP_
#define EXPERIMENTAL__SCENE_HPP_

#define EIGEN_MPL2_ONLY

#include "../scene.hpp"

#include <autoware/behavior_velocity_rtc_interface/experimental/scene_module_interface_with_rtc.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>
namespace autoware::behavior_velocity_planner::experimental
{

class DetectionAreaModule : public SceneModuleInterfaceWithRTC
{
public:
  enum class State { GO, STOP };

  struct DebugData
  {
    double base_link2front;
    std::vector<geometry_msgs::msg::Pose> stop_poses;
    std::vector<geometry_msgs::msg::Pose> dead_line_poses;
    geometry_msgs::msg::Pose first_stop_pose;
    std::vector<geometry_msgs::msg::Point> obstacle_points;
  };

  struct PlannerParam
  {
    double stop_margin;
    bool use_dead_line;
    double dead_line_margin;
    double state_clear_time;
    double hold_stop_margin_distance;
    double distance_to_judge_over_stop_line;
    bool suppress_pass_judge_when_stopping;
    bool enable_detected_obstacle_logging;

    // Unified unstoppable situation handling
    std::string unstoppable_policy;  // "go", "force_stop", or "stop_after_stopline"
    double max_deceleration;
    double delay_response_time;

    autoware::behavior_velocity_planner::DetectionAreaModule::PlannerParam::TargetFiltering
      target_filtering;
  };

  DetectionAreaModule(
    const lanelet::Id module_id, const lanelet::Id lane_id,
    const lanelet::autoware::DetectionArea & detection_area_reg_elem,
    const PlannerParam & planner_param, const rclcpp::Logger & logger,
    const rclcpp::Clock::SharedPtr clock,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
    const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
      planning_factor_interface);

  bool modifyPathVelocity(
    Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const PlannerData & planner_data) override;

  visualization_msgs::msg::MarkerArray createDebugMarkerArray() override;
  autoware::motion_utils::VirtualWalls createVirtualWalls() override;

  std::vector<lanelet::Id> getRegulatoryElementIds() const override
  {
    return {detection_area_reg_elem_.id()};
  }
  std::vector<lanelet::Id> getLaneletIds() const override { return {lane_id_}; }
  std::vector<lanelet::Id> getLineIds() const override
  {
    return {detection_area_reg_elem_.stopLine().id()};
  }

private:
  // Lane id
  lanelet::Id lane_id_;

  // Key Feature
  const lanelet::autoware::DetectionArea & detection_area_reg_elem_;

  // State
  State state_;
  std::shared_ptr<const rclcpp::Time> last_obstacle_found_time_;
  double forward_offset_to_stop_line_{0.0};

  // Parameter
  PlannerParam planner_param_;

  // Debug
  DebugData debug_data_;

  /**
   * @brief Print positions of detected obstacle, time elapsed since last detection, and ego vehicle
   * position
   * @param obstacle_points Points representing detected obstacles in the detection area
   * @param self_pose Current pose of the ego vehicle
   */
  void print_detected_obstacle(
    const std::vector<geometry_msgs::msg::Point> & obstacle_points,
    const geometry_msgs::msg::Pose & self_pose) const;

  /**
   * @brief Finalize stop point by inserting it, logging, and creating stop reason
   * @param path Path to modify
   * @param stop_pose Original stop pose
   * @param modified_stop_pose Modified stop pose to insert
   * @param modified_stop_line_seg_idx Modified stop line segment index
   * @param self_pose Current vehicle pose
   * @param detection_source Source of detection
   * @param policy_name Name of policy being applied (for logging)
   * @param prev_state Previous state before this operation
   * @param planner_data Planner data for odometry information
   */
  void finalizeStopPoint(
    Trajectory * path, const geometry_msgs::msg::Pose & stop_pose,
    const geometry_msgs::msg::Pose & modified_stop_pose, const size_t modified_stop_line_seg_idx,
    const geometry_msgs::msg::Pose & self_pose, const std::string & detection_source,
    const std::string & policy_name, const State & prev_state, const PlannerData & planner_data);

  /**
   * @brief Handle "go" policy for unstoppable situation
   * @return true if should pass through
   */
  bool handleUnstoppableGoPolicy();

  /**
   * @brief Handle "force_stop" policy for unstoppable situation
   * @param path Path to modify
   * @param stop_pose Original stop pose
   * @param modified_stop_pose Modified stop pose
   * @param modified_stop_line_seg_idx Modified stop line segment index
   * @param self_pose Current vehicle pose
   * @param detection_source Source of detection
   * @param planner_data Planner data for odometry information
   * @return true if handled
   */
  bool handleUnstoppableForceStopPolicy(
    Trajectory * path, const geometry_msgs::msg::Pose & stop_pose,
    const geometry_msgs::msg::Pose & modified_stop_pose, const size_t modified_stop_line_seg_idx,
    const geometry_msgs::msg::Pose & self_pose, const std::string & detection_source,
    const PlannerData & planner_data);

  /**
   * @brief Handle "stop_after_stopline" policy for unstoppable situation
   * @param path Path to modify
   * @param original_path Original path before modification
   * @param stop_pose Original stop pose
   * @param modified_stop_pose Modified stop pose (may be changed)
   * @param modified_stop_line_seg_idx Modified stop line segment index (may be changed)
   * @param self_pose Current vehicle pose
   * @param current_velocity Current vehicle velocity
   * @param stop_dist Distance to stop line
   * @param detection_source Source of detection
   * @param planner_data Planner data for odometry information
   * @return true if handled
   */
  bool handleUnstoppableStopAfterLinePolicy(
    Trajectory * path, const PathWithLaneId & original_path,
    const geometry_msgs::msg::Pose & stop_pose, geometry_msgs::msg::Pose & modified_stop_pose,
    size_t & modified_stop_line_seg_idx, const geometry_msgs::msg::Pose & self_pose,
    const double current_velocity, const double stop_dist, const std::string & detection_source,
    const PlannerData & planner_data);
};
}  // namespace autoware::behavior_velocity_planner::experimental

#endif  // EXPERIMENTAL__SCENE_HPP_
