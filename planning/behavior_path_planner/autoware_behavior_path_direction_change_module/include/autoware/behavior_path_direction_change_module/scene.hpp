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

#ifndef AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_

#include "autoware/behavior_path_direction_change_module/data_structs.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{
using autoware_internal_planning_msgs::msg::PathWithLaneId;

// State for tracking which path segment to publish
enum class PathSegmentState {
  IDLE = 0,           // Module is idle/inactive
  FORWARD_FOLLOWING,  // Following path in forward direction
  APPROACHING_CUSP,   // Approaching a cusp point
  AT_CUSP,            // At a cusp point
  REVERSE_FOLLOWING,  // Following path in reverse direction
  COMPLETED           // Module has completed processing
};

class DirectionChangeModule : public SceneModuleInterface
{
public:
  DirectionChangeModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<DirectionChangeParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> planning_factor_interface);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  bool isReadyForNextRequest(
    const double & min_request_time_sec, bool override_requests = false) const noexcept;
  void updateData() override;
  BehaviorModuleOutput plan() override;
  BehaviorModuleOutput planWaitingApproval() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;

  void setParameters(const std::shared_ptr<DirectionChangeParameters> & parameters);

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<DirectionChangeParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override { return false; }

  void initVariables();

  // Helper functions
  bool shouldActivateModule() const;
  /// Push current odometry and return true if velocity has been below stop_velocity_threshold for
  /// at least th_stopped_time
  bool isSustainedStoppedForDirectionSwitch();

  // Member variables
  PathWithLaneId reference_path_{};
  PathWithLaneId modified_path_{};
  std::shared_ptr<DirectionChangeParameters> parameters_;

  // Direction change data
  std::vector<size_t> cusp_point_indices_{};

  // Path segment state tracking for separate forward/backward publishing (multi-cusp)
  PathSegmentState current_segment_state_{PathSegmentState::FORWARD_FOLLOWING};
  size_t current_segment_index_{0};  // Segment index (0 = first forward, 1 = first reverse, ...)
  geometry_msgs::msg::Point
    first_cusp_position_;  // Position of current segment end (cusp) for debug/log
  bool has_valid_cusp_{false};

  // Sustained stop: timestamp when velocity first dropped below stop_velocity_threshold at cusp
  std::optional<rclcpp::Time> cusp_stopped_since_{};

  // Sustained stop: buffer (timestamp, velocity) for direction switch at cusp
  std::deque<std::pair<rclcpp::Time, double>> odometry_buffer_direction_switch_{};

  // Debug data
  mutable DirectionChangeDebugData debug_data_;
  void setDebugMarkersVisualization() const;

  // Publisher for processed path with reversed orientations
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    path_publisher_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_
