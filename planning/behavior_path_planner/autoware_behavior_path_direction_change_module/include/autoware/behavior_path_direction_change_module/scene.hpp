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
#include <geometry_msgs/msg/pose.hpp>

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
  IDLE = 0,
  FORWARD_FOLLOWING,
  AT_CUSP,
  REVERSE_FOLLOWING,
  COMPLETED
};

inline const char * pathSegmentStateToString(const PathSegmentState state)
{
  switch (state) {
    case PathSegmentState::IDLE:
      return "IDLE";
    case PathSegmentState::FORWARD_FOLLOWING:
      return "FORWARD_FOLLOWING";
    case PathSegmentState::AT_CUSP:
      return "AT_CUSP";
    case PathSegmentState::REVERSE_FOLLOWING:
      return "REVERSE_FOLLOWING";
    case PathSegmentState::COMPLETED:
      return "COMPLETED";
    default:
      return "UNKNOWN";
  }
}

class DirectionChangeModule : public SceneModuleInterface
{
public:
  DirectionChangeModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<DirectionChangeParameters> & parameters,
    const std::shared_ptr<DirectionChangePersistentState> & persistent_state,
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

  /// Build route lanelet groups and tagged centerline once per module activation.
  bool buildRouteContextIfNeeded() const;

  /// Detect cusps on the tagged centerline and store stable path indices.
  void initializeCuspPointsFromTaggedCenterline();

  // Helper functions
  bool shouldActivateModule() const;

  /// Set drivable_area_info on output based on current segment state and published path.
  void updateDrivableAreaInfo(BehaviorModuleOutput & output);

  /// Keep cusp terminal point on the current segment lanelet for drivable area generation.
  void filterLaneletsAtCusp(BehaviorModuleOutput & output);

  /// Set turn_signal_info on output based on current segment state and published path.
  void updateTurnSignalInfo(BehaviorModuleOutput & output);

  void initializeManeuverState();

  void updateManeuverStateMachine(const PathWithLaneId & reference_path);

  double calcDistanceToNextCusp(
    const PathWithLaneId & maneuver_path, const geometry_msgs::msg::Pose & ego_pose) const;

  const PathWithLaneId & getTaggedLaneletCenterlinePath() const;

  const CuspPoint * getFirstUnvisitedCusp() const;
  const CuspPoint * getLastVisitedCusp() const;
  bool allCuspsVisited() const;
  size_t countUnvisitedCusps() const;

  bool isManeuverCompletedForCurrentRoute() const;
  void markManeuverCompletedForCurrentRoute();

  // Member variables
  PathWithLaneId reference_path_{};
  PathWithLaneId modified_path_{};
  mutable DirectionChangeRouteContext route_context_;
  mutable bool is_route_context_initialized_{false};
  std::shared_ptr<DirectionChangeParameters> parameters_;
  std::shared_ptr<DirectionChangePersistentState> persistent_state_;

  // Cusp transition poses discovered from reference_path; visited after stop at AT_CUSP
  std::vector<CuspPoint> cusp_points_{};

  PathSegmentState current_segment_state_{PathSegmentState::IDLE};
  bool is_ego_driving_forward_wrt_lane_{true};

  std::optional<rclcpp::Time> cusp_stopped_since_{};

  rclcpp::Publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>::SharedPtr
    path_publisher_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DIRECTION_CHANGE_MODULE__SCENE_HPP_
