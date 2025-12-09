// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_DRY_STEERING_AVOIDANCE_MODULE__DRY_STEERING_AVOIDANCE_MODULE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_DRY_STEERING_AVOIDANCE_MODULE__DRY_STEERING_AVOIDANCE_MODULE_HPP_

#include "autoware/behavior_path_dry_steering_avoidance_module/data_structs.hpp"
#include "autoware/behavior_path_planner_common/interface/scene_module_interface.hpp"
#include "autoware/behavior_path_planner_common/utils/parking_departure/geometric_parallel_parking.hpp"

#include <autoware/boundary_departure_checker/boundary_departure_checker.hpp>
#include <autoware/route_handler/route_handler.hpp>
#include <autoware/vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::behavior_path_planner
{

using autoware_perception_msgs::msg::PredictedObject;
using geometry_msgs::msg::Pose;

struct PullOutPath
{
  std::vector<PathWithLaneId> partial_paths{};
  std::vector<std::pair<double, double>> pairs_terminal_velocity_and_accel{};
  Pose start_pose{};
  Pose end_pose{};
};

class DrySteeringAvoidanceModule : public SceneModuleInterface
{
public:
  DrySteeringAvoidanceModule(
    const std::string & name, rclcpp::Node & node,
    const std::shared_ptr<DrySteeringAvoidanceParameters> & parameters,
    const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
    std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
      objects_of_interest_marker_interface_ptr_map,
    const std::shared_ptr<PlanningFactorInterface> & planning_factor_interface);

  bool isExecutionRequested() const override;
  bool isExecutionReady() const override;
  BehaviorModuleOutput plan() override;
  CandidateOutput planCandidate() const override;
  void processOnEntry() override;
  void processOnExit() override;
  void updateData() override;
  bool canTransitSuccessState() override;
  bool canTransitFailureState() override;

  void updateModuleParams(const std::any & parameters) override
  {
    parameters_ = std::any_cast<std::shared_ptr<DrySteeringAvoidanceParameters>>(parameters);
  }

  void acceptVisitor(
    [[maybe_unused]] const std::shared_ptr<SceneModuleVisitor> & visitor) const override
  {
  }

private:
  // Activation condition
  bool isStopped() const;
  bool hasStaticObjectInFront() const;
  bool checkActivationCondition();

  // Object detection
  std::optional<PredictedObject> findNearestStaticObjectInCurrentLanes() const;
  double getObjectLateralEdgeOffset(const PredictedObject & object, bool right_side) const;

  // End pose generation
  std::vector<Pose> generateEndPoseCandidates(const PredictedObject & object) const;

  // Path planning
  std::optional<PullOutPath> planAvoidancePath();
  PathWithLaneId getCurrentPath() const;

  // Termination condition
  bool hasReachedGoal() const;
  bool hasObstacleDisappearedAtStart() const;

  // Utilities
  lanelet::ConstLanelets getCurrentLanes() const;
  lanelet::ConstLanelets getRightLanes() const;
  double calcLongitudinalDistanceFromEgo(
    const Pose & target_pose, const lanelet::ConstLanelets & current_lanes) const;
  bool isObjectInLanes(
    const PredictedObject & object, const lanelet::ConstLanelets & lanes) const;
  TurnSignalInfo calcTurnSignalInfo() const;

  // Drivable area
  std::vector<DrivableLanes> generateDrivableLanes() const;

  // Debug visualization
  void setDebugData();

  // Parameters
  std::shared_ptr<DrySteeringAvoidanceParameters> parameters_;

  // Vehicle info
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // Geometric planner
  GeometricParallelParking geometric_planner_;
  std::shared_ptr<autoware::boundary_departure_checker::BoundaryDepartureChecker>
    boundary_departure_checker_;

  // Avoidance path (holds during execution)
  std::optional<PullOutPath> avoidance_path_;

  // Drivable lanes (cached at activation time)
  std::vector<DrivableLanes> drivable_lanes_;

  // Activation timing
  Pose start_pose_;
  rclcpp::Time condition_start_time_;
  bool condition_check_started_{false};

  // Debug data
  std::vector<Pose> debug_end_pose_candidates_;
  std::optional<PredictedObject> debug_target_object_;
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_DRY_STEERING_AVOIDANCE_MODULE__DRY_STEERING_AVOIDANCE_MODULE_HPP_
