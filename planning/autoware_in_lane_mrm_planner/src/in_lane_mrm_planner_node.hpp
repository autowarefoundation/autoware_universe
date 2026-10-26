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

#ifndef IN_LANE_MRM_PLANNER_NODE_HPP_
#define IN_LANE_MRM_PLANNER_NODE_HPP_

#include "in_lane_mrm_trajectory_modifier.hpp"
#include "in_lane_mrm_trajectory_planner.hpp"
#include "in_lane_mrm_trajectory_validator.hpp"
#include "mrm_stop_velocity_planner.hpp"
#include "path_planner.hpp"
#include "predicted_objects_latcher.hpp"
#include "stop_profile.hpp"
#include "trajectory_latcher.hpp"
#include "trajectory_selector_stub.hpp"
#include "trajectory_smoother.hpp"
#include "trigger_edge_detector.hpp"
#include "type_alias.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <in_lane_mrm_planner_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <tier4_system_msgs/msg/in_lane_stop_trigger.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace autoware::in_lane_mrm_planner
{
namespace namespace_polling = autoware::agnocast_wrapper::polling;

class InLaneMrmPlannerNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit InLaneMrmPlannerNode(const rclcpp::NodeOptions & options);

private:
  using Float32MultiArrayStamped = autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;
  using InLaneStopTrigger = tier4_system_msgs::msg::InLaneStopTrigger;
  using ProfileFlags = std::array<bool, kNumStopProfiles>;

  struct DebugStatus
  {
    int reason_code{99};
    bool trigger_active{false};
    bool is_latched{false};
    bool has_latest_candidate{false};
    bool data_ready{false};
    bool plan_ok{false};
    bool validation_ok{false};
    size_t planned_points{0};
    size_t published_points{0};
    double cycle_time_ms{0.0};
    double odom_vx{0.0};
    size_t sanitized_points{0};
    uint8_t requested_profile{InLaneStopTrigger::PROFILE_UNKNOWN};
    uint8_t latched_profile{InLaneStopTrigger::PROFILE_UNKNOWN};
  };

  struct InputData
  {
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr;
    LaneletRoute::ConstSharedPtr route_ptr;
    Odometry::ConstSharedPtr odometry_ptr;
    AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr;
    PredictedObjects::ConstSharedPtr objects_ptr;
    InLaneStopTrigger::ConstSharedPtr trigger_ptr;
  };

  void on_timer();
  // Plans the path once and fills the velocity profile of every deceleration profile. Returns
  // which profiles got a new validated candidate this cycle.
  ProfileFlags plan_candidates(
    const Odometry & odom, const AccelWithCovarianceStamped & accel,
    const PredictedObjects & live_objects, DebugStatus & status);
  StopProfile resolve_requested_profile(const InLaneStopTrigger & trigger);
  InputData take_data();
  bool is_data_ready(const InputData & input_data) const;
  void update_params();
  void publish_debug_status(const DebugStatus & status);

  std::shared_ptr<::in_lane_mrm_planner::ParamListener> param_listener_;
  Params params_;
  VehicleInfo vehicle_info_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  std::unique_ptr<PathPlanner> path_planner_;
  std::unique_ptr<InLaneMrmTrajectoryPlanner> trajectory_planner_;
  TrajectorySmoother trajectory_smoother_;
  InLaneMrmTrajectoryModifier trajectory_modifier_;
  MrmStopVelocityPlanner velocity_planner_;
  InLaneMrmTrajectoryValidator trajectory_validator_;
  TrajectorySelectorStub trajectory_selector_;
  TrajectoryLatcher trajectory_latcher_;
  PredictedObjectsLatcher objects_latcher_;
  TriggerEdgeDetector trigger_edge_detector_;

  namespace_polling::PollingSubscriber<
    LaneletRoute, namespace_polling::polling_policy::Newest>::SharedPtr route_subscriber_;
  LaneletRoute::ConstSharedPtr route_ptr_;

  namespace_polling::PollingSubscriber<
    LaneletMapBin, namespace_polling::polling_policy::Newest>::SharedPtr vector_map_subscriber_;
  LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr_;

  namespace_polling::PollingSubscriber<Odometry>::SharedPtr kinematic_state_subscriber_;
  Odometry::ConstSharedPtr odometry_ptr_;

  namespace_polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    acceleration_subscriber_;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_;

  namespace_polling::PollingSubscriber<PredictedObjects>::SharedPtr objects_subscriber_;
  PredictedObjects::ConstSharedPtr objects_ptr_;

  namespace_polling::PollingSubscriber<InLaneStopTrigger>::SharedPtr trigger_subscriber_;
  InLaneStopTrigger::ConstSharedPtr trigger_ptr_;

  AUTOWARE_PUBLISHER_PTR(Trajectory) pub_trajectory_;
  AUTOWARE_PUBLISHER_PTR(Float32MultiArrayStamped) pub_debug_status_;
  AUTOWARE_TIMER_PTR timer_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // IN_LANE_MRM_PLANNER_NODE_HPP_
