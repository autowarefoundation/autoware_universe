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
#include "trajectory_latcher.hpp"
#include "trajectory_selector_stub.hpp"
#include "trajectory_smoother.hpp"
#include "trigger_edge_detector.hpp"
#include "type_alias.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <in_lane_mrm_planner_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_multi_array_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstddef>
#include <memory>

namespace autoware::in_lane_mrm_planner
{

class InLaneMrmPlannerNode : public rclcpp::Node
{
public:
  explicit InLaneMrmPlannerNode(const rclcpp::NodeOptions & options);

private:
  using Float32MultiArrayStamped = autoware_internal_debug_msgs::msg::Float32MultiArrayStamped;

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
  };

  struct InputData
  {
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr;
    LaneletRoute::ConstSharedPtr route_ptr;
    Odometry::ConstSharedPtr odometry_ptr;
    AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr;
    PredictedObjects::ConstSharedPtr objects_ptr;
    std_msgs::msg::Bool::ConstSharedPtr trigger_ptr;
  };

  void on_timer();
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

  LaneletRoute::ConstSharedPtr route_ptr_;
  LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr_;
  Odometry::ConstSharedPtr odometry_ptr_;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_;
  PredictedObjects::ConstSharedPtr objects_ptr_;
  std_msgs::msg::Bool::ConstSharedPtr trigger_ptr_;

  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_;
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_;
  autoware_utils::InterProcessPollingSubscriber<Odometry> kinematic_state_subscriber_;
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    acceleration_subscriber_;
  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> objects_subscriber_;
  autoware_utils::InterProcessPollingSubscriber<std_msgs::msg::Bool> trigger_subscriber_;

  rclcpp::Publisher<Trajectory>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<Float32MultiArrayStamped>::SharedPtr pub_debug_status_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // IN_LANE_MRM_PLANNER_NODE_HPP_
