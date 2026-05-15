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

#ifndef MINIMUM_RULE_BASED_PLANNER_HPP_
#define MINIMUM_RULE_BASED_PLANNER_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"
#include "path_planner.hpp"
#include "velocity_smoother.hpp"

#include <autoware/trajectory_modifier/trajectory_modifier_structs.hpp>
#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
using TrajectoryModifierData = trajectory_modifier::TrajectoryModifierData;

class MinimumRuleBasedPlannerNode : public rclcpp::Node
{
public:
  explicit MinimumRuleBasedPlannerNode(const rclcpp::NodeOptions & options);

  /**
   * @brief aggregated input data consumed each planning cycle
   */
  struct InputData
  {
    LaneletRoute::ConstSharedPtr route_ptr;
    LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr;
    Odometry::ConstSharedPtr odometry_ptr;
    AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr;
    PredictedObjects::ConstSharedPtr predicted_objects_ptr;
    PathWithLaneId::ConstSharedPtr test_path_with_lane_id_ptr;
  };

private:
  /**
   ***********************************************************
   * @defgroup core pipeline
   * on_timer() is the main entry point, called at planning_frequency_hz.
   * @{
   */
  void on_timer();
  InputData take_data();
  bool is_data_ready(const InputData & input_data);

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<::minimum_rule_based_planner::ParamListener> param_listener_;
  const UUID generator_uuid_;
  const VehicleInfo vehicle_info_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup path planning
   * PathPlanner handles route/map initialisation, path planning,
   * trajectory shifting, and conversion.
   * @{
   */
  //! PathPlanner encapsulates path planning, trajectory shifting, and conversion
  std::unique_ptr<PathPlanner> path_planner_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup optimizer-plugins trajectory optimizer plugins
   * @{
   */
  void load_optimizer_plugins();

  std::unique_ptr<OptimizerPluginLoader> plugin_loader_;
  std::shared_ptr<OptimizerPluginInterface> path_smoother_;
  std::unique_ptr<VelocitySmoother> velocity_smoother_;
  std::map<std::string, rclcpp::Publisher<Trajectory>::SharedPtr>
    pub_debug_optimizer_module_trajectories_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup modifier-plugins trajectory modifier plugins
   * @{
   */
  void load_modifier_plugins();

  void load_plugin(const std::string & name);
  void unload_plugin(const std::string & name);

  void set_modifier_data(const MinimumRuleBasedPlannerNode::InputData & input_data);

  bool initialized_modifiers_{false};
  ModifierPluginLoader modifier_plugin_loader_;
  std::vector<std::shared_ptr<trajectory_modifier::plugin::TrajectoryModifierPluginBase>>
    modifier_plugins_;
  trajectory_modifier_params::Params modifier_params_;
  std::map<std::string, rclcpp::Publisher<Trajectory>::SharedPtr>
    pub_debug_modifier_module_trajectories_;

  std::shared_ptr<TrajectoryModifierData> modifier_data_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup subscribers and publishers
   * @{
   */
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    route_subscriber_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  LaneletRoute::ConstSharedPtr route_ptr_;

  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    vector_map_subscriber_{this, "~/input/vector_map", rclcpp::QoS{1}.transient_local()};
  LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr_;

  autoware_utils::InterProcessPollingSubscriber<Odometry> odometry_subscriber_{
    this, "~/input/odometry"};
  Odometry::ConstSharedPtr odometry_ptr_;

  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    acceleration_subscriber_{this, "~/input/acceleration"};
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_;

  autoware_utils::InterProcessPollingSubscriber<PredictedObjects> objects_subscriber_{
    this, "~/input/objects"};
  PredictedObjects::ConstSharedPtr predicted_objects_ptr_;

  //! test input: bypasses path planning when provided
  autoware_utils::InterProcessPollingSubscriber<
    PathWithLaneId, autoware_utils::polling_policy::Newest>
    test_path_with_lane_id_subscriber_{this, "~/input/test/path_with_lane_id"};
  PathWithLaneId::ConstSharedPtr test_path_with_lane_id_ptr;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_debug_path_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_trajectory_;
  rclcpp::Publisher<Trajectory>::SharedPtr pub_debug_shifted_trajectory_;
  /** @} */
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MINIMUM_RULE_BASED_PLANNER_HPP_
