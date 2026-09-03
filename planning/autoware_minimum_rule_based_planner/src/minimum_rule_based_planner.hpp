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

#include "autoware/trajectory_processor/trajectory_processor_context.hpp"
#include "autoware/trajectory_processor/trajectory_processor_data.hpp"
#include "path_planner.hpp"
#include "velocity_smoother.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware_trajectory_processor/trajectory_processor_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::minimum_rule_based_planner
{
namespace namespace_polling = autoware::agnocast_wrapper::polling;

class MinimumRuleBasedPlannerNode : public autoware::agnocast_wrapper::Node
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
    PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr;
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
  void update_params();

  std::optional<PathWithLaneId> plan_path(const InputData & input_data);
  Trajectory shift_trajectory_to_ego(
    const Trajectory & trajectory, const InputData & input_data) const;
  Trajectory smooth_trajectory(const Trajectory & trajectory, const InputData & input_data) const;
  void apply_modifiers(Trajectory & trajectory, const InputData & input_data) const;
  Trajectory optimize_velocity(const Trajectory & trajectory, const InputData & input_data) const;

  void publish_candidate_trajectories(const Trajectory & trajectory) const;

  void publish_debug_trajectory(
    const std::string & plugin_name, const TrajectoryPoints & traj_points) const;

  AUTOWARE_TIMER_PTR timer_;
  std::shared_ptr<::minimum_rule_based_planner::ParamListener> param_listener_;
  const UUID generator_uuid_;
  const VehicleInfo vehicle_info_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  AUTOWARE_PUBLISHER_PTR(autoware_utils_debug::ProcessingTimeDetail)
  debug_processing_time_detail_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
  debug_processing_time_pub_;
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  minimum_rule_based_planner::Params params_;
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
  std::shared_ptr<autoware::trajectory_processor::TrajectoryProcessorContext> optimizer_context_;
  std::unique_ptr<VelocitySmoother> velocity_smoother_;
  std::map<std::string, AUTOWARE_PUBLISHER_PTR(Trajectory)>
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

  bool initialized_modifiers_{false};
  ModifierPluginLoader modifier_plugin_loader_;
  std::vector<std::shared_ptr<plugin::PluginInterface>> modifier_plugins_;
  std::map<std::string, AUTOWARE_PUBLISHER_PTR(Trajectory)> pub_debug_modifier_module_trajectories_;

  std::shared_ptr<plugin::ModifierContext> modifier_context_;
  /** @} */

private:
  /**
   ***********************************************************
   * @defgroup subscribers and publishers
   * @{
   */
  namespace_polling::PollingSubscriber<
    LaneletRoute, namespace_polling::polling_policy::Newest>::SharedPtr route_subscriber_;
  LaneletRoute::ConstSharedPtr route_ptr_;

  namespace_polling::PollingSubscriber<
    LaneletMapBin, namespace_polling::polling_policy::Newest>::SharedPtr vector_map_subscriber_;
  LaneletMapBin::ConstSharedPtr lanelet_map_bin_ptr_;

  namespace_polling::PollingSubscriber<Odometry>::SharedPtr odometry_subscriber_;
  Odometry::ConstSharedPtr odometry_ptr_;

  namespace_polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    acceleration_subscriber_;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr_;

  namespace_polling::PollingSubscriber<PredictedObjects>::SharedPtr objects_subscriber_;
  PredictedObjects::ConstSharedPtr predicted_objects_ptr_;

  namespace_polling::PollingSubscriber<PointCloud2>::SharedPtr pointcloud_subscriber_;
  PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr_;

  //! test input: bypasses path planning when provided
  namespace_polling::PollingSubscriber<PathWithLaneId, namespace_polling::polling_policy::Newest>::
    SharedPtr test_path_with_lane_id_subscriber_;
  PathWithLaneId::ConstSharedPtr test_path_with_lane_id_ptr;

  AUTOWARE_PUBLISHER_PTR(CandidateTrajectories) pub_trajectories_;
  AUTOWARE_PUBLISHER_PTR(PathWithLaneId) pub_debug_path_;
  AUTOWARE_PUBLISHER_PTR(Trajectory) pub_debug_trajectory_;
  AUTOWARE_PUBLISHER_PTR(Trajectory) pub_debug_shifted_trajectory_;
  /** @} */
};

}  // namespace autoware::minimum_rule_based_planner

#endif  // MINIMUM_RULE_BASED_PLANNER_HPP_
