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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__PLUGIN_BASE_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__PLUGIN_BASE_HPP_

#include "autoware/trajectory_processor/trajectory_optimizer_structs.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace trajectory_modifier_params
{
struct Params;
}  // namespace trajectory_modifier_params

namespace autoware::trajectory_processor::plugin
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using TrajectoryModifierParams = trajectory_modifier_params::Params;
using TrajectoryOptimizerParams = autoware::trajectory_optimizer::TrajectoryOptimizerParams;

struct InputData
{
  nav_msgs::msg::Odometry::ConstSharedPtr current_odometry = nullptr;
  geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr current_acceleration = nullptr;
  autoware::trajectory_optimizer::SemanticSpeedTracker * semantic_speed_tracker = nullptr;
  autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr predicted_objects = nullptr;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr obstacle_pointcloud = nullptr;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map = nullptr;
  autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr route = nullptr;
  autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr traffic_light_signals =
    nullptr;
};

struct NodeContext
{
  /// Create shared plugin context from the owning node and a time keeper.
  explicit NodeContext(
    rclcpp::Node & node, std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    bool listen_tf = false)
  : node_ptr{&node},
    time_keeper{std::move(time_keeper)},
    vehicle_info{autoware::vehicle_info_utils::VehicleInfoUtils(node).getVehicleInfo()},
    tf_buffer{std::make_shared<tf2_ros::Buffer>(node.get_clock())}
  {
    if (!this->time_keeper) {
      throw std::invalid_argument{"NodeContext requires a valid TimeKeeper"};
    }
    if (listen_tf) {
      tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, &node);
    }
  }

  rclcpp::Node * node_ptr;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
};

class PluginBase
{
public:
  virtual ~PluginBase() = default;

  /// Initialize the plugin name, shared context.
  virtual void initialize(const std::string & name, std::shared_ptr<NodeContext> context)
  {
    name_ = name;
    short_name_ = [&name]() {
      const auto npos = name.find_last_of(':');
      return npos != std::string::npos ? name.substr(npos + 1) : name;
    }();
    context_ = std::move(context);
    set_up_params();
  }

  /// Modify trajectory points in place using input data.
  virtual bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) = 0;

  /// Declare and read plugin-specific ROS parameters.
  virtual void set_up_params() {}

  /// Handle dynamic ROS parameter updates for the plugin.
  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  /// Update optimizer-wide parameters shared by optimizer plugins.
  virtual void update_params([[maybe_unused]] const TrajectoryOptimizerParams & params)
  {
    optimizer_params_ = params;
  }

  /// Update modifier-wide parameters shared by modifier plugins.
  virtual void update_params([[maybe_unused]] const TrajectoryModifierParams & params) {}

  /// Publish plugin-specific debug data under the given namespace.
  virtual void publish_debug_data([[maybe_unused]] const std::string & ns) const {}

  /// Publish accumulated planning factors, if the plugin owns an interface.
  virtual void publish_planning_factor()
  {
    if (planning_factor_interface_) {
      planning_factor_interface_->publish();
    }
  }

  /// Return accumulated planning factors, or an empty list when unused.
  [[nodiscard]] std::vector<PlanningFactor> get_planning_factors() const
  {
    if (planning_factor_interface_ != nullptr) {
      return planning_factor_interface_->get_factors();
    }
    return {};
  }

  /// Return the full plugin name used at initialization.
  [[nodiscard]] std::string get_name() const { return name_; }
  /// Return the short plugin name derived from the full name.
  [[nodiscard]] std::string get_short_name() const { return short_name_; }

protected:
  /// Return the owning ROS node.
  [[nodiscard]] rclcpp::Node * get_node_ptr() const
  {
    return context_ ? context_->node_ptr : nullptr;
  }
  /// Return the shared timing utility.
  [[nodiscard]] std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const
  {
    return context_ ? context_->time_keeper : nullptr;
  }
  /// Return the owning node clock.
  [[nodiscard]] rclcpp::Clock::SharedPtr get_clock() const
  {
    const auto node = get_node_ptr();
    return node ? node->get_clock() : nullptr;
  }

  /// Run an optimizer-style callback after adapting shared input data.
  template <class Optimizer>
  bool run_optimizer(
    Optimizer && optimizer, TrajectoryPoints & traj_points, const InputData & input)
  {
    if (!input.current_odometry || !input.current_acceleration) {
      return false;
    }

    autoware::trajectory_optimizer::TrajectoryOptimizerData data;
    data.current_odometry = *input.current_odometry;
    data.current_acceleration = *input.current_acceleration;
    if (input.semantic_speed_tracker) {
      data.semantic_speed_tracker = *input.semantic_speed_tracker;
    }

    std::forward<Optimizer>(optimizer)(traj_points, optimizer_params_, data);

    if (input.semantic_speed_tracker) {
      *input.semantic_speed_tracker = data.semantic_speed_tracker;
    }
    return true;
  }

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  std::shared_ptr<NodeContext> context_{nullptr};
  bool enabled_{true};
  double trajectory_time_step_{0.1};
  TrajectoryOptimizerParams optimizer_params_;

private:
  std::string name_{"unnamed_plugin"};
  std::string short_name_{"unnamed_plugin"};
};
}  // namespace autoware::trajectory_processor::plugin

namespace autoware::trajectory_modifier::plugin
{
using autoware::trajectory_processor::plugin::InputData;
using autoware::trajectory_processor::plugin::NodeContext;
using autoware::trajectory_processor::plugin::PlanningFactor;
using autoware::trajectory_processor::plugin::PluginBase;
using autoware::trajectory_processor::plugin::TrajectoryModifierParams;
using autoware::trajectory_processor::plugin::TrajectoryPoint;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
}  // namespace autoware::trajectory_modifier::plugin

namespace autoware::trajectory_optimizer::plugin
{
using autoware::trajectory_processor::plugin::InputData;
using autoware::trajectory_processor::plugin::NodeContext;
using autoware::trajectory_processor::plugin::PluginBase;
using autoware::trajectory_processor::plugin::TrajectoryPoint;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
}  // namespace autoware::trajectory_optimizer::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__PLUGIN_BASE_HPP_
