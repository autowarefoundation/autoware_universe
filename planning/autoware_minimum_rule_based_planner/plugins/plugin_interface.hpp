// Copyright 2025 TIER IV, Inc.
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

// NOLINTNEXTLINE
#ifndef PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__PLUGIN_INTERFACE_HPP_
// NOLINTNEXTLINE
#define PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__PLUGIN_INTERFACE_HPP_

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware/vehicle_info_utils/vehicle_info.hpp>
#include <autoware_minimum_rule_based_planner/minimum_rule_based_planner_parameters.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::minimum_rule_based_planner::plugin
{
using autoware::vehicle_info_utils::VehicleInfo;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using MinimumRuleBasedPlannerParams = ::minimum_rule_based_planner::Params;

struct ModifierData
{
  explicit ModifierData(rclcpp::Node * node) : tf_buffer{node->get_clock()}, tf_listener{tf_buffer}
  {
  }

  Odometry::ConstSharedPtr odometry_ptr;
  AccelWithCovarianceStamped::ConstSharedPtr acceleration_ptr;
  PointCloud2::ConstSharedPtr obstacle_pointcloud_ptr;
  PredictedObjects::ConstSharedPtr predicted_objects_ptr;
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
};

class PluginInterface
{
public:
  PluginInterface() = default;

  void initialize(
    std::string name, rclcpp::Node * node_ptr,
    const std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper,
    const std::shared_ptr<ModifierData> & modifier_data, const VehicleInfo & vehicle_info,
    [[maybe_unused]] const MinimumRuleBasedPlannerParams & params)
  {
    name_ = std::move(name);
    node_ptr_ = node_ptr;
    time_keeper_ = time_keeper;
    data_ = modifier_data;
    vehicle_info_ = vehicle_info;
    RCLCPP_DEBUG(node_ptr_->get_logger(), "instantiated PluginInterface: %s", name_.c_str());
    on_initialize(params);
  }

  virtual ~PluginInterface() = default;
  virtual void run(TrajectoryPoints & traj_points) = 0;
  std::string get_name() const { return name_; }
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const { return time_keeper_; }
  virtual void update_params(const MinimumRuleBasedPlannerParams & params) = 0;

  virtual void publish_planning_factor()
  {
    if (planning_factor_interface_) {
      planning_factor_interface_->publish();
    }
  }
  std::vector<PlanningFactor> get_planning_factors() const
  {
    if (planning_factor_interface_ != nullptr) {
      return planning_factor_interface_->get_factors();
    }
    return {};
  }

protected:
  virtual void on_initialize(const MinimumRuleBasedPlannerParams & params) = 0;
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  VehicleInfo vehicle_info_;
  std::shared_ptr<ModifierData> data_;

  rclcpp::Clock::SharedPtr get_clock() const { return node_ptr_->get_clock(); }

private:
  std::string name_;
  rclcpp::Node * node_ptr_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};
}  // namespace autoware::minimum_rule_based_planner::plugin

// NOLINTNEXTLINE
#endif  // PLANNING__AUTOWARE_MINIMUM_RULE_BASED_PLANNER__PLUGINS__PLUGIN_INTERFACE_HPP_
