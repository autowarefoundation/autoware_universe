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

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_processor::plugin
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

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
  rclcpp::Node * node_ptr{nullptr};
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper{nullptr};
  autoware::vehicle_info_utils::VehicleInfo vehicle_info{};
  tf2_ros::Buffer * tf_buffer{nullptr};
};

class PluginBase
{
public:
  PluginBase() = default;
  virtual ~PluginBase() = default;

  virtual void initialize(const std::string & name, std::shared_ptr<NodeContext> context)
  {
    name_ = name;
    short_name_ = std::invoke([&name]() {
      const auto npos = name.find_last_of(':');
      return npos != std::string::npos ? name.substr(npos + 1) : name;
    });
    context_ = std::move(context);
  }

  virtual bool modify_trajectory(TrajectoryPoints & traj_points, const InputData & input) = 0;

  virtual rcl_interfaces::msg::SetParametersResult on_parameter(
    [[maybe_unused]] const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    return result;
  }

  virtual void update_params(
    [[maybe_unused]] const autoware::trajectory_optimizer::TrajectoryOptimizerParams & params)
  {
  }

  virtual void publish_debug_data([[maybe_unused]] const std::string & ns) const {}
  virtual void publish_debug_markers() const {}

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

  std::string get_name() const { return name_; }
  std::string get_short_name() const { return short_name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return context_ ? context_->node_ptr : nullptr; }
  std::shared_ptr<autoware_utils_debug::TimeKeeper> get_time_keeper() const
  {
    return context_ ? context_->time_keeper : nullptr;
  }
  rclcpp::Clock::SharedPtr get_clock() const { return get_node_ptr()->get_clock(); }

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  std::shared_ptr<NodeContext> context_{nullptr};
  bool enabled_{true};

private:
  std::string name_{"unnamed_plugin"};
  std::string short_name_{"unnamed_plugin"};
};
}  // namespace autoware::trajectory_processor::plugin

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__PLUGIN_BASE_HPP_
