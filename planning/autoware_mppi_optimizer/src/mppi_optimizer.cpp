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

#include "autoware/mppi_optimizer/mppi_optimizer.hpp"

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params_ros.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_vehicle_params_ros.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <memory>
#include <optional>
#include <stdexcept>

namespace autoware::mppi_optimizer
{

MppiOptimizer::MppiOptimizer(const rclcpp::NodeOptions & options) : Node("mppi_optimizer", options)
{
  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&MppiOptimizer::on_trajectory, this, std::placeholders::_1));
  trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  mppi_interface_ = std::make_unique<FirstOrderDubinsMppiInterface>();
  declare_first_order_dubins_mppi_cost_params(*this);
  declare_first_order_dubins_mppi_vehicle_dynamics_params(*this);
  mppi_interface_->setCostParams(get_first_order_dubins_mppi_cost_params(*this));
  mppi_interface_->setVehicleParams(get_first_order_dubins_mppi_vehicle_params(*this));
  const bool enable_debug_log = declare_parameter<bool>("enable_debug_trajectory_log", false);
  const std::string debug_log_dir =
    declare_parameter<std::string>("debug_trajectory_log_directory", "/tmp/mppi_debug_log");
  mppi_interface_->setDebugTrajectoryLogging(enable_debug_log, debug_log_dir);
  mppi_interface_->setAblationOptions(
    declare_parameter<bool>("ignore_obstacles", false),
    declare_parameter<bool>("ignore_drivable_area", false),
    declare_parameter<bool>("force_cold_start_each_step", false));
}

void MppiOptimizer::on_trajectory(const Trajectory::ConstSharedPtr msg)
{
  if (!msg || msg->points.empty()) {
    return;
  }

  const auto odometry = sub_odometry_.take_data();
  if (!odometry) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for odometry...");
    return;
  }

  const auto steering_status = sub_steering_.take_data();
  const std::optional<SteeringReport> ego_steering =
    steering_status ? std::make_optional(*steering_status) : std::nullopt;

  try {
    const autoware_perception_msgs::msg::TrackedObjects empty_tracked_objects;
    const auto result = mppi_interface_->optimizeTrajectory(
      *msg, *odometry, std::nullopt, ego_steering, empty_tracked_objects);
    Trajectory tracked = result.trajectory;
    tracked.header = msg->header;
    trajectory_pub_->publish(tracked);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(get_logger(), "MPPI tracking failed: " << e.what());
    trajectory_pub_->publish(*msg);
  }
}

}  // namespace autoware::mppi_optimizer

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::mppi_optimizer::MppiOptimizer)
