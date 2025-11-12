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

#ifndef AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_HPP_

#include "autoware/trajectory_optimizer/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer_structs.hpp"

#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_optimizer
{

using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class TrajectoryOptimizer : public rclcpp::Node
{
public:
  explicit TrajectoryOptimizer(const rclcpp::NodeOptions & options);

private:
  void on_traj(const CandidateTrajectories::ConstSharedPtr msg);
  void set_up_params();
  void initialize_optimizers();
  void load_plugin(const std::string & plugin_name);
  bool initialized_optimizers_{false};

  /**
   * @brief Callback for parameter updates
   * @param parameters Vector of updated parameters
   * @return Set parameters result
   */
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  // Pluginlib loader and plugin storage
  std::unique_ptr<pluginlib::ClassLoader<plugin::TrajectoryOptimizerPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::TrajectoryOptimizerPluginBase>> plugins_;

  // interface subscriber
  rclcpp::Subscription<CandidateTrajectories>::SharedPtr trajectories_sub_;
  // interface publisher
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr trajectories_pub_;

  autoware_utils::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "~/input/odometry"};
  autoware_utils::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};

  Odometry::ConstSharedPtr current_odometry_ptr_;  // current odometry
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_ptr_;

  rclcpp::Publisher<autoware_utils::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};

  TrajectoryOptimizerParams params_;
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
};

}  // namespace autoware::trajectory_optimizer

#endif  // AUTOWARE__TRAJECTORY_OPTIMIZER__TRAJECTORY_OPTIMIZER_HPP_
