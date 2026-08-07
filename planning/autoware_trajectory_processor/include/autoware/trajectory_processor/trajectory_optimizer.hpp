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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_HPP_

#include "autoware/trajectory_processor/trajectory_optimizer_plugins/trajectory_optimizer_plugin_base.hpp"
#include "autoware/trajectory_processor/trajectory_optimizer_structs.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_debug_msgs/msg/float64_stamped.hpp>
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

class TrajectoryOptimizer : public autoware::agnocast_wrapper::Node
{
public:
  explicit TrajectoryOptimizer(const rclcpp::NodeOptions & options);

private:
  void on_traj(AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) msg);
  void publish_processing_time_ms(double processing_time_ms);
  void update_params();
  void initialize_optimizers();
  void load_plugin(const std::string & plugin_name);
  bool initialized_optimizers_{false};

  // Pluginlib loader and plugin storage
  std::unique_ptr<pluginlib::ClassLoader<plugin::TrajectoryOptimizerPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::TrajectoryOptimizerPluginBase>> plugins_;

  // interface subscriber
  AUTOWARE_SUBSCRIPTION_PTR(CandidateTrajectories) trajectories_sub_;
  // interface publisher
  AUTOWARE_PUBLISHER_PTR(Trajectory) trajectory_pub_;
  AUTOWARE_PUBLISHER_PTR(CandidateTrajectories) trajectories_pub_;

  // Polling subscribers are created in the constructor through the wrapper so they route
  // through Agnocast when enabled.
  autoware::agnocast_wrapper::polling::PollingSubscriber<Odometry>::SharedPtr sub_current_odometry_;
  autoware::agnocast_wrapper::polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr
    sub_current_acceleration_;

  // polling::PollingSubscriber::take_data() returns a plain std::shared_ptr in both
  // ENABLE_AGNOCAST=0/1 builds.
  std::shared_ptr<const Odometry> current_odometry_ptr_;  // current odometry
  std::shared_ptr<const AccelWithCovarianceStamped> current_acceleration_ptr_;
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;

  AUTOWARE_PUBLISHER_PTR(autoware_utils::ProcessingTimeDetail) debug_processing_time_detail_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_internal_debug_msgs::msg::Float64Stamped)
  debug_processing_time_pub_;
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_{nullptr};

  std::unique_ptr<trajectory_optimizer_node_params::ParamListener> param_listener_;
  TrajectoryOptimizerParams params_;
};

}  // namespace autoware::trajectory_optimizer

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_OPTIMIZER_HPP_
