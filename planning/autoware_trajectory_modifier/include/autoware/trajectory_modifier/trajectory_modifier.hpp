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

#ifndef AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_HPP_
#define AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_HPP_

#include "autoware/trajectory_modifier/trajectory_modifier_plugins/stop_point_fixer.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_structs.hpp"

#include <autoware_trajectory_modifier/trajectory_modifier_param.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_modifier
{

using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class TrajectoryModifier : public rclcpp::Node
{
public:
  explicit TrajectoryModifier(const rclcpp::NodeOptions & options);

private:
  void on_traj(const CandidateTrajectories::ConstSharedPtr msg);
  void initialize_modifiers();
  void load_plugin(const std::string & name);
  void unload_plugin(const std::string & name);
  void reset_previous_data();
  void set_data();
  bool initialized_modifiers_{false};

  std::unique_ptr<trajectory_modifier_params::ParamListener> param_listener_;
  trajectory_modifier_params::Params params_;

  void update_params();

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr trajectories_sub_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr trajectories_pub_;
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_;

  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_current_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_current_acceleration_{this, "~/input/acceleration"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PointCloud2> sub_pointcloud_{
    this, "~/input/pointcloud", autoware_utils::single_depth_sensor_qos()};

  Odometry::ConstSharedPtr current_odometry_ptr_;
  AccelWithCovarianceStamped::ConstSharedPtr current_acceleration_ptr_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  pluginlib::ClassLoader<plugin::TrajectoryModifierPluginBase> plugin_loader_;
  std::vector<std::shared_ptr<plugin::TrajectoryModifierPluginBase>> plugins_;

  std::shared_ptr<TrajectoryModifierData> data_;
};

}  // namespace autoware::trajectory_modifier

#endif  // AUTOWARE__TRAJECTORY_MODIFIER__TRAJECTORY_MODIFIER_HPP_
