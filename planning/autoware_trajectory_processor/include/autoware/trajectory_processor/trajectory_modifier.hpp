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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_HPP_

#include "autoware/trajectory_processor/trajectory_modifier_context.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/input_data.hpp"
#include "autoware/trajectory_processor/trajectory_modifier_plugins/trajectory_modifier_plugin_base.hpp"

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/polling_subscriber.hpp>
#include <autoware_trajectory_processor/trajectory_modifier_param.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
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

namespace polling = autoware::agnocast_wrapper::polling;

class TrajectoryModifier : public autoware::agnocast_wrapper::Node
{
public:
  explicit TrajectoryModifier(const rclcpp::NodeOptions & options);

private:
  void on_traj(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(CandidateTrajectories) & msg);
  void on_map(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::LaneletMapBin) & msg);
  void load_plugin(const std::string & name);
  void unload_plugin(const std::string & name);
  tl::expected<plugin::InputData, std::string> make_input_data();
  bool initialized_modifiers_{false};

  std::unique_ptr<trajectory_modifier_params::ParamListener> param_listener_;
  trajectory_modifier_params::Params params_;

  void update_params();

  AUTOWARE_SUBSCRIPTION_PTR(CandidateTrajectories) trajectories_sub_;
  AUTOWARE_PUBLISHER_PTR(CandidateTrajectories) trajectories_pub_;

  polling::PollingSubscriber<Odometry>::SharedPtr sub_current_odometry_ =
    polling::create_polling_subscriber<Odometry>(this, "~/input/odometry");
  polling::PollingSubscriber<AccelWithCovarianceStamped>::SharedPtr sub_current_acceleration_ =
    polling::create_polling_subscriber<AccelWithCovarianceStamped>(this, "~/input/acceleration");
  polling::PollingSubscriber<PredictedObjects>::SharedPtr sub_objects_ =
    polling::create_polling_subscriber<PredictedObjects>(this, "~/input/objects");
  polling::PollingSubscriber<PointCloud2>::SharedPtr sub_pointcloud_ =
    polling::create_polling_subscriber<PointCloud2>(
      this, "~/input/pointcloud", autoware_utils_rclcpp::single_depth_sensor_qos());

  polling::PollingSubscriber<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr
    sub_traffic_lights_ =
      polling::create_polling_subscriber<autoware_perception_msgs::msg::TrafficLightGroupArray>(
        this, "~/input/traffic_signals");
  AUTOWARE_SUBSCRIPTION_PTR(autoware_map_msgs::msg::LaneletMapBin) sub_map_;
  polling::PollingSubscriber<autoware_planning_msgs::msg::LaneletRoute>::SharedPtr sub_route_ =
    polling::create_polling_subscriber<autoware_planning_msgs::msg::LaneletRoute>(
      this, "~/input/route", rclcpp::QoS{1}.transient_local());

  AUTOWARE_PUBLISHER_PTR(autoware_utils_debug::ProcessingTimeDetail)
  debug_processing_time_detail_pub_;
  std::shared_ptr<autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    pub_processing_time_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  pluginlib::ClassLoader<plugin::TrajectoryModifierPluginBase> plugin_loader_;
  std::vector<std::shared_ptr<plugin::TrajectoryModifierPluginBase>> plugins_;

  std::shared_ptr<TrajectoryModifierContext> context_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
};

}  // namespace autoware::trajectory_modifier

#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_HPP_
