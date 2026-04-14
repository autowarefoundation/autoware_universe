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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_NODE_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_NODE_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_trajectory_validator/autoware_trajectory_validator_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils_diagnostics::DiagnosticsInterface;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;

class TrajectoryValidator : public rclcpp::Node
{
public:
  explicit TrajectoryValidator(const rclcpp::NodeOptions & node_options);

private:
  void process(const CandidateTrajectories::ConstSharedPtr msg);

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  void load_metric(const std::string & name);

  /**
   * @brief Unloads a metric plugin
   * @param name Metric plugin name to unload
   */
  void unload_metric(const std::string & name);
  void update_diagnostic(
    const CandidateTrajectories & input_trajectories,
    const CandidateTrajectories & filtered_trajectories);

  validator::ParamListener listener_;
  validator::Params params_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<AccelWithCovarianceStamped>
    sub_acceleration_{this, "~/input/acceleration"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_lights_{this, "~/input/traffic_signals"};

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  pluginlib::ClassLoader<plugin::ValidatorInterface> plugin_loader_;
  std::vector<std::shared_ptr<plugin::ValidatorInterface>> plugins_;

  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  DiagnosticsInterface diagnostics_interface_{this, "trajectory_validator"};
};

}  // namespace autoware::trajectory_validator

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__TRAJECTORY_VALIDATOR_NODE_HPP_
