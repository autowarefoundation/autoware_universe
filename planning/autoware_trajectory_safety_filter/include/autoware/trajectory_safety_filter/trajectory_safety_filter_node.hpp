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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__TRAJECTORY_SAFETY_FILTER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__TRAJECTORY_SAFETY_FILTER_NODE_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_trajectory_safety_filter_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_safety_filter
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using nav_msgs::msg::Odometry;

class TrajectorySafetyFilter : public rclcpp::Node
{
public:
  explicit TrajectorySafetyFilter(const rclcpp::NodeOptions & node_options);

private:
  void process(const CandidateTrajectories::ConstSharedPtr msg);

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  bool validate_trajectory_basics(const CandidateTrajectory & trajectory) const;
  bool check_finite(const TrajectoryPoint & point) const;

  void load_metric(const std::string & name);

  /**
   * @brief Unloads a metric plugin
   * @param name Metric plugin name to unload
   */
  void unload_metric(const std::string & name);

  std::unique_ptr<safety_filter::ParamListener> listener_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};

  rclcpp::Subscription<LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;

  pluginlib::ClassLoader<plugin::SafetyFilterInterface> plugin_loader_;
  std::vector<std::shared_ptr<plugin::SafetyFilterInterface>> plugins_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
};

}  // namespace autoware::trajectory_safety_filter

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__TRAJECTORY_SAFETY_FILTER_NODE_HPP_
