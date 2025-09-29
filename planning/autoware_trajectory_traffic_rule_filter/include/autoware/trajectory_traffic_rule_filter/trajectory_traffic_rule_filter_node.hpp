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

#ifndef AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAJECTORY_TRAFFIC_RULE_FILTER_NODE_HPP_
#define AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAJECTORY_TRAFFIC_RULE_FILTER_NODE_HPP_

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>

#include <map>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;

class TrajectoryTrafficRuleFilter : public rclcpp::Node
{
public:
  explicit TrajectoryTrafficRuleFilter(const rclcpp::NodeOptions & node_options);

private:
  void process(const CandidateTrajectories::ConstSharedPtr msg) override;

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  lanelet::ConstLanelets get_lanelets_from_trajectory(
    const TrajectoryPoints & trajectory_points) const;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  pluginlib::ClassLoader<plugin::SafetyFilterInterface> plugin_loader_;
  std::vector<std::shared_ptr<plugin::SafetyFilterInterface>> plugins_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
};

}  // namespace autoware::trajectory_traffic_rule_filter

#endif  // AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAJECTORY_TRAFFIC_RULE_FILTER_NODE_HPP_
