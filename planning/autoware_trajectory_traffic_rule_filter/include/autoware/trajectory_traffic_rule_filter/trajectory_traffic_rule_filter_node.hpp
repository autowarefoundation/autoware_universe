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

#include "autoware/trajectory_traffic_rule_filter/traffic_rule_filter_interface.hpp"

#include <autoware_trajectory_traffic_rule_filter/autoware_trajectory_traffic_rule_filter_param.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/subscription.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_routing/Forward.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_traffic_rule_filter
{
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

class TrajectoryTrafficRuleFilter : public rclcpp::Node
{
public:
  explicit TrajectoryTrafficRuleFilter(const rclcpp::NodeOptions & node_options);

private:
  void process(const CandidateTrajectories::ConstSharedPtr msg);

  void map_callback(const LaneletMapBin::ConstSharedPtr msg);

  void load_metric(const std::string & name);
  void unload_metric(const std::string & name);
  void update_diagnostic(const CandidateTrajectories & filtered_trajectories);

  lanelet::ConstLanelets get_lanelets_from_trajectory(
    const TrajectoryPoints & trajectory_points) const;

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr sub_map_;
  autoware_utils_rclcpp::InterProcessPollingSubscriber<
    autoware_perception_msgs::msg::TrafficLightGroupArray>
    sub_traffic_lights_{this, "~/input/traffic_signals"};

  rclcpp::Subscription<CandidateTrajectories>::SharedPtr sub_trajectories_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_trajectories_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;

  pluginlib::ClassLoader<plugin::TrafficRuleFilterInterface> plugin_loader_;
  std::vector<std::shared_ptr<plugin::TrafficRuleFilterInterface>> plugins_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  std::unique_ptr<traffic_rule_filter::ParamListener> listener_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    debug_processing_time_detail_pub_;
  mutable std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_{nullptr};
  autoware_utils_diagnostics::DiagnosticsInterface diagnostics_interface_{
    this, "trajectory_traffic_rule_filter"};
};

}  // namespace autoware::trajectory_traffic_rule_filter

#endif  // AUTOWARE__TRAJECTORY_TRAFFIC_RULE_FILTER__TRAJECTORY_TRAFFIC_RULE_FILTER_NODE_HPP_
