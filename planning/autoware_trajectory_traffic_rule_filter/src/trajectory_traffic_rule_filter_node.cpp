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

#include "autoware/trajectory_traffic_rule_filter/trajectory_traffic_rule_filter_node.hpp"

#include "utils.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/logging.hpp>

#include <autoware_internal_planning_msgs/msg/detail/path_point_with_lane_id__struct.hpp>
#include <autoware_new_planning_msgs/msg/detail/trajectories__struct.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_routing/Forward.h>

#include <algorithm>
#include <memory>

namespace autoware::trajectory_traffic_rule_filter
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_internal_planning_msgs::msg::PathWithLaneId;

TrajectoryTrafficRuleFilter::TrajectoryTrafficRuleFilter(const rclcpp::NodeOptions & node_options)
: Node{"trajectory_traffic_rule_filter_node", node_options},
  plugin_loader(
    "autoware_trajectory_traffic_rule_filter",
    "autoware::trajectory_traffic_rule_filter::plugin::TrafficRuleFilterInterface"),
  vehicle_info_{autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()}
{
  debug_processing_time_detail_pub_ =
    this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
      "~/debug/processing_time_detail_ms", 1);
  time_keeper_ =
    std::make_shared<autoware_utils_debug::TimeKeeper>(debug_processing_time_detail_pub_);
  sub_map_ = create_subscription<LaneletMapBin>(
    "~/input/lanelet2_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrajectoryTrafficRuleFilter::map_callback, this, std::placeholders::_1));

  autoware::boundary_departure_checker::Param boundary_departure_checker_params;
  boundary_departure_checker_ =
    std::make_shared<autoware::boundary_departure_checker::BoundaryDepartureChecker>(
      boundary_departure_checker_params, vehicle_info_, time_keeper_);
}

void TrajectoryTrafficRuleFilter::process(const CandidateTrajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto trajectories = traffic_light_check(msg);
  publish(trajectories);
}

void TrajectoryTrafficRuleFilter::map_callback(const LaneletMapBin::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

lanelet::ConstLanelets TrajectoryTrafficRuleFilter::get_lanelets_from_trajectory(
  const TrajectoryPoints & trajectory_points) const
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  lanelet::ConstLanelets lanes;
  PathWithLaneId path;
  path.points.reserve(trajectory_points.size());
  for (const auto & point : trajectory_points) {
    PathPointWithLaneId path_point;
    path_point.point.pose = point.pose;
    path_point.point.longitudinal_velocity_mps = point.longitudinal_velocity_mps;
    path_point.point.lateral_velocity_mps = point.lateral_velocity_mps;
    path_point.point.heading_rate_rps = point.heading_rate_rps;
    path.points.push_back(path_point);
  }
  const auto lanelet_distance_pair =
    boundary_departure_checker_->getLaneletsFromPath(lanelet_map_ptr_, path);
  if (lanelet_distance_pair.empty()) {
    RCLCPP_WARN(get_logger(), "No lanelets found in the map");
    return lanes;
  }

  for (const auto & lanelet_distance : lanelet_distance_pair) {
    const auto & lanelet = lanelet_distance.second;
    lanes.push_back(lanelet);
  }
  return lanes;
}

Trajectories::ConstSharedPtr ValidTrajectoryFilterNode::traffic_light_check(
  const Trajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  auto trajectories = msg->trajectories;
  const auto traffic_signal_msg = traffic_signals_subscriber_.take_data();

  if (traffic_signal_msg) {
    traffic_light_id_map_.clear();
    for (const auto & signal : traffic_signal_msg->traffic_light_groups) {
      TrafficSignalStamped traffic_signal;
      traffic_signal.stamp = traffic_signal_msg->stamp;
      traffic_signal.signal = signal;
      traffic_light_id_map_[signal.traffic_light_group_id] = traffic_signal;
    }
  }

  if (!lanelet_map_ptr_) return std::make_shared<Trajectories>();

  const auto itr =
    std::remove_if(trajectories.begin(), trajectories.end(), [&](const auto & trajectory) {
      // TODO(go-sakayori): this query is slow, consider using other methods similar to
      // BoundaryDepartureChecker::getFusedLaneletPolygonForPath?
      const auto lanes = get_lanelets_from_trajectory(trajectory.points);

      for (const auto & lane : lanes) {
        for (const auto & element : lane.template regulatoryElementsAs<lanelet::TrafficLight>()) {
          if (traffic_light_id_map_.count(element->id()) == 0) continue;

          if (autoware::traffic_light_utils::isTrafficSignalStop(
                lane, traffic_light_id_map_.at(element->id()).signal)) {
            return true;
          }
        }
      }
      return false;
    });
  trajectories.erase(itr, trajectories.end());

  const auto new_trajectories = autoware_new_planning_msgs::build<Trajectories>()
                                  .trajectories(trajectories)
                                  .generator_info(msg->generator_info);

  return std::make_shared<Trajectories>(new_trajectories);
}

Trajectories::ConstSharedPtr ValidTrajectoryFilterNode::stop_line_check(
  const Trajectories::ConstSharedPtr msg)
{
  autoware_utils_debug::ScopedTimeTrack st(__func__, *time_keeper_);
  auto trajectories = msg->trajectories;
  if (!lanelet_map_ptr_) return std::make_shared<Trajectories>();
  const lanelet::ConstLanelets lanelets(
    lanelet_map_ptr_->laneletLayer.begin(), lanelet_map_ptr_->laneletLayer.end());

  const auto itr =
    std::remove_if(trajectories.begin(), trajectories.end(), [lanelets](const auto & trajectory) {
      const auto lanes = utils::get_lanes_from_trajectory(trajectory.points, lanelets);
      if (lanes.size() < 2) return false;
      for (size_t i = 0; i < lanes.size() - 1; i++) {
        for (const auto & reg_elem :
             lanes[i].template regulatoryElementsAs<lanelet::TrafficSign>()) {
          if (reg_elem->type() != "stop_sign") continue;
          if (lanes[i].id() != lanes[i + 1].id()) return true;
        }
      }
      return false;
    });
  trajectories.erase(itr, trajectories.end());

  const auto new_trajectories = autoware_new_planning_msgs::build<Trajectories>()
                                  .trajectories(trajectories)
                                  .generator_info(msg->generator_info);

  return std::make_shared<Trajectories>(new_trajectories);
}

}  // namespace autoware::trajectory_traffic_rule_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::trajectory_traffic_rule_filter::TrajectoryTrafficRuleFilterNode)
