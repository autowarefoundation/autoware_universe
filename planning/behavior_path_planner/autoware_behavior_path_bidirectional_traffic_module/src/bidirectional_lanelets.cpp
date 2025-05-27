// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"

#include "autoware/behavior_path_bidirectional_traffic_module/parameter.hpp"
#include "autoware/behavior_path_bidirectional_traffic_module/utils.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"
#include "autoware_utils_system/lru_cache.hpp"

#include <Eigen/Core>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <boost/geometry/algorithms/area.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/algorithms/length.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <list>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

bool is_bidirectional_lanelets_pair(
  const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b)
{
  std::string_view turn_direction_a = lanelet_a.attributeOr("turn_direction", "none");
  if (turn_direction_a == "right" || turn_direction_a == "left") {
    return false;
  }
  std::string_view turn_direction_b = lanelet_b.attributeOr("turn_direction", "none");
  if (turn_direction_b == "right" || turn_direction_b == "left") {
    return false;
  }
  return (lanelet_a.leftBound().id() == lanelet_b.rightBound().id()) &&
         (lanelet_a.rightBound().id() == lanelet_b.leftBound().id());
}

void remove_lanelet_with_turn_direction(lanelet::ConstLanelets * lanelets)
{
  lanelets->erase(
    std::remove_if(
      lanelets->begin(), lanelets->end(),
      [](const lanelet::ConstLanelet & lanelet) {
        std::string_view turn_direction = lanelet.attributeOr("turn_direction", "none");
        return turn_direction == "right" || turn_direction == "left";
      }),
    lanelets->end());
}

void search_connected_bidirectional_lanelet_in_one_direction(
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets,
  const lanelet::ConstLanelet & lanelet_a,             //
  const lanelet::ConstLanelet & lanelet_b,             //
  std::list<lanelet::ConstLanelet> * lanelets_list_a,  //
  std::list<lanelet::ConstLanelet> * lanelets_list_b)
{
  auto next_lanelets_a = get_next_lanelets(lanelet_a);
  auto prev_lanelets_b = get_prev_lanelets(lanelet_b);
  remove_lanelet_with_turn_direction(&next_lanelets_a);
  remove_lanelet_with_turn_direction(&prev_lanelets_b);
  if (next_lanelets_a.size() != 1 || prev_lanelets_b.size() != 1) return;
  auto next_lanelet_a = next_lanelets_a.front();
  auto prev_lanelet_b = prev_lanelets_b.front();
  if (is_bidirectional_lanelets_pair(next_lanelet_a, prev_lanelet_b)) {
    lanelets_list_a->emplace_back(next_lanelet_a);
    lanelets_list_b->emplace_front(prev_lanelet_b);
    search_connected_bidirectional_lanelet_in_one_direction(
      get_next_lanelets, get_prev_lanelets, next_lanelet_a, prev_lanelet_b, lanelets_list_a,
      lanelets_list_b);
  }
}

ConnectedBidirectionalLanelets::ConnectedBidirectionalLanelets(
  lanelet::ConstLanelets bidirectional_lanelets, lanelet::ConstLanelets lanelets_before_entering,
  lanelet::ConstLanelets lanelets_after_entering)
: bidirectional_lanelets_(std::move(bidirectional_lanelets)),
  lanelets_before_entering_(std::move(lanelets_before_entering)),
  lanelets_after_entering_(std::move(lanelets_after_entering))
{
}

std::pair<
  ConnectedBidirectionalLanelets::SharedConstPtr, ConnectedBidirectionalLanelets::SharedConstPtr>
ConnectedBidirectionalLanelets::make_bidirectional_lane_pair(
  const std::pair<lanelet::ConstLanelet, lanelet::ConstLanelet> & one_bidirectional_lanelet_pair,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets)
{
  const auto & [lanelet_a, lanelet_b] = one_bidirectional_lanelet_pair;

  std::list<lanelet::ConstLanelet> lanelets_list_a = {lanelet_a};
  std::list<lanelet::ConstLanelet> lanelets_list_b = {lanelet_b};

  search_connected_bidirectional_lanelet_in_one_direction(
    get_next_lanelets, get_prev_lanelets, lanelet_a, lanelet_b, &lanelets_list_a, &lanelets_list_b);
  search_connected_bidirectional_lanelet_in_one_direction(
    get_next_lanelets, get_prev_lanelets, lanelet_b, lanelet_a, &lanelets_list_b, &lanelets_list_a);

  lanelet::ConstLanelets lanelets_a(lanelets_list_a.begin(), lanelets_list_a.end());
  lanelet::ConstLanelets lanelets_b(lanelets_list_b.begin(), lanelets_list_b.end());
  lanelet::ConstLanelets lanelets_inflow_to_a = get_inflow_lanelets(lanelets_a, get_prev_lanelets);
  lanelet::ConstLanelets lanelets_outflow_from_a =
    get_outflow_lanelets(lanelets_a, get_next_lanelets);
  lanelet::ConstLanelets lanelets_inflow_to_b = get_inflow_lanelets(lanelets_b, get_prev_lanelets);
  lanelet::ConstLanelets lanelets_outflow_from_b =
    get_outflow_lanelets(lanelets_b, get_next_lanelets);

  auto bidirectional_lanelets_a = std::make_shared<ConnectedBidirectionalLanelets>(
    ConnectedBidirectionalLanelets{lanelets_a, lanelets_inflow_to_a, lanelets_outflow_from_a});

  auto bidirectional_lanelets_b = std::make_shared<ConnectedBidirectionalLanelets>(
    ConnectedBidirectionalLanelets{lanelets_b, lanelets_inflow_to_b, lanelets_outflow_from_b});

  bidirectional_lanelets_a->opposite_ = bidirectional_lanelets_b;
  bidirectional_lanelets_b->opposite_ = bidirectional_lanelets_a;

  return {bidirectional_lanelets_a, bidirectional_lanelets_b};
}

std::vector<ConnectedBidirectionalLanelets::SharedConstPtr>
ConnectedBidirectionalLanelets::search_bidirectional_lanes_on_map(
  const lanelet::LaneletMap & map,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_prev_lanelets)
{
  std::vector<ConnectedBidirectionalLanelets::SharedConstPtr> bidirectional_lanelets;

  std::unordered_set<lanelet::ConstLanelet> searched_lanelets;

  for (auto lanelet_itr_a = map.laneletLayer.begin(); lanelet_itr_a != map.laneletLayer.end();
       ++lanelet_itr_a) {
    for (auto lanelet_itr_b = std::next(lanelet_itr_a); lanelet_itr_b != map.laneletLayer.end();
         ++lanelet_itr_b) {
      if (
        is_bidirectional_lanelets_pair(*lanelet_itr_a, *lanelet_itr_b) &&
        searched_lanelets.find(*lanelet_itr_a) == searched_lanelets.end() &&
        searched_lanelets.find(*lanelet_itr_b) == searched_lanelets.end()) {
        auto [bidirectional_lanelets_a, bidirectional_lanelets_b] = make_bidirectional_lane_pair(
          {*lanelet_itr_a, *lanelet_itr_b}, get_next_lanelets, get_prev_lanelets);
        searched_lanelets.insert(
          bidirectional_lanelets_a->get_lanelets().begin(),
          bidirectional_lanelets_a->get_lanelets().end());
        searched_lanelets.insert(
          bidirectional_lanelets_b->get_lanelets().begin(),
          bidirectional_lanelets_b->get_lanelets().end());
        bidirectional_lanelets.emplace_back(bidirectional_lanelets_a);
        bidirectional_lanelets.emplace_back(bidirectional_lanelets_b);
      }
    }
  }
  return bidirectional_lanelets;
}

std::optional<experimental::trajectory::Interval>
ConnectedBidirectionalLanelets::get_overlap_interval(
  const experimental::trajectory::Trajectory<
    autoware_internal_planning_msgs::msg::PathPointWithLaneId> & trajectory) const
{
  std::unordered_set<lanelet::Id> lane_ids_set;
  for (const auto & lanelet : bidirectional_lanelets_) {
    lane_ids_set.insert(lanelet.id());
  }

  auto interval = experimental::trajectory::find_intervals(
    trajectory,
    [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) -> bool {
      for (const auto & lane_id : point.lane_ids) {
        return lane_ids_set.find(lane_id) != lane_ids_set.end();
      }
      return false;
    });
  if (interval.empty()) return std::nullopt;
  return interval.front();
}

Eigen::Vector2d calc_pose_direction(const geometry_msgs::msg::Pose & pose)
{
  double yaw = tf2::getYaw(pose.orientation);
  return {cos(yaw), sin(yaw)};
}

Eigen::Vector2d calc_lanelet_direction(const lanelet::ConstLanelet & lanelet)
{
  return (lanelet.centerline2d().back().basicPoint2d() -
          lanelet.centerline2d().front().basicPoint2d())
    .normalized();
}

bool is_same_direction(
  const Eigen::Vector2d & a, const Eigen::Vector2d & b, const double & eps = 1.0)
{
  return std::abs(a.dot(b) - 1) < eps;
}

bool ConnectedBidirectionalLanelets::is_object_on_this_lane(
  const geometry_msgs::msg::Pose & obj_pose,
  const autoware::universe_utils::Polygon2d & obj_polygon) const
{
  Eigen::Vector2d obj_direction = calc_pose_direction(obj_pose);

  for (const auto & lanelet : bidirectional_lanelets_) {
    auto lanelet_polygon = lanelet.polygon2d().basicPolygon();
    if (boost::geometry::disjoint(obj_polygon, lanelet_polygon)) {
      continue;
    }
    Eigen::Vector2d lanelet_direction = calc_lanelet_direction(lanelet);
    if (is_same_direction(obj_direction, lanelet_direction)) {
      return true;
    }
  }
  return false;
}

bool ConnectedBidirectionalLanelets::is_object_on_this_lane(
  const autoware_perception_msgs::msg::PredictedObject & obj) const
{
  autoware::universe_utils::Polygon2d obj_polygon = autoware::universe_utils::toPolygon2d(obj);
  const geometry_msgs::msg::Pose obj_pose = obj.kinematics.initial_pose_with_covariance.pose;
  return is_object_on_this_lane(obj_pose, obj_polygon);
}

[[nodiscard]] ConnectedBidirectionalLanelets::SharedConstPtr
ConnectedBidirectionalLanelets::get_opposite() const
{
  return opposite_.lock();
}

[[nodiscard]] const lanelet::ConstLanelets & ConnectedBidirectionalLanelets::get_lanelets() const
{
  return bidirectional_lanelets_;
}

[[nodiscard]] const lanelet::ConstLanelets &
ConnectedBidirectionalLanelets::get_lanelets_before_entering() const
{
  return lanelets_before_entering_;
}

[[nodiscard]] const lanelet::ConstLanelets &
ConnectedBidirectionalLanelets::get_lanelets_after_exiting() const
{
  return lanelets_after_entering_;
}

[[nodiscard]] experimental::trajectory::Trajectory<geometry_msgs::msg::Pose>
ConnectedBidirectionalLanelets::get_center_line() const
{
  // add cache
  static autoware_utils_system::LRUCache<
    lanelet::ConstLanelets, experimental::trajectory::Trajectory<geometry_msgs::msg::Point>,
    ConstLaneletsHashMap>
    cache(1000);
  if (cache.contains(bidirectional_lanelets_)) {
    return experimental::trajectory::Trajectory<geometry_msgs::msg::Pose>{
      cache.get(bidirectional_lanelets_).value()};
  }
  std::vector<geometry_msgs::msg::Point> center_line;
  for (const auto & lane : bidirectional_lanelets_) {
    for (const auto & point : lane.centerline()) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      center_line.emplace_back(p);
    }
    if (lane != bidirectional_lanelets_.back()) {
      center_line.pop_back();
    }
  }
  auto trajectory =
    experimental::trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(center_line);

  if (!trajectory) {
    throw std::runtime_error("Failed to build trajectory in ConnectedBidirectionalLanelets");
  }

  cache.put(bidirectional_lanelets_, *trajectory);

  return experimental::trajectory::Trajectory<geometry_msgs::msg::Pose>{*trajectory};
}

[[nodiscard]] double ConnectedBidirectionalLanelets::average_lane_width() const
{
  static autoware_utils_system::LRUCache<lanelet::ConstLanelets, double, ConstLaneletsHashMap>
    cache(1000);
  if (cache.contains(bidirectional_lanelets_)) {
    return cache.get(bidirectional_lanelets_).value();
  }
  long double length_sum = 0;
  double area_sum = 0;
  for (const auto & lane : bidirectional_lanelets_) {
    length_sum += boost::geometry::length(lane.centerline2d());
    area_sum += boost::geometry::area(lane.polygon2d().basicPolygon());
  }

  auto result = static_cast<double>(area_sum / length_sum);
  cache.put(bidirectional_lanelets_, result);
  return result;
}

std::optional<ConnectedBidirectionalLanelets::SharedConstPtr>
get_bidirectional_lanelets_where_ego_is(
  const geometry_msgs::msg::Pose & ego_pose, const EgoParameters & ego_params,
  const std::vector<ConnectedBidirectionalLanelets::SharedConstPtr> & all_bidirectional_lanes)
{
  for (const auto & bidirectional_lane : all_bidirectional_lanes) {
    if (bidirectional_lane->is_object_on_this_lane(ego_pose, ego_params.ego_polygon(ego_pose))) {
      return bidirectional_lane;
    }
  }
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
