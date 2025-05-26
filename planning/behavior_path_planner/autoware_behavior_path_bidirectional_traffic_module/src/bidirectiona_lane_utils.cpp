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

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lane_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"
#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/universe_utils/geometry/boost_geometry.hpp"
#include "autoware/universe_utils/geometry/boost_polygon_utils.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>
#include <rclcpp/logging.hpp>

#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/detail/disjoint/interface.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <list>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
namespace autoware::behavior_path_planner
{

bool is_bidirectional_lanes(
  const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b)
{
  return (lanelet_a.leftBound().id() == lanelet_b.rightBound().id()) &&
         (lanelet_a.rightBound().id() == lanelet_b.leftBound().id());
}

void remove_lanelet_with_turn_direction(lanelet::ConstLanelets * lanelets)
{
  lanelets->erase(
    std::remove_if(
      lanelets->begin(), lanelets->end(),
      [](const lanelet::ConstLanelet & lanelet) {
        std::string turn_direction = lanelet.attributeOr("turn_direction", "none");
        return turn_direction == "right" || turn_direction == "left";
      }),
    lanelets->end());
}

void search_connected_bidirectional_lanelet_in_one_direction(
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> &
    get_previous_lanelets,
  const lanelet::ConstLanelet & lanelet_a,                             //
  const lanelet::ConstLanelet & lanelet_b,                             //
  std::list<lanelet::ConstLanelet> * connected_bidirectional_lanes_a,  //
  std::list<lanelet::ConstLanelet> * connected_bidirectional_lanes_b)
{
  auto next_lanelets_a = get_next_lanelets(lanelet_a);
  auto prev_lanelets_b = get_previous_lanelets(lanelet_b);
  remove_lanelet_with_turn_direction(&next_lanelets_a);
  remove_lanelet_with_turn_direction(&prev_lanelets_b);
  if (next_lanelets_a.size() != 1 || prev_lanelets_b.size() != 1) return;
  auto next_lanelet_a = next_lanelets_a.front();
  auto prev_lanelet_b = prev_lanelets_b.front();
  if (is_bidirectional_lanes(next_lanelet_a, prev_lanelet_b)) {
    connected_bidirectional_lanes_a->emplace_back(next_lanelet_a);
    connected_bidirectional_lanes_b->emplace_front(prev_lanelet_b);
    search_connected_bidirectional_lanelet_in_one_direction(
      get_next_lanelets, get_previous_lanelets, next_lanelet_a, prev_lanelet_b,
      connected_bidirectional_lanes_a, connected_bidirectional_lanes_b);
  }
}

std::pair<ConnectedBidirectionalLanelets, ConnectedBidirectionalLanelets>
ConnectedBidirectionalLanelets::search_connected_bidirectional_lanelets(
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> &
    get_previous_lanelets,
  const lanelet::ConstLanelet & lanelet_a, const lanelet::ConstLanelet & lanelet_b)
{
  std::list<lanelet::ConstLanelet> connected_bidirectional_lanes_a = {lanelet_a};
  std::list<lanelet::ConstLanelet> connected_bidirectional_lanes_b = {lanelet_b};
  search_connected_bidirectional_lanelet_in_one_direction(
    get_next_lanelets, get_previous_lanelets, lanelet_a, lanelet_b,
    &connected_bidirectional_lanes_a, &connected_bidirectional_lanes_b);
  search_connected_bidirectional_lanelet_in_one_direction(
    get_next_lanelets, get_previous_lanelets, lanelet_b, lanelet_a,
    &connected_bidirectional_lanes_b, &connected_bidirectional_lanes_a);
  ConnectedBidirectionalLanelets connected_bidirectional_lanelets_a(
    connected_bidirectional_lanes_a.begin(), connected_bidirectional_lanes_a.end());
  ConnectedBidirectionalLanelets connected_bidirectional_lanelets_b(
    connected_bidirectional_lanes_b.begin(), connected_bidirectional_lanes_b.end());
  connected_bidirectional_lanelets_a.set_opposite_bidirectional_lanes(
    connected_bidirectional_lanelets_b);
  connected_bidirectional_lanelets_b.set_opposite_bidirectional_lanes(
    connected_bidirectional_lanelets_a);
  return {connected_bidirectional_lanelets_a, connected_bidirectional_lanelets_b};
}

std::vector<ConnectedBidirectionalLanelets>
ConnectedBidirectionalLanelets::search_bidirectional_lanes_on_map(
  const lanelet::LaneletMap & map,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> & get_next_lanelets,
  const std::function<lanelet::ConstLanelets(const lanelet::ConstLanelet &)> &
    get_previous_lanelets)
{
  std::vector<ConnectedBidirectionalLanelets> bidirectional_lanes;

  auto contains = [&bidirectional_lanes](const lanelet::ConstLanelet & lanelet) -> bool {
    return std::any_of(
      bidirectional_lanes.begin(), bidirectional_lanes.end(),
      [&](const ConnectedBidirectionalLanelets & lanelets) {
        return std::any_of(
          lanelets.bidirectional_lanes_.begin(), lanelets.bidirectional_lanes_.end(),
          [&lanelet](const lanelet::ConstLanelet & bidirectional_lanelet) {
            return bidirectional_lanelet.id() == lanelet.id();
          });
      });
  };

  for (auto lanelet_itr_a = map.laneletLayer.begin(); lanelet_itr_a != map.laneletLayer.end();
       ++lanelet_itr_a) {
    for (auto lanelet_itr_b = std::next(lanelet_itr_a); lanelet_itr_b != map.laneletLayer.end();
         ++lanelet_itr_b) {
      if (
        is_bidirectional_lanes(*lanelet_itr_a, *lanelet_itr_b) && !contains(*lanelet_itr_a) &&
        !contains(*lanelet_itr_b)) {
        auto [connected_bidirectional_lanes_a, connected_bidirectional_lanes_b] =
          search_connected_bidirectional_lanelets(
            get_next_lanelets, get_previous_lanelets, *lanelet_itr_a, *lanelet_itr_b);
        bidirectional_lanes.emplace_back(std::move(connected_bidirectional_lanes_a));
        bidirectional_lanes.emplace_back(std::move(connected_bidirectional_lanes_b));
      }
    }
  }
  return bidirectional_lanes;
}

std::optional<trajectory::Interval> ConnectedBidirectionalLanelets::get_overlap_interval(
  const trajectory::Trajectory<tier4_planning_msgs::msg::PathPointWithLaneId> & trajectory) const
{
  lanelet::Ids lane_ids(bidirectional_lanes_.size());
  std::transform(
    bidirectional_lanes_.begin(), bidirectional_lanes_.end(), lane_ids.begin(),
    [](const lanelet::ConstLanelet & lanelet) { return lanelet.id(); });

  std::unordered_set<lanelet::Id> lane_ids_set(lane_ids.begin(), lane_ids.end());

  auto interval = trajectory::find_intervals(
    trajectory, [&](const tier4_planning_msgs::msg::PathPointWithLaneId & point) -> bool {
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
  double x = pose.orientation.x;
  double y = pose.orientation.y;
  double z = pose.orientation.z;
  double w = pose.orientation.w;
  double yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
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

  for (const auto & lanelet : bidirectional_lanes_) {
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
ConnectedBidirectionalLanelets::ConnectedBidirectionalLanelets(
  const ConnectedBidirectionalLanelets & other)
{
  bidirectional_lanes_ = other.bidirectional_lanes_;
  set_opposite_bidirectional_lanes(*other.opposite_bidirectional_lanes_);
  opposite_bidirectional_lanes_->set_opposite_bidirectional_lanes(*this);
}

ConnectedBidirectionalLanelets & ConnectedBidirectionalLanelets::operator=(
  const ConnectedBidirectionalLanelets & other)
{
  if (this != &other) {
    bidirectional_lanes_ = other.bidirectional_lanes_;
    set_opposite_bidirectional_lanes(*other.opposite_bidirectional_lanes_);
    opposite_bidirectional_lanes_->set_opposite_bidirectional_lanes(*this);
  }
  return *this;
}

[[nodiscard]] ConnectedBidirectionalLanelets
ConnectedBidirectionalLanelets::get_opposite_bidirectional_lanes() const
{
  return *opposite_bidirectional_lanes_;
}

[[nodiscard]] trajectory::Trajectory<geometry_msgs::msg::Point>
ConnectedBidirectionalLanelets::get_center_line() const
{
  std::vector<geometry_msgs::msg::Point> center_line;
  for (const auto & lane : bidirectional_lanes_) {
    for (const auto & point : lane.centerline()) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      center_line.emplace_back(p);
    }
    if (lane != bidirectional_lanes_.back()) {
      center_line.pop_back();
    }
  }
  auto trajectory = trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(center_line);

  if (!trajectory) {
    throw std::runtime_error("Failed to build trajectory in ConnectedBidirectionalLanelets");
  }

  return *trajectory;
}

[[nodiscard]] trajectory::Trajectory<geometry_msgs::msg::Point>
ConnectedBidirectionalLanelets::get_left_line() const
{
  std::vector<geometry_msgs::msg::Point> left_line;
  for (const auto & lane : bidirectional_lanes_) {
    for (const auto & point : lane.leftBound()) {
      geometry_msgs::msg::Point p;
      p.x = point.x();
      p.y = point.y();
      p.z = point.z();
      left_line.push_back(p);
    }
    if (lane != bidirectional_lanes_.back()) {
      left_line.pop_back();
    }
  }

  auto trajectory = trajectory::Trajectory<geometry_msgs::msg::Point>::Builder{}.build(left_line);

  if (!trajectory) {
    throw std::runtime_error("Failed to build trajectory in ConnectedBidirectionalLanelets");
  }

  return *trajectory;
}

std::optional<ConnectedBidirectionalLanelets> get_current_bidirectional_lane(
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware::universe_utils::Polygon2d & ego_polygon,
  const std::vector<ConnectedBidirectionalLanelets> & all_bidirectional_lanes)
{
  for (const auto & bidirectional_lane : all_bidirectional_lanes) {
    if (bidirectional_lane.is_object_on_this_lane(ego_pose, ego_polygon)) {
      return bidirectional_lane;
    }
  }
  return std::nullopt;
}

}  // namespace autoware::behavior_path_planner
