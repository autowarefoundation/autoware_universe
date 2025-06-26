// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/motion_velocity_planner_common/velocity_planning_result.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>
#include <rclcpp/time.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/index/predicates.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <cstdint>
#include <deque>
#include <iomanip>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

/// @brief indicates the position of an intersection with the ego footprint
enum IntersectionPosition {
  front_left,            // intersect with the front_left linestring
  front_right,           // intersect with the front_right linestring
  rear_left,             // intersect with the rear_left linestring
  rear_right,            // intersect with the rear_right linestring
  rear,                  // intersect with the rear segment
  front,                 // intersect with the front segment
  inside_front_polygon,  // inside the front polygon (polygon from front left and right linestrings)
  inside_rear_polygon,   // inside the rear polygon (polygon from rear left and right linestrings)
  inside_both_polygons   // inside both the front and rear polygons
};

/// @brief footprint represented by linestrings corresponding to the path of 4 footprint corners
struct CornerFootprint
{
  std::vector<universe_utils::LineString2d> corner_linestrings{4};

  [[nodiscard]] size_t size() const
  {
    return corner_linestrings[IntersectionPosition::front_left].size();
  }
};
/// @brief corner footprint of an object with the timestep separating each footprint point
struct ObjectPredictedPathFootprint
{
  CornerFootprint predicted_path_footprint;
  double time_step{};
};

/// @brief footprint intersection with the corresponding time (for ego and the object), position,
/// and angle
struct FootprintIntersection
{
  double ego_time{};     // [s] time when ego is predicted to reach the intersection point
  double object_time{};  // [s] time when the object is predicted to reach the intersection point
  universe_utils::Point2d intersection;  // intersection point
  IntersectionPosition position;  // intersection position relative to the ego trajectory footprint
  double arc_length{};            // [m] arc length of the intersection along the ego trajectory
  double yaw_diff{};  // [rad] yaw difference between ego and the object at the intersection
  double ego_vel{};   // [m/s] ego velocity at the intersection
  double vel_diff{};  // [m/s] velocity difference between ego and the object at the intersection
};

/// @brief a set of footprint intersections between ego and a specific object
struct FootprintIntersections
{
  std::vector<FootprintIntersection> intersections;
  std::string uuid;

  explicit FootprintIntersections(const std::string & id) { uuid = id; }
};

namespace bgi = boost::geometry::index;
using SegmentNode = std::pair<universe_utils::Segment2d, size_t>;
using SegmentRtree = bgi::rtree<SegmentNode, bgi::rstar<16>>;
using PolygonNode = std::pair<universe_utils::Box2d, size_t>;
class PolygonRtree : bgi::rtree<PolygonNode, bgi::rstar<16>>
{
  static std::vector<PolygonNode> prepare_nodes(
    const std::vector<universe_utils::LinearRing2d> & polygons)
  {
    std::vector<PolygonNode> nodes;
    nodes.reserve(polygons.size());
    for (auto i = 0UL; i < polygons.size(); ++i) {
      nodes.emplace_back(boost::geometry::return_envelope<universe_utils::Box2d>(polygons[i]), i);
    }
    return nodes;
  }

public:
  PolygonRtree() = default;
  explicit PolygonRtree(const std::vector<universe_utils::LinearRing2d> & polygons)
  : bgi::rtree<PolygonNode, bgi::rstar<16>>(PolygonRtree::prepare_nodes(polygons))
  {
  }

  /// @brief check if the given geometry is disjoint from the polygons contained in the rtree
  template <class T>
  bool is_geometry_disjoint_from_rtree_polygons(
    const T & geometry, const std::vector<universe_utils::LinearRing2d> & polygons) const
  {
    std::vector<PolygonNode> query_results;
    query(!bgi::disjoint(geometry), std::back_inserter(query_results));
    for (const auto & query_result : query_results) {
      const auto & polygon = polygons[query_result.second];
      if (!boost::geometry::disjoint(geometry, polygon)) {
        return false;
      }
    }
    return true;
  }
};
using FootprintSegmentNode =
  std::pair<universe_utils::Segment2d, std::pair<IntersectionPosition, size_t>>;
using FootprintSegmentRtree = bgi::rtree<FootprintSegmentNode, bgi::rstar<16>>;

/// @brief the corner footprint of the ego trajectory
struct TrajectoryCornerFootprint
{
  CornerFootprint predicted_path_footprint;
  FootprintSegmentRtree segments_rtree;
  std::vector<universe_utils::LinearRing2d>
    front_polygons;  // polygons built from the front linestrings
  std::vector<universe_utils::LinearRing2d>
    rear_polygons;  // polygons built from the rear linestrings
  PolygonRtree front_polygons_rtree;
  PolygonRtree rear_polygons_rtree;
  double max_longitudinal_offset;  // [m] distance between baselink and the front of the vehicle
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> ego_trajectory;

  /// @brief get the rear footprint segment of the given index
  [[nodiscard]] universe_utils::Segment2d get_rear_segment(const size_t index) const
  {
    return {
      predicted_path_footprint.corner_linestrings[rear_left][index],
      predicted_path_footprint.corner_linestrings[rear_right][index]};
  }
  /// @brief get the front footprint segment of the given index
  [[nodiscard]] universe_utils::Segment2d get_front_segment(const size_t index) const
  {
    return {
      predicted_path_footprint.corner_linestrings[front_left][index],
      predicted_path_footprint.corner_linestrings[front_right][index]};
  }
};

/// @brief the time interval where a vehicle overlaps the path of another vehicle
struct TimeOverlapInterval
{
  double from{};
  double to{};
  FootprintIntersection first_intersection;
  FootprintIntersection last_intersection;
  TimeOverlapInterval(
    const double from_, const double to_, FootprintIntersection first_intersection_,
    FootprintIntersection last_intersection_)
  : from(from_),
    to(to_),
    first_intersection(std::move(first_intersection_)),
    last_intersection(std::move(last_intersection_))
  {
  }

  [[nodiscard]] bool precedes(const TimeOverlapInterval & o, const double tolerance = 0.0) const
  {
    return to + tolerance < o.from;
  }
  [[nodiscard]] bool succeeds(const TimeOverlapInterval & o, const double tolerance = 0.0) const
  {
    return from > o.to + tolerance;
  }
  [[nodiscard]] bool overlaps(const TimeOverlapInterval & o, const double tolerance = 0.0) const
  {
    return !precedes(o, tolerance) && !succeeds(o, tolerance) && !o.precedes(*this, tolerance) &&
           !o.succeeds(*this, tolerance);
  };

  void expand(const TimeOverlapInterval & o)
  {
    if (o.from < from) {
      from = o.from;
      first_intersection = o.first_intersection;
    }
    if (o.to > to) {
      to = o.to;
      last_intersection = o.last_intersection;
    }
  }
};

inline std::ostream & operator<<(std::ostream & os, const TimeOverlapInterval & i)
{
  std::stringstream ss;
  ss << std::setprecision(3) << "[" << i.from << ", " << i.to << "]";
  os << ss.str();
  return os;
}
/// @brief Type of collision between ego and an object
enum CollisionType { pass_first_no_collision, ignored_collision, collision, no_collision };
/// @brief Collision between ego and an object with the corresponding time intervals and type
struct Collision
{
  TimeOverlapInterval ego_time_interval;
  TimeOverlapInterval object_time_interval;
  CollisionType type = no_collision;
  double ego_collision_time{};  // [s] predicted time of the collision for ego (only used when type
                                // is 'collision')
  std::string explanation;

  Collision(TimeOverlapInterval ego, TimeOverlapInterval object)
  : ego_time_interval(std::move(ego)), object_time_interval(std::move(object))
  {
  }
};
/// @brief Decision type
enum DecisionType { stop, slowdown, nothing };
/// @brief Decision with the corresponding collision, type, and stop point
struct Decision
{
  std::optional<Collision> collision = std::nullopt;
  DecisionType type = nothing;
  std::optional<geometry_msgs::msg::Point> stop_point;  // stop point calculated from this decision
  std::optional<SlowdownInterval> slowdown_interval;    // slowdown calculated from this decision
  std::string explanation;

  Decision() = default;
  Decision(Collision collision_, const DecisionType type_)
  : collision(std::move(collision_)), type(type_)
  {
  }
};
/// @brief History of decision and the corresponding decision times
struct DecisionHistory
{
  std::deque<Decision> decisions;
  std::deque<double> times;
  std::optional<geometry_msgs::msg::Point> latest_stop;

  /// @brief remove outdated history, keeping at most one item above the max_history_duration
  void remove_outdated(const rclcpp::Time & now, const double max_history_duration)
  {
    while (times.size() > 1UL && (now.seconds() - times[1]) >= max_history_duration) {
      times.pop_front();
      decisions.pop_front();
    }
  }

  void add_decision(const double now, const Decision & decision)
  {
    times.push_back(now);
    decisions.push_back(decision);
  }
};
/// @brief Tracker for the decisions toward predicted objects
struct ObjectDecisionsTracker
{
  std::unordered_map<std::string, DecisionHistory> history_per_object;
  /// @brief get the decision history of the given object
  std::optional<DecisionHistory> get(const std::string & object) const
  {
    if (history_per_object.count(object) < 1) {
      return std::nullopt;
    }
    return history_per_object.at(object);
  }
};
/// @brief Object represented by its uuid, corner footprints, current footprint, position,
/// collisions with ego, classification label
struct Object
{
  std::shared_ptr<motion_velocity_planner::PlannerData::Object> object;
  std::string uuid;
  std::vector<ObjectPredictedPathFootprint>
    predicted_path_footprints;  // footprint of each predicted path
  universe_utils::Polygon2d current_footprint;
  universe_utils::Point2d position;
  bool is_stopped = false;
  uint8_t label;
  bool has_target_label = false;
  std::vector<Collision> collisions;  // collisions with the ego trajectory
  bool ignore_but_preserve_predicted_paths = false;
};
/// @brief data to filter predicted paths and collisions
struct FilteringData
{
  std::vector<universe_utils::LinearRing2d> ignore_objects_polygons;
  PolygonRtree ignore_objects_rtree;
  std::vector<universe_utils::LinearRing2d> ignore_collisions_polygons;
  PolygonRtree ignore_collisions_rtree;
  std::vector<universe_utils::Segment2d> cut_predicted_paths_segments;
  SegmentRtree cut_predicted_paths_rtree;
  std::vector<universe_utils::Segment2d> strict_cut_predicted_paths_segments;
  SegmentRtree strict_cut_predicted_paths_rtree;
};
using FilteringDataPerLabel = std::vector<FilteringData>;

}  // namespace autoware::motion_velocity_planner::run_out

#endif  // TYPES_HPP_
