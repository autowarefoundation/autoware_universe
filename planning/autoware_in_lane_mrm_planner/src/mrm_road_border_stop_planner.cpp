// Copyright 2026 TIER IV, Inc.
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

#include "mrm_road_border_stop_planner.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/primitives/LineString.h>

#include <algorithm>
#include <cinttypes>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::in_lane_mrm_planner
{
namespace
{
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_utils_geometry::Box2d;
using autoware_utils_geometry::LineString2d;
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;
using autoware_utils_geometry::Segment2d;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

constexpr size_t kBisectionIterations = 6;  // 0.5 m spacing / 2^6 ≈ 0.01 m
constexpr const char * kModuleName = "in_lane_mrm_road_border_stop";

// Equivalent to bg::closest_points, which is not available in Boost 1.74 (ROS 2 Humble).
geometry_msgs::msg::Point closest_point_on_segment(
  const Segment2d & segment, const geometry_msgs::msg::Point & p, const double z)
{
  const double ax = segment.first.x();
  const double ay = segment.first.y();
  const double bx = segment.second.x();
  const double by = segment.second.y();
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > std::numeric_limits<double>::epsilon()) {
    t = std::clamp(((p.x - ax) * dx + (p.y - ay) * dy) / len2, 0.0, 1.0);
  }
  geometry_msgs::msg::Point out;
  out.x = ax + t * dx;
  out.y = ay + t * dy;
  out.z = z;
  return out;
}
}  // namespace

// ---------------------------------------------------------------------------
// BoundarySegmentIndex
// ---------------------------------------------------------------------------

void BoundarySegmentIndex::build(
  const lanelet::LaneletMapPtr & map, const std::vector<std::string> & boundary_types)
{
  clear();
  if (!map) return;

  std::vector<Value> values;
  for (const auto & linestring : map->lineStringLayer) {
    const auto type = linestring.attributeOr(lanelet::AttributeName::Type, std::string());
    if (type.empty()) continue;
    if (std::find(boundary_types.begin(), boundary_types.end(), type) == boundary_types.end()) {
      continue;
    }
    const auto basic = linestring.basicLineString();
    for (size_t i = 0; i + 1 < basic.size(); ++i) {
      Entry entry;
      entry.segment = Segment2d{
        Point2d{basic.at(i).x(), basic.at(i).y()},
        Point2d{basic.at(i + 1).x(), basic.at(i + 1).y()}};
      entry.z_min = std::min(basic.at(i).z(), basic.at(i + 1).z());
      entry.z_max = std::max(basic.at(i).z(), basic.at(i + 1).z());
      entry.linestring_id = linestring.id();
      Box2d box;
      bg::envelope(entry.segment, box);
      values.emplace_back(box, entries_.size());
      entries_.push_back(entry);
    }
  }
  rtree_ = RTree(values.begin(), values.end());
  map_identity_ = map.get();
  boundary_types_ = boundary_types;
}

bool BoundarySegmentIndex::is_built_for(
  const lanelet::LaneletMapPtr & map, const std::vector<std::string> & boundary_types) const
{
  return map && map_identity_ == map.get() && boundary_types_ == boundary_types;
}

void BoundarySegmentIndex::clear()
{
  entries_.clear();
  rtree_ = RTree();
  map_identity_ = nullptr;
  boundary_types_.clear();
}

std::vector<const BoundarySegmentIndex::Entry *> BoundarySegmentIndex::query(
  const Box2d & box) const
{
  std::vector<Value> hits;
  rtree_.query(bgi::intersects(box), std::back_inserter(hits));
  std::vector<const Entry *> result;
  result.reserve(hits.size());
  for (const auto & hit : hits) {
    result.push_back(&entries_.at(hit.second));
  }
  return result;
}

// ---------------------------------------------------------------------------
// MrmRoadBorderStopPlanner
// ---------------------------------------------------------------------------

void MrmRoadBorderStopPlanner::initialize(
  rclcpp::Node * node, const VehicleInfo & vehicle_info, const Params & params)
{
  node_ = node;
  vehicle_info_ = vehicle_info;
  update_params(params);

  if (node_) {
    planning_factor_interface_ =
      std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
        node_, kModuleName);
    debug_marker_pub_ = node_->create_publisher<MarkerArray>("~/road_border_stop/debug/marker", 1);
  }
}

void MrmRoadBorderStopPlanner::update_params(const Params & params)
{
  params_ = params.road_border_stop;
  base_footprint_ =
    vehicle_info_.createFootprint(params_.lateral_margin, params_.longitudinal_margin);
  if (
    lanelet_map_ptr_ &&
    !boundary_index_.is_built_for(lanelet_map_ptr_, params_.boundary_types_to_detect)) {
    boundary_index_.build(lanelet_map_ptr_, params_.boundary_types_to_detect);
  }
}

void MrmRoadBorderStopPlanner::set_lanelet_map(const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  if (!lanelet_map_ptr_) {
    boundary_index_.clear();
    return;
  }
  if (!boundary_index_.is_built_for(lanelet_map_ptr_, params_.boundary_types_to_detect)) {
    boundary_index_.build(lanelet_map_ptr_, params_.boundary_types_to_detect);
    if (node_) {
      RCLCPP_INFO(
        node_->get_logger(), "[In-lane MRM RoadBorderStop] Built boundary index: %zu segments",
        boundary_index_.size());
    }
  }
}

std::optional<RoadBorderContact> MrmRoadBorderStopPlanner::apply(
  TrajectoryPoints & points, const Odometry & odom)
{
  last_contact_.reset();

  if (!params_.enable || points.size() < 2 || boundary_index_.empty()) {
    return std::nullopt;
  }

  const auto & ego_position = odom.pose.pose.position;
  const size_t ego_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(points, ego_position);
  const double ego_arc_length =
    autoware::motion_utils::calcSignedArcLength(points, size_t{0}, ego_position);

  auto contact = find_first_contact(points, odom.pose.pose, ego_segment_idx, ego_arc_length);
  if (contact) {
    set_stop_point(points, *contact, odom);
    last_contact_ = contact;
  }
  publish_debug_markers(odom);
  return contact;
}

std::optional<RoadBorderContact> MrmRoadBorderStopPlanner::find_first_contact(
  const TrajectoryPoints & points, const geometry_msgs::msg::Pose & ego_pose,
  const size_t ego_segment_idx, const double ego_arc_length) const
{
  if (points.size() < 2 || ego_segment_idx >= points.size() || boundary_index_.empty()) {
    return std::nullopt;
  }

  const auto make_contact = [&](
                              const BoundarySegmentIndex::Entry & entry, const double arc_length,
                              const geometry_msgs::msg::Pose & pose) {
    RoadBorderContact contact;
    contact.ego_arc_length = ego_arc_length;
    contact.linestring_id = entry.linestring_id;
    contact.segment = entry.segment;
    contact.contact_arc_length = arc_length;
    contact.contact_pose = pose;
    contact.contact_point =
      closest_point_on_segment(contact.segment, pose.position, pose.position.z);
    return contact;
  };

  // The first footprint is the ego itself (not the trajectory point behind it at the segment
  // start).
  if (const auto hit = find_intersecting_segment(create_footprint(ego_pose), ego_pose.position.z)) {
    return make_contact(**hit, ego_arc_length, ego_pose);
  }

  geometry_msgs::msg::Pose pose_prev = ego_pose;
  double arc_prev = ego_arc_length;
  double arc = autoware::motion_utils::calcSignedArcLength(points, 0, ego_segment_idx);
  for (size_t i = ego_segment_idx + 1; i < points.size(); ++i) {
    arc += autoware_utils_geometry::calc_distance2d(points.at(i - 1), points.at(i));
    if (arc <= ego_arc_length) continue;  // not ahead of the ego (degenerate projection)
    if (arc - ego_arc_length > params_.max_check_length) break;

    const auto & pose = points.at(i).pose;
    const auto hit = find_intersecting_segment(create_footprint(pose), pose.position.z);
    if (!hit) {
      pose_prev = pose;
      arc_prev = arc;
      continue;
    }
    const auto [contact_arc, contact_pose] = refine_contact(pose_prev, pose, arc_prev, arc);
    return make_contact(**hit, contact_arc, contact_pose);
  }
  return std::nullopt;
}

Polygon2d MrmRoadBorderStopPlanner::create_footprint(const geometry_msgs::msg::Pose & pose) const
{
  const auto ring = autoware_utils_geometry::transform_vector(
    base_footprint_, autoware_utils_geometry::pose2transform(pose));
  Polygon2d polygon;
  polygon.outer() = ring;
  bg::correct(polygon);
  return polygon;
}

bool MrmRoadBorderStopPlanner::is_within_height(
  const BoundarySegmentIndex::Entry & entry, const double pose_z) const
{
  if (!params_.use_height_filter) return true;
  const double h = vehicle_info_.vehicle_height_m;
  return pose_z >= entry.z_min - h && pose_z <= entry.z_max + h;
}

std::optional<const BoundarySegmentIndex::Entry *>
MrmRoadBorderStopPlanner::find_intersecting_segment(
  const Polygon2d & footprint, const double pose_z) const
{
  Box2d box;
  bg::envelope(footprint, box);
  for (const auto * entry : boundary_index_.query(box)) {
    if (!is_within_height(*entry, pose_z)) continue;
    LineString2d segment_ls{entry->segment.first, entry->segment.second};
    if (bg::intersects(footprint, segment_ls)) {
      return entry;
    }
  }
  return std::nullopt;
}

std::pair<double, geometry_msgs::msg::Pose> MrmRoadBorderStopPlanner::refine_contact(
  const geometry_msgs::msg::Pose & pose_prev, const geometry_msgs::msg::Pose & pose_contact,
  const double arc_prev, const double arc_contact) const
{
  if (arc_contact <= arc_prev) return {arc_contact, pose_contact};

  double lo = 0.0;  // ratio: no contact
  double hi = 1.0;  // ratio: contact
  geometry_msgs::msg::Pose pose_hi = pose_contact;
  for (size_t k = 0; k < kBisectionIterations; ++k) {
    const double mid = 0.5 * (lo + hi);
    const auto pose = autoware_utils_geometry::calc_interpolated_pose(
      pose_prev, pose_contact, mid, /*set_orientation_from_position_direction=*/false);
    if (find_intersecting_segment(create_footprint(pose), pose.position.z)) {
      hi = mid;
      pose_hi = pose;
    } else {
      lo = mid;
    }
  }
  return {arc_prev + hi * (arc_contact - arc_prev), pose_hi};
}

void MrmRoadBorderStopPlanner::set_stop_point(
  TrajectoryPoints & points, RoadBorderContact & contact, const Odometry & odom)
{
  // Stop `stop_margin` before the footprint touches the border, but never behind the ego.
  contact.stop_arc_length =
    std::max(contact.contact_arc_length - params_.stop_margin, contact.ego_arc_length);

  const auto stop_idx = autoware::motion_utils::insertStopPoint(contact.stop_arc_length, points);
  if (!stop_idx) return;
  contact.stop_pose = points.at(*stop_idx).pose;

  if (planning_factor_interface_) {
    planning_factor_interface_->add(
      points, odom.pose.pose, points.at(*stop_idx).pose, PlanningFactor::STOP, safety_factors_);
  }
  if (node_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 500,
      "[In-lane MRM RoadBorderStop] Inserted stop point at arc length %.2f m (contact %.2f m, "
      "linestring %" PRId64 ")",
      contact.stop_arc_length, contact.contact_arc_length,
      static_cast<int64_t>(contact.linestring_id));
  }
}

void MrmRoadBorderStopPlanner::publish_planning_factor()
{
  if (planning_factor_interface_) {
    planning_factor_interface_->publish();
  }
}

void MrmRoadBorderStopPlanner::publish_latched(
  const TrajectoryPoints & latched_points, const Odometry & odom)
{
  if (
    planning_factor_interface_ && last_contact_ && last_contact_->stop_pose &&
    latched_points.size() >= 2) {
    planning_factor_interface_->add(
      latched_points, odom.pose.pose, *last_contact_->stop_pose, PlanningFactor::STOP,
      safety_factors_);
  }
  publish_planning_factor();
  publish_debug_markers(odom);
}

void MrmRoadBorderStopPlanner::publish_debug_markers(const Odometry & odom) const
{
  if (!node_ || !debug_marker_pub_) return;
  if (debug_marker_pub_->get_subscription_count() == 0 && !last_contact_) return;

  const auto now = node_->get_clock()->now();
  const double ego_z = odom.pose.pose.position.z;
  MarkerArray marker_array;

  const auto add_polygon_marker = [&](
                                    const Polygon2d & polygon, const std::string & ns, const int id,
                                    const std_msgs::msg::ColorRGBA & color, const double width) {
    Marker marker = autoware_utils::create_default_marker(
      "map", now, ns, id, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(width, width, width), color);
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);
    for (const auto & p : polygon.outer()) {
      marker.points.push_back(autoware_utils_geometry::create_point(p.x(), p.y(), ego_z));
    }
    if (!marker.points.empty()) marker.points.push_back(marker.points.front());
    marker_array.markers.push_back(marker);
  };

  if (last_contact_) {
    const auto & c = *last_contact_;
    const auto red = autoware_utils::create_marker_color(1.0, 0.2, 0.2, 0.9);
    add_polygon_marker(create_footprint(c.contact_pose), "contact_footprint", 0, red, 0.15);

    Marker seg = autoware_utils::create_default_marker(
      "map", now, "contact_segment", 0, Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.3, 0.3, 0.3), red);
    seg.lifetime = rclcpp::Duration::from_seconds(0.2);
    seg.points.push_back(
      autoware_utils_geometry::create_point(c.segment.first.x(), c.segment.first.y(), ego_z));
    seg.points.push_back(
      autoware_utils_geometry::create_point(c.segment.second.x(), c.segment.second.y(), ego_z));
    marker_array.markers.push_back(seg);

    Marker pt = autoware_utils::create_default_marker(
      "map", now, "contact_point", 0, Marker::SPHERE,
      autoware_utils::create_marker_scale(0.5, 0.5, 0.5), red);
    pt.lifetime = rclcpp::Duration::from_seconds(0.2);
    pt.pose.position = c.contact_point;
    marker_array.markers.push_back(pt);

    if (c.stop_pose) {
      const auto wall = autoware::motion_utils::createStopVirtualWallMarker(
        *c.stop_pose, kModuleName, now, 0, vehicle_info_.max_longitudinal_offset_m);
      marker_array.markers.insert(
        marker_array.markers.end(), wall.markers.begin(), wall.markers.end());
    }
  } else {
    // Clear previous virtual wall by publishing a DELETE for the same namespace.
    const auto wall = autoware::motion_utils::createDeletedStopVirtualWallMarker(now, 0);
    marker_array.markers.insert(
      marker_array.markers.end(), wall.markers.begin(), wall.markers.end());
  }

  debug_marker_pub_->publish(marker_array);
}

}  // namespace autoware::in_lane_mrm_planner
