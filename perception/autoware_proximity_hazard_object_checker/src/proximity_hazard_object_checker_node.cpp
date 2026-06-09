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

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker_node.hpp"

#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <boost/geometry/algorithms/append.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/strategies/buffer.hpp>
#include <boost/geometry/strategies/strategies.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::proximity_hazard_object_checker
{
ProximityHazardObjectCheckerNode::ProximityHazardObjectCheckerNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("proximity_hazard_object_checker", options)
{
  param_listener_ =
    std::make_shared<proximity_hazard_object::ParamListener>(get_node_parameters_interface());

  const double node_hz = param_listener_->get_params().node_hz;
  const double node_period_sec = 1.0 / node_hz;
  timer_ = rclcpp::create_timer(
    this, get_clock(), rclcpp::Duration::from_seconds(node_period_sec),
    std::bind(&ProximityHazardObjectCheckerNode::on_timer, this));

  const auto vehicle_info = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  vehicle_footprint_ = vehicle_info.createFootprint();

  impl_ = std::make_unique<ProximityHazardObjectChecker>(
    param_listener_->get_params(), vehicle_footprint_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pub_hazards_ = create_publisher<ProximityHazardObjects>(
    "~/output/proximity_hazards", rclcpp::QoS{1}.reliable());
  pub_debug_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "~/debug/sector_markers", rclcpp::QoS{1});
}

void ProximityHazardObjectCheckerNode::on_timer()
{
  const auto object_ptr = sub_objects_.take_data();

  if (!object_ptr) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to take predicted objects data");
    return;
  }

  const auto odometry_ptr = sub_odometry_.take_data();
  if (!odometry_ptr) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to take predicted odometry data");
    return;
  }

  geometry_msgs::msg::TransformStamped to_base_link;
  try {
    to_base_link = tf_buffer_->lookupTransform(
      odometry_ptr->child_frame_id, object_ptr->header.frame_id, object_ptr->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "TF lookup %s -> %s failed: %s",
      object_ptr->header.frame_id.c_str(), odometry_ptr->child_frame_id.c_str(), ex.what());
    return;
  }

  pub_hazards_->publish(impl_->process(*object_ptr, to_base_link));
  publish_sector_markers(odometry_ptr->child_frame_id);
}

void ProximityHazardObjectCheckerNode::publish_sector_markers(const std::string & frame_id)
{
  using visualization_msgs::msg::Marker;
  using visualization_msgs::msg::MarkerArray;
  namespace bg = boost::geometry;

  const auto params = param_listener_->get_params();
  const double range = params.max_detection_range_m;
  const auto & ego = impl_->ego_center();
  const auto stamp = now();

  // Build the detection-zone outline: Minkowski sum of the vehicle footprint
  // with a disc of radius `range`. This is exactly the set of points within
  // `range` of the footprint -- i.e. the actual detection zone, not a circle
  // around ego_center.
  Polygon2d footprint_polygon;
  for (const auto & p : vehicle_footprint_) {
    bg::append(footprint_polygon, p);
  }
  bg::correct(footprint_polygon);

  constexpr int k_points_per_circle = 36;
  const bg::strategy::buffer::distance_symmetric<double> distance_strategy(range);
  const bg::strategy::buffer::join_round join_strategy(k_points_per_circle);
  const bg::strategy::buffer::end_round end_strategy(k_points_per_circle);
  const bg::strategy::buffer::point_circle point_strategy(k_points_per_circle);
  const bg::strategy::buffer::side_straight side_strategy;

  bg::model::multi_polygon<Polygon2d> buffered_mp;
  bg::buffer(
    footprint_polygon, buffered_mp, distance_strategy, side_strategy, join_strategy, end_strategy,
    point_strategy);

  if (buffered_mp.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Buffered footprint is empty; skipping debug marker publish.");
    return;
  }
  const auto & outline = buffered_mp.front().outer();

  // Distance from ego along unit direction (dx, dy) to the first outline edge
  // hit by the ray. ego_center is inside the buffered region (it's inside the
  // footprint), so the ray exits at exactly one boundary point.
  auto ray_to_outline = [&](double dx, double dy) -> double {
    double t_min = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < outline.size(); ++i) {
      const double ax = outline[i].x();
      const double ay = outline[i].y();
      const double bx = outline[i + 1].x();
      const double by = outline[i + 1].y();
      const double ex = bx - ax;
      const double ey = by - ay;
      const double det = dy * ex - dx * ey;
      if (std::abs(det) < 1e-12) {
        continue;
      }
      const double rx = ax - ego.x();
      const double ry = ay - ego.y();
      const double t = (ry * ex - rx * ey) / det;
      const double s = (dx * ry - dy * rx) / det;
      if (t > 0.0 && s >= 0.0 && s <= 1.0 && t < t_min) {
        t_min = t;
      }
    }
    return t_min;
  };

  // Sectors in the same order as bearing_to_sector (CCW around the vehicle).
  struct SectorVis
  {
    const char * name;
    const std::vector<double> & range_deg;
  };
  const std::array<SectorVis, 8> sectors = {{
    {"FRONT", params.sector_range.front},
    {"FRONT_LEFT", params.sector_range.front_left},
    {"LEFT", params.sector_range.left},
    {"REAR_LEFT", params.sector_range.rear_left},
    {"REAR", params.sector_range.rear},
    {"REAR_RIGHT", params.sector_range.rear_right},
    {"RIGHT", params.sector_range.right},
    {"FRONT_RIGHT", params.sector_range.front_right},
  }};

  std_msgs::msg::ColorRGBA color;
  color.r = 1.0f;
  color.g = 1.0f;
  color.b = 1.0f;
  color.a = 0.7f;

  MarkerArray out;
  out.markers.reserve(2 + sectors.size());

  // Detection-zone outline (replaces the circle).
  {
    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "proximity_hazard_sectors";
    m.id = 0;
    m.type = Marker::LINE_STRIP;
    m.action = Marker::ADD;
    m.frame_locked = true;
    m.scale.x = 0.03;
    m.color = color;
    m.pose.orientation.w = 1.0;
    m.points.reserve(outline.size());
    for (const auto & p : outline) {
      geometry_msgs::msg::Point pt;
      pt.x = p.x();
      pt.y = p.y();
      pt.z = 0.0;
      m.points.push_back(pt);
    }
    out.markers.push_back(m);
  }

  // Sector boundary lines: each line starts at ego_center and extends along the
  // sector's start angle out to the buffered outline, so it actually shows the
  // detection extent in that direction.
  {
    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "proximity_hazard_sectors";
    m.id = 1;
    m.type = Marker::LINE_LIST;
    m.action = Marker::ADD;
    m.frame_locked = true;
    m.scale.x = 0.03;
    m.color = color;
    m.pose.orientation.w = 1.0;
    m.points.reserve(sectors.size() * 2);
    for (const auto & s : sectors) {
      const double a = autoware_utils_math::deg2rad(s.range_deg.front());
      const double dx = std::cos(a);
      const double dy = std::sin(a);
      const double t = ray_to_outline(dx, dy);
      if (!std::isfinite(t)) {
        continue;
      }
      geometry_msgs::msg::Point p0;
      p0.x = ego.x();
      p0.y = ego.y();
      p0.z = 0.0;
      geometry_msgs::msg::Point p1;
      p1.x = ego.x() + t * dx;
      p1.y = ego.y() + t * dy;
      p1.z = 0.0;
      m.points.push_back(p0);
      m.points.push_back(p1);
    }
    out.markers.push_back(m);
  }

  // Sector labels placed at the sector's angular midpoint, slightly past the
  // buffered outline so they don't sit on the line.
  constexpr double k_label_offset_m = 0.5;
  int label_id = 100;
  for (const auto & s : sectors) {
    const double start = s.range_deg.front();
    const double end = s.range_deg.back();
    // Wraparound: if end < start, the sector crosses +/-180; midpoint shifts by 180.
    double mid_deg = (end > start) ? 0.5 * (start + end) : 0.5 * (start + end + 360.0);
    if (mid_deg >= 180.0) {
      mid_deg -= 360.0;
    }
    const double mid_rad = autoware_utils_math::deg2rad(mid_deg);
    const double dx = std::cos(mid_rad);
    const double dy = std::sin(mid_rad);
    const double t_outline = ray_to_outline(dx, dy);
    if (!std::isfinite(t_outline)) {
      continue;
    }
    const double t = t_outline + k_label_offset_m;

    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "proximity_hazard_sectors";
    m.id = label_id++;
    m.type = Marker::TEXT_VIEW_FACING;
    m.action = Marker::ADD;
    m.frame_locked = true;
    m.pose.position.x = ego.x() + t * dx;
    m.pose.position.y = ego.y() + t * dy;
    m.pose.position.z = 0.0;
    m.pose.orientation.w = 1.0;
    m.scale.z = 0.3;  // text height in meters
    m.color = color;
    m.text = s.name;
    out.markers.push_back(m);
  }

  pub_debug_markers_->publish(out);
}

}  // namespace autoware::proximity_hazard_object_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::proximity_hazard_object_checker::ProximityHazardObjectCheckerNode)
