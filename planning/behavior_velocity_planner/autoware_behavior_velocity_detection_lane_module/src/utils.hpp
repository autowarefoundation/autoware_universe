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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "structs.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
visualization_msgs::msg::MarkerArray create_polygon_marker_array(
  const lanelet::BasicPolygons3d & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray msg;

  size_t i = 0;
  for (const auto & polygon : polygons) {
    auto marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++,
      visualization_msgs::msg::Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.2, 0.0, 0.0), color);

    for (const auto & p : polygon) {
      marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), p.z()));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray create_polygon_marker_array(
  const std::vector<autoware_utils::Polygon3d> & polygons, const std::string & ns,
  const std_msgs::msg::ColorRGBA & color)
{
  visualization_msgs::msg::MarkerArray msg;

  size_t i = 0;
  for (const auto & polygon : polygons) {
    auto marker = autoware_utils::create_default_marker(
      "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns, i++,
      visualization_msgs::msg::Marker::LINE_STRIP,
      autoware_utils::create_marker_scale(0.05, 0.0, 0.0), color);

    for (const auto & p : polygon.outer()) {
      marker.points.push_back(autoware_utils::create_point(p.x(), p.y(), p.z()));
    }
    if (!marker.points.empty()) {
      marker.points.push_back(marker.points.front());
    }
    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::msg::MarkerArray create_pointcloud_object_marker_array(
  const PointCloudObjects & objects, const std::string & ns)
{
  visualization_msgs::msg::MarkerArray msg;

  size_t i = 0L;
  for (const auto & object : objects) {
    const auto sphere_color = [&object]() {
      if (object.tracking_duration < 0.3) {
        return autoware_utils::create_marker_color(1.0, 0.67, 0.0, 0.999);
      }
      if (object.ignore) {
        return autoware_utils::create_marker_color(0.5, 0.5, 0.5, 0.999);
      }
      return object.safe ? autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.999)
                         : autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999);
    }();

    {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_sphere", i++,
        visualization_msgs::msg::Marker::SPHERE, autoware_utils::create_marker_scale(1.0, 1.0, 1.0),
        sphere_color);
      marker.pose = object.pose;
      msg.markers.push_back(marker);
    }

    {
      const auto x_scale = object.velocity * 0.36;
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_arrow", i++,
        visualization_msgs::msg::Marker::ARROW,
        autoware_utils::create_marker_scale(x_scale, 0.3, 0.3), sphere_color);
      marker.pose = object.pose;
      msg.markers.push_back(marker);
    }

    {
      auto marker = autoware_utils::create_default_marker(
        "map", rclcpp::Clock{RCL_ROS_TIME}.now(), ns + "_text", i++,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        autoware_utils::create_marker_scale(0.5, 0.5, 0.5),
        autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999));

      std::ostringstream ss;
      ss << std::fixed << std::setprecision(2);
      ss << "TrackingDuration:" << object.tracking_duration
         << "[s]\nDistance(w/DC):" << object.distance
         << "[m]\nDistance(w/oDC):" << object.distance_with_delay_compensation
         << "[m]\nVelocity:" << object.velocity << "[m/s]\nTTC" << object.ttc
         << "[s]\nFurthestLaneID:" << object.furthest_lane.id();

      marker.text = ss.str();
      marker.pose = object.pose;
      marker.pose.position.z += 1.0;
      msg.markers.push_back(marker);
    }
  }

  return msg;
}

std::pair<lanelet::BasicPoint2d, double> get_smallest_enclosing_circle(
  const lanelet::BasicPolygon2d & poly)
{
  // The `eps` is used to avoid precision bugs in circle inclusion checks.
  // If the value of `eps` is too small, this function doesn't work well. More than 1e-10 is
  // recommended.
  const double eps = 1e-5;
  lanelet::BasicPoint2d center(0.0, 0.0);
  double radius_squared = 0.0;

  auto cross = [](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> double {
    return p1.x() * p2.y() - p1.y() * p2.x();
  };

  auto make_circle_3 = [&](
                         const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2,
                         const lanelet::BasicPoint2d & p3) -> void {
    // reference for circumcenter vector https://en.wikipedia.org/wiki/Circumscribed_circle
    const double a = (p2 - p3).squaredNorm();
    const double b = (p3 - p1).squaredNorm();
    const double c = (p1 - p2).squaredNorm();
    const double s = cross(p2 - p1, p3 - p1);
    if (std::abs(s) < eps) return;
    center = (a * (b + c - a) * p1 + b * (c + a - b) * p2 + c * (a + b - c) * p3) / (4 * s * s);
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto make_circle_2 =
    [&](const lanelet::BasicPoint2d & p1, const lanelet::BasicPoint2d & p2) -> void {
    center = (p1 + p2) * 0.5;
    radius_squared = (center - p1).squaredNorm() + eps;
  };

  auto in_circle = [&](const lanelet::BasicPoint2d & p) -> bool {
    return (center - p).squaredNorm() <= radius_squared;
  };

  // mini disc
  for (size_t i = 1; i < poly.size(); i++) {
    const auto p1 = poly[i];
    if (in_circle(p1)) continue;

    // mini disc with point
    const auto p0 = poly[0];
    make_circle_2(p0, p1);
    for (size_t j = 0; j < i; j++) {
      const auto p2 = poly[j];
      if (in_circle(p2)) continue;

      // mini disc with two points
      make_circle_2(p1, p2);
      for (size_t k = 0; k < j; k++) {
        const auto p3 = poly[k];
        if (in_circle(p3)) continue;

        // mini disc with tree points
        make_circle_3(p1, p2, p3);
      }
    }
  }

  return std::make_pair(center, radius_squared);
}

PointCloud::Ptr filter_by_range(
  const PointCloud::Ptr & points, const geometry_msgs::msg::Point & center, const double range,
  const bool outer = false)
{
  PointCloud ret;
  const auto threshold = range * range;
  for (const auto & p : *points) {
    const double squared_dist =
      (center.x - p.x) * (center.x - p.x) + (center.y - p.y) * (center.y - p.y);
    if (outer) {
      if (squared_dist > threshold) {
        ret.push_back(p);
      }
    } else {
      if (squared_dist < threshold) {
        ret.push_back(p);
      }
    }
  }
  return std::make_shared<PointCloud>(ret);
}

PointCloud::Ptr get_obstacle_points(
  const lanelet::BasicPolygons3d & polygons, const PointCloud & points)
{
  PointCloud ret;
  for (const auto & polygon : polygons) {
    const auto circle = get_smallest_enclosing_circle(lanelet::utils::to2D(polygon));
    for (const auto & p : points) {
      const double squared_dist = (circle.first.x() - p.x) * (circle.first.x() - p.x) +
                                  (circle.first.y() - p.y) * (circle.first.y() - p.y);
      if (squared_dist > circle.second) {
        continue;
      }
      if (boost::geometry::within(
            autoware_utils::Point2d{p.x, p.y}, lanelet::utils::to2D(polygon))) {
        ret.push_back(p);
      }
    }
  }
  return std::make_shared<PointCloud>(ret);
}

bool is_green_arrow(
  const autoware_perception_msgs::msg::TrafficLightGroup & tl_state, const uint8_t & lamp_shape)
{
  const auto it_lamp =
    std::find_if(tl_state.elements.begin(), tl_state.elements.end(), [&lamp_shape](const auto & x) {
      return x.shape == lamp_shape &&
             x.color == autoware_perception_msgs::msg::TrafficLightElement::GREEN;
    });

  return it_lamp != tl_state.elements.end();
}
}  // namespace autoware::behavior_velocity_planner

#endif  // UTILS_HPP_
