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

#ifndef AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_MARKERS_HPP_
#define AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_MARKERS_HPP_

#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::mppi_optimizer
{

using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

inline ColorRGBA makeColor(const float r, const float g, const float b, const float a)
{
  ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

inline ColorRGBA costGradientColor(const float cost, const float min_cost, const float max_cost)
{
  float t = 0.5F;
  if (max_cost > min_cost) {
    t = (cost - min_cost) / (max_cost - min_cost);
  }
  t = std::clamp(t, 0.0F, 1.0F);
  return makeColor(t, 1.0F - t, 0.0F, 0.35F);
}

inline Marker makeDeleteAllMarker(
  const rclcpp::Time & stamp, const std::string & frame_id, const std::string & ns)
{
  Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = 0;
  marker.action = Marker::DELETEALL;
  return marker;
}

inline Marker makeLineStripMarker(
  const rclcpp::Time & stamp, const std::string & frame_id, const std::string & ns, const int id,
  const ColorRGBA & color, const double line_width,
  const std::vector<std::pair<float, float>> & points, const float z_val = 0.0F)
{
  Marker marker;
  marker.header.stamp = stamp;
  marker.header.frame_id = frame_id;
  marker.ns = ns;
  marker.id = id;
  marker.type = Marker::LINE_STRIP;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = line_width;
  marker.color = color;
  marker.lifetime = builtin_interfaces::msg::Duration{}.set__sec(0).set__nanosec(500000000);
  marker.points.reserve(points.size());
  for (const auto & [x, y] : points) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z_val;
    marker.points.push_back(point);
  }
  return marker;
}

inline MarkerArray createMppiDebugMarkers(
  const FirstOrderDubinsMppiDebug & debug, const std::string & frame_id, const rclcpp::Time & stamp)
{
  MarkerArray marker_array;

  if (!debug.reference_trajectory.points.empty()) {
    std::vector<std::pair<float, float>> reference_points;
    reference_points.reserve(debug.reference_trajectory.points.size());
    for (const auto & point : debug.reference_trajectory.points) {
      reference_points.emplace_back(
        static_cast<float>(point.pose.position.x), static_cast<float>(point.pose.position.y));
    }
    marker_array.markers.push_back(makeLineStripMarker(
      stamp, frame_id, "mppi_reference", 0, makeColor(0.0F, 1.0F, 1.0F, 0.9F), 0.15,
      reference_points, 0.5F));
  }

  if (!debug.optimal_horizon.empty()) {
    marker_array.markers.push_back(makeLineStripMarker(
      stamp, frame_id, "mppi_optimal", 0, makeColor(1.0F, 0.0F, 0.0F, 1.0F), 0.2,
      debug.optimal_horizon, 0.5F));
  }

  marker_array.markers.push_back(makeDeleteAllMarker(stamp, frame_id, "mppi_rollout"));

  if (!debug.rollouts.empty()) {
    float min_cost = debug.rollouts.front().cost;
    float max_cost = debug.rollouts.front().cost;
    for (const auto & rollout : debug.rollouts) {
      min_cost = std::min(min_cost, rollout.cost);
      max_cost = std::max(max_cost, rollout.cost);
    }

    int rollout_id = 1;
    for (const auto & rollout : debug.rollouts) {
      if (rollout.points.size() < 2U) {
        continue;
      }
      marker_array.markers.push_back(makeLineStripMarker(
        stamp, frame_id, "mppi_rollout", rollout_id,
        costGradientColor(rollout.cost, min_cost, max_cost), 0.04, rollout.points));
      ++rollout_id;
    }
  }

  return marker_array;
}

}  // namespace autoware::mppi_optimizer

#endif  // AUTOWARE__MPPI_OPTIMIZER__MPPI_DEBUG_MARKERS_HPP_
