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

#include "autoware/simpl/debug/marker.hpp"

#include <rclcpp/duration.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simpl::debug
{
visualization_msgs::msg::MarkerArray create_history_marker_array(
  const std::unordered_map<std::string, archetype::AgentHistory> & history_map,
  const std_msgs::msg::Header & header)
{
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  MarkerArray marker_array;
  for (const auto & [agent_id, history] : history_map) {
    Marker position_marker, velocity_marker;

    // ===== Position Marker =====
    position_marker.header = header;
    position_marker.ns = "history/position/" + agent_id;
    position_marker.id = 0;
    position_marker.type = Marker::SPHERE_LIST;
    position_marker.action = Marker::ADD;
    position_marker.pose.orientation.w = 1.0;
    position_marker.scale.x = 0.3;
    position_marker.scale.y = 0.3;
    position_marker.scale.z = 0.3;
    position_marker.color.r = 0.0f;
    position_marker.color.g = 0.5f;
    position_marker.color.b = 1.0f;
    position_marker.color.a = 1.0f;
    position_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

    // ===== Velocity Marker =====
    velocity_marker.header = header;
    velocity_marker.ns = "history/velocity/" + agent_id;
    velocity_marker.id = 0;
    velocity_marker.type = Marker::LINE_LIST;
    velocity_marker.pose.orientation.w = 1.0;
    velocity_marker.scale.x = 0.3;
    velocity_marker.scale.y = 0.3;
    velocity_marker.scale.z = 0.3;
    velocity_marker.color.r = 0.0f;
    velocity_marker.color.g = 0.5f;
    velocity_marker.color.b = 1.0f;
    velocity_marker.color.a = 1.0f;
    velocity_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
    for (const auto & state : history) {
      geometry_msgs::msg::Point pt;
      pt.x = state.x;
      pt.y = state.y;
      pt.z = state.z;
      position_marker.points.emplace_back(pt);

      geometry_msgs::msg::Point dir = pt;

      dir.x += state.vx * 1.0;
      dir.y += state.vy * 1.0;
      velocity_marker.points.emplace_back(pt);   // start
      velocity_marker.points.emplace_back(dir);  // end
    }
    marker_array.markers.emplace_back(position_marker);
    marker_array.markers.emplace_back(velocity_marker);
  }

  return marker_array;
}

visualization_msgs::msg::MarkerArray create_polyline_marker_array(
  const std::vector<archetype::Polyline> & polylines, const std_msgs::msg::Header & header)
{
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  Marker points_marker, dir_marker;
  MarkerArray marker_array;

  // ===== Points Marker =====
  points_marker.header = header;
  points_marker.ns = "map_points";
  points_marker.id = 0;
  points_marker.type = Marker::SPHERE_LIST;
  points_marker.action = Marker::ADD;
  points_marker.pose.orientation.w = 1.0;
  points_marker.scale.x = 0.3;
  points_marker.scale.y = 0.3;
  points_marker.scale.z = 0.3;
  points_marker.color.r = 0.0f;
  points_marker.color.g = 0.5f;
  points_marker.color.b = 1.0f;
  points_marker.color.a = 1.0f;
  points_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  // ======== Direction Marker (arrows) ========
  dir_marker = points_marker;
  dir_marker.id = 1;
  dir_marker.type = Marker::LINE_LIST;
  dir_marker.ns = "map_directions";
  dir_marker.scale.x = 0.1;
  dir_marker.color.r = 1.0f;
  dir_marker.color.g = 0.5f;
  dir_marker.color.b = 0.0f;
  dir_marker.lifetime = rclcpp::Duration::from_seconds(0.1);

  for (size_t i = 0; i < polylines.size(); ++i) {
    const auto & polyline = polylines.at(i);
    for (const auto & point : polyline) {
      geometry_msgs::msg::Point pt;
      pt.x = point.x;
      pt.y = point.y;
      pt.z = point.z;
      points_marker.points.emplace_back(pt);

      if (i < polyline.size() - 1) {
        const auto & next = polyline.at(i + 1);
        geometry_msgs::msg::Point dir = pt;
        auto dx = next.x - point.x;
        auto dy = next.y - point.y;
        auto dz = next.z - point.z;
        const auto norm = std::hypot(dx, dy, dz);
        if (norm > 1e-6) {
          dx /= norm;
          dy /= norm;
          dz /= norm;
        } else {
          dx = 0.0;
          dy = 0.0;
          dz = 0.0;
        }

        dir.x += dx;
        dir.y += dy;
        dir.z += dz;
        dir_marker.points.emplace_back(pt);   // start
        dir_marker.points.emplace_back(dir);  // end
      }
    }
  }

  marker_array.markers.emplace_back(points_marker);
  marker_array.markers.emplace_back(dir_marker);

  return marker_array;
}

visualization_msgs::msg::MarkerArray create_processed_map_marker_array(
  const std::vector<archetype::Polyline> & polylines, const processing::NodePoints & node_centers,
  const processing::NodePoints & node_vectors, const std_msgs::msg::Header & header)
{
  using Marker = visualization_msgs::msg::Marker;
  using MarkerArray = visualization_msgs::msg::MarkerArray;

  MarkerArray marker_array;
  int id = 0;

  for (const auto & polyline : polylines) {
    // LINE_STRIP for polyline
    Marker line;
    line.header = header;
    line.header.frame_id = "base_link";
    line.ns = "processed_map_polylines";
    line.id = id++;
    line.type = Marker::LINE_STRIP;
    line.action = Marker::ADD;
    line.scale.x = 0.1;
    line.color.r = 0.0f;
    line.color.g = 1.0f;
    line.color.b = 0.0f;
    line.color.a = 1.0f;
    line.pose.orientation.w = 1.0;
    line.lifetime = rclcpp::Duration::from_seconds(0.1);

    for (const auto & pt : polyline) {
      geometry_msgs::msg::Point p;
      p.x = pt.x;
      p.y = pt.y;
      line.points.emplace_back(p);
    }
    marker_array.markers.emplace_back(line);
  }

  for (size_t i = 0; i < node_centers.size(); ++i) {
    Marker arrow;
    arrow.header = header;
    arrow.header.frame_id = "base_link";
    arrow.ns = "processed_map_node_centers";
    arrow.id = id++;
    arrow.type = Marker::ARROW;
    arrow.action = Marker::ADD;
    arrow.scale.x = 0.1;
    arrow.scale.y = 0.2;
    arrow.scale.z = 0.2;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.0f;
    arrow.color.b = 0.0f;
    arrow.color.a = 1.0f;
    arrow.lifetime = rclcpp::Duration::from_seconds(0.1);

    geometry_msgs::msg::Point start, end;
    const auto & center = node_centers.at(i);
    const auto & vector = node_vectors.at(i);
    start.x = center.x;
    start.y = center.y;

    end.x = center.x + vector.x;
    end.y = center.y + vector.y;

    arrow.points = {start, end};

    marker_array.markers.emplace_back(arrow);
  }

  return marker_array;
}
}  // namespace autoware::simpl::debug
