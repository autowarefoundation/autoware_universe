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

#include "debug.hpp"

#include "type_alias.hpp"

#include <std_msgs/msg/detail/color_rgba__struct.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace color
{
using std_msgs::msg::ColorRGBA;

inline ColorRGBA blue(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 0., 1., a);
}

inline ColorRGBA yellow(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 0., a);
}

inline ColorRGBA green(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 1., 0., a);
}

inline ColorRGBA aqua(float a = 0.99)
{
  return autoware_utils::create_marker_color(0., 1., 1., a);
}

inline ColorRGBA magenta(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 0., 1., a);
}

inline ColorRGBA medium_orchid(float a = 0.99)
{
  return autoware_utils::create_marker_color(0.729, 0.333, 0.827, a);
}

inline ColorRGBA light_pink(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 0.713, 0.756, a);
}

inline ColorRGBA light_yellow(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 0.878, a);
}

inline ColorRGBA light_steel_blue(float a = 0.99)
{
  return autoware_utils::create_marker_color(0.690, 0.768, 0.870, a);
}

inline ColorRGBA white(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 1., 1., a);
}

inline ColorRGBA grey(float a = 0.99)
{
  return autoware_utils::create_marker_color(.5, .5, .5, a);
}
}  // namespace color

namespace autoware::motion_velocity_planner::experimental::debug
{
Marker create_ego_sides_marker(
  const EgoSides & ego_sides_from_footprints, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;
  marker.points.reserve(ego_sides_from_footprints.size() * 4);
  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  for (const auto & ego_footprint_sides : ego_sides_from_footprints) {
    const auto & left = ego_footprint_sides.left;
    marker.points.push_back(to_geom(left.first));
    marker.points.push_back(to_geom(left.second));
    const auto & right = ego_footprint_sides.right;
    marker.points.push_back(to_geom(right.first));
    marker.points.push_back(to_geom(right.second));
  }

  return marker;
}

template <typename T>
Marker create_projections_to_bound_marker(
  const T & projections_to_bound, Marker marker, const std::string & type_str,
  const std::string & side_key_str, const double base_link_z)
{
  marker.ns = type_str + "_projection_to_bound_" + side_key_str;
  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  for (const auto & pt : projections_to_bound) {
    marker.color = color::blue();
    marker.points.push_back(to_geom(pt.pt_on_ego));
    marker.points.push_back(to_geom(pt.pt_on_bound));
    marker.points.push_back(to_geom(pt.nearest_bound_seg.first));
    marker.points.push_back(to_geom(pt.nearest_bound_seg.second));
  }
  return marker;
}

MarkerArray create_projections_type_wall_marker(
  const std::vector<ClosestProjectionToBound> & projections_to_bound,
  [[maybe_unused]] const Trajectory & ego_traj, const rclcpp::Time & curr_time,
  const std::string & side_key_str, const double base_link_z)
{
  int32_t id{0};
  auto marker_near_bound = create_default_marker(
    "map", curr_time, "departure_type_line_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::POINTS, create_marker_scale(0.25, 0.25, 1.0), color::green());
  auto marker_approaching = create_default_marker(
    "map", curr_time, "departure_type_line_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::POINTS, create_marker_scale(0.25, 0.25, 1.0), color::yellow());
  auto marker_critical = create_default_marker(
    "map", curr_time, "departure_type_line_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::POINTS, create_marker_scale(0.25, 0.25, 1.0),
    color::magenta());

  auto marker_others = create_default_marker(
    "map", curr_time, "departure_type_line_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::POINTS, create_marker_scale(0.25, 0.25, 1.0), color::white());

  MarkerArray marker_array;

  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };

  for (const auto & pt : projections_to_bound) {
    if (pt.departure_type == DepartureType::NEAR_BOUNDARY) {
      marker_near_bound.points.push_back(to_geom(pt.pt_on_bound));
    } else if (pt.departure_type == DepartureType::APPROACHING_DEPARTURE) {
      marker_approaching.points.push_back(to_geom(pt.pt_on_bound));
    } else if (pt.departure_type == DepartureType::CRITICAL_DEPARTURE) {
      marker_critical.points.push_back(to_geom(pt.pt_on_bound));
    } else {
      marker_others.points.push_back(to_geom(pt.pt_on_ego));
    }
  }
  marker_array.markers.push_back(marker_near_bound);
  marker_array.markers.push_back(marker_approaching);
  marker_array.markers.push_back(marker_critical);
  marker_array.markers.push_back(marker_others);
  return marker_array;
}

Marker create_departure_points_marker(
  const DeparturePoints & departure_points, const rclcpp::Time & curr_time,
  const std::string & side_key_str, const double base_link_z)
{
  int32_t id{0};
  auto marker = create_default_marker(
    "map", curr_time, "departure_points_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::SPHERE_LIST, create_marker_scale(0.25, 0.25, 1.0),
    color::yellow());
  for (const auto & pt : departure_points) {
    marker.points.push_back(pt.to_geom_pt(base_link_z));
  }
  return marker;
}

Marker create_footprint_marker(
  const Footprints & footprints, const rclcpp::Time & curr_time, const std::string & type_str,
  const double base_link_z, const std_msgs::msg::ColorRGBA & color)
{
  int32_t id{0};
  auto marker_ll = create_default_marker(
    "map", curr_time, type_str + "_footprint", id, visualization_msgs::msg::Marker::LINE_LIST,
    create_marker_scale(0.05, 0, 0), color);
  if (!footprints.empty()) {
    marker_ll.points.reserve(footprints.size() * footprints.front().size());
  }

  for (const auto & footprint : footprints) {
    for (size_t i = 0; i + 1 < footprint.size(); ++i) {
      const auto & p1 = footprint.at(i);
      const auto & p2 = footprint.at(i + 1);

      marker_ll.points.push_back(autoware_utils::to_msg(p1.to_3d(base_link_z)));
      marker_ll.points.push_back(autoware_utils::to_msg(p2.to_3d(base_link_z)));
    }
  }

  return marker_ll;
}

Marker create_boundary_segments_marker(
  const BoundarySideWithIdx & boundaries, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;

  const auto to_geom = [base_link_z](const auto & pt) { return to_msg(pt.to_3d(base_link_z)); };
  marker.color = color::medium_orchid();
  for (const auto & [segment, id] : boundaries.left) {
    marker.points.push_back(to_geom(segment.first));
    marker.points.push_back(to_geom(segment.second));
  }
  for (const auto & [segment, id] : boundaries.right) {
    marker.points.push_back(to_geom(segment.first));
    marker.points.push_back(to_geom(segment.second));
  }
  return marker;
}

MarkerArray create_slow_down_interval(
  const std::vector<std::tuple<Pose, Pose, double>> & slow_down_points,
  const rclcpp::Time & curr_time)
{
  int32_t id{0};
  auto marker_1 = create_default_marker(
    "map", curr_time, "start_slow", id, visualization_msgs::msg::Marker::POINTS,
    create_marker_scale(0.25, 0.25, 1.0), color::light_steel_blue());

  auto marker_2 = create_default_marker(
    "map", curr_time, "stop_slow", id, visualization_msgs::msg::Marker::POINTS,
    create_marker_scale(0.25, 0.25, 1.0), color::light_pink());
  for (const auto & [start, stop, vel] : slow_down_points) {
    marker_1.points.push_back(start.position);
    marker_2.points.push_back(stop.position);
  }

  MarkerArray marker_array;
  marker_array.markers = {marker_1, marker_2};
  return marker_array;
}

Marker create_departure_interval_marker(
  const DepartureIntervals & departure_intervals, Marker marker, std::string && ns)
{
  marker.ns = ns;
  marker.color = color::magenta();
  for (const auto & departure_interval : departure_intervals) {
    marker.points.push_back(departure_interval.start.pose.position);
    marker.points.push_back(departure_interval.end.pose.position);
  }
  return marker;
}

MarkerArray create_debug_marker_array(
  const Output & output, const Trajectory & ego_traj, const rclcpp::Clock::SharedPtr & clock_ptr,
  const double base_link_z, const NodeParam & node_param)
{
  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;
  const auto curr_time = clock_ptr->now();
  const auto color = color::green();
  const auto m_scale = create_marker_scale(0.05, 0, 0);

  const auto get_type_str = [&](const AbnormalityType type) {
    auto type_str = std::string(magic_enum::enum_name(type));
    std::transform(type_str.begin(), type_str.end(), type_str.begin(), [](unsigned char c) {
      return std::tolower(c);
    });
    return type_str;
  };

  const auto get_side_key_str = [&](const SideKey side_key) {
    auto side_key_str = std::string(magic_enum::enum_name(side_key));
    std::transform(
      side_key_str.begin(), side_key_str.end(), side_key_str.begin(),
      [](unsigned char c) { return std::tolower(c); });
    return side_key_str;
  };

  MarkerArray marker_array;

  auto marker = create_default_marker("map", curr_time, "", 0, line_list, m_scale, color);

  marker_array.markers.push_back(create_boundary_segments_marker(
    output.abnormalities_data.boundary_segments, marker, "boundary_segments", base_link_z));
  marker_array.markers.push_back(
    create_departure_interval_marker(output.departure_intervals, marker, "departure interval"));
  for (const auto type : node_param.bdc_param.abnormality_types_to_compensate) {
    const auto type_str = get_type_str(type);
    marker_array.markers.push_back(create_footprint_marker(
      output.abnormalities_data.footprints[type], curr_time, type_str, base_link_z, color::aqua()));

    for (const auto side_key : g_side_keys) {
      const auto side_key_str = get_side_key_str(side_key);
      marker_array.markers.push_back(create_projections_to_bound_marker(
        output.abnormalities_data.projections_to_bound[type][side_key], marker, type_str,
        side_key_str, base_link_z));
    }
  }

  for (const auto side_key : g_side_keys) {
    const auto side_key_str = get_side_key_str(side_key);

    marker_array.markers.push_back(create_departure_points_marker(
      output.departure_points[side_key], curr_time, side_key_str, base_link_z));
    marker_array.markers.push_back(create_projections_to_bound_marker(
      output.closest_projections_to_bound[side_key], marker, "closest", side_key_str, base_link_z));
    autoware_utils::append_marker_array(
      create_projections_type_wall_marker(
        output.closest_projections_to_bound[side_key], ego_traj, curr_time, side_key_str,
        base_link_z),
      &marker_array);
  }
  autoware_utils::append_marker_array(
    create_slow_down_interval(output.slowdown_intervals, curr_time), &marker_array);

  return marker_array;
}

}  // namespace autoware::motion_velocity_planner::experimental::debug
