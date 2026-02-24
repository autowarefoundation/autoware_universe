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

#include <autoware_utils_visualization/marker_helper.hpp>

#include <std_msgs/msg/detail/color_rgba__struct.hpp>

#include <string>
#include <vector>

namespace color
{
using std_msgs::msg::ColorRGBA;
inline ColorRGBA magenta(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 0., 1., a);
}
inline ColorRGBA light_pink(float a = 0.99)
{
  return autoware_utils::create_marker_color(1., 0.713, 0.756, a);
}
inline ColorRGBA light_steel_blue(float a = 0.99)
{
  return autoware_utils::create_marker_color(0.690, 0.768, 0.870, a);
}
}  // namespace color

namespace autoware::motion_velocity_planner::experimental::debug
{
MarkerArray create_slow_down_interval(
  const std::vector<std::tuple<Pose, Pose, double>> & slow_down_points,
  const rclcpp::Time & curr_time)
{
  int32_t id{0};
  auto marker_1 = autoware_utils::create_default_marker(
    "map", curr_time, "start_slow", id, visualization_msgs::msg::Marker::POINTS,
    autoware_utils::create_marker_scale(0.25, 0.25, 1.0), color::light_steel_blue());

  auto marker_2 = autoware_utils::create_default_marker(
    "map", curr_time, "stop_slow", id, visualization_msgs::msg::Marker::POINTS,
    autoware_utils::create_marker_scale(0.25, 0.25, 1.0), color::light_pink());
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
  const Output & output, const Trajectory & ego_traj, const rclcpp::Time & curr_time,
  const double base_link_z, const NodeParam & node_param)
{
  auto marker_array = autoware::boundary_departure_checker::debug::create_debug_marker_array(
    output.abnormalities_data, ego_traj, curr_time, base_link_z, node_param.bdc_param);

  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;
  const auto color = color::magenta();
  const auto m_scale = autoware_utils::create_marker_scale(0.05, 0, 0);
  auto marker =
    autoware_utils::create_default_marker("map", curr_time, "", 0, line_list, m_scale, color);
  marker_array.markers.push_back(
    create_departure_interval_marker(output.departure_intervals, marker, "departure interval"));
  autoware_utils::append_marker_array(
    create_slow_down_interval(output.slowdown_intervals, curr_time), &marker_array);

  return marker_array;
}

}  // namespace autoware::motion_velocity_planner::experimental::debug
