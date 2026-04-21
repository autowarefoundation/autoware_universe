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

#include "autoware/boundary_departure_checker/debug.hpp"

#include "autoware/boundary_departure_checker/footprints_generator.hpp"
#include "autoware/boundary_departure_checker/hysteresis_logic.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <std_msgs/msg/detail/color_rgba__struct.hpp>

#include <string>

namespace color
{
using std_msgs::msg::ColorRGBA;

inline ColorRGBA blue(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(0., 0., 1., a);
}

inline ColorRGBA yellow(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(1., 1., 0., a);
}

inline ColorRGBA red(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(1., 0., 0., a);
}

inline ColorRGBA aqua(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(0., 1., 1., a);
}

inline ColorRGBA magenta(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(1., 0., 1., a);
}

}  // namespace color

namespace autoware::boundary_departure_checker::debug
{
MarkerArray create_departure_footprint_marker(
  const ProjectionsToBound & projections_to_bound, const footprints::Footprints & footprints,
  const rclcpp::Time & curr_time, const double base_link_z)
{
  int32_t id{0};
  const auto add_marker = [&](
                            const auto color, const ProjectionToBound & projection_to_bound,
                            const std::string & type) -> Marker {
    auto marker_ll = autoware_utils_visualization::create_default_marker(
      "map", curr_time, "footprint_" + type, ++id, visualization_msgs::msg::Marker::LINE_LIST,
      autoware_utils_visualization::create_marker_scale(0.05, 0, 0), color);

    const auto & footprint = footprints.at(projection_to_bound.pose_index);
    for (size_t i = 0; i + 1 < footprint.size(); ++i) {
      const auto & p1 = footprint.at(i);
      const auto & p2 = footprint.at(i + 1);

      marker_ll.points.push_back(autoware_utils_geometry::to_msg(p1.to_3d(base_link_z)));
      marker_ll.points.push_back(autoware_utils_geometry::to_msg(p2.to_3d(base_link_z)));
    }
    return marker_ll;
  };

  MarkerArray marker_array;
  marker_array.markers.reserve(footprints.size());

  for (const auto & pt : projections_to_bound) {
    if (pt.is_none_departure()) {
      continue;
    }

    if (footprints.size() <= pt.pose_index) {
      continue;
    }

    if (pt.is_critical()) {
      marker_array.markers.push_back(add_marker(color::red(), pt, "critical"));
    }
  }
  return marker_array;
}

Marker create_projections_points_marker(
  const ProjectionsToBound & projections_to_bound, const std::string & side_key_str,
  const rclcpp::Time & curr_time, const double base_link_z)
{
  auto marker = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "" + side_key_str, 0, visualization_msgs::msg::Marker::SPHERE_LIST,
    autoware_utils_visualization::create_marker_scale(0.25, 0.25, 1.0), color::yellow());
  marker.ns = side_key_str + "_projected_points";

  const auto to_geom = [base_link_z](const auto & p) {
    return autoware_utils_geometry::to_msg(p.to_3d(base_link_z));
  };

  for (const auto & pt : projections_to_bound) {
    marker.points.push_back(to_geom(pt.pt_on_ego));
    marker.points.push_back(to_geom(pt.pt_on_bound));
    marker.points.push_back(to_geom(pt.nearest_bound_seg.first));
    marker.points.push_back(to_geom(pt.nearest_bound_seg.second));
  }
  return marker;
}

Marker create_projections_points_connection_marker(
  const ProjectionsToBound & projections_to_bound, const std::string & side_key_str,
  const rclcpp::Time & curr_time, const double base_link_z)
{
  auto marker = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "" + side_key_str, 0, visualization_msgs::msg::Marker::LINE_LIST,
    autoware_utils_visualization::create_marker_scale(0.25, 0.25, 1.0), color::blue());
  marker.ns = side_key_str + "_projected_points_connection";

  const auto to_geom = [base_link_z](const auto & p) {
    return autoware_utils_geometry::to_msg(p.to_3d(base_link_z));
  };

  for (const auto & pt : projections_to_bound) {
    marker.points.push_back(to_geom(pt.pt_on_ego));
    marker.points.push_back(to_geom(pt.pt_on_bound));
  }

  return marker;
}

Marker create_projected_segment_bound_marker(
  const ProjectionsToBound & projections_to_bound, const std::string & side_key_str,
  const rclcpp::Time & curr_time, const double base_link_z)
{
  auto marker = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "" + side_key_str, 0, visualization_msgs::msg::Marker::LINE_LIST,
    autoware_utils_visualization::create_marker_scale(0.1, 0.0, 0.0), color::aqua());
  marker.ns = side_key_str + "_projected_segment_bound";

  const auto to_geom = [base_link_z](const auto & p) {
    return autoware_utils_geometry::to_msg(p.to_3d(base_link_z));
  };

  for (const auto & pt : projections_to_bound) {
    marker.points.push_back(to_geom(pt.nearest_bound_seg.first));
    marker.points.push_back(to_geom(pt.nearest_bound_seg.second));
  }

  return marker;
}

MarkerArray create_debug_markers(
  const HysteresisState & hysteresis_state, const footprints::Footprints & footprints,
  const EgoDynamicState & ego_state, const bool enable_developer_marker)
{
  builtin_interfaces::msg::Time curr_time = std::invoke([&ego_state]() {
    const auto current_time_s = ego_state.current_time_s;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(current_time_s);
    stamp.nanosec = static_cast<uint32_t>((current_time_s - stamp.sec) * 1e9);
    return stamp;
  });
  const double base_link_z = ego_state.pose_with_cov.pose.position.z;
  MarkerArray marker_array;
  hysteresis_state.critical_departure_history.for_each_side([&](auto & side_value) {
    autoware_utils_visualization::append_marker_array(
      create_departure_footprint_marker(side_value, footprints, curr_time, base_link_z),
      &marker_array);
  });

  if (!enable_developer_marker) {
    return marker_array;
  }

  hysteresis_state.critical_departure_history.for_each([&](auto key_constant, auto & side_value) {
    const std::string side_str = to_string(key_constant.value);
    marker_array.markers.push_back(
      create_projections_points_marker(side_value, side_str, curr_time, base_link_z));
    marker_array.markers.push_back(
      create_projections_points_connection_marker(side_value, side_str, curr_time, base_link_z));
    marker_array.markers.push_back(
      create_projected_segment_bound_marker(side_value, side_str, curr_time, base_link_z));
  });

  return marker_array;
}

}  // namespace autoware::boundary_departure_checker::debug
