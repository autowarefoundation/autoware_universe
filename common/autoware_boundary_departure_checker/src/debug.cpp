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
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <std_msgs/msg/detail/color_rgba__struct.hpp>

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

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

inline ColorRGBA green(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(0., 1., 0., a);
}

inline ColorRGBA aqua(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(0., 1., 1., a);
}

inline ColorRGBA magenta(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(1., 0., 1., a);
}

inline ColorRGBA medium_orchid(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(0.729, 0.333, 0.827, a);
}

inline ColorRGBA white(float a = 0.99)
{
  return autoware_utils_visualization::create_marker_color(1., 1., 1., a);
}
}  // namespace color

namespace autoware::boundary_departure_checker::debug
{
namespace
{
void add_projection_to_marker(
  Marker & marker, const ProjectionToBound & pt, const double base_link_z)
{
  const auto to_geom = [base_link_z](const auto & p) {
    return autoware_utils_geometry::to_msg(p.to_3d(base_link_z));
  };
  marker.points.push_back(to_geom(pt.pt_on_ego));
  marker.points.push_back(to_geom(pt.pt_on_bound));
  marker.points.push_back(to_geom(pt.nearest_bound_seg.first));
  marker.points.push_back(to_geom(pt.nearest_bound_seg.second));
}

void append_footprint_lines(
  Marker & marker, const footprints::Footprint & footprint, const double base_link_z)
{
  for (size_t i = 0; i + 1 < footprint.size(); ++i) {
    const auto & p1 = footprint.at(i);
    const auto & p2 = footprint.at(i + 1);

    marker.points.push_back(autoware_utils_geometry::to_msg(p1.to_3d(base_link_z)));
    marker.points.push_back(autoware_utils_geometry::to_msg(p2.to_3d(base_link_z)));
  }
}
}  // namespace

MarkerArray create_history_footprints_marker(
  const ProjectionsToBound & projections_to_bound, const footprints::Footprints & footprints,
  const rclcpp::Time & curr_time, const double base_link_z)
{
  int32_t id{0};
  const auto add_marker = [&](
                            const auto color, const ProjectionToBound & projection_to_bound,
                            const std::string & type) -> Marker {
    auto marker_ll = autoware_utils_visualization::create_default_marker(
      "map", curr_time, "history_footprint_" + type, ++id,
      visualization_msgs::msg::Marker::LINE_LIST,
      autoware_utils_visualization::create_marker_scale(0.05, 0, 0), color);

    if (projection_to_bound.is_none_departure()) {
      return marker_ll;
    }

    const auto & footprint = footprints.at(projection_to_bound.pose_index);
    append_footprint_lines(marker_ll, footprint, base_link_z);
    return marker_ll;
  };

  MarkerArray marker_array;
  marker_array.markers.reserve(footprints.size());

  for (const auto & pt : projections_to_bound) {
    if (pt.is_none_departure()) {
      continue;
    }
    if (pt.is_critical()) {
      marker_array.markers.push_back(add_marker(color::yellow(), pt, "critical"));
    }
  }
  return marker_array;
}

MarkerArray create_evaluated_pair_markers(
  const CriticalPointPair & result_pair, const rclcpp::Time & curr_time,
  const footprints::Footprints & footprints, const double base_link_z,
  const std::string & side_key_str)
{
  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;
  const auto add_marker = [&](
                            const auto color, const ProjectionToBound & projection_to_bound,
                            const std::string & type) -> Marker {
    auto marker = autoware_utils_visualization::create_default_marker(
      "map", curr_time, "evaluated_" + side_key_str + "_" + type, 0, line_list,
      autoware_utils_visualization::create_marker_scale(0.05, 0, 0), color);

    const auto & footprint = footprints.at(projection_to_bound.pose_index);
    append_footprint_lines(marker, footprint, base_link_z);
    return marker;
  };

  MarkerArray marker_array;
  marker_array.markers.reserve(4);
  marker_array.markers.push_back(
    add_marker(color::magenta(), result_pair.physical_departure_point, "physical_departure_point"));
  marker_array.markers.push_back(
    add_marker(color::green(), result_pair.safety_buffer_start, "safety_buffer_start"));

  const auto color = color::green();
  const auto m_scale = autoware_utils_visualization::create_marker_scale(0.05, 0, 0);
  int32_t id{0};
  auto marker = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "", 0, line_list, m_scale, color);
  marker.ns = "evaluated_" + side_key_str + "_pair";
  marker.id = ++id;
  marker.color = color::aqua();
  add_projection_to_marker(marker, result_pair.physical_departure_point, base_link_z);
  add_projection_to_marker(marker, result_pair.safety_buffer_start, base_link_z);
  marker_array.markers.push_back(marker);
  return marker_array;
}

Marker create_boundary_segments_marker(
  const BoundarySegmentsBySide & boundaries, Marker marker, std::string && ns,
  const double base_link_z)
{
  marker.ns = ns;

  const auto to_geom = [base_link_z](const auto & pt) {
    return autoware_utils_geometry::to_msg(pt.to_3d(base_link_z));
  };
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

MarkerArray create_debug_markers(
  const DepartureData & departure_data, const rclcpp::Time & curr_time, const double base_link_z)
{
  const auto line_list = visualization_msgs::msg::Marker::LINE_LIST;
  const auto color = color::green();
  const auto m_scale = autoware_utils_visualization::create_marker_scale(0.05, 0, 0);

  MarkerArray marker_array;

  auto marker = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "", 0, line_list, m_scale, color);

  marker_array.markers.push_back(create_boundary_segments_marker(
    departure_data.boundary_segments, marker, "boundary_segments", base_link_z));

  departure_data.critical_departure_history.for_each_side([&](auto & side_value) {
    autoware_utils_visualization::append_marker_array(
      create_history_footprints_marker(
        side_value, departure_data.footprints, curr_time, base_link_z),
      &marker_array);
  });

  departure_data.evaluated_projections.for_each([&](auto key_constant, auto & side_value_opt) {
    if (side_value_opt.has_value()) {
      autoware_utils_visualization::append_marker_array(
        create_evaluated_pair_markers(
          *side_value_opt, curr_time, departure_data.footprints, base_link_z,
          to_string(key_constant)),
        &marker_array);
    }
  });

  return marker_array;
}

}  // namespace autoware::boundary_departure_checker::debug
