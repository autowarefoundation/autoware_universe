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

template <typename T>
Marker create_projections_to_bound_marker(
  const T & projections_to_bound, Marker marker, const std::string & type_str,
  const std::string & side_key_str, const double base_link_z)
{
  marker.ns = type_str + "_projection_to_bound_" + side_key_str;
  marker.color = color::blue();
  for (const auto & pt : projections_to_bound) {
    add_projection_to_marker(marker, pt, base_link_z);
  }
  return marker;
}

Marker create_result_projection_marker(
  const CriticalPointPair & result_pair, Marker marker, const std::string & side_key_str,
  const double base_link_z)
{
  marker.ns = "result_projection_to_bound_" + side_key_str;
  marker.color = color::aqua();
  add_projection_to_marker(marker, result_pair.physical_departure_point, base_link_z);
  add_projection_to_marker(marker, result_pair.safety_buffer_start, base_link_z);
  return marker;
}

MarkerArray create_projections_type_wall_marker(
  const ProjectionsToBound & projections_to_bound, const rclcpp::Time & curr_time,
  const std::string & side_key_str, const double base_link_z)
{
  int32_t id{0};
  auto marker_approaching = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "departure_type_line_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::POINTS,
    autoware_utils_visualization::create_marker_scale(0.25, 0.25, 1.0), color::yellow());
  auto marker_critical = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "departure_type_line_" + side_key_str, ++id,
    visualization_msgs::msg::Marker::POINTS,
    autoware_utils_visualization::create_marker_scale(0.25, 0.25, 1.0), color::magenta());

  MarkerArray marker_array;

  const auto to_geom = [base_link_z](const auto & pt) {
    return autoware_utils_geometry::to_msg(pt.to_3d(base_link_z));
  };

  for (const auto & pt : projections_to_bound) {
    if (pt.is_none_departure()) {
      continue;
    }
    if (pt.is_approaching()) {
      marker_approaching.points.push_back(to_geom(pt.pt_on_bound));
    } else if (pt.is_critical()) {
      marker_critical.points.push_back(to_geom(pt.pt_on_bound));
    }
  }
  marker_array.markers.push_back(marker_approaching);
  marker_array.markers.push_back(marker_critical);
  return marker_array;
}

Marker create_footprint_marker(
  const footprints::Footprints & footprints, const rclcpp::Time & curr_time,
  const double base_link_z, const std_msgs::msg::ColorRGBA & color)
{
  int32_t id{0};
  auto marker_ll = autoware_utils_visualization::create_default_marker(
    "map", curr_time, "current_footprints", id, visualization_msgs::msg::Marker::LINE_LIST,
    autoware_utils_visualization::create_marker_scale(0.05, 0, 0), color);
  if (!footprints.empty()) {
    marker_ll.points.reserve(footprints.size() * footprints.front().size());
  }

  for (const auto & footprint : footprints) {
    for (size_t i = 0; i + 1 < footprint.size(); ++i) {
      const auto & p1 = footprint.at(i);
      const auto & p2 = footprint.at(i + 1);

      marker_ll.points.push_back(autoware_utils_geometry::to_msg(p1.to_3d(base_link_z)));
      marker_ll.points.push_back(autoware_utils_geometry::to_msg(p2.to_3d(base_link_z)));
    }
  }

  return marker_ll;
}

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

    if (projection_to_bound.is_none_departure()) {
      return marker_ll;
    }

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
    if (pt.is_approaching()) {
      marker_array.markers.push_back(add_marker(color::yellow(), pt, "approaching"));
    } else if (pt.is_critical()) {
      marker_array.markers.push_back(add_marker(color::magenta(), pt, "critical"));
    }
  }
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

  departure_data.projections_to_bound.for_each([&](auto key_constant, auto & side_value) {
    const std::string side_str = to_string(key_constant.value);
    marker_array.markers.push_back(
      create_projections_to_bound_marker(side_value, marker, "closest", side_str, base_link_z));
    autoware_utils_visualization::append_marker_array(
      create_projections_type_wall_marker(side_value, curr_time, side_str, base_link_z),
      &marker_array);
    autoware_utils_visualization::append_marker_array(
      create_departure_footprint_marker(
        side_value, departure_data.footprints, curr_time, base_link_z),
      &marker_array);
  });

  departure_data.evaluated_projections.for_each([&](auto key_constant, auto & side_value) {
    if (side_value.has_value()) {
      marker_array.markers.push_back(create_result_projection_marker(
        side_value.value(), marker, to_string(key_constant.value), base_link_z));
    }
  });

  return marker_array;
}

}  // namespace autoware::boundary_departure_checker::debug
