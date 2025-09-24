// Copyright 2025 TIER IV, Inc. All rights reserved.
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

#include "map_data.hpp"

#include <autoware/motion_velocity_planner_common/planner_data.hpp>
#include <autoware/universe_utils/geometry/boost_geometry.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <boost/geometry/algorithms/envelope.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <string>
#include <vector>

namespace autoware::motion_velocity_planner::run_out
{

namespace
{
bool contains_type(const std::vector<std::string> & types, const std::string & type)
{
  return std::find(types.begin(), types.end(), type) != types.end();
}
}  // namespace

lanelet::BoundingBox2d prepare_relevent_bounding_box(
  const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects)
{
  lanelet::BoundingBox2d bounding_box;
  boost::geometry::convert(ego_footprint.segments_rtree.bounds(), bounding_box);
  for (const auto & o : objects) {
    const auto p = o->predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    bounding_box.extend(universe_utils::Point2d(p.x, p.y));
  }
  return bounding_box;
}

void add_ignore_and_cut_lanelets(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::Lanelet> & lanelets,
  const std::vector<uint8_t> & labels, const std::vector<ObjectParameters> & params_per_label)
{
  for (const auto & ll : lanelets) {
    const auto lanelet_subtype = ll.attributeOr(lanelet::AttributeName::Subtype, std::string());
    for (const auto label : labels) {
      auto & data = data_per_label[label];
      const auto & params = params_per_label[label];
      if (contains_type(params.cut_lanelet_subtypes, lanelet_subtype)) {
        for (auto i = 0UL; i < ll.polygon2d().numSegments(); ++i) {
          data.cut_predicted_paths_segments.push_back(convert(ll.polygon2d().segment(i)));
        }
      }
      if (contains_type(params.strict_cut_lanelet_subtypes, lanelet_subtype)) {
        for (auto i = 0UL; i < ll.polygon2d().numSegments(); ++i) {
          data.strict_cut_predicted_paths_segments.push_back(convert(ll.polygon2d().segment(i)));
        }
      }
      if (contains_type(params.ignore_objects_lanelet_subtypes, lanelet_subtype)) {
        universe_utils::LinearRing2d polygon;
        boost::geometry::convert(ll.polygon2d().basicPolygon(), polygon);
        data.ignore_objects_polygons.push_back(polygon);
      }
      if (contains_type(params.ignore_collisions_lanelet_subtypes, lanelet_subtype)) {
        universe_utils::LinearRing2d polygon;
        boost::geometry::convert(ll.polygon2d().basicPolygon(), polygon);
        data.ignore_collisions_polygons.push_back(polygon);
      }
    }
  }
}

void add_ignore_and_cut_polygons(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::Polygon3d> & polygons,
  const std::vector<uint8_t> & labels, const std::vector<ObjectParameters> & params_per_label)
{
  for (const auto & p : polygons) {
    const auto polygon_type = p.attributeOr(lanelet::AttributeName::Type, std::string());
    for (const auto label : labels) {
      const auto & params = params_per_label[label];
      if (contains_type(params.cut_polygon_types, polygon_type)) {
        for (auto i = 0UL; i < p.numSegments(); ++i) {
          data_per_label[label].cut_predicted_paths_segments.push_back(convert(p.segment(i)));
        }
      }
      if (contains_type(params.strict_cut_polygon_types, polygon_type)) {
        for (auto i = 0UL; i < p.numSegments(); ++i) {
          data_per_label[label].strict_cut_predicted_paths_segments.push_back(
            convert(p.segment(i)));
        }
      }
      if (contains_type(params.ignore_objects_polygon_types, polygon_type)) {
        universe_utils::LinearRing2d polygon;
        for (const auto & pt : p) {
          polygon.emplace_back(pt.x(), pt.y());
        }
        data_per_label[label].ignore_objects_polygons.push_back(polygon);
      }
      if (contains_type(params.ignore_collisions_polygon_types, polygon_type)) {
        universe_utils::LinearRing2d polygon;
        for (const auto & pt : p) {
          polygon.emplace_back(pt.x(), pt.y());
        }
        data_per_label[label].ignore_collisions_polygons.push_back(polygon);
      }
    }
  }
}

void add_cut_segments(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::LineString3d> & linestrings,
  const std::vector<uint8_t> & labels, const std::vector<ObjectParameters> & params_per_label)
{
  for (const auto & ls : linestrings) {
    for (const auto label : labels) {
      const auto & params = params_per_label[label];
      const auto attribute = ls.attributeOr(lanelet::AttributeName::Type, std::string());
      const auto & types = params.cut_linestring_types;
      if (std::find(types.begin(), types.end(), attribute) != types.end()) {
        for (auto i = 0UL; i < ls.numSegments(); ++i) {
          data_per_label[label].cut_predicted_paths_segments.push_back(convert(ls.segment(i)));
        }
      }
      const auto & strict_types = params.strict_cut_linestring_types;
      if (std::find(strict_types.begin(), strict_types.end(), attribute) != strict_types.end()) {
        for (auto i = 0UL; i < ls.numSegments(); ++i) {
          data_per_label[label].strict_cut_predicted_paths_segments.push_back(
            convert(ls.segment(i)));
        }
      }
    }
  }
}

FilteringDataPerLabel calculate_filtering_data(
  const lanelet::LaneletMapPtr & map_ptr, const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects, const Parameters & parameters)
{
  const auto bounding_box = prepare_relevent_bounding_box(ego_footprint, objects);
  FilteringDataPerLabel data_per_label;
  const auto & params_per_label = parameters.object_parameters_per_label;
  data_per_label.resize(Parameters::all_labels().size());
  const auto target_labels = parameters.target_labels();
  for (const auto label : target_labels) {
    if (params_per_label[label].cut_if_crossing_ego_from_behind) {
      data_per_label[label].cut_predicted_paths_segments.push_back(
        ego_footprint.get_rear_segment(0));
    }
  }
  const auto lanelets_in_range = map_ptr->laneletLayer.search(bounding_box);
  const auto polygons_in_range = map_ptr->polygonLayer.search(bounding_box);
  add_ignore_and_cut_lanelets(data_per_label, lanelets_in_range, target_labels, params_per_label);
  add_ignore_and_cut_polygons(data_per_label, polygons_in_range, target_labels, params_per_label);
  const auto linestrings_in_range = map_ptr->lineStringLayer.search(bounding_box);
  add_cut_segments(data_per_label, linestrings_in_range, target_labels, params_per_label);
  for (const auto label : target_labels) {
    const auto & params = parameters.object_parameters_per_label[label];
    if (params.ignore_if_on_ego_trajectory) {
      auto & data = data_per_label[label];
      data.ignore_objects_polygons.reserve(
        data.ignore_objects_polygons.size() + ego_footprint.front_polygons.size() +
        ego_footprint.rear_polygons.size());
      data.ignore_objects_polygons.insert(
        data.ignore_objects_polygons.end(), ego_footprint.front_polygons.begin(),
        ego_footprint.front_polygons.end());
      data.ignore_objects_polygons.insert(
        data.ignore_objects_polygons.end(), ego_footprint.rear_polygons.begin(),
        ego_footprint.rear_polygons.end());
    }
  }
  // prepare rtree objects
  for (const auto label : target_labels) {
    auto & data = data_per_label[label];
    std::vector<SegmentNode> nodes;
    std::vector<SegmentNode> strict_nodes;
    nodes.reserve(data.cut_predicted_paths_segments.size());
    strict_nodes.reserve(data.strict_cut_predicted_paths_segments.size());
    for (auto i = 0UL; i < data.cut_predicted_paths_segments.size(); ++i) {
      nodes.emplace_back(data.cut_predicted_paths_segments[i], i);
    }
    for (auto i = 0UL; i < data.strict_cut_predicted_paths_segments.size(); ++i) {
      strict_nodes.emplace_back(data.strict_cut_predicted_paths_segments[i], i);
    }
    data.cut_predicted_paths_rtree = SegmentRtree(nodes);
    data.strict_cut_predicted_paths_rtree = SegmentRtree(strict_nodes);
  }
  for (const auto label : target_labels) {
    auto & data = data_per_label[label];
    data.ignore_objects_rtree = PolygonRtree(data.ignore_objects_polygons);
    data.ignore_collisions_rtree = PolygonRtree(data.ignore_collisions_polygons);
  }
  return data_per_label;
}
}  // namespace autoware::motion_velocity_planner::run_out
