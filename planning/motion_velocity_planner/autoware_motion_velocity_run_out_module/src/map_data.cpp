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

#include <autoware/motion_velocity_planner_common_universe/planner_data.hpp>
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

lanelet::BoundingBox2d prepare_relevent_bounding_box(
  const TrajectoryCornerFootprint & ego_footprint,
  const std::vector<std::shared_ptr<PlannerData::Object>> & objects)
{
  lanelet::BoundingBox2d bounding_box(ego_footprint.get_rear_segment().first);
  for (const auto & p : ego_footprint.front_polygon.outer()) {
    bounding_box.extend(p);
  }
  bounding_box.extend(universe_utils::Point2d(ego_footprint.get_rear_segment().second));
  for (const auto & o : objects) {
    const auto p = o->predicted_object.kinematics.initial_pose_with_covariance.pose.position;
    bounding_box.extend(universe_utils::Point2d(p.x, p.y));
  }
  return bounding_box;
}

void add_ignore_polygons(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::Lanelet> & lanelets,
  const std::vector<uint8_t> & all_labels, const std::vector<ObjectParameters> & params_per_label)
{
  for (const auto & ll : lanelets) {
    const auto attribute = ll.attributeOr(lanelet::AttributeName::Subtype, std::string());
    for (const auto label : all_labels) {
      auto & data = data_per_label[label];
      const auto & params = params_per_label[label];
      const auto & types = params.cut_polygon_types;
      if (std::find(types.begin(), types.end(), attribute) != types.end()) {
        for (auto i = 0UL; i < ll.polygon2d().numSegments(); ++i) {
          data.cut_predicted_paths_segments.push_back(convert(ll.polygon2d().segment(i)));
        }
      }
      if (params.ignore_if_on_crosswalk && attribute == lanelet::AttributeValueString::Crosswalk) {
        universe_utils::LinearRing2d polygon;
        boost::geometry::convert(ll.polygon2d().basicPolygon(), polygon);
        data.ignore_polygons.push_back(polygon);
      }
    }
  }
}

void add_cut_segments(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::Polygon3d> & polygons,
  const std::vector<uint8_t> & all_labels, const std::vector<ObjectParameters> & params_per_label)
{
  for (const auto & p : polygons) {
    const auto attribute = p.attributeOr(lanelet::AttributeName::Subtype, std::string());
    for (const auto label : all_labels) {
      auto & data = data_per_label[label];
      const auto & params = params_per_label[label];
      const auto & types = params.cut_polygon_types;
      if (std::find(types.begin(), types.end(), attribute) != types.end()) {
        for (auto i = 0UL; i < p.numSegments(); ++i) {
          data.cut_predicted_paths_segments.push_back(convert(p.segment(i)));
        }
      }
    }
  }
}

void add_cut_segments(
  FilteringDataPerLabel & data_per_label, const std::vector<lanelet::LineString3d> & linestrings,
  const std::vector<uint8_t> & all_labels, const std::vector<ObjectParameters> & params_per_label)
{
  for (const auto & ls : linestrings) {
    for (const auto label : all_labels) {
      auto & data = data_per_label[label];
      const auto & params = params_per_label[label];
      const auto attribute = ls.attributeOr(lanelet::AttributeName::Subtype, std::string());
      const auto & types = params.cut_linestring_types;
      if (std::find(types.begin(), types.end(), attribute) != types.end()) {
        for (auto i = 0UL; i < ls.numSegments(); ++i) {
          data.cut_predicted_paths_segments.push_back(convert(ls.segment(i)));
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
  const auto all_labels = Parameters::all_labels();
  data_per_label.resize(all_labels.size());
  for (const auto label : all_labels) {
    data_per_label[label].cut_predicted_paths_segments.push_back(ego_footprint.get_rear_segment());
  }
  const auto lanelets_in_range = map_ptr->laneletLayer.search(bounding_box);
  const auto polygons_in_range = map_ptr->polygonLayer.search(bounding_box);
  add_ignore_polygons(data_per_label, lanelets_in_range, all_labels, params_per_label);
  add_cut_segments(data_per_label, polygons_in_range, all_labels, params_per_label);
  const auto linestrings_in_range = map_ptr->lineStringLayer.search(bounding_box);
  add_cut_segments(data_per_label, linestrings_in_range, all_labels, params_per_label);
  for (const auto label : all_labels) {
    auto & data = data_per_label[label];
    const auto & params = parameters.object_parameters_per_label[label];
    if (params.ignore_if_on_ego_trajectory) {
      data.ignore_polygons.push_back(ego_footprint.front_polygon.outer());
      data.ignore_polygons.push_back(ego_footprint.rear_polygon.outer());
    }
  }
  // prepare rtrees
  for (const auto label : all_labels) {
    auto & data = data_per_label[label];
    std::vector<SegmentNode> nodes;
    nodes.reserve(data.cut_predicted_paths_segments.size());
    for (auto i = 0UL; i < data.cut_predicted_paths_segments.size(); ++i) {
      nodes.emplace_back(data.cut_predicted_paths_segments[i], i);
    }
    data.cut_predicted_paths_rtree = SegmentRtree(nodes);
  }
  for (const auto label : all_labels) {
    auto & data = data_per_label[label];
    std::vector<PolygonNode> nodes;
    nodes.reserve(data.ignore_polygons.size());
    for (auto i = 0UL; i < data.ignore_polygons.size(); ++i) {
      nodes.emplace_back(
        boost::geometry::return_envelope<universe_utils::Box2d>(data.ignore_polygons[i]), i);
    }
    data.ignore_rtree = PolygonRtree(nodes);
  }
  return data_per_label;
}
}  // namespace autoware::motion_velocity_planner::run_out
