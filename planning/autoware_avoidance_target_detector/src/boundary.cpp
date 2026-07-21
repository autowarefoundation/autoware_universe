// Copyright 2026 Autoware Foundation
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

#include "autoware/avoidance_target_detector/boundary.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>

#include <autoware_planning_msgs/msg/path_point.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/CompoundLineString.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

namespace traffic_rules
{

namespace
{
lanelet::traffic_rules::RegisterTrafficRules<GoalPurposeRules> register_goal_purpose_rule(
  k_goal_purpose_location, lanelet::Participants::Vehicle);
}  // namespace

lanelet::Optional<bool> GoalPurposeRules::canPass(
  const lanelet::RegulatoryElementConstPtrs & /*reg_elems*/) const
{
  return {};
}

lanelet::Optional<bool> GoalPurposeRules::canPass(
  const std::string & type, const std::string & /*location*/) const
{
  using ParticipantsMap = std::map<std::string, std::vector<std::string>>;
  using Value = lanelet::AttributeValueString;

  static const ParticipantsMap participant_map{
    {"", {lanelet::Participants::Vehicle}},
    {Value::Road, {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle}},
    {"road_shoulder",
     {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle,
      lanelet::Participants::Pedestrian}},
    {"pedestrian_lane",
     {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle,
      lanelet::Participants::Pedestrian}},
    {Value::Highway, {lanelet::Participants::Vehicle}},
    {Value::BicycleLane, {lanelet::Participants::Vehicle, lanelet::Participants::Bicycle}},
    {Value::PlayStreet,
     {lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle,
      lanelet::Participants::Vehicle}},
    {Value::BusLane,
     {lanelet::Participants::VehicleBus, lanelet::Participants::VehicleEmergency,
      lanelet::Participants::VehicleTaxi}},
    {Value::EmergencyLane, {lanelet::Participants::VehicleEmergency}},
    {Value::Exit,
     {lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle,
      lanelet::Participants::Vehicle}},
    {Value::Walkway, {lanelet::Participants::Pedestrian}},
    {Value::Crosswalk, {lanelet::Participants::Pedestrian}},
    {Value::Stairs, {lanelet::Participants::Pedestrian}},
    {Value::SharedWalkway, {lanelet::Participants::Pedestrian, lanelet::Participants::Bicycle}}};

  const auto participants = participant_map.find(type);
  if (participants == participant_map.end()) {
    return {};
  }

  return std::any_of(
    participants->second.begin(), participants->second.end(),
    [this](const std::string & allowed_participant) {
      return this->participant().compare(0, allowed_participant.size(), allowed_participant) == 0;
    });
}

const lanelet::traffic_rules::CountrySpeedLimits & GoalPurposeRules::countrySpeedLimits() const
{
  return speed_limits_;
}

lanelet::Optional<lanelet::traffic_rules::SpeedLimitInformation> GoalPurposeRules::speedLimit(
  const lanelet::RegulatoryElementConstPtrs & /*reg_elems*/) const
{
  return {};
}

lanelet::routing::RoutingGraphConstPtr create_goal_purpose_routing_graph(
  const lanelet::LaneletMap & lanelet_map)
{
  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    k_goal_purpose_location, lanelet::Participants::Vehicle);
  return lanelet::routing::RoutingGraph::build(lanelet_map, *traffic_rules);
}

std::optional<lanelet::ConstLanelet> get_left_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet)
{
  if (const auto left_lanelet = routing_graph.left(lanelet)) {
    return *left_lanelet;
  }
  if (const auto adjacent_left_lanelet = routing_graph.adjacentLeft(lanelet)) {
    return *adjacent_left_lanelet;
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLanelet> get_right_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet)
{
  if (const auto right_lanelet = routing_graph.right(lanelet)) {
    return *right_lanelet;
  }
  if (const auto adjacent_right_lanelet = routing_graph.adjacentRight(lanelet)) {
    return *adjacent_right_lanelet;
  }
  return std::nullopt;
}

}  // namespace traffic_rules

namespace
{

bool exists_in_map(const lanelet::LaneletMap & map, const lanelet::Id id)
{
  return map.laneletLayer.exists(id);
}

bool are_laterally_connected(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::LaneletMap & map,
  const lanelet::Id a, const lanelet::Id b)
{
  if (!exists_in_map(map, a) || !exists_in_map(map, b)) {
    return false;
  }

  const auto lanelet_a = map.laneletLayer.get(a);
  if (const auto left = traffic_rules::get_left_lanelet(routing_graph, lanelet_a)) {
    if (left->id() == b) {
      return true;
    }
  }
  if (const auto right = traffic_rules::get_right_lanelet(routing_graph, lanelet_a)) {
    if (right->id() == b) {
      return true;
    }
  }
  return false;
}

void insert_lateral_chain_to_edge(
  lanelet::ConstLanelet lanelet, const lanelet::routing::RoutingGraph & routing_graph,
  const bool to_left, std::set<int64_t> & ids)
{
  constexpr size_t k_max_lateral_hops = 4;
  for (size_t hop = 0; hop < k_max_lateral_hops; ++hop) {
    const auto neighbor = to_left ? traffic_rules::get_left_lanelet(routing_graph, lanelet)
                                  : traffic_rules::get_right_lanelet(routing_graph, lanelet);
    if (!neighbor) {
      break;
    }
    if (!ids.insert(neighbor->id()).second) {
      break;
    }
    lanelet = *neighbor;
  }
}

std::vector<std::vector<int64_t>> find_lateral_connected_components(
  const std::vector<int64_t> & primitive_ids, const lanelet::LaneletMap & map,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  std::set<int64_t> unvisited(primitive_ids.begin(), primitive_ids.end());
  for (const auto id : primitive_ids) {
    if (!exists_in_map(map, id)) {
      continue;
    }
    const auto lanelet = map.laneletLayer.get(id);
    insert_lateral_chain_to_edge(lanelet, routing_graph, true, unvisited);
    insert_lateral_chain_to_edge(lanelet, routing_graph, false, unvisited);
  }

  std::vector<std::vector<int64_t>> components;
  while (!unvisited.empty()) {
    const auto seed = *unvisited.begin();
    std::vector<int64_t> component;
    std::vector<int64_t> queue{seed};
    unvisited.erase(seed);

    while (!queue.empty()) {
      const auto current = queue.back();
      queue.pop_back();
      component.push_back(current);

      for (auto it = unvisited.begin(); it != unvisited.end();) {
        const auto neighbor = *it;
        if (are_laterally_connected(routing_graph, map, current, neighbor)) {
          queue.push_back(neighbor);
          it = unvisited.erase(it);
        } else {
          ++it;
        }
      }
    }

    components.push_back(std::move(component));
  }

  return components;
}

std::optional<int64_t> find_left_neighbor_in_set(
  const int64_t id, const std::set<int64_t> & id_set, const lanelet::LaneletMap & map,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  if (!exists_in_map(map, id)) {
    return std::nullopt;
  }

  const auto lanelet = map.laneletLayer.get(id);
  if (const auto left = traffic_rules::get_left_lanelet(routing_graph, lanelet)) {
    if (id_set.count(left->id()) > 0) {
      return left->id();
    }
  }
  return std::nullopt;
}

std::optional<int64_t> find_right_neighbor_in_set(
  const int64_t id, const std::set<int64_t> & id_set, const lanelet::LaneletMap & map,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  if (!exists_in_map(map, id)) {
    return std::nullopt;
  }

  const auto lanelet = map.laneletLayer.get(id);
  if (const auto right = traffic_rules::get_right_lanelet(routing_graph, lanelet)) {
    if (id_set.count(right->id()) > 0) {
      return right->id();
    }
  }
  return std::nullopt;
}

std::vector<int64_t> sort_left_to_right(
  const std::vector<int64_t> & primitive_ids, const lanelet::LaneletMap & map,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  if (primitive_ids.empty()) {
    return {};
  }

  const std::set<int64_t> id_set(primitive_ids.begin(), primitive_ids.end());

  int64_t leftmost = primitive_ids.front();
  for (const auto id : primitive_ids) {
    if (!find_left_neighbor_in_set(id, id_set, map, routing_graph)) {
      leftmost = id;
      break;
    }
  }

  std::vector<int64_t> ordered;
  ordered.reserve(primitive_ids.size());
  std::set<int64_t> visited;
  auto current = leftmost;

  while (visited.count(current) == 0) {
    ordered.push_back(current);
    visited.insert(current);
    if (const auto right = find_right_neighbor_in_set(current, id_set, map, routing_graph)) {
      current = *right;
    } else {
      break;
    }
  }

  for (const auto id : primitive_ids) {
    if (visited.count(id) == 0) {
      ordered.push_back(id);
    }
  }

  return ordered;
}

std::vector<int64_t> collect_sibling_primitives(
  const std::vector<int64_t> & prev_ordered_primitives,
  const std::vector<int64_t> & /*current_ordered_primitives*/,
  const std::vector<int64_t> & next_ordered_primitives, const lanelet::LaneletMap & map,
  const lanelet::routing::RoutingGraph & routing_graph)
{
  std::vector<int64_t> siblings;
  std::set<int64_t> added;

  for (int64_t next_id : next_ordered_primitives) {
    if (!exists_in_map(map, next_id)) {
      continue;
    }

    for (int64_t prev_id : prev_ordered_primitives) {
      if (!exists_in_map(map, prev_id)) {
        continue;
      }

      const auto prev_lanelet = map.laneletLayer.get(prev_id);
      for (const auto & middle : routing_graph.following(prev_lanelet)) {
        if (added.count(middle.id()) > 0) {
          continue;
        }

        const auto next_lanelets = routing_graph.following(middle);
        const bool connects_to_next = std::any_of(
          next_lanelets.begin(), next_lanelets.end(),
          [&](const auto & next) { return next.id() == next_id; });
        if (!connects_to_next) {
          continue;
        }

        added.insert(middle.id());
        siblings.push_back(middle.id());
      }
    }
  }

  return siblings;
}

lanelet::LineString2d remove_const(const lanelet::ConstLineString2d & line_string)
{
  lanelet::LineString2d linestring(lanelet::utils::removeConst(line_string.constData()));
  return line_string.inverted() ? linestring.invert() : linestring;
}

void add_bounds_linestring_to_map(
  lanelet::LaneletMap & map, const lanelet::LineString2d & linestring_2d, const std::string & type)
{
  if (linestring_2d.empty()) {
    return;
  }
  lanelet::LineString3d linestring = lanelet::utils::to3D(linestring_2d);
  linestring.setAttribute(lanelet::AttributeName::Type, type);
  map.add(linestring);
}

std::vector<lanelet::ConstLanelet> get_nearest_lanelets(
  const lanelet::LaneletMap & route_map, const lanelet::BasicPoint2d & search_point)
{
  constexpr size_t k_max_nearest_lanelets = 5;
  std::vector<lanelet::ConstLanelet> nearest_lanelets;
  nearest_lanelets.reserve(k_max_nearest_lanelets);

  route_map.laneletLayer.nearestUntil(
    search_point, [&](const lanelet::BoundingBox2d & bbox, const lanelet::ConstLanelet & lanelet) {
      if (lanelet::geometry::inside(lanelet, search_point)) {
        nearest_lanelets.push_back(lanelet);
        return nearest_lanelets.size() >= k_max_nearest_lanelets;
      }
      constexpr double k_bbox_touch_epsilon_m = 1e-3;
      return lanelet::geometry::distance2d(bbox, search_point) > k_bbox_touch_epsilon_m;
    });

  return nearest_lanelets;
}

bool is_road_shoulder(const lanelet::ConstLanelet & lanelet)
{
  return std::string(lanelet.attributeOr(lanelet::AttributeName::Subtype, "none")) ==
         "road_shoulder";
}

bool is_pedestrian_lane(const lanelet::ConstLanelet & lanelet)
{
  return std::string(lanelet.attributeOr(lanelet::AttributeName::Subtype, "none")) ==
         "pedestrian_lane";
}

std::optional<double> read_speed_limit_from_lanelet(const lanelet::ConstLanelet & lanelet)
{
  constexpr const char * k_speed_limit_attribute = "speed_limit";
  if (!lanelet.hasAttribute(k_speed_limit_attribute)) {
    return std::nullopt;
  }
  const auto v = lanelet.attribute(k_speed_limit_attribute).asDouble();
  if (!v) {
    return std::nullopt;
  }
  return v.get();
}

lanelet::LaneletMap build_debug_map(lanelet::LaneletMap & route_map)
{
  lanelet::LaneletMap debug_map;
  for (const auto & lanelet : route_map.laneletLayer) {
    debug_map.add(lanelet);
  }
  for (const auto & linestring : route_map.lineStringLayer) {
    debug_map.add(linestring);
  }
  return debug_map;
}
}  // namespace

ExtendedLaneletSegments::ExtendedLaneletSegments(const LaneletRoute & route) : route_{route}
{
  segments_.reserve(route.segments.size());
  for (const auto & segment : route.segments) {
    Segment extended_segment;
    extended_segment.preferred_primitive = segment.preferred_primitive.id;
    segments_.push_back(std::move(extended_segment));
  }
}

void ExtendedLaneletSegments::build(
  const lanelet::LaneletMap & map, const lanelet::routing::RoutingGraph & routing_graph)
{
  segments_.clear();
  segments_.reserve(route_.segments.size());

  for (const auto & route_segment : route_.segments) {
    Segment segment;
    segment.preferred_primitive = route_segment.preferred_primitive.id;

    std::vector<int64_t> original_primitive_ids;
    original_primitive_ids.reserve(route_segment.primitives.size());
    for (const auto & primitive : route_segment.primitives) {
      if (!exists_in_map(map, primitive.id)) {
        continue;
      }
      original_primitive_ids.push_back(primitive.id);
    }

    const auto components =
      find_lateral_connected_components(original_primitive_ids, map, routing_graph);

    std::vector<int64_t> largest_component;
    for (const auto & component : components) {
      if (component.size() > largest_component.size()) {
        largest_component = component;
      }
    }

    segment.ordered_primitives = sort_left_to_right(largest_component, map, routing_graph);
    segment.siblings_included_primitives = segment.ordered_primitives;

    const std::set<int64_t> original_ids(
      original_primitive_ids.begin(), original_primitive_ids.end());
    segment.original_ordered_primitives.reserve(segment.ordered_primitives.size());
    for (const auto id : segment.ordered_primitives) {
      if (original_ids.count(id) > 0) {
        segment.original_ordered_primitives.push_back(id);
      }
    }

    const std::set<int64_t> ordered_ids(
      segment.ordered_primitives.begin(), segment.ordered_primitives.end());
    for (const auto id : original_primitive_ids) {
      if (ordered_ids.count(id) == 0) {
        segment.floating_primitives.push_back(id);
      }
    }

    segments_.push_back(std::move(segment));
  }

  for (size_t i = 0; i + 2 < segments_.size(); ++i) {
    auto & current_segment = segments_[i + 1];
    const auto siblings = collect_sibling_primitives(
      segments_[i].ordered_primitives, current_segment.ordered_primitives,
      segments_[i + 2].ordered_primitives, map, routing_graph);

    // When there are siblings not originally included in the segment
    if (siblings.size() > current_segment.ordered_primitives.size()) {
      current_segment.siblings_included_primitives = siblings;
    }
  }
}

ExtendedRouteHandler::ExtendedRouteHandler(const LaneletMapBin & map, const LaneletRoute & route)
: route_{route},
  route_map_{std::make_shared<lanelet::LaneletMap>()},
  extended_lanelet_segments_{route},
  original_route_handler_{std::make_shared<RouteHandler>()}
{
  original_route_handler_->setMap(map);
  original_route_handler_->setRoute(route);
}

void ExtendedRouteHandler::create_map()
{
  const auto map = original_route_handler_->getLaneletMapPtr();
  extended_routing_graph_ = traffic_rules::create_goal_purpose_routing_graph(*map);
  extended_lanelet_segments_.build(*map, *extended_routing_graph_);

  std::set<lanelet::Id> lanelet_ids;
  for (const auto & segment : extended_lanelet_segments_.segments()) {
    for (const auto id : segment.siblings_included_primitives) {
      lanelet_ids.insert(id);
    }
    for (const auto id : segment.floating_primitives) {
      lanelet_ids.insert(id);
    }
  }

  route_map_ = std::make_shared<lanelet::LaneletMap>();
  for (const auto lanelet_id : lanelet_ids) {
    if (!map->laneletLayer.exists(lanelet_id)) {
      continue;
    }
    lanelet::Lanelet lanelet = map->laneletLayer.get(lanelet_id);
    const auto reg_elements = lanelet.regulatoryElements();
    for (const auto & elem : reg_elements) {
      lanelet.removeRegulatoryElement(elem);
    }
    route_map_->add(lanelet);
  }

  constexpr double k_road_border_near_distance_m = 1.5;
  std::set<lanelet::Id> road_border_ids;
  for (const auto lanelet_id : lanelet_ids) {
    if (!map->laneletLayer.exists(lanelet_id)) {
      continue;
    }
    const auto lanelet = map->laneletLayer.get(lanelet_id);
    const auto nearby_linestrings =
      lanelet::geometry::findWithin2d(map->lineStringLayer, lanelet, k_road_border_near_distance_m);
    for (const auto & nearby_linestring : nearby_linestrings) {
      const auto & linestring = nearby_linestring.second;
      if (road_border_ids.count(linestring.id()) > 0) {
        continue;
      }
      constexpr auto no_type = "none";
      const std::string type = linestring.attributeOr(lanelet::AttributeName::Type, no_type);
      if (type != "road_border") {
        continue;
      }
      road_border_ids.insert(linestring.id());
      route_map_->add(linestring);
    }
  }

  std::vector<const std::vector<int64_t> *> original_primitive_lists;
  std::vector<const std::vector<int64_t> *> extended_primitive_lists;
  original_primitive_lists.reserve(extended_lanelet_segments_.segments().size());
  extended_primitive_lists.reserve(extended_lanelet_segments_.segments().size());
  for (const auto & segment : extended_lanelet_segments_.segments()) {
    original_primitive_lists.push_back(&segment.original_ordered_primitives);
    extended_primitive_lists.push_back(&segment.siblings_included_primitives);
  }
  original_route_bounds_ = build_route_bounds(original_primitive_lists);
  extended_route_bounds_ = build_route_bounds(extended_primitive_lists);

  route_map_routing_graph_ = traffic_rules::create_goal_purpose_routing_graph(*route_map_);
}

void ExtendedRouteHandler::export_debug_map() const
{
  const auto package_share_directory =
    ament_index_cpp::get_package_share_directory("autoware_avoidance_target_detector");
  const auto debug_map_path_str = package_share_directory + "/debug_map.osm";

  constexpr double origin_lat = 35.22312494103;
  constexpr double origin_lon = 138.80245834626;
  lanelet::Origin origin({origin_lat, origin_lon});
  lanelet::projection::MGRSProjector projector(origin);
  projector.setMGRSCode(lanelet::GPSPoint{origin_lat, origin_lon, 0.0});

  auto debug_map = build_debug_map(*route_map_);

  const auto original_bounds = get_original_route_bounds();
  add_bounds_linestring_to_map(debug_map, original_bounds.first, "original_route");
  add_bounds_linestring_to_map(debug_map, original_bounds.second, "original_route");

  const auto extended_bounds = get_extended_route_bounds();
  add_bounds_linestring_to_map(debug_map, extended_bounds.first, "extended_route");
  add_bounds_linestring_to_map(debug_map, extended_bounds.second, "extended_route");

  lanelet::write(debug_map_path_str, debug_map, projector);
}

std::vector<lanelet::LineString2d> ExtendedRouteHandler::get_road_borders() const
{
  std::vector<lanelet::LineString2d> road_borders;
  for (const auto & ls : route_map_->lineStringLayer) {
    const std::string type = ls.attributeOr(lanelet::AttributeName::Type, "none");
    if (type != "road_border") {
      continue;
    }
    road_borders.push_back(lanelet::utils::to2D(ls));
  }
  return road_borders;
}

std::pair<lanelet::LineString2d, lanelet::LineString2d>
ExtendedRouteHandler::get_primitive_set_bounds(const std::vector<int64_t> & primitives) const
{
  if (
    !exists_in_map(*route_map_, primitives.front()) ||
    !exists_in_map(*route_map_, primitives.back())) {
    return std::make_pair(lanelet::LineString2d(), lanelet::LineString2d());
  }

  const lanelet::Lanelet start_lanelet = route_map_->laneletLayer.get(primitives.front());
  const lanelet::Lanelet end_lanelet = route_map_->laneletLayer.get(primitives.back());
  const lanelet::LineString2d start_left_bound = remove_const(start_lanelet.leftBound2d());
  const lanelet::LineString2d end_right_bound = remove_const(end_lanelet.rightBound2d());
  return std::make_pair(start_left_bound, end_right_bound);
}

std::pair<lanelet::LineString2d, lanelet::LineString2d> ExtendedRouteHandler::build_route_bounds(
  const std::vector<const std::vector<int64_t> *> & segment_primitives) const
{
  std::vector<lanelet::LineString2d> left_bounds;
  std::vector<lanelet::LineString2d> right_bounds;
  left_bounds.reserve(segment_primitives.size());
  right_bounds.reserve(segment_primitives.size());
  for (const auto * primitives : segment_primitives) {
    const auto bounds = get_primitive_set_bounds(*primitives);
    left_bounds.push_back(bounds.first);
    right_bounds.push_back(bounds.second);
  }
  const auto left_compound_bound = lanelet::CompoundLineString2d(left_bounds);
  const auto right_compound_bound = lanelet::CompoundLineString2d(right_bounds);

  lanelet::LineString2d left_bound(lanelet::utils::getId());
  lanelet::LineString2d right_bound(lanelet::utils::getId());
  left_bound.reserve(left_compound_bound.size());
  right_bound.reserve(right_compound_bound.size());
  for (const auto & point : left_compound_bound) {
    left_bound.push_back(
      lanelet::Point2d(point.id(), {point.basicPoint().x(), point.basicPoint().y(), 0.0}));
  }
  for (const auto & point : right_compound_bound) {
    right_bound.push_back(
      lanelet::Point2d(point.id(), {point.basicPoint().x(), point.basicPoint().y(), 0.0}));
  }

  return std::make_pair(left_bound, right_bound);
}

std::optional<std::size_t> ExtendedRouteHandler::find_segment_index_for_lanelet(
  const lanelet::Id lanelet_id) const
{
  const auto & segments = extended_lanelet_segments_.segments();
  for (std::size_t i = 0; i < segments.size(); ++i) {
    const auto & primitives = segments.at(i).siblings_included_primitives;
    if (std::find(primitives.begin(), primitives.end(), lanelet_id) != primitives.end()) {
      return i;
    }
  }
  return std::nullopt;
}

std::optional<std::size_t> ExtendedRouteHandler::find_segment_index_for_point(
  const geometry_msgs::msg::Point & point) const
{
  const lanelet::BasicPoint2d search_point{point.x, point.y};

  const auto searched_lanelets = route_map_->laneletLayer.nearest(search_point, 1);
  if (searched_lanelets.empty()) {
    return std::nullopt;
  }

  const auto & closest_lanelet = searched_lanelets.front();

  return find_segment_index_for_lanelet(closest_lanelet.id());
}

std::vector<ExtendedLaneletSegments::Segment> ExtendedRouteHandler::get_near_segments(
  const geometry_msgs::msg::Point & prev_end_point,
  const geometry_msgs::msg::Point & following_end_point) const
{
  const auto start_idx = find_segment_index_for_point(prev_end_point);
  const auto end_idx = find_segment_index_for_point(following_end_point);
  if (!start_idx || !end_idx) {
    return {};
  }

  if (*end_idx < *start_idx) {
    return {};
  }

  const auto & all_segments = extended_lanelet_segments_.segments();
  return {
    all_segments.begin() + static_cast<std::ptrdiff_t>(*start_idx),
    all_segments.begin() + static_cast<std::ptrdiff_t>(*end_idx) + 1};
}

lanelet::BasicPolygon2d ExtendedRouteHandler::get_near_segment_polygon(
  const geometry_msgs::msg::Point & prev_end_point,
  const geometry_msgs::msg::Point & following_end_point) const
{
  const auto segments = get_near_segments(prev_end_point, following_end_point);
  if (segments.empty()) {
    return {};
  }

  std::vector<lanelet::LineString2d> left_bounds;
  std::vector<lanelet::LineString2d> right_bounds;
  left_bounds.reserve(segments.size());
  right_bounds.reserve(segments.size());

  for (const auto & segment : segments) {
    if (segment.siblings_included_primitives.empty()) {
      continue;
    }
    const auto bounds = get_primitive_set_bounds(segment.siblings_included_primitives);
    if (bounds.first.empty() || bounds.second.empty()) {
      continue;
    }
    left_bounds.push_back(bounds.first);
    right_bounds.push_back(bounds.second.invert());
  }

  if (left_bounds.empty() || right_bounds.empty()) {
    return {};
  }

  std::reverse(right_bounds.begin(), right_bounds.end());

  const auto left_compound_bound = lanelet::CompoundLineString2d(left_bounds);
  const auto right_compound_bound = lanelet::CompoundLineString2d(right_bounds);

  lanelet::BasicLineString2d ring;
  ring.reserve(left_compound_bound.size() + right_compound_bound.size() + 1);

  for (const auto & point : left_compound_bound) {
    ring.emplace_back(point.basicPoint2d());
  }

  for (const auto & point : right_compound_bound) {
    ring.emplace_back(point.basicPoint2d());
  }

  if (ring.size() < 3) {
    return {};
  }

  ring.emplace_back(ring.front());

  return lanelet::BasicPolygon2d(ring);
}

std::optional<double> ExtendedRouteHandler::get_velocity_limit(
  const lanelet::BasicPoint2d & point) const
{
  const auto nearest_lanelets = get_nearest_lanelets(*route_map_, point);
  if (nearest_lanelets.empty()) {
    return std::nullopt;
  }

  std::optional<double> velocity_limit;

  for (const auto & lanelet : nearest_lanelets) {
    // If the lanelet has a speed limit attribute, use it.
    const auto speed_limit = read_speed_limit_from_lanelet(lanelet);
    if (speed_limit) {
      velocity_limit = (velocity_limit) ? std::min(*velocity_limit, *speed_limit) : *speed_limit;
      continue;
    }

    // If the lanelet has no speed limit attribute, but is a road_shoulder or pedestrian_lane, get
    // the speed limit from the left and right lanelets.
    if (is_road_shoulder(lanelet) || is_pedestrian_lane(lanelet)) {
      const auto left_lanelet = traffic_rules::get_left_lanelet(*extended_routing_graph_, lanelet);
      const auto right_lanelet =
        traffic_rules::get_right_lanelet(*extended_routing_graph_, lanelet);
      if (right_lanelet) {
        const auto right_speed_limit = read_speed_limit_from_lanelet(*right_lanelet);
        if (right_speed_limit) {
          velocity_limit =
            (velocity_limit) ? std::min(*velocity_limit, *right_speed_limit) : *right_speed_limit;
          continue;
        }
      }
      if (left_lanelet) {
        const auto left_speed_limit = read_speed_limit_from_lanelet(*left_lanelet);
        if (left_speed_limit) {
          velocity_limit =
            (velocity_limit) ? std::min(*velocity_limit, *left_speed_limit) : *left_speed_limit;
          continue;
        }
      }
    }
  }

  return velocity_limit;
}

std::optional<double> ExtendedRouteHandler::get_velocity_limit(const lanelet::Point2d & point) const
{
  return get_velocity_limit(lanelet::BasicPoint2d(point.x(), point.y()));
}

std::optional<double> ExtendedRouteHandler::get_velocity_limit(
  const geometry_msgs::msg::Point & point) const
{
  return get_velocity_limit(lanelet::BasicPoint2d(point.x, point.y));
}

Path to_path_msg(const RouteBounds & bounds, const Trajectory & trajectory)
{
  auto to_geometry_points = [](const lanelet::LineString2d & linestring) {
    std::vector<geometry_msgs::msg::Point> points;
    points.reserve(linestring.size());
    for (const auto & point : linestring) {
      geometry_msgs::msg::Point geometry_point;
      geometry_point.x = point.x();
      geometry_point.y = point.y();
      geometry_point.z = 0.0;
      points.push_back(geometry_point);
    }
    return points;
  };

  Path path;
  path.header = trajectory.header;
  path.left_bound = to_geometry_points(bounds.first);
  path.right_bound = to_geometry_points(bounds.second);

  path.points.reserve(trajectory.points.size());
  for (const auto & trajectory_point : trajectory.points) {
    autoware_planning_msgs::msg::PathPoint path_point;
    path_point.pose = trajectory_point.pose;
    path_point.longitudinal_velocity_mps = trajectory_point.longitudinal_velocity_mps;
    path_point.lateral_velocity_mps = trajectory_point.lateral_velocity_mps;
    path_point.heading_rate_rps = trajectory_point.heading_rate_rps;
    path.points.push_back(path_point);
  }

  return path;
}

}  // namespace autoware::avoidance_target_detector
