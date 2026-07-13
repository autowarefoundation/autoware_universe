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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__BOUNDARY_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__BOUNDARY_HPP_

#include <autoware/route_handler/route_handler.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Polygon.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/GenericTrafficRules.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::avoidance_target_detector
{

using autoware::route_handler::RouteHandler;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;

/** Left and right route boundary linestrings. */
using RouteBounds = std::pair<lanelet::LineString2d, lanelet::LineString2d>;

namespace traffic_rules
{

/** Location name registered for custom GoalPurposeRules traffic rules. */
inline constexpr const char * k_goal_purpose_location = "goal_purpose";

class GoalPurposeRules : public lanelet::traffic_rules::GenericTrafficRules
{
public:
  using lanelet::traffic_rules::GenericTrafficRules::GenericTrafficRules;

protected:
  lanelet::Optional<bool> canPass(
    const lanelet::RegulatoryElementConstPtrs & reg_elems) const override;

  lanelet::Optional<bool> canPass(
    const std::string & type, const std::string & location) const override;

  const lanelet::traffic_rules::CountrySpeedLimits & countrySpeedLimits() const override;

  lanelet::Optional<lanelet::traffic_rules::SpeedLimitInformation> speedLimit(
    const lanelet::RegulatoryElementConstPtrs & reg_elems) const override;

private:
  lanelet::traffic_rules::CountrySpeedLimits speed_limits_;
};

/** Build a routing graph using goal_purpose traffic rules. */
lanelet::routing::RoutingGraphConstPtr create_goal_purpose_routing_graph(
  const lanelet::LaneletMap & lanelet_map);

/** Get the left neighbor lanelet using goal_purpose routing rules. */
std::optional<lanelet::ConstLanelet> get_left_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet);

/** Get the right neighbor lanelet using goal_purpose routing rules. */
std::optional<lanelet::ConstLanelet> get_right_lanelet(
  const lanelet::routing::RoutingGraph & routing_graph, const lanelet::ConstLanelet & lanelet);

}  // namespace traffic_rules

class ExtendedLaneletSegments
{
public:
  explicit ExtendedLaneletSegments(const LaneletRoute & route);

  struct Segment
  {
    int64_t preferred_primitive{0};
    std::vector<int64_t> original_ordered_primitives;
    std::vector<int64_t> ordered_primitives;
    std::vector<int64_t> siblings_included_primitives;
    std::vector<int64_t> floating_primitives;
  };

  void build(const lanelet::LaneletMap & map, const lanelet::routing::RoutingGraph & routing_graph);

  [[nodiscard]] const std::vector<Segment> & segments() const { return segments_; }

private:
  LaneletRoute route_;
  std::vector<Segment> segments_;
};

class ExtendedRouteHandler
{
public:
  ExtendedRouteHandler(const LaneletMapBin & map, const LaneletRoute & route);

  /** Build the extended route map and routing graph from the original map and route. */
  void create_map();

  /** Write the route map to the debug OSM file. Temporary debug code. Must be removed before
   * release. */
  void export_debug_map() const;

  [[nodiscard]] const std::shared_ptr<RouteHandler> & getOriginalRouteHandler() const
  {
    return original_route_handler_;
  }

  [[nodiscard]] lanelet::routing::RoutingGraphConstPtr getExtendedRoutingGraph() const
  {
    return extended_routing_graph_;
  }

  [[nodiscard]] lanelet::routing::RoutingGraphConstPtr getRouteMapRoutingGraph() const
  {
    return route_map_routing_graph_;
  }

  [[nodiscard]] lanelet::LaneletMapPtr getRouteMap() const { return route_map_; }

  [[nodiscard]] std::vector<lanelet::LineString2d> get_road_borders() const;

  [[nodiscard]] RouteBounds get_primitive_set_bounds(const std::vector<int64_t> & primitives) const;

  [[nodiscard]] const RouteBounds & get_original_route_bounds() const
  {
    return original_route_bounds_;
  }

  [[nodiscard]] const RouteBounds & get_extended_route_bounds() const
  {
    return extended_route_bounds_;
  }

  [[nodiscard]] lanelet::BasicPolygon2d get_near_segment_polygon(
    const geometry_msgs::msg::Point & prev_end_point,
    const geometry_msgs::msg::Point & following_end_point) const;

private:
  [[nodiscard]] std::optional<std::size_t> find_segment_index_for_point(
    const geometry_msgs::msg::Point & point) const;

  [[nodiscard]] std::optional<std::size_t> find_segment_index_for_lanelet(
    const lanelet::Id lanelet_id) const;

  [[nodiscard]] std::vector<ExtendedLaneletSegments::Segment> get_near_segments(
    const geometry_msgs::msg::Point & prev_end_point,
    const geometry_msgs::msg::Point & following_end_point) const;

  [[nodiscard]] RouteBounds build_route_bounds(
    const std::vector<const std::vector<int64_t> *> & segment_primitives) const;

  LaneletRoute route_;
  lanelet::LaneletMapPtr route_map_;
  ExtendedLaneletSegments extended_lanelet_segments_;
  lanelet::routing::RoutingGraphConstPtr extended_routing_graph_;
  lanelet::routing::RoutingGraphConstPtr route_map_routing_graph_;
  std::shared_ptr<RouteHandler> original_route_handler_;
  RouteBounds original_route_bounds_;
  RouteBounds extended_route_bounds_;
};

/**
 * @brief Convert route bounds into an autoware_planning_msgs/Path for publishing.
 * @param bounds Left and right boundary linestrings.
 * @param trajectory Reference trajectory; header and centerline points are copied into the path.
 * @return Path message with left_bound and right_bound populated.
 */
Path to_path_msg(const RouteBounds & bounds, const Trajectory & trajectory);

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__BOUNDARY_HPP_
