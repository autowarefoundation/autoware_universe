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

#ifndef AUTOWARE__CANDIDATE_BANK__PREPROCESSING__LANE_SEGMENTS_HPP_
#define AUTOWARE__CANDIDATE_BANK__PREPROCESSING__LANE_SEGMENTS_HPP_

#include "autoware/candidate_bank/candidate/map_conflict_context.hpp"
#include "autoware/candidate_bank/conversion/lanelet.hpp"
#include "autoware/candidate_bank/dimensions.hpp"
#include "autoware/candidate_bank/preprocessing/traffic_signals.hpp"
#include "autoware/traffic_light_utils/traffic_light_utils.hpp"

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/CompoundPolygon.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/utility/Optional.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::candidate_bank::preprocess
{
using autoware::candidate_bank::LanePoint;
using autoware::candidate_bank::LaneSegment;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;

struct RouteTrafficSignalStop
{
  lanelet::Id traffic_light_id{0};
  uint8_t color{autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN};
  bool signal_observed{false};
  LanePoint stop_line_center_ego = LanePoint::Zero();
};
struct CurrentLaneBoundary
{
  lanelet::Id boundary_id{0};
  lanelet::Id adjacent_lane_id{0};
  std::vector<RouteTrafficSignalStop> adjacent_traffic_signal_stops;
  Polyline points_map;
};

struct CurrentLaneRuleContext
{
  bool valid{false};
  lanelet::Id lane_id{0};
  std::vector<RouteTrafficSignalStop> traffic_signal_stops;
  std::vector<CurrentLaneBoundary> shared_boundaries_map;
};

/**
 * @brief Context class that encapsulates static lane segment processing data and operations.
 *
 * This class combines the commonly used static parameters (matrix, mappings, and lanelet map)
 * that are determined at initialization time, separate from dynamic per-frame data like
 * transform_matrix and traffic_light_id_map.
 */
class LaneSegmentContext
{
public:
  /**
   * @brief Constructor that initializes the context with static data determined at initialization.
   *
   * @param lanelet_map_ptr Shared pointer to the lanelet map.
   */
  explicit LaneSegmentContext(
    const std::shared_ptr<const lanelet::LaneletMap> & lanelet_map_ptr,
    double line_string_max_step_m = 5.0);

  /**
   * @brief Select route segment indices based on route and constraints.
   *
   * @param route The lanelet route to process.
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param center_z Z-coordinate of the center point.
   * @param max_segments Maximum number of segments to select.
   * @return Vector of lane segment indices.
   */
  std::vector<int64_t> select_route_segment_indices(
    const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
    const int64_t max_segments) const;

  /**
   * @brief Select lane segment indices based on distances and constraints.
   *
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param max_segments Maximum number of segments to select.
   * @return Vector of lane segment indices.
   */
  std::vector<int64_t> select_lane_segment_indices(
    const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y,
    const int64_t max_segments) const;

  /**
   * @brief Create tensor data from selected segment indices.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
   * @param segment_indices Vector of segment indices to process.
   * @param max_segments Maximum number of segments for output tensor.
   * @return Pair of lane tensor data and speed limit vector.
   */
  std::pair<std::vector<float>, std::vector<float>> create_tensor_data_from_indices(
    const Eigen::Matrix4d & transform_matrix,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
    const std::vector<int64_t> & segment_indices, const int64_t max_segments) const;

  /**
   * @brief Get the first traffic light on the route from ego position forward.
   *
   * @param route The lanelet route.
   * @param center_x X-coordinate of ego center.
   * @param center_y Y-coordinate of ego center.
   * @param center_z Z-coordinate of ego center.
   * @param traffic_light_id_map Map of traffic light IDs to signal data.
   * @return TrafficLightGroup: cached signal if perception available, UNKNOWN element if not, or
   *         empty (traffic_light_group_id=0, elements empty) if no traffic light on route.
   */
  autoware_perception_msgs::msg::TrafficLightGroup get_first_traffic_light_on_route(
    const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map) const;

  /**
   * @brief Collect traffic-light stop lines on the selected forward route.
   *
   * Each result is associated with the same regulatory-element ID used by perception. The stop-line
   * center is transformed into the current ego frame. Duplicate regulatory elements are removed.
   */
  std::vector<RouteTrafficSignalStop> get_route_traffic_signal_stops(
    const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
    const Eigen::Matrix4d & map_to_ego_transform,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map) const;
  /**
   * @brief Resolve the lane currently occupied by the ego reference point and collect only that
   * lane's traffic-light stop lines plus boundaries shared with another lanelet.
   *
   * The previous lane ID is retained while the ego point still lies in that lane polygon, which
   * suppresses frame-to-frame flips while the reference point is exactly on a lane boundary.
   */
  CurrentLaneRuleContext get_current_lane_rule_context(
    const double center_x, const double center_y, const Eigen::Matrix4d & map_to_ego_transform,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
    double adjacent_signal_search_distance_m = 80.0) const;
  /**
   * @brief Build a compact route/competing-lane graph for dynamic-object relevance filtering.
   *
   * The forward route is searched for geometric centerline crossings.  For every crossing,
   * predecessor lanelets are traced backwards using directed connectivity inferred from centerline
   * endpoint continuity.  This experimental adapter avoids owning a Lanelet2 RoutingGraph.
   * The returned snapshot is map-frame geometry only and is safe to pass to CandidateManager.
   */
  CandidateMapConflictContext create_candidate_map_conflict_context(
    const LaneletRoute & route, const double center_x, const double center_y, const double center_z,
    double forward_search_distance_m, double backtrace_distance_m) const;

  /**
   * @brief Get the mapping from lanelet ID to array index.
   *
   * @return Map of lanelet IDs to their corresponding array indices.
   */
  const std::map<lanelet::Id, size_t> & get_lanelet_id_to_array_index() const
  {
    return lanelet_id_to_array_index_;
  }

  // Create polygon and linestring tensor data
  std::vector<float> create_polygon_tensor(
    const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y) const
  {
    return create_line_tensor(
      lanelet_map_.polygons, transform_matrix, center_x, center_y, NUM_POLYGONS, POINTS_PER_POLYGON,
      POLYGON_TYPE_NUM);
  }
  std::vector<float> create_line_string_tensor(
    const Eigen::Matrix4d & transform_matrix, const double center_x, const double center_y) const
  {
    return create_line_tensor(
      lanelet_map_.line_strings, transform_matrix, center_x, center_y, NUM_LINE_STRINGS,
      POINTS_PER_LINE_STRING, LINE_STRING_TYPE_NUM);
  }

private:
  /**
   * @brief Create line tensor data from elements with points and type.
   *
   * @tparam T Element type with `.points` (std::vector<LanePoint>) and `.type` (enum).
   * @param elements Vector of elements to process.
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param num_elements Maximum number of elements to include.
   * @param num_points Number of points per element.
   * @param num_types Number of type categories for one-hot encoding.
   * @return Vector of float tensor data with shape [num_elements, num_points, 2 + num_types].
   */
  template <typename T>
  std::vector<float> create_line_tensor(
    const std::vector<T> & elements, const Eigen::Matrix4d & transform_matrix,
    const double center_x, const double center_y, const int64_t num_elements,
    const int64_t num_points, const int64_t num_types) const;

  const std::shared_ptr<const lanelet::LaneletMap> lanelet_map_ptr_;
  const autoware::candidate_bank::LaneletMap lanelet_map_;
  const std::map<lanelet::Id, size_t> lanelet_id_to_array_index_;
  const std::map<lanelet::Id, std::vector<CurrentLaneBoundary>> shared_lane_boundaries_by_lane_id_;
  mutable std::optional<lanelet::Id> candidate_current_lane_id_;
  const std::map<lanelet::Id, std::vector<lanelet::Id>> predecessor_lane_ids_;
  const std::map<lanelet::Id, std::vector<lanelet::Id>> successor_lane_ids_;
};

}  // namespace autoware::candidate_bank::preprocess

#endif  // AUTOWARE__CANDIDATE_BANK__PREPROCESSING__LANE_SEGMENTS_HPP_
