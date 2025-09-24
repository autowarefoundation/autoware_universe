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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__LANE_SEGMENTS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__LANE_SEGMENTS_HPP_

#include "autoware/diffusion_planner/conversion/lanelet.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/traffic_signals.hpp"
#include "autoware/traffic_light_utils/traffic_light_utils.hpp"

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

namespace autoware::diffusion_planner::preprocess
{
using autoware::diffusion_planner::LanePoint;
using autoware::diffusion_planner::LaneSegment;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;

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
  explicit LaneSegmentContext(const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr);

  /**
   * @brief Select route segment indices based on route and constraints.
   *
   * @param route The lanelet route to process.
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param max_segments Maximum number of segments to select.
   * @return Vector of lane segment indices.
   */
  std::vector<int64_t> select_route_segment_indices(
    const LaneletRoute & route, const double center_x, const double center_y,
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
   * @brief Get the mapping from lanelet ID to array index.
   *
   * @return Map of lanelet IDs to their corresponding array indices.
   */
  const std::map<lanelet::Id, size_t> & get_lanelet_id_to_array_index() const
  {
    return lanelet_id_to_array_index_;
  }

private:
  const std::vector<autoware::diffusion_planner::LaneSegment> lane_segments_;
  const std::map<lanelet::Id, size_t> lanelet_id_to_array_index_;
};

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__LANE_SEGMENTS_HPP_
