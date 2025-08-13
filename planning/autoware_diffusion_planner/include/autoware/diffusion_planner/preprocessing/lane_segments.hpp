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
using autoware_planning_msgs::msg::LaneletRoute;
/**
 * @brief Represents a column index with its associated distance and whether it is inside a mask
 * range.
 */
struct ColWithDistance
{
  int64_t index;           //!< Column index in the input matrix.
  float distance_squared;  //!< Squared distance from the center.
  bool inside;             //!< Whether the column is within the mask range.
};

/**
 * @brief Maps between lanelet IDs and matrix column indices.
 */
struct ColLaneIDMaps
{
  std::map<lanelet::Id, int64_t> lane_id_to_matrix_col;  //!< Lanelet ID to matrix column index.
  std::map<int64_t, lanelet::Id> matrix_col_to_lane_id;  //!< Matrix column index to lanelet ID.
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
  explicit LaneSegmentContext(const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr);

  /**
   * @brief Get route segments and transform them to ego-centric coordinates.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
   * @param current_lanes List of current lanelets to extract.
   * @return Flattened vectors containing the transformed route segments and speed limits.
   */
  std::pair<std::vector<float>, std::vector<float>> get_route_segments(
    const Eigen::Matrix4f & transform_matrix,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
    const lanelet::ConstLanelets & current_lanes) const;

  /**
   * @brief Transform and select columns based on proximity to a center point.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param m Maximum number of columns (segments) to select.
   * @return Tuple of the transformed matrix and updated column ID mapping.
   */
  std::tuple<Eigen::MatrixXf, ColLaneIDMaps> transform_and_select_rows(
    const Eigen::Matrix4f & transform_matrix,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map, const float center_x,
    const float center_y, const int64_t m) const;

private:
  /**
   * @brief Add traffic light one-hot encoding to a segment matrix.
   *
   * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
   * @param segment_matrix The segment matrix to modify (in-place).
   * @param row_idx The row index in the matrix corresponding to the segment.
   * @param col_counter The column counter for the segment.
   */
  void add_traffic_light_one_hot_encoding_to_segment(
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
    Eigen::MatrixXf & segment_matrix, const int64_t row_idx, const int64_t col_counter) const;

  /**
   * @brief Apply coordinate transforms to the output matrix for all segments.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param output_matrix Matrix to transform (in-place).
   * @param num_segments Number of segments to transform.
   */
  void apply_transforms(
    const Eigen::Matrix4f & transform_matrix, Eigen::MatrixXf & output_matrix,
    int64_t num_segments) const;

  /**
   * @brief Compute distances of lane segments from a center point.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param distances Output vector to store column indices, distances, and mask inclusion.
   * @param center_x X-coordinate of the center point.
   * @param center_y Y-coordinate of the center point.
   * @param mask_range Range within which columns are considered "inside" the mask.
   */
  void compute_distances(
    const Eigen::Matrix4f & transform_matrix, std::vector<ColWithDistance> & distances,
    const float center_x, const float center_y, const float mask_range = 100.0) const;

  /**
   * @brief Transform and select columns from input matrix based on distances.
   *
   * @param transform_matrix Transformation matrix to apply to the points.
   * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
   * @param distances Vector of columns with distances, used to select columns.
   * @param m Maximum number of columns (segments) to select.
   * @return Tuple of the transformed matrix and updated column ID mapping.
   */
  std::tuple<Eigen::MatrixXf, ColLaneIDMaps> transform_points_and_add_traffic_info(
    const Eigen::Matrix4f & transform_matrix,
    const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
    const std::vector<ColWithDistance> & distances, int64_t m) const;

  // variables
  Eigen::MatrixXf map_lane_segments_matrix_;
  ColLaneIDMaps col_id_mapping_;
  const std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
};

/**
 * @brief Extracts lane tensor data from ego-centric lane segments.
 *
 * @param lane_segments_matrix Matrix containing ego-centric lane segment data.
 * @return A flattened vector of lane tensor data.
 */
std::vector<float> extract_lane_tensor_data(const Eigen::MatrixXf & lane_segments_matrix);

/**
 * @brief Extracts lane speed tensor data from ego-centric lane segments.
 *
 * @param lane_segments_matrix Matrix containing ego-centric lane segment data.
 * @return A flattened vector of lane speed tensor data.
 */
std::vector<float> extract_lane_speed_tensor_data(const Eigen::MatrixXf & lane_segments_matrix);

/**
 * @brief Sorts the columns by their squared distances in ascending order.
 *
 * @param distances Vector of columns with distances to be sorted.
 */
inline void sort_indices_by_distance(std::vector<ColWithDistance> & distances)
{
  std::sort(distances.begin(), distances.end(), [&](auto & a, auto & b) {
    return a.distance_squared < b.distance_squared;
  });
}

/**
 * @brief Returns a one-hot encoded row vector for the traffic signal state.
 *
 * @param signal The traffic light group message.
 * @return Row vector with one-hot encoding for [green, amber, red, unknown].
 */
Eigen::Matrix<float, 1, TRAFFIC_LIGHT_ONE_HOT_DIM> get_traffic_signal_row_vector(
  const autoware_perception_msgs::msg::TrafficLightGroup & signal);

/**
 * @brief Transforms selected rows of the output matrix using a transformation matrix.
 *
 * @param transform_matrix Transformation matrix to apply.
 * @param output_matrix Matrix to transform (in-place).
 * @param num_segments Number of segments to transform.
 * @param row_idx Index of the row to transform.
 * @param do_translation Whether to apply translation during the transformation.
 */
void transform_selected_rows(
  const Eigen::Matrix4f & transform_matrix, Eigen::MatrixXf & output_matrix, int64_t num_segments,
  int64_t row_idx, bool do_translation = true);

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__LANE_SEGMENTS_HPP_
