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
 * @brief Extracts route segments from the map lane segments matrix and transforms them to
 * ego-centric coordinates.
 *
 * @param map_lane_segments_matrix Matrix containing lane segment data in map coordinates.
 * @param transform_matrix Transformation matrix to apply to the points.
 * @param col_id_mapping Map of segment IDs to their corresponding column indices in the lane
 * segments matrix.
 * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
 * @param lanelet_map_ptr Shared pointer to the lanelet map.
 * @param current_lanes List of current lanelets to extract.
 * @return Flattened vectors containing the transformed route segments in ego-centric coordinates
 * and speed limits.
 */
std::pair<std::vector<float>, std::vector<float>> get_route_segments(
  const Eigen::MatrixXf & map_lane_segments_matrix, const Eigen::Matrix4f & transform_matrix,
  const ColLaneIDMaps & col_id_mapping,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::ConstLanelets & current_lanes);

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
 * @brief Processes multiple lane segments and converts them into a single matrix.
 *
 * @param lane_segments Vector of lane segments to process.
 * @param col_id_mapping Map to store the starting column index of each segment in the resulting
 * matrix.
 * @return A matrix containing the processed lane segment data (transposed: columns are segments).
 * @throws std::runtime_error If any segment matrix does not have the expected number of rows.
 */
Eigen::MatrixXf process_segments_to_matrix(
  const std::vector<LaneSegment> & lane_segments, ColLaneIDMaps & col_id_mapping);

/**
 * @brief Processes a single lane segment and converts it into a matrix representation.
 *
 * @param segment The lane segment to process.
 * @return A matrix containing the processed lane segment data, or an empty matrix if the segment is
 * invalid.
 */
Eigen::MatrixXf process_segment_to_matrix(const LaneSegment & segment);

/**
 * @brief Computes distances of lane segments from a center point and stores the results.
 *
 * @param input_matrix Input matrix containing lane segment data.
 * @param transform_matrix Transformation matrix to apply to the points.
 * @param distances Output vector to store column indices, distances, and mask inclusion.
 * @param center_x X-coordinate of the center point.
 * @param center_y Y-coordinate of the center point.
 * @param mask_range Range within which columns are considered "inside" the mask.
 */
void compute_distances(
  const Eigen::MatrixXf & input_matrix, const Eigen::Matrix4f & transform_matrix,
  std::vector<ColWithDistance> & distances, const float center_x, const float center_y,
  const float mask_range = 100.0);

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
 * @brief Adds one-hot encoded traffic light information to a segment matrix.
 *
 * @param segment_matrix The segment matrix to modify (in-place).
 * @param col_id_mapping Mapping between lanelet IDs and matrix columns.
 * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
 * @param lanelet_map_ptr Shared pointer to the lanelet map.
 * @param row_idx The row index in the matrix corresponding to the segment.
 * @param col_counter The column counter for the segment.
 */
void add_traffic_light_one_hot_encoding_to_segment(
  [[maybe_unused]] Eigen::MatrixXf & segment_matrix, const ColLaneIDMaps & col_id_mapping,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, const int64_t row_idx,
  [[maybe_unused]] const int64_t col_counter);

/**
 * @brief Returns a one-hot encoded row vector for the traffic signal state.
 *
 * @param signal The traffic light group message.
 * @return Row vector with one-hot encoding for [green, amber, red, unknown].
 */
Eigen::Matrix<float, 1, TRAFFIC_LIGHT_ONE_HOT_DIM> get_traffic_signal_row_vector(
  const autoware_perception_msgs::msg::TrafficLightGroup & signal);

/**
 * @brief Applies coordinate transforms to the output matrix for all segments.
 *
 * @param transform_matrix Transformation matrix to apply.
 * @param output_matrix Matrix to transform (in-place).
 * @param num_segments Number of segments to transform.
 */
void apply_transforms(
  const Eigen::Matrix4f & transform_matrix, Eigen::MatrixXf & output_matrix, int64_t num_segments);

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

/**
 * @brief Transforms and selects columns from the input matrix based on distances, and adds traffic
 * info.
 *
 * @param input_matrix Input matrix containing lane segment data.
 * @param transform_matrix Transformation matrix to apply to the points.
 * @param distances Vector of columns with distances, used to select columns.
 * @param lanelet_map_ptr Shared pointer to the lanelet map.
 * @param m Maximum number of columns (segments) to select.
 * @return Tuple of the transformed matrix and updated column ID mapping.
 */
std::tuple<Eigen::MatrixXf, ColLaneIDMaps> transform_points_and_add_traffic_info(
  const Eigen::MatrixXf & input_matrix, const Eigen::Matrix4f & transform_matrix,
  const std::vector<ColWithDistance> & distances, const ColLaneIDMaps & col_id_mapping,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, int64_t m);

/**
 * @brief Transforms and selects columns from the input matrix based on proximity to a center point.
 *
 * @param input_matrix Input matrix containing lane segment data.
 * @param transform_matrix Transformation matrix to apply to the points.
 * @param col_id_mapping Mapping between lanelet IDs and matrix columns.
 * @param traffic_light_id_map Map of lanelet IDs to traffic signal information.
 * @param lanelet_map_ptr Shared pointer to the lanelet map.
 * @param center_x X-coordinate of the center point.
 * @param center_y Y-coordinate of the center point.
 * @param m Maximum number of columns (segments) to select.
 * @return Tuple of the transformed matrix and updated column ID mapping.
 * @throws std::invalid_argument If input_matrix dimensions are not correct or if m <= 0.
 */
std::tuple<Eigen::MatrixXf, ColLaneIDMaps> transform_and_select_rows(
  const Eigen::MatrixXf & input_matrix, const Eigen::Matrix4f & transform_matrix,
  const ColLaneIDMaps & col_id_mapping,
  const std::map<lanelet::Id, TrafficSignalStamped> & traffic_light_id_map,
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, const float center_x,
  const float center_y, const int64_t m);

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__LANE_SEGMENTS_HPP_
