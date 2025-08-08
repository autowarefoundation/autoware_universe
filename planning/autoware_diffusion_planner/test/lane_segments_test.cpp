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

#include "lane_segments_test.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/preprocessing/lane_segments.hpp"

#include <Eigen/Dense>

#include <gtest/gtest.h>

#include <algorithm>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace autoware::diffusion_planner::test
{

TEST_F(LaneSegmentsTest, ProcessSegmentToMatrixThrowsOnInvalidInput)
{
  // Empty polyline
  LaneSegment invalid_segment = lane_segments_.front();
  invalid_segment.polyline = Polyline();
  EXPECT_EQ(preprocess::process_segment_to_matrix(invalid_segment).size(), 0);

  // Empty left boundary
  invalid_segment = lane_segments_.front();
  invalid_segment.left_boundaries.clear();
  EXPECT_EQ(preprocess::process_segment_to_matrix(invalid_segment).size(), 0);

  // Empty right boundary
  invalid_segment = lane_segments_.front();
  invalid_segment.right_boundaries.clear();
  EXPECT_EQ(preprocess::process_segment_to_matrix(invalid_segment).size(), 0);

  // Wrong number of points
  invalid_segment = lane_segments_.front();
  auto wrong_polyline = invalid_segment.polyline;
  // Remove a point to make it invalid
  auto points = wrong_polyline.waypoints();
  points.pop_back();
  Polyline short_polyline(MapType::Lane, points);
  invalid_segment.polyline = short_polyline;
  EXPECT_THROW(preprocess::process_segment_to_matrix(invalid_segment), std::runtime_error);
}

TEST_F(LaneSegmentsTest, ProcessSegmentsToMatrixThrowsOnEmptyInput)
{
  std::vector<LaneSegment> empty_segments;
  preprocess::ColLaneIDMaps col_id_mapping;
  EXPECT_THROW(
    preprocess::process_segments_to_matrix(empty_segments, col_id_mapping), std::runtime_error);
}

TEST_F(LaneSegmentsTest, ProcessSegmentsToMatrixThrowsOnWrongRows)
{
  std::vector<LaneSegment> segments = lane_segments_;
  // Patch segment to have wrong number of rows
  LaneSegment bad_segment = segments.front();
  auto polyline = bad_segment.polyline;
  auto points = polyline.waypoints();
  points.pop_back();
  Polyline short_polyline(MapType::Lane, points);
  bad_segment.polyline = short_polyline;
  segments.push_back(bad_segment);

  preprocess::ColLaneIDMaps col_id_mapping;
  EXPECT_THROW(
    preprocess::process_segments_to_matrix(segments, col_id_mapping), std::runtime_error);
}

TEST_F(LaneSegmentsTest, ComputeDistancesThrowsOnBadCols)
{
  preprocess::ColLaneIDMaps col_id_mapping;
  auto input_matrix = preprocess::process_segments_to_matrix(lane_segments_, col_id_mapping);
  // Remove a column to break divisibility
  Eigen::MatrixXf bad_matrix = input_matrix.leftCols(input_matrix.cols() - 1);
  std::vector<preprocess::ColWithDistance> distances;
  EXPECT_THROW(
    preprocess::compute_distances(bad_matrix, Eigen::Matrix4f::Identity(), distances, 0, 0, 100.0),
    std::runtime_error);
}

TEST_F(LaneSegmentsTest, TransformSelectedRowsNoTranslation)
{
  preprocess::ColLaneIDMaps col_id_mapping;
  auto input_matrix = preprocess::process_segments_to_matrix(lane_segments_, col_id_mapping);

  Eigen::MatrixXf matrix = input_matrix;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // Apply a translation
  transform(0, 3) = 5.0f;
  transform(1, 3) = -2.0f;

  // Only dX/dY should not be translated
  preprocess::transform_selected_rows(transform, matrix, 1, dX, false);
  // The values should remain unchanged for dX row
  for (int i = 0; i < matrix.cols(); ++i) {
    EXPECT_FLOAT_EQ(matrix(dX, i), input_matrix(dX, i));
  }
}

TEST_F(LaneSegmentsTest, TransformAndSelectRowsThrowsOnInvalidInput)
{
  preprocess::ColLaneIDMaps col_id_mapping;
  auto input_matrix = preprocess::process_segments_to_matrix(lane_segments_, col_id_mapping);

  // Wrong number of rows
  Eigen::MatrixXf bad_matrix = input_matrix.topRows(input_matrix.rows() - 1);
  std::map<lanelet::Id, preprocess::TrafficSignalStamped> traffic_light_id_map;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr;
  EXPECT_THROW(
    preprocess::transform_and_select_rows(
      bad_matrix, Eigen::Matrix4f::Identity(), col_id_mapping, traffic_light_id_map,
      lanelet_map_ptr, 0, 0, 1),
    std::invalid_argument);

  // m <= 0
  EXPECT_THROW(
    preprocess::transform_and_select_rows(
      input_matrix, Eigen::Matrix4f::Identity(), col_id_mapping, traffic_light_id_map,
      lanelet_map_ptr, 0, 0, 0),
    std::invalid_argument);
}

TEST_F(LaneSegmentsTest, ExtractLaneTensorDataAndSpeedTensorData)
{
  preprocess::ColLaneIDMaps col_id_mapping;
  auto input_matrix = preprocess::process_segments_to_matrix(lane_segments_, col_id_mapping);

  auto lane_tensor = preprocess::extract_lane_tensor_data(input_matrix);
  auto lane_speed_tensor = preprocess::extract_lane_speed_tensor_data(input_matrix);

  EXPECT_EQ(lane_tensor.size(), SEGMENT_POINT_DIM * LANES_SHAPE[1] * POINTS_PER_SEGMENT);
  EXPECT_EQ(lane_speed_tensor.size(), LANES_SPEED_LIMIT_SHAPE[2] * LANES_SPEED_LIMIT_SHAPE[1]);
}

}  // namespace autoware::diffusion_planner::test
