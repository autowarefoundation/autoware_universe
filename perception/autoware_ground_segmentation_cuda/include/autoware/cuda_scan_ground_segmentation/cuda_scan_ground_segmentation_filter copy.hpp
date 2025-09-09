// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_

#include <autoware/cuda_pointcloud_preprocessor/point_types.hpp>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <iostream>
#include <memory>

#include "cuda_mempool_wrapper.hpp"
#include "cuda_stream_wrapper.hpp"

namespace autoware::cuda_ground_segmentation
{

enum SegmentationMode : uint8_t { UNINITIALIZED = 0, CONTINUOUS, DISCONTINUOUS, BREAK };

struct PointTypeStruct
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
};

enum class PointType : uint8_t {
  INIT = 0,
  GROUND,
  NON_GROUND,
  POINT_FOLLOW,
  UNKNOWN,
  VIRTUAL_GROUND,
  OUT_OF_RANGE
};

struct ClassifiedPointTypeStruct
{
  float z;
  PointType type;
  float radius;
  size_t origin_index;  // index in the original point cloud

  ClassifiedPointTypeStruct() : z(0.0), type(PointType::INIT), radius(-1.0), origin_index(0) {}
};

struct CellCentroid
{
  uint32_t cell_id;  // cell_id = sector_id * number_cells_per_sector + grid_index
  uint32_t sector_id;
  uint32_t cell_id_in_sector;
  uint32_t num_points;
  size_t start_point_index;  // start index of points in classified_points_dev
  float gnd_radius_avg;
  float gnd_height_avg;
  float gnd_height_min;
  uint32_t num_ground_points;
  // initialize constructor
  CellCentroid()
  : cell_id(0),
    sector_id(0),
    cell_id_in_sector(0),
    num_points(0),
    start_point_index(0),
    gnd_radius_avg(0.0f),
    gnd_height_avg(0.0f),
    gnd_height_min(0.0f),
    num_ground_points(0)
  {
  }
};

// structure to hold parameter values
struct FilterParameters
{
  float max_radius;
  float non_ground_height_threshold;
  // common thresholds
  float global_slope_max_angle_rad;  // radians
  float local_slope_max_angle_rad;   // radians
  float global_slope_max_ratio;
  float local_slope_max_ratio;
  float split_points_distance_tolerance;  // distance in meters between concentric divisions
  // common parameters
  float sector_angle_rad;  // radial sector angle in radians
  float inv_sector_angle_rad;

  // cell mode parameters
  float recheck_start_distance;  // distance to start rechecking ground cluster
  float detection_range_z_max;
  // cell parameters
  float cell_divider_size_m;
  float center_x{0.0f};
  float center_y{0.0f};
  uint32_t max_num_cells;
  uint32_t max_num_cells_per_sector;      // number of cells per sector
  uint32_t num_sectors;                   // number of radial sectors
  uint32_t gnd_grid_continual_thresh{3};  // threshold for continual ground grid
  uint32_t use_recheck_ground_cluster;    // to enable recheck ground cluster
  const uint32_t gnd_cell_buffer_size{5};
};

class CudaScanGroundSegmentationFilter
{
public:
  explicit CudaScanGroundSegmentationFilter(
    const FilterParameters & filter_parameters, const int64_t max_mem_pool_size_in_byte);
  ~CudaScanGroundSegmentationFilter() = default;

  // Method to process the point cloud data and filter ground points
  void classifyPointcloud(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    cuda_blackboard::CudaPointCloud2::SharedPtr output_points,
    cuda_blackboard::CudaPointCloud2::SharedPtr ground_points);

  uint32_t number_input_points_;
  uint32_t num_process_points_host_;
  uint32_t input_pointcloud_step_;
  uint32_t input_xyzi_offset_[4];
  float center_x_{0.0f};
  float center_y_{0.0f};
  const uint32_t gnd_grid_continual_thresh_{3};  // threshold for continual ground grid
  const uint32_t continual_gnd_grid_thresh_{5};  // threshold for continual ground grid with recheck
  // Parameters
  FilterParameters filter_parameters_;

private:
  // Internal methods for ground segmentation logic

  template <typename T>
  T * allocateBufferFromPool(size_t num_elements);

  template <typename T>
  void returnBufferToPool(T * buffer);
  void scanObstaclePoints(
    const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
    PointTypeStruct * output_points_dev, size_t * num_output_points,
    CellCentroid * centroid_cells_list_dev);
  /*
   * This function calc the cell_id for each point
   * Assign the point with initialized class into temp memory for classification
   * Memory size of each cell is depend on predefined cell point num
   *
   */
  void assignPointToClassifyPoint(
    const PointTypeStruct * input_points_dev, const CellCentroid * centroid_cells_list_dev,
    const FilterParameters * filter_parameters_dev, uint32_t * cell_counts_dev,
    ClassifiedPointTypeStruct * classified_points_dev);

  void getCellFirstPointIndex(
    CellCentroid * centroid_cells_list_dev, uint32_t * num_points_per_cell_dev,
    size_t * cell_first_point_indices_dev);

  void sortPointsInCells(
    const uint32_t * num_points_per_cell_dev, ClassifiedPointTypeStruct * classified_points_dev);
  void scanPerSectorGroundReference(
    ClassifiedPointTypeStruct * classified_points_dev, CellCentroid * centroid_cells_list_dev,
    const FilterParameters * filter_parameters_dev);

  /*
   * Extract obstacle points from classified_points_dev into
   */
  void extractNonGroundPoints(
    const PointTypeStruct * input_points_dev, ClassifiedPointTypeStruct * classified_points_dev,
    PointTypeStruct * output_points_dev, uint32_t & num_output_points_host,
    const PointType pointtype);

  void countCellPointNum(
    const PointTypeStruct * input_points_dev, CellCentroid * indices_list_dev,
    const FilterParameters * filter_parameters_dev);

  std::shared_ptr<CudaStream> ground_segment_stream_;
  std::shared_ptr<CudaMempool> mempool_;
};
}  // namespace autoware::cuda_ground_segmentation

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_
