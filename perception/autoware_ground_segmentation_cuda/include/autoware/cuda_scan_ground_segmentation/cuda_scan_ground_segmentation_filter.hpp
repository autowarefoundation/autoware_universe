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

#include "cuda_mempool_wrapper.hpp"
#include "cuda_point_cloud2.hpp"
#include "cuda_stream_wrapper.hpp"

#include <autoware/cuda_pointcloud_preprocessor/point_types.hpp>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <iostream>
#include <memory>

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

struct alignas(16) ClassifiedPointType
{
  float z;
  PointType type;
  float radius;
  size_t origin_index;  // index in the original point cloud

  CUDAH ClassifiedPointType() : z(0.0), type(PointType::INIT), radius(-1.0), origin_index(0) {}
};

struct alignas(16) Cell
{
  float gnd_radius_avg;
  float gnd_height_avg;
  float gnd_height_min;
  uint32_t num_ground_points;
  // initialize constructor

  CUDAH Cell()
  : gnd_radius_avg(0.0f), gnd_height_avg(0.0f), gnd_height_min(0.0f), num_ground_points(0)
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
  void classifyPointCloud(
    const sensor_msgs::msg::PointCloud2 & input_points,
    sensor_msgs::msg::PointCloud2 & output_points, sensor_msgs::msg::PointCloud2 & ground_points);

private:
  std::unique_ptr<cuda::PointCloud2> dev_input_points_, dev_output_points_, dev_ground_points_;
  std::unique_ptr<device_vector<int>> empty_cell_mark_;

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
  // Remove points too far from the origin
  void removeOutliers();

  /*
   * This function calc the cell_id for each point
   * Assign the point with initialized class into temp memory for classification
   * Memory size of each cell is depend on predefined cell point num
   *
   */

  void scanPerSectorGroundReference(
    device_vector<Cell> & cell_list, device_vector<int> & starting_pid,
    device_vector<ClassifiedPointType> & classified_points);

  /*
   * Extract obstacle points from classified_points_dev into
   */
  template <typename CheckerType>
  void extractPoints(
    device_vector<ClassifiedPointType> & classified_points, cuda::PointCloud2 & input,
    cuda::PointCloud2 & output);

  // Distribute points to cells
  void sort_points(
    device_vector<Cell> & cell_list, device_vector<int> & starting_pid,
    device_vector<ClassifiedPointType> & classified_points);

  std::shared_ptr<CudaStream> stream_;
  std::shared_ptr<CudaMempool> mempool_;
};

}  // namespace autoware::cuda_ground_segmentation

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_SCAN_GROUND_SEGMENTATION_FILTER_HPP_
