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

#include "autoware/cuda_scan_ground_segmentation/cuda_scan_ground_segmentation_filter.hpp"

#include <autoware/cuda_scan_ground_segmentation/cuda_common.hpp>
#include <autoware/cuda_scan_ground_segmentation/cuda_utility.cuh>
#include <cub/cub.cuh>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

namespace autoware::cuda_ground_segmentation
{

  __device__ __forceinline__ float fastAtan2_0_2Pi(float y, float x)
{
  return fmodf(atan2(y, x) + 2.0f * M_PI, 2.0f * M_PI);
}

// Initialize the cell list
__global__ void cellInit(Cell * __restrict__ cell_list, int max_num_cells)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;
  Cell new_cell;

  for (int i = index; i < max_num_cells; i += stride) {
    new_cell.gnd_height_min = 1e6f;

    cell_list[i] = new_cell;
  }
}

/**
 * @brief CUDA kernel to count the number of points in each grid cell for ground segmentation.
 *
 * This kernel processes each input point, computes its polar coordinates (radius and angle)
 * relative to a specified center, and determines which cell in a polar grid the point belongs to.
 * It then atomically increments the point count for the corresponding cell.
 *
 * @param input_points Pointer to the array of input points (device memory).
 * @param num_input_points Number of input points in the array.
 * @param filter_parameters_dev Pointer to filter parameters structure (device memory), containing
 *        grid and segmentation configuration such as center coordinates, sector angle, and cell
 * size.
 * @param centroid_cells_list_dev Pointer to the array of cell centroid structures (device memory),
 *        where each cell maintains a count of points assigned to it.
 *
 * @note Each thread processes one point. Atomic operations are used to safely increment the
 *       point count in each cell when multiple threads access the same cell concurrently.
 * @note Points that fall outside the defined grid (cell_id >= max_num_cells) are ignored.
 */
__global__ void computeCellId(
  const cuda::PointCloud2::Ptr input_points, FilterParameters param, int * cell_id, int * count,
  ClassifiedPointType * classified_points)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;
  ClassifiedPointType cp;

  cp.type = PointType::INIT;

  for (int i = index; i < input_points.size(); i += stride) {
    auto p = input_points[i];
    float dx = p.x - param.center_x;
    float dy = p.y - param.center_y;
    float radius = hypotf(dx, dy);
    float angle = fastAtan2_0_2Pi(dy, dx);
    int sector_id = static_cast<int>(angle * param.inv_sector_angle_rad);
    int cell_id_in_sector = static_cast<int>(radius / param.cell_divider_size_m);
    int point_cell_id = cell_id_in_sector * param.num_sectors + sector_id;

    // Save this so we don't have to recalculate later
    cell_id[i] = point_cell_id;

    // ALso, initialize classified points
    cp.z = p.z;
    cp.radius = radius;
    cp.origin_index = i;

    classified_points[i] = cp;

    // Also update the number of points in each cell atomically
    atomicAdd(count + point_cell_id, 1);
  }
}

__global__ void distributePointsToCell(
  ClassifiedPointType * input, ClassifiedPointType * output, int * cell_id, int point_num,
  int * writing_loc)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    int cid = cell_id[i];
    int wloc = atomicAdd(writing_loc + cid, 1);

    output[wloc] = input[i];
  }
}

CudaScanGroundSegmentationFilter::CudaScanGroundSegmentationFilter(
  const FilterParameters & filter_parameters, const int64_t max_mem_pool_size_in_byte)
: filter_parameters_(filter_parameters)
{
  stream_.reset(new CudaStream());

  int current_device_id = 0;

  CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
  mempool_.reset(new CudaMempool(current_device_id, max_mem_pool_size_in_byte));

  dev_input_points_.reset(new cuda::PointCloud2(stream_, mempool_));
  dev_output_points_.reset(new cuda::PointCloud2(stream_, mempool_));
  dev_ground_points_.reset(new cuda::PointCloud2(stream_, mempool_));
}

// ============= Sort points in each cell by radius =============
void CudaScanGroundSegmentationFilter::sortPointsInCells(
  const uint32_t * num_points_per_cell_dev, ClassifiedPointTypeStruct * classified_points_dev)
{
  (void)num_points_per_cell_dev;
  (void)classified_points_dev;
}

// ============ Scan per sector to get ground reference and Non-Ground points =============
void CudaScanGroundSegmentationFilter::scanPerSectorGroundReference(
  device_vector<Cell> & cell_list,
  device_vector<int> & starting_pid,
  device_vector<ClassifiedPointType> & classified_points
)
{
  const uint32_t num_sectors = filter_parameters_.num_sectors;
  if (num_sectors == 0) {
    return;
  }

  CHECK_CUDA_ERROR(cuda::launchAsync<BLOCK_DIM_X>(
    (int)(num_sectors * WARP_SIZE), 
    (BLOCK_DIM_X / WARP_SIZE) * sizeof(Cell) * filter_parameters_.gnd_cell_buffer_size,
    stream_->get(),
    cell_list.data(), 
    classifeid_points.data(),
    starting_pid.data(),
    filter_parameters_
  ));
}

__forceinline__ __device__ SegmentationMode checkSegmentMode(
  int cell_id, int closest_gnd_cell_id, int furthest_gnd_cell_id, int gnd_cell_buffer_size,
  int gnd_cell_continual_threshold)
{
  if (closest_gnd_cell_id < 0) {
    return SegmentationMode::UNINITIALIZED;
  }

  if (cell_id - closest_gnd_cell_id >= gnd_cell_continual_threshold) {
    return SegmentationMode::BREAK;
  }

  if (cell_id - furthest_gnd_cell_id <= gnd_cell_buffer_size) {
    return SegmentationMode::CONTINUOUS;
  }

  return SegmentationMode::DISCONTINUOUS;
}

__forceinline__ __device__ void segmentUninitializedPoint(
  ClassifiedPointType & p, const FilterParameters & param)
{
  if (p.z > param.detection_range_z_max || p.z < param.non_ground_height_threshold) {
    p.type = PointType::OUT_OF_RANGE;
    return;
  }

  float global_height_th = p.radius * param.global_slope_max_ratio;

  if (p.z > global_height_th && p.z > param.non_ground_height_threshold) {
    p.type = PointType::NON_GROUND;
    return;
  }

  float abs_pz = abs(p.z);

  if (abs_pz < global_height_th && abs_pz < param.non_ground_height_threshold) {
    p.type = PointType::GROUND;
  }
}

__forceinline__ __device__ void segmentContinuousPoint(
  ClassifiedPointType & p, const FilterParameters & param, float slope,
  float prev_cell_gnd_height_avg, float prev_cell_gnd_radius_avg)
{
  if (p.z - prev_cell_gnd_height_avg > param.detection_range_z_max) {
    p.type = PointType::OUT_OF_RANGE;
    return;
  }

  float d_radius = p.radius - prev_cell_gnd_radius_avg + param.cell_divider_size_m;
  float dz = p.z - prev_cell_gnd_height_avg;

  if (p.z > param.global_slope_max_ratio * p.radius) {
    p.type = PointType::NON_GROUND;
    return;
  }

  if (dz > param.local_slope_max_ratio * d_radius) {
    p.type = PointType::NON_GROUND;
    return;
  }

  float estimated_ground_z = prev_cell_gnd_height_avg + slope * param.cell_divider_size_m;

  if (p.z > estimated_ground_z + param.non_ground_height_threshold) {
    p.type = PointType::NON_GROUND;
    return;
  }

  if (
    p.z < estimated_ground_z - param.non_ground_height_threshold ||
    dz < -param.local_slope_max_ratio * d_radius ||
    p.z < -param.global_slope_max_ratio * p.radius) {
    p.type = PointType::OUT_OF_RANGE;
    return;
  }

  p.type = PointType::GROUND;
}

__forceinline__ __device__ void segmentDiscontinuousPoint(
  ClassifiedPointType & p, const FilterParameters & param, float prev_cell_gnd_height_avg,
  float prev_cell_gnd_radius_avg)
{
  if (p.z - prev_cell_gnd_height_avg > param.detection_range_z_max) {
    p.type = PointType::OUT_OF_RANGE;
    return;
  }

  float dz = p.z - prev_cell_gnd_height_avg;
  float d_radius = p.radius - prev_cell_gnd_radius_avg + param.cell_divider_size_m;
  float global_height_threshold = p.radius * param.global_slope_max_ratio;
  float local_height_threshold = param.local_slope_max_ratio * d_radius;

  if (p.z > global_height_threshold || dz > local_height_threshold) {
    p.type = PointType::NON_GROUND;
    return;
  }

  if (dz < -local_height_threshold || p.z < -global_height_threshold) {
    p.type = PointType::OUT_OF_RANGE;
    return;
  }

  p.type = PointType::GROUND;
}

__forceinline__ __device__ void segmentBreakPoint(
  ClassifiedPointType & p, const FilterParameters & param, float prev_cell_gnd_height_avg,
  float prev_cell_gnd_radius_avg)
{
  if (p.z - prev_cell_gnd_height_avg > param.detection_range_z_max) {
    p.type = PointType::OUT_OF_RANGE;
    return;
  }

  float dz = p.z - prev_cell_gnd_height_avg;
  float d_radius = p.radius - prev_cell_gnd_radius_avg;
  float global_height_threshold = d_radius * param.global_slope_max_ratio;

  p.type = (dz > global_height_threshold)
             ? PointType::NON_GROUND
             : ((dz < -global_height_threshold) ? PointType::OUT_OF_RANGE : PointType::GROUND);
}

__forceinline__ __device__ void segmentCell(
  const int wid,  // Index of the thread in the warp
  ClassifiedPointType * classified_points, 
  const FilterParameters & param, 
  Cell & cell, float slope,     // Slope of the line connect the previous ground cells
  int start_pid, int end_pid,  // Start and end indices of points in the current cell
  float prev_cell_gnd_radius_avg, float prev_cell_gnd_height_avg, const SegmentationMode & mode)
{
  // Here I assume each thread handles no more than 64 points
  int64_t ground_mask = 0, recheck_mask = 0, backup_mask = 0;
  // For computing the cell statistic
  float t_gnd_radius_sum = 0;
  float t_gnd_height_sum = 0;
  float t_gnd_height_min = FLT_MAX;
  int t_gnd_point_num = 0;
  // For recheck
  float minus_t_gnd_radius_sum = 0;
  float minus_t_gnd_height_sum = 0;
  float minus_t_gnd_point_num = 0;
  ClassifiedPointType p;

  // Fit line from the recent ground cells
  for (int j = start_pid + wid; j < end_pid; j += WARP_SIZE) {
    p = classified_points[j];

    switch (mode) {
      case (SegmentationMode::UNINITIALIZED): {
        segmentUninitializedPoint(p, param);
        break;
      }
      case (SegmentationMode::CONTINUOUS): {
        segmentContinuousPoint(p, param, slope, prev_cell_gnd_height_avg, prev_cell_gnd_radius_avg);
        break;
      }
      case (SegmentationMode::DISCONTINUOUS): {
        segmentDiscontinuousPoint(p, param, prev_cell_gnd_height_avg, prev_cell_gnd_radius_avg);
        break;
      }
      case (SegmentationMode::BREAK): {
        segmentBreakPoint(p, param, prev_cell_gnd_height_avg, prev_cell_gnd_radius_avg);
        break;
      }
    }

    if (p.type == PointType::GROUND) {
      t_gnd_radius_sum += p.radius;
      t_gnd_height_sum += p.z;
      t_gnd_point_num += 1;
      t_gnd_height_min = (t_gnd_height_min > p.z) ? p.z : t_gnd_height_min;

      // Set the corresponding bit to 1
      // In the recheck we don't have to check every point, just jump to the ground point
      ground_mask |= 1 << ((j - start_pid - wid) / WARP_SIZE);
    }
  }

  // Wait for all threads in the warp to finish
  __syncwarp();
  // Find the min height and the number of ground points first

  // Use reduction to compute the cell's stat
  for (int offset = WARP_SIZE >> 1; offset > 0; offset >> 1) {
    t_gnd_height_sum += __shfl_down_sync(FULL_MASK, t_gnd_height_sum, offset);
    t_gnd_radius_sum += __shfl_down_sync(FULL_MASK, t_gnd_radius_sum, offset);
    t_gnd_point_num += __shfl_down_sync(FULL_MASK, t_gnd_point_num, offset);

    float other_height_min = __shfl_down_sync(FULL_MASK, t_gnd_height_min, offset);
    t_gnd_height_min = min(t_gnd_height_min, other_height_min);
  }

  // Now broadcast the min height and the number of ground points to all threads
  float cell_gnd_height_min = __shfl_sync(FULL_MASK, t_gnd_height_min, 0);
  int cell_gnd_point_num = __shfl_sync(FULL_MASK, t_gnd_point_num, 0);

  // This is to record the remaining ground point in each thread
  // After looping, ground_mask will be cleared, so we need to back it up
  recheck_mask = backup_mask = ground_mask;   

  if (param.use_recheck_ground_cluster && cell_gnd_point_num > 1 &&
      t_gnd_radius_sum / (float)(t_gnd_point_num) > param.recheck_start_distance) {

    // Now recheck the points using the height_min
    // This is to record the remaining ground point in each thread
    recheck_mask = ground_mask; 
    // After looping, ground_mask will be cleared, so we need to back it up
    backup_mask = ground_mask;

    while (ground_mask != 0) {
      // Get the offset of the ground point
      int pos = __ffs(ground_mask) - 1;
      int gnd_pid = pos * WARP_SIZE  + start_pid + wid;
      
      // Clear the bit
      ground_mask &= (ground_mask - 1);
      p = classified_points[gnd_pid];

      if (p.z > cell_gnd_height_min + param.non_ground_height_threshold && 
            cell_gnd_point_num > 1) {
        minus_t_gnd_height_sum += p.z;
        minus_t_gnd_radius_sum += p.radius;
        ++minus_t_gnd_point_num;

        // If the point is no longer ground, clear the bit
        recheck_mask &= (recheck_mask - 1);
      }
    }

    // Now use ballot sync to see if there are any ground points remaining
    uint32_t recheck_res = __ballot_sync(FULL_MASK, recheck_mask > 0);
    uint32_t backup_res = __ballot_sync(FULL_MASK, backup_mask > 0);

    // If no ground point remains, we have to keep the last ground point 
    if (recheck_res == 0 && backup_res > 0) {
      // Get the index of the last thread that detects ground point
      int last_tid = 32 - __ffs(backup_res);

      // Only the last point in that thread remain
      if (wid == last_tid) {
        // Luckily, the last point still remains
        minus_t_gnd_height_sum -= p.z;
        minus_t_gnd_radius_sum -= p.radius;
        --minus_t_gnd_point_num;
        recheck_mask |= (1 << (31 - __clz(backup_res)));
      }
    }

    __syncwarp();

    // Final update 
    for (int offset = WARP_SIZE >> 1; offset > 0; offset >> 1) {
      minus_t_gnd_height_sum += __shfl_down_sync(FULL_MASK, minus_t_gnd_height_sum, offset);
      minus_t_gnd_radius_sum += __shfl_down_sync(FULL_MASK, minus_t_gnd_radius_sum, offset);
      minus_t_gnd_point_num += __shfl_down_sync(FULL_MASK, minus_t_gnd_point_num, offset);
    }

    if (wid == 0) {
      t_gnd_height_sum -= minus_t_gnd_height_sum;
      t_gnd_radius_sum -= minus_t_gnd_radius_sum;
      t_gnd_point_sum -= minus_t_gnd_point_sum;
    }
  }

  // Now update the point type in the global memory
  while (recheck_mask != 0) {
    // Get the offset of the ground point
    int pos = __ffs(recheck_mask) - 1;
    int gnd_pid = pos * WARP_SIZE  + start_pid + wid;
   
    classified_points[gnd_pid].type = PointType::GROUND;
    // Clear the bit
    recheck_mask &= (recheck_mask - 1);
  }

  // Finally, thread 0 update the cell stat
  if (wid == 0) {
    cell.gnd_radius_avg = t_gnd_radius_sum / (float)(t_gnd_point_num);
    cell.gnd_height_avg = t_gnd_height_sum / (float)(t_gnd_point_num);
    cell.gnd_height_min = t_gnd_height_min;
    cell.num_ground_points = t_gnd_point_num;
  }
}

__global__ void sectorProcessingKernel(
  Cell * __restrict__ cell_list, ClassifiedPointType * __restrict__ classied_points,
  int * starting_pid, FilterParameters param)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  // Each warp handles one sector
  int sector_stride = (blockDim.x * gridDim.x) / WARP_SIZE;
  int wid = index % WARP_SIZE;  // Index of the thread within the warp
  // Shared memory to backup the cell data
  extern __shared__ Cell gnd_cell_queue[];
  const int queue_id = (threadIdx.x / WARP_SIZE) * param.gnd_cell_buffer_size;

  // Loop on sectors
  for (int sector_id = index / WARP_SIZE; sector_id < param.num_sector; sector_id += sector_stride ) {
    // For storing the previous ground cells
    int closest_gnd_cell_id, furthest_gnd_cell_id, num_latest_gnd_cells = 0;
    int head = 0, tail = 0;
    float sum_x, sum_y, sum_xx, sum_xy;

    // Initially no ground cell is identified
    closest_gnd_cell_id = furthest_gnd_cell_id = -1;
    sum_x = sum_y = sum_xx = sum_xy = 0.0;

    // Loop on the cells in a sector
    for (int i = 0; i < param.max_num_cells_per_sector; ++i) {
      auto mode = checkSegmentMode(
        i, closest_gnd_cell_id, furthest_gnd_cell_id, 
        param.gnd_cell_buffer_size,
        param.gnd_grid_continual_thresh);

      int global_cell_id = i * param.num_sector + sector_id;
      Cell cell;

      // Classify the points in the cell
      segmentCell(
        wid, classified_points, slope, param, cell, 
        starting_pid[global_cell_id], starting_pid[global_cell_id + 1], 
        prev_cell_gnd_radius_avg,
        prev_cell_gnd_height_avg, mode
      );

      // Update the indices of the previous ground cells if the cell contains ground points
      if (wid == 0 && cell.num_ground_points > 0) {
        if (num_latest_gnd_cells >= param.gnd_cell_buffer_size) {
          // If the number of previous ground cell reach maximum, drop the first one
          Cell head_cell = gnd_cell_queue[head++];
          head = (head == param.gnd_cell_buffer_size) ? 0 : head;
          --num_latest_gnd_cells;
          sum_x -= head_cell.gnd_radius_avg;
          sum_y -= head_cell.gnd_height_avg;
          sum_xx -= head_cell.gnd_radius_avg * head_cell.gnd_radius_avg;
          sum_xy -= head_cell.gnd_radius_avg * head_cell.gnd_height_avg;
        } 

        // Otherwise, add the new ground cell to the queue
        gnd_cell_queue[tail++] = cell;
        tail = (tail == param.gnd_cell_buffer_size) ? 0 : tail; 
        ++num_latest_gnd_cells;
        // Update the stats
        closest_gnd_cell_id = i;
        sum_x += cell.gnd_radius_avg;
        sum_y += cell.gnd_height_avg;
        sum_xx += cell.gnd_radius_avg * cell.gnd_radius_avg;
        sum_xy += cell.gnd_radius_avg * cell.gnd_height_avg;

        float denom = (num_latest_gnd_cells * sum_xx - sum_x * sum_x);

        if (fbasf(denom) < 1e-6f) {
          Cell head_cell = cell_queue[queue_id + head];
          slope = head_cell.gnd_height_avg / head_cell.gnd_radius_avg;
        } else {
          slope = (num_latest_gnd_cells * sum_xy - sum_x * sum_y) / denom;
          slope = fmax(fminf(slope, param.global_slope_max_ratio), -param.global_slope_max_ratio);
        }

        // Write the cell to the global memory
        cell_list[global_cell_id] = cell;
      }
      __syncwarp();
    }
  }
}

void CudaScanGroundSegmentationFilter::sort_points(
  device_vector<Cell> & cell_list, device_vector<int> & starting_pid,
  device_vector<ClassifiedPointType> & classified_points)
{
  if (dev_input_points_.empty() || filter_parameters_.max_num_cells == 0) {
    return;
  }

  int point_num = dev_input_points_.size();

  cell_list.resize(filter_parameters_.max_num_cells);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      (int)(cell_list.size()), 0, stream_->get(), cellInit, cell_list.data(),
      (int)(cell_list.size())));

  starting_pid.resize(point_num + 1);

  device_vector<int> cell_id(point_num, stream_, mempool_);
  device_vector<ClassifiedPointType> tmp_classified_points(point_num, stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::fill(starting_pid, 0));
  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), computeCellId, dev_input_points_.data(), filter_parameters_,
      cell_id.data(), starting_pid.data(), tmp_classified_points.data()));

  CHECK_CUDA_ERROR(cuda::ExclusiveScan(starting_pid));

  device_vector<int> writing_loc(starting_pid, stream_, mempool_);

  classified_points.resize(point_num);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), distributePointsToCell, tmp_classified_points.data(),
      classified_points.data(), cell_id.data(), point_num, writing_loc.data()));
}


struct NonGroundChecker {
  CUDAH bool operator()(const ClassifiedPointType & p) const {
    return (p.type == PointType::NON_GROUND);
  }
};

struct GroundChecker {
  CUDAH bool operator()(const ClassifiedPointType & p) const {
    return (p.type == PointType::GROUND);
  }
};

template <typename CheckerType>
__global__ void markingPoints(
  ClassifiedPointType * classified_points, int point_num, int * mark,
  CheckerType checker
)
)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    auto p = classified_points[i];

    mark[p.original_index] = (checker(p)) ? 1 : 0;
  }
}

__global__ void extract(
  cuda::PointCloud2::Ptr dev_input_points, int point_num, int * writing_loc,
  cuda::PointCloud2::Ptr dev_output_points
)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    int2 wloc = *(int2*)(writing_loc + i);

    if (wloc.x < wloc.y) {
      dev_output_points[wloc.x] = dev_input_points[i];
    }
  }
}

// ============= Extract non-ground points =============
template <typename CheckerType>
void CudaScanGroundSegmentationFilter::extractPoints(
  device_vector<ClassifiedPointType> & classified_points,
  cuda::PointCloud2 & input,
  cuda::PointCloud2 & output
)
{
  int point_num = dev_input_points_->size();
  device_vector<int> point_mark(point_num + 1, stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::fill(non_ground_mark, 0));

  // Mark non-ground points
  CHECK_CUDA_ERROR(cuda::launchAsync<BLOCK_DIM_X>(
    point_num, 0, stream_->get(),
    markingPoints,
    classified_points.data(),
    point_num,
    point_mark.data(),
    CheckerType()
  ));

  // Exclusive scan
  device_vector<int> writing_loc(stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::ExclusiveScan(point_mark, writing_loc));

  // Reserve the output
  int output_size = writing_loc[point_num];

  if (output_size <= 0) {
    return;
  }

  output.resize(output_size);

  // Get non-ground 
  CHECK_CUDA_ERROR(cuda::launchAsync<BLOCK_DIM_X>(
    point_num, 0, stream_->get(),
    extract,
    input.data(),
    point_num,
    writing_loc.data(),
    output.data()
  ));
}

// ============= Extract non-ground points =============
void CudaScanGroundSegmentationFilter::extractGroundPoints(
  device_vector<ClassifiedPointType> & classified_points
)
{
  int point_num = dev_input_points_->size();
  device_vector<int> non_ground_mark(point_num + 1, stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::fill(non_ground_mark, 0));

  // Mark non-ground points
  CHECK_CUDA_ERROR(cuda::launchAsync<BLOCK_DIM_X>(
    point_num, 0, stream_->get(),
    markNonGroundPoints,
    classified_points.data(),
    point_num,
    non_ground_mark.data()
  ));

  // Exclusive scan
  device_vector<int> writing_loc(stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::ExclusiveScan(non_ground_mark, writing_loc));

  // Reserve the output
  int non_ground_point_num = writing_loc[point_num];

  if (non_ground_point_num <= 0) {
    return;
  }

  dev_output_points_.reset(new cuda::PointCloud2(non_ground_point_num, stream_, mempool_));

  // Get non-ground 
  CHECK_CUDA_ERROR(cuda::launchAsync<BLOCK_DIM_X>(
    point_num, 0, stream_->get(),
    getNonGroundPoints,
    dev_input_points_->data(),
    point_num,
    writing_loc.data(),
    dev_output_points_->data()
  ));
}

void CudaScanGroundSegmentationFilter::classifyPointCloud(
  sensor_msgs::msg::PointCloud2 & input_points, sensor_msgs::msg::PointCloud2 & output_points,
  sensor_msgs::msg::PointCloud2 & ground_points)
{
  dev_input_points_.from_point_cloud2(input_points);

  device_vector<Cell> cell_list(stream_, mempool_);
  device_vector<int> starting_pid(stream_, mempool_);
  device_vector<ClassifiedPointType> classified_points(stream_, mempool_);

  sort_points(cell_list, starting_pid, classified_points);
  scanPerSectorGroundReference(cell_list, starting_pid, classified_points);
  // Extract non-ground points
  e(classified_points);

  dev_output_points_.to_point_cloud2(output_points);
}

}  // namespace autoware::cuda_ground_segmentation
