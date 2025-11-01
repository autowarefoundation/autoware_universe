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

#include <sys/time.h>

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
  const PointTypeStruct * __restrict__ input_points, int point_num, FilterParameters param,
  int * __restrict__ cell_id, int * __restrict__ count,
  ClassifiedPointType * __restrict__ classified_points)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;
  ClassifiedPointType cp;

  cp.type = PointType::INIT;

  for (int i = index; i < point_num; i += stride) {
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

  // Pre-allocate the PointCloud2 objects
  // The CudaPointCloud2 still use sync cudaMalloc to allocate memory
  // Those calls are costly, so it should be done only once.
  dev_input_points_.reset(new cuda_blackboard::CudaPointCloud2);

  // Warm-up the memory pool a bit
  empty_cell_mark_.reset(new device_vector<int>(100000, stream_, mempool_));
}

__forceinline__ __device__ SegmentationMode checkSegmentationMode(
  int cell_id, int closest_gnd_cell_id, int furthest_gnd_cell_id, int gnd_cell_buffer_size,
  int gnd_cell_continual_threshold, int empty_cell_num)
{
  if (closest_gnd_cell_id < 0) {
    return SegmentationMode::UNINITIALIZED;
  }

  if (cell_id - closest_gnd_cell_id >= gnd_cell_continual_threshold) {
    return SegmentationMode::BREAK;
  }

  if (cell_id - furthest_gnd_cell_id - empty_cell_num <= gnd_cell_buffer_size) {
    return SegmentationMode::CONTINUOUS;
  }

  return SegmentationMode::DISCONTINUOUS;
}

__forceinline__ __device__ void segmentUninitializedPoint(
  ClassifiedPointType & p, const FilterParameters & param)
{
  if (p.z > param.detection_range_z_max || p.z < -param.non_ground_height_threshold) {
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
    p.z < estimated_ground_z -
            param.non_ground_height_threshold ||  // p.z < estimated_ground_z -
                                                  // param.non_ground_height_threshold
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
  // float d_radius = p.radius - prev_cell_gnd_radius_avg + param.cell_divider_size_m;
  float d_radius = p.radius - prev_cell_gnd_radius_avg;
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
  ClassifiedPointType * classified_points, const FilterParameters & param, Cell & cell,
  float slope,                 // Slope of the line connect the previous ground cells
  int start_pid, int end_pid,  // Start and end indices of points in the current cell
  float prev_cell_gnd_radius_avg, float prev_cell_gnd_height_avg, const SegmentationMode & mode)
{
  if (start_pid >= end_pid) {
    return;
  }

  // For computing the cell statistic
  float t_gnd_radius_sum = 0;
  float t_gnd_height_sum = 0;
  float t_gnd_height_min = FLT_MAX;
  int t_gnd_point_num = 0;
  // For recheck
  float minus_t_gnd_radius_sum = 0;
  float minus_t_gnd_height_sum = 0;
  float minus_t_gnd_point_num = 0;
  ClassifiedPointType last_gnd_point, p;
  int last_gnd_idx;
  int local_gnd_point_num;

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
      ++t_gnd_point_num;
      t_gnd_height_min = (t_gnd_height_min > p.z) ? p.z : t_gnd_height_min;
    }

    classified_points[j] = p;
  }

  // Wait for all threads in the warp to finish
  local_gnd_point_num = t_gnd_point_num;
  __syncwarp();

  // Find the min height and the number of ground points first

  // Use reduction to compute the cell's stat
  for (int offset = WARP_SIZE >> 1; offset > 0; offset >>= 1) {
    t_gnd_height_sum += __shfl_down_sync(FULL_MASK, t_gnd_height_sum, offset);
    t_gnd_radius_sum += __shfl_down_sync(FULL_MASK, t_gnd_radius_sum, offset);
    t_gnd_point_num += __shfl_down_sync(FULL_MASK, t_gnd_point_num, offset);

    float other_height_min = __shfl_down_sync(FULL_MASK, t_gnd_height_min, offset);
    t_gnd_height_min = min(t_gnd_height_min, other_height_min);
  }

  // Now broadcast the min height and the number of ground points to all threads
  float cell_gnd_height_min = __shfl_sync(FULL_MASK, t_gnd_height_min, 0);
  int cell_gnd_point_num = __shfl_sync(FULL_MASK, t_gnd_point_num, 0);
  float cell_gnd_radius_sum = __shfl_sync(FULL_MASK, t_gnd_radius_sum, 0);

  if (
    param.use_recheck_ground_cluster && cell_gnd_point_num > 1 &&
    cell_gnd_radius_sum / (float)(cell_gnd_point_num) > param.recheck_start_distance) {
    // Now recheck the points using the height_min
    for (int j = start_pid + wid; j < end_pid; j += WARP_SIZE) {
      auto p = classified_points[j];

      if (
        p.type == PointType::GROUND &&
        p.z > cell_gnd_height_min + param.non_ground_height_threshold && cell_gnd_point_num > 1) {
        last_gnd_point = p;
        minus_t_gnd_height_sum += p.z;
        minus_t_gnd_radius_sum += p.radius;
        ++minus_t_gnd_point_num;
        p.type = PointType::NON_GROUND;
        last_gnd_idx = j;
      }

      classified_points[j] = p;
    }

    __syncwarp();

    // Now use ballot sync to see if there are any ground points remaining
    uint32_t recheck_res = __ballot_sync(FULL_MASK, local_gnd_point_num > minus_t_gnd_point_num);
    uint32_t backup_res = __ballot_sync(FULL_MASK, local_gnd_point_num > 0);

    // If no ground point remains, we have to keep the last ground point
    if (recheck_res == 0 && backup_res > 0) {
      // Get the index of the last thread that detects ground point
      int last_tid = 32 - __ffs(backup_res);

      // Only the last point in that thread remain
      if (wid == last_tid) {
        // Keep the last point's stat
        minus_t_gnd_height_sum -= last_gnd_point.z;
        minus_t_gnd_radius_sum -= last_gnd_point.radius;
        --minus_t_gnd_point_num;
        classified_points[last_gnd_idx] = last_gnd_point;
      }
    }
    __syncwarp();

    // Final update
    for (int offset = WARP_SIZE >> 1; offset > 0; offset >>= 1) {
      minus_t_gnd_height_sum += __shfl_down_sync(FULL_MASK, minus_t_gnd_height_sum, offset);
      minus_t_gnd_radius_sum += __shfl_down_sync(FULL_MASK, minus_t_gnd_radius_sum, offset);
      minus_t_gnd_point_num += __shfl_down_sync(FULL_MASK, minus_t_gnd_point_num, offset);
    }

    if (wid == 0) {
      t_gnd_height_sum -= minus_t_gnd_height_sum;
      t_gnd_radius_sum -= minus_t_gnd_radius_sum;
      t_gnd_point_num -= minus_t_gnd_point_num;
    }
  }

  // Finally, thread 0 update the cell stat
  if (wid == 0 && t_gnd_point_num > 0) {
    cell.gnd_radius_avg = t_gnd_radius_sum / (float)(t_gnd_point_num);
    cell.gnd_height_avg = t_gnd_height_sum / (float)(t_gnd_point_num);
    cell.gnd_height_min = t_gnd_height_min;
    cell.num_ground_points = t_gnd_point_num;
  }

  __syncwarp();
  cell.num_ground_points = __shfl_sync(FULL_MASK, cell.num_ground_points, 0);
}

__global__ void sectorProcessingKernel(
  Cell * __restrict__ cell_list, ClassifiedPointType * __restrict__ classified_points,
  int * starting_pid, FilterParameters param, int * empty_cell_mark)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  // Each warp handles one sector
  int sector_stride = (blockDim.x * gridDim.x) / WARP_SIZE;
  int wid = index % WARP_SIZE;  // Index of the thread within the warp
  // Shared memory to backup the cell data
  extern __shared__ Cell shared_buffer[];
  Cell * cell_queue = shared_buffer + (threadIdx.x / WARP_SIZE) * param.gnd_cell_buffer_size;
  int * cell_id_queue =
    (int *)(shared_buffer + param.gnd_cell_buffer_size * (blockDim.x / WARP_SIZE)) +
    (threadIdx.x / WARP_SIZE) * param.gnd_cell_buffer_size;

  // Loop on sectors
  for (int sector_id = index / WARP_SIZE; sector_id < param.num_sectors;
       sector_id += sector_stride) {
    // For storing the previous ground cells
    int closest_gnd_cell_id, furthest_gnd_cell_id, num_latest_gnd_cells = 0;
    int head = 0, tail = 0;
    float sum_x, sum_y, sum_xx, sum_xy, slope;
    float prev_gnd_radius_avg, prev_gnd_height_avg;
    int empty_cell_num = 0;

    // Initially no ground cell is identified
    closest_gnd_cell_id = furthest_gnd_cell_id = -1;
    sum_x = sum_y = sum_xx = sum_xy = slope = 0.0;

    // Loop on the cells in a sector
    for (int i = 0; i < param.max_num_cells_per_sector; ++i) {
      if (num_latest_gnd_cells > 0) {
        furthest_gnd_cell_id = cell_id_queue[head];
        empty_cell_num = empty_cell_mark[i * param.num_sectors + sector_id] -
                         empty_cell_mark[furthest_gnd_cell_id * param.num_sectors + sector_id];
      }

      int gid = i * param.num_sectors + sector_id;
      int sid = starting_pid[gid], eid = starting_pid[gid + 1];
      Cell cell;

      // Skip empty cells
      if (sid >= eid) {
        continue;
      }

      auto mode = checkSegmentationMode(
        i, closest_gnd_cell_id, furthest_gnd_cell_id, param.gnd_cell_buffer_size,
        param.gnd_grid_continual_thresh, empty_cell_num);

      // Classify the points in the cell
      segmentCell(
        wid, classified_points, param, cell, slope, sid, eid, prev_gnd_radius_avg,
        prev_gnd_height_avg, mode);

      // Update the indices of the previous ground cells if the cell contains ground points
      if (cell.num_ground_points > 0) {
        if (num_latest_gnd_cells >= param.gnd_cell_buffer_size) {
          if (wid == 0) {
            // If the number of previous ground cell reach maximum,
            // remove the cell at the queue head
            Cell head_cell = cell_queue[head];
            sum_x -= head_cell.gnd_radius_avg;
            sum_y -= head_cell.gnd_height_avg;
            sum_xx -= head_cell.gnd_radius_avg * head_cell.gnd_radius_avg;
            sum_xy -= head_cell.gnd_radius_avg * head_cell.gnd_height_avg;
          }

          // Now remove the entry at the head of the queue
          head = (head + 1) % param.gnd_cell_buffer_size;
          --num_latest_gnd_cells;
        }

        ++num_latest_gnd_cells;

        if (wid == 0) {
          // Add the new cell to the queue
          cell_queue[tail] = cell;
          cell_id_queue[tail] = i;

          // Update the stats and estimate the local slope
          sum_x += cell.gnd_radius_avg;
          sum_y += cell.gnd_height_avg;
          sum_xx += cell.gnd_radius_avg * cell.gnd_radius_avg;
          sum_xy += cell.gnd_radius_avg * cell.gnd_height_avg;

          float denom = (num_latest_gnd_cells * sum_xx - sum_x * sum_x);

          if (fabsf(denom) < 1e-6f) {
            Cell head_cell = cell_queue[head];
            slope = head_cell.gnd_height_avg / head_cell.gnd_radius_avg;
          } else {
            slope = (num_latest_gnd_cells * sum_xy - sum_x * sum_y) / denom;
            slope = fmax(fminf(slope, param.global_slope_max_ratio), -param.global_slope_max_ratio);
          }

          // Write the cell to the global memory
          cell_list[gid] = cell;
        }

        // Wait for the thread 0 to finish its work
        __syncwarp();
        // Now remove the cell at the end of the queue
        tail = (tail + 1) % param.gnd_cell_buffer_size;
        // Distribute the new slope to all threads in the warp
        slope = __shfl_sync(FULL_MASK, slope, 0);
        prev_gnd_radius_avg = __shfl_sync(FULL_MASK, cell.gnd_radius_avg, 0);
        prev_gnd_height_avg = __shfl_sync(FULL_MASK, cell.gnd_height_avg, 0);
        closest_gnd_cell_id = i;
      }
    }
  }
}

__global__ void markEmptyCells(int * starting_pid, int cell_num, int * mark)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < cell_num; i += stride) {
    mark[i] = (starting_pid[i] >= starting_pid[i + 1]) ? 1 : 0;
  }
}

__global__ void prefixSumEmptyCells(int * mark, int sector_num, int cell_per_sector_num)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < sector_num; i += stride) {
    int sum = 0;

    for (int j = 0, cell_global_id = i; j < cell_per_sector_num;
         ++j, cell_global_id += sector_num) {
      int val_j = mark[cell_global_id];

      mark[cell_global_id] = sum;
      sum += val_j;
    }
  }
}

void CudaScanGroundSegmentationFilter::sort_points(
  device_vector<Cell> & cell_list, device_vector<int> & starting_pid,
  device_vector<ClassifiedPointType> & classified_points)
{
  int point_num = dev_input_points_->height * dev_input_points_->width;

  if (point_num == 0 || filter_parameters_.max_num_cells == 0) {
    return;
  }

  int cell_num = filter_parameters_.max_num_cells;

  cell_list.resize(cell_num);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      (int)(cell_list.size()), 0, stream_->get(), cellInit, cell_list.data(),
      (int)(cell_list.size())));

  starting_pid.resize(cell_num + 1);

  device_vector<int> cell_id(point_num, stream_, mempool_);
  device_vector<ClassifiedPointType> tmp_classified_points(point_num, stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::fill(starting_pid, 0));
  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), computeCellId,
      reinterpret_cast<const PointTypeStruct *>(dev_input_points_->data.get()),
      (int)(dev_input_points_->height * dev_input_points_->width), filter_parameters_,
      cell_id.data(), starting_pid.data(), tmp_classified_points.data()));

  CHECK_CUDA_ERROR(cuda::ExclusiveScan(starting_pid));

  device_vector<int> writing_loc(starting_pid, stream_, mempool_);

  classified_points.resize(point_num);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), distributePointsToCell, tmp_classified_points.data(),
      classified_points.data(), cell_id.data(), point_num, writing_loc.data()));

  // Compute the number of empty cells between every pair of consecutive non-empty cell
  empty_cell_mark_->resize(cell_num);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      cell_num, 0, stream_->get(), markEmptyCells, starting_pid.data(), cell_num,
      empty_cell_mark_->data()));

  empty_cell_mark_->resize(cell_num);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      (int)(filter_parameters_.num_sectors), 0, stream_->get(), prefixSumEmptyCells,
      empty_cell_mark_->data(), (int)filter_parameters_.num_sectors,
      (int)filter_parameters_.max_num_cells_per_sector));
}

// ============ Scan per sector to get ground reference and Non-Ground points =============
void CudaScanGroundSegmentationFilter::scanPerSectorGroundReference(
  device_vector<Cell> & cell_list, device_vector<int> & starting_pid,
  device_vector<ClassifiedPointType> & classified_points)
{
  const uint32_t num_sectors = filter_parameters_.num_sectors;
  if (num_sectors == 0) {
    return;
  }

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      (int)(num_sectors * WARP_SIZE),
      (BLOCK_SIZE_X / WARP_SIZE) * (sizeof(Cell) + sizeof(int)) *
        filter_parameters_.gnd_cell_buffer_size,
      stream_->get(), sectorProcessingKernel, cell_list.data(), classified_points.data(),
      starting_pid.data(), filter_parameters_, empty_cell_mark_->data()));
}

struct NonGroundChecker
{
  CUDAH bool operator()(const ClassifiedPointType & p) const
  {
    return (p.type != PointType::GROUND);
  }
};

struct GroundChecker
{
  CUDAH bool operator()(const ClassifiedPointType & p) const
  {
    return (p.type == PointType::GROUND);
  }
};

template <typename CheckerType>
__global__ void markingPoints(
  ClassifiedPointType * classified_points, int point_num, int * mark, CheckerType checker
)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    auto p = classified_points[i];

    mark[p.origin_index] = (checker(p)) ? 1 : 0;
  }
}

__global__ void extract(
  const PointTypeStruct * __restrict__ dev_input_points, int point_num, int * writing_loc,
  PointTypeStruct * __restrict__ dev_output_points)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    int wloc = writing_loc[i];

    if (wloc < writing_loc[i + 1]) {
      dev_output_points[wloc] = dev_input_points[i];
    }
  }
}

// ============= Extract non-ground points =============
template <typename CheckerType>
void CudaScanGroundSegmentationFilter::extractPoints(
  device_vector<ClassifiedPointType> & classified_points,
  const cuda_blackboard::CudaPointCloud2 & input, cuda_blackboard::CudaPointCloud2 & output)
{
  int point_num = dev_input_points_->height * dev_input_points_->width;
  device_vector<int> point_mark(point_num + 1, stream_, mempool_);

  // Mark non-ground points
  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), markingPoints, classified_points.data(), point_num,
      point_mark.data(), CheckerType()
    ));

  // Exclusive scan
  device_vector<int> writing_loc(stream_, mempool_);

  CHECK_CUDA_ERROR(cuda::ExclusiveScan(point_mark, writing_loc));

  // Reserve the output
  int output_size = writing_loc[point_num];

  if (output_size <= 0) {
    return;
  }

  cuda::copyPointCloud2Metadata(output, input);

  output.data = cuda_blackboard::make_unique<uint8_t[]>(output_size * output.point_step);
  output.height = 1;
  output.width = output_size;

  // Get the points
  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), extract,
      reinterpret_cast<const PointTypeStruct *>(input.data.get()),
      (int)(input.height * input.width), writing_loc.data(),
      reinterpret_cast<PointTypeStruct *>(output.data.get())));
}

void CudaScanGroundSegmentationFilter::classifyPointCloud(
  const cuda_blackboard::CudaPointCloud2 & input, cuda_blackboard::CudaPointCloud2 & ground,
  cuda_blackboard::CudaPointCloud2 & non_ground)
{
  device_vector<Cell> cell_list(stream_, mempool_);
  device_vector<int> starting_pid(stream_, mempool_);
  device_vector<ClassifiedPointType> classified_points(stream_, mempool_);

  removeOutliers(input);
  sort_points(cell_list, starting_pid, classified_points);
  scanPerSectorGroundReference(cell_list, starting_pid, classified_points);

  int point_num = dev_input_points_->height * dev_input_points_->width;

  // Extract non-ground points
  extractPoints<NonGroundChecker>(classified_points, *dev_input_points_, non_ground);

  // Extract ground points
  extractPoints<GroundChecker>(classified_points, *dev_input_points_, ground);
}

__global__ void markNonOutliers(
  const PointTypeStruct * __restrict__ cloud, int point_num, FilterParameters param,
  float max_radius, int * __restrict__ mark)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    auto p = cloud[i];
    int val = (p.x < param.min_x || p.x > param.max_x || p.y < param.min_y || 
                p.y > param.max_y || p.z < param.min_z || p.z > param.max_z) ? 0 : 1;
    mark[i] = val;
  }
}

__global__ void getValidPoints(
  const PointTypeStruct * __restrict__ cloud, int point_num, int * __restrict__ writing_loc,
  PointTypeStruct * __restrict__ out_cloud)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < point_num; i += stride) {
    int wloc = writing_loc[i];

    if (wloc < writing_loc[i + 1]) {
      out_cloud[wloc] = cloud[i];
    }
  }
}

void CudaScanGroundSegmentationFilter::removeOutliers(
  const cuda_blackboard::CudaPointCloud2 & input)
{
  float max_radius = filter_parameters_.max_radius;
  int point_num = input.width * input.height;

  if (point_num <= 0) {
    return;
  }

  device_vector<int> mark(point_num + 1, stream_, mempool_);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), markNonOutliers,
      reinterpret_cast<const PointTypeStruct *>(input.data.get()),
      (int)(input.height * input.width), filter_parameters_, max_radius, mark.data()));

  CHECK_CUDA_ERROR(cuda::ExclusiveScan(mark));

  int remain_size = mark[point_num];

  cuda::copyPointCloud2Metadata(*dev_input_points_, input);

  dev_input_points_->height = 1;
  dev_input_points_->width = remain_size;
  dev_input_points_->data = cuda_blackboard::make_unique<uint8_t[]>(remain_size * dev_input_points_->point_step);

  CHECK_CUDA_ERROR(
    cuda::launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), getValidPoints,
      reinterpret_cast<const PointTypeStruct *>(input.data.get()),
      (int)(input.height * input.width), mark.data(),
      reinterpret_cast<PointTypeStruct *>(dev_input_points_->data.get())));
}

}  // namespace autoware::cuda_ground_segmentation
