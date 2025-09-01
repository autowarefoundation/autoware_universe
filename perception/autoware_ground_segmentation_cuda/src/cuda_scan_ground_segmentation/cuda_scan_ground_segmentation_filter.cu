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

#include <cub/cub.cuh>

#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

namespace autoware::cuda_ground_segmentation
{
namespace
{

template <typename T>
__device__ const T getElementValue(
  const uint8_t * data, const size_t point_index, const size_t point_step, const size_t offset)
{
  return *reinterpret_cast<const T *>(data + offset + point_index * point_step);
}
__device__ __forceinline__ float fastAtan2_0_2Pi(float y, float x)
{
  float angle = atan2f(y, x);  // Returns [-π, π]
  if (angle < 0.0f) {
    angle += 2.0f * M_PI;  // Convert to [0, 2π]
  }
  return angle;
}

/**
 * @brief CUDA kernel to initialize an array of ClassifiedPointTypeStruct elements.
 *
 * This kernel sets the `z` field to 0.0f, the `type` field to PointType::INIT,
 * the `radius` field to -1.0f, and the `origin_index` field to 0 for each element
 * in the input array up to N elements. Each thread processes one element.
 *
 * @param arr Pointer to the array of ClassifiedPointTypeStruct to initialize.
 * @param N The number of points in the array to initialize.
 */
__global__ void initPoints(ClassifiedPointTypeStruct * arr, uint32_t N)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  arr[idx].z = 0.0f;
  arr[idx].type = PointType::INIT;
  arr[idx].radius = -1.0f;
  arr[idx].origin_index = 0;
}

/**
 * @brief CUDA kernel to set elements of a flags array to a specified value for classified points
 * extraction.
 *
 * This kernel assigns the given value to each element of the `flags` array up to `n` elements.
 *
 * @param[in,out] flags Pointer to the array of uint32_t flags to be set.
 * @param[in] n Number of elements in the flags array to set.
 * @param[in] value The value to assign to each element in the flags array.
 */
__global__ void setFlagsKernel(uint32_t * flags, uint32_t n, const uint32_t value)
{
  uint32_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) flags[i] = value;  // write real uint32_t  0 or 1
}

/**
 * @brief CUDA kernel to extract the number of points from each cell centroid.
 *
 * This kernel iterates over an array of cell centroids and writes the number of points
 * in each cell to a corresponding output array. Each thread processes one cell.
 *
 * @param[in] centroid_cells_list_dev Pointer to the device array of cell centroids.
 * @param[in] num_cells Number of maximum cells to process.
 * @param[out] num_points_per_cell Pointer to the device array where the number of points per cell
 * will be stored.
 */
__global__ void getCellNumPointsKernel(
  const CellCentroid * __restrict__ centroid_cells_list_dev, const uint32_t num_cells,
  uint32_t * __restrict__ num_points_per_cell)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_cells) {
    return;  // Out of bounds
  }
  num_points_per_cell[idx] = centroid_cells_list_dev[idx].num_points;
}

/**
 * @brief CUDA kernel to assign each point in the input point cloud to a classified cell.
 *
 * This kernel splits the input point cloud into sectors and cells based on the angle and distance
 * from a specified center. Each point is assigned to a cell, and the number of points in each cell
 * is tracked. The classified points are stored in an output array, with each point's metadata
 * updated accordingly.
 *
 * @param[in]  input_points              Pointer to the input point cloud array.
 * @param[in]  num_points                Number of points in the input point cloud.
 * @param[in]  cells_centroid_dev        Pointer to the array of cell centroid information.
 * @param[in,out] cell_counts_dev        Pointer to the array tracking the number of points per cell
 * (atomic).
 * @param[in]  filter_parameters_dev     Pointer to the filter parameters (e.g., sector angle, cell
 * size, center).
 * @param[out] classified_points_dev     Pointer to the output array of classified points.
 *
 * @details
 * - Each thread processes one point from the input point cloud.
 * - The angle and radius from the center are computed for each point.
 * - The sector and cell indices are determined based on these values.
 * - Points are atomically assigned to a slot in their corresponding cell.
 * - The classified point's metadata (z, type, radius, origin index) is set in the output array.
 * - Out-of-bounds checks are performed at each step to ensure memory safety.
 *
 * @note
 * - The function assumes that the output arrays and counters are properly initialized.
 * - The kernel uses atomic operations to safely increment cell counts in parallel.
 * - The function does not update cell centroid values; it only assigns points to cells.
 */
__global__ void assignPointToClassifyPointKernel(
  const PointTypeStruct * __restrict__ input_points, const uint32_t num_points,
  const CellCentroid * __restrict__ cells_centroid_dev, uint32_t * __restrict__ cell_counts_dev,
  const FilterParameters * __restrict__ filter_parameters_dev,
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev)
{
  uint32_t idx = static_cast<uint32_t>(blockIdx.x * blockDim.x + threadIdx.x);
  if (idx >= num_points) {
    return;  // Out of bounds
  }
  const auto & inv_sector_angle_rad = filter_parameters_dev->inv_sector_angle_rad;
  // Calculate the angle and distance from the center
  const float x = input_points[idx].x - filter_parameters_dev->center_x;
  const float y = input_points[idx].y - filter_parameters_dev->center_y;
  const float radius = sqrtf(x * x + y * y);
  const float angle = fastAtan2_0_2Pi(y, x);  // replace with approximate atan
  // Determine the radial division index
  auto division_sector_index = static_cast<uint32_t>(angle * inv_sector_angle_rad);
  auto cell_index_in_sector =
    static_cast<uint32_t>(radius / filter_parameters_dev->cell_divider_size_m);
  auto cell_id =
    division_sector_index * filter_parameters_dev->max_num_cells_per_sector + cell_index_in_sector;
  if (cell_id >= filter_parameters_dev->max_num_cells) {
    return;  // Out of bounds
  }

  uint32_t slot_idx = atomicAdd(&cell_counts_dev[cell_id], 1);
  // check local bounds for slot_idx
  if (slot_idx >= cells_centroid_dev[cell_id].num_points) {
    return;  // Out of bounds
  }
  auto classify_point_idx =
    static_cast<uint32_t>(cells_centroid_dev[cell_id].start_point_index + slot_idx);
  // Check global bounds
  if (classify_point_idx >= num_points) {
    return;  // Out of bounds
  }
  // add pointcloud to output grid list
  auto & assign_classified_point_dev = classified_points_dev[classify_point_idx];
  assign_classified_point_dev.z = input_points[idx].z;
  assign_classified_point_dev.type = PointType::INIT;

  assign_classified_point_dev.radius = radius;
  assign_classified_point_dev.origin_index = idx;
}

/**
 * @brief Updates the ground point statistics in a cell with a new ground point.
 *
 * This function updates the running average and minimum height of ground points
 * within a cell by incorporating a newly classified ground point. It recalculates
 * the average ground height and radius based on the new point, increments the
 * ground point count, and updates the minimum ground height if the new point is lower.
 *
 * @param[in,out] cell Reference to the cell centroid structure to update.
 * @param[in] gnd_point The newly classified ground point to incorporate.
 */
__device__ void updateGndPointInCell(
  CellCentroid & cell, const ClassifiedPointTypeStruct & gnd_point)
{
  cell.gnd_height_avg = cell.gnd_height_avg * cell.num_ground_points + gnd_point.z;
  cell.gnd_radius_avg = cell.gnd_radius_avg * cell.num_ground_points + gnd_point.radius;

  cell.num_ground_points++;

  cell.gnd_height_avg = cell.gnd_height_avg / cell.num_ground_points;
  cell.gnd_radius_avg = cell.gnd_radius_avg / cell.num_ground_points;
  // Update the min height
  if (gnd_point.z < cell.gnd_height_min) {
    cell.gnd_height_min = gnd_point.z;
  }
}

/**
 * @brief Removes a ground point from a cell and updates the cell's ground statistics.
 *
 * This device function updates the average ground height and radius of a cell by removing
 * the contribution of a specified ground point. It decrements the number of ground points
 * in the cell and recalculates the averages accordingly.
 *
 * @param cell Reference to the CellCentroid structure representing the cell to update.
 * @param point The ClassifiedPointTypeStruct representing the ground point to remove.
 */
__device__ void removeGndPointInCell(CellCentroid & cell, const ClassifiedPointTypeStruct & point)
{
  cell.gnd_height_avg =
    (cell.gnd_height_avg * cell.num_ground_points - point.z) / (cell.num_ground_points - 1);
  cell.gnd_radius_avg =
    (cell.gnd_radius_avg * cell.num_ground_points - point.radius) / (cell.num_ground_points - 1);
  cell.num_ground_points--;
}

/**
 * @brief Determines the segmentation mode based on the position of the current cell and the history
 * of ground cells.
 *
 * This device function analyzes the relationship between the current cell index within a sector and
 * the indices of previously detected ground cells. It sets the segmentation mode accordingly:
 * - UNINITIALIZED: No ground cells have been detected yet.
 * - BREAK: The current cell is too far from the last detected ground cell, indicating a break in
 * ground continuity.
 * - CONTINUOUS: The current cell is within a buffer threshold of the most recent ground cell,
 * indicating continuous ground.
 * - DISCONTINUOUS: The current cell is not within the buffer threshold, but not far enough to be
 * considered a break.
 *
 * @param[in] cell_index_in_sector         Index of the current cell within the sector.
 * @param[in] last_gnd_cells_dev           Device pointer to an array of indices of the most recent
 * ground cells in the sector.
 * @param[in] num_latest_gnd_cells         Number of valid entries in last_gnd_cells_dev.
 * @param[in] gnd_cell_continual_thresh    Threshold for determining a break in ground cell
 * continuity.
 * @param[in] gnd_cell_buffer_size         Buffer size for determining continuous ground.
 * @param[out] mode                        Reference to the segmentation mode to be set by this
 * function.
 */
__device__ void checkSegmentMode(
  const uint32_t cell_index_in_sector, const uint32_t * last_gnd_cells_dev,
  const uint32_t num_latest_gnd_cells, const uint32_t gnd_cell_continual_thresh,
  const uint32_t gnd_cell_buffer_size, SegmentationMode & mode)
{
  if (num_latest_gnd_cells == 0) {
    mode = SegmentationMode::UNINITIALIZED;
    return;  // No ground cells found, set mode to UNINITIALIZED
  }
  const auto & last_gnd_idx_in_sector = last_gnd_cells_dev[0];
  if (cell_index_in_sector - last_gnd_idx_in_sector >= gnd_cell_continual_thresh) {
    mode = SegmentationMode::BREAK;  // If the latest ground cell is too far, set mode to BREAK
    return;
  }
  const auto & front_gnd_idx_in_sector = last_gnd_cells_dev[num_latest_gnd_cells - 1];
  if (cell_index_in_sector - front_gnd_idx_in_sector <= gnd_cell_buffer_size) {
    mode = SegmentationMode::CONTINUOUS;  // If the latest ground cell is within threshold, set
                                          // mode to CONTINUOUS
    return;
  }
  mode = SegmentationMode::DISCONTINUOUS;  // If the latest ground cell is not within threshold, set
                                           // mode to DISCONTINUOUS
  return;
}

/**
 * @brief Recursively searches for ground cells in a sector and collects their indices.
 *
 * This device function traverses the sector's cells in reverse order (from the given index down to
 * 0), collecting the indices of cells that contain ground points. The search stops when either the
 * required number of ground cells is found or the first cell is reached.
 *
 * @param[in] sector_cells_list_dev      Pointer to the array of cell centroids in the sector
 * (device memory).
 * @param[in] last_gnd_cells_num_threshold  Maximum number of ground cells to collect.
 * @param[in] cell_index_in_sector       Current cell index in the sector to check.
 * @param[out] last_gnd_cells_dev        Pointer to the array where found ground cell indices are
 * stored (device memory).
 * @param[in,out] num_latest_gnd_cells   Reference to the current count of found ground cells
 * (incremented as cells are found).
 */
__device__ void RecursiveGndCellSearch(
  const CellCentroid * __restrict__ sector_cells_list_dev,
  const uint32_t last_gnd_cells_num_threshold, const uint32_t cell_index_in_sector,
  uint32_t * last_gnd_cells_dev, uint32_t & num_latest_gnd_cells)
{
  if (num_latest_gnd_cells >= last_gnd_cells_num_threshold) {
    return;  // Stop if we have enough ground cells
  }
  const auto & current_cell = sector_cells_list_dev[cell_index_in_sector];

  if (current_cell.num_ground_points > 0) {
    // If the cell has ground points, add it to the list
    last_gnd_cells_dev[num_latest_gnd_cells++] = cell_index_in_sector;
  }
  if (cell_index_in_sector == 0) {
    return;  // Base case: no more cells to check
  }
  // Continue searching in the previous cell
  RecursiveGndCellSearch(
    sector_cells_list_dev, last_gnd_cells_num_threshold, cell_index_in_sector - 1,
    last_gnd_cells_dev, num_latest_gnd_cells);
}
/**
 * @brief Fits a line to ground cell centroids using the least squares method and returns the slope.
 *
 * This device function computes the slope (a) of a line fitted to a set of ground cell centroids,
 * represented by their average radius and height, using the least squares method. The function
 * handles special cases where there are zero or one fitting cells, and clamps the resulting slope
 * to a maximum allowed ratio specified in the filter parameters.
 *
 * @param sector_cells_list_dev Pointer to the array of CellCentroid structures representing the
 * sector's cells (device memory).
 * @param last_gnd_cells_indices_dev Pointer to the array of indices indicating which cells are used
 * for fitting (device memory).
 * @param num_fitting_cells Number of cells to use for fitting.
 * @param filter_parameters_dev Pointer to the FilterParameters structure containing fitting
 * thresholds (device memory).
 * @return The slope (a) of the fitted line. If fitting is not possible, returns 0.0f.
 */

__device__ float fitLineFromGndCell(
  const CellCentroid * __restrict__ sector_cells_list_dev,
  const uint32_t * last_gnd_cells_indices_dev, const uint32_t num_fitting_cells,
  const FilterParameters * __restrict__ filter_parameters_dev)
{
  float a = 0.0f;
  // float b = 0.0f;

  if (num_fitting_cells == 0) {
    a = 0.0f;  // No fitting cells, set to zero
    // b = 0.0f;  // No fitting cells, set to zero
    return a;  // No fitting cells, return zero slope
  }

  if (num_fitting_cells == 1) {
    auto cell_idx_in_sector = last_gnd_cells_indices_dev[0];
    const auto & cell = sector_cells_list_dev[cell_idx_in_sector];
    a = cell.gnd_height_avg / cell.gnd_radius_avg;
    // b = 0.0f;  // Only one point, no line fitting needed
    return a;  // Return the slope based on the single point
  }

  // calculate the line by least squares method
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xy = 0.0f;
  float sum_xx = 0.0f;
  for (int i = 0; i < num_fitting_cells; ++i) {
    const auto & cell_idx_in_sector = last_gnd_cells_indices_dev[i];
    const auto & cell = sector_cells_list_dev[cell_idx_in_sector];
    float x = cell.gnd_radius_avg;
    float y = cell.gnd_height_avg;
    sum_x += x;
    sum_y += y;
    sum_xy += x * y;
    sum_xx += x * x;
  }
  const float denominator = (num_fitting_cells * sum_xx - sum_x * sum_x);
  if (fabsf(denominator) < 1e-6f) {
    const auto & cell_idx_in_sector = last_gnd_cells_indices_dev[0];
    const auto & cell = sector_cells_list_dev[cell_idx_in_sector];
    a = cell.gnd_height_avg / cell.gnd_radius_avg;
    // b = 0.0f;
    return a;  // If denominator is zero, return slope based on the first cell
  } else {
    a = (num_fitting_cells * sum_xy - sum_x * sum_y) / denominator;  // slope
    a = a > filter_parameters_dev->global_slope_max_ratio
          ? filter_parameters_dev->global_slope_max_ratio
          : a;  // Clamp to threshold
    a = a < -filter_parameters_dev->global_slope_max_ratio
          ? -filter_parameters_dev->global_slope_max_ratio
          : a;  // Clamp to threshold
    // b = (sum_y * sum_xx - sum_x * sum_xy) / denominator;  // intercept
  }
  return a;
}
/**
 * @brief Rechecks the classification of points within a cell to ensure ground points meet height
 * criteria.
 *
 * This device function iterates through all ground points in the given cell and verifies if each
 * point still satisfies the ground height threshold. If a ground point's height (z) exceeds the
 * minimum ground height of the cell plus the non-ground height threshold from the filter
 * parameters, the point is reclassified as non-ground and removed from the cell's ground point
 * count.
 *
 * @param[in,out] cell Reference to the cell centroid structure containing cell statistics and
 * indices.
 * @param[in,out] classify_points Pointer to the array of classified point structures.
 * @param[in] filter_parameters_dev Pointer to the filter parameters structure in device memory.
 */

__device__ void recheckCell(
  CellCentroid & cell, ClassifiedPointTypeStruct * __restrict__ classify_points,
  const FilterParameters * __restrict__ filter_parameters_dev)
{
  auto const idx_start_point_of_cell = cell.start_point_index;
  if (cell.num_ground_points < 2) {
    // If the cell has less than 2 ground points, we can skip rechecking
    return;
  }
  for (int i = 0; i < cell.num_points; i++) {
    auto point_idx = static_cast<size_t>(idx_start_point_of_cell + i);
    auto & point = classify_points[point_idx];
    if (point.type != PointType::GROUND) {
      continue;
    }
    if (point.z > cell.gnd_height_min + filter_parameters_dev->non_ground_height_threshold) {
      point.type = PointType::NON_GROUND;
      removeGndPointInCell(cell, point);
    }
  }
}

/**
 * @brief Segments and classifies points within a cell as ground, non-ground, or out-of-range.
 *
 * This device function iterates over all points in a given cell, classifies each point based on
 * height and slope criteria, and updates the cell's ground point statistics. Optionally, it can
 * recheck the ground cluster in the cell if certain conditions are met.
 *
 * @param sector_cells_list_dev         Pointer to the array of cell centroids (device memory).
 * @param cell_classify_points_dev      Pointer to the array of classified points (device memory).
 * @param filter_parameters_dev         Pointer to the filter parameters structure (device memory).
 * @param cell_idx_in_sector            Index of the current cell within the sector.
 *
 * Classification logic:
 * - Marks points as OUT_OF_RANGE if their height is outside the detection range.
 * - Marks points as NON_GROUND if their height exceeds a global slope-based threshold.
 * - Marks points as GROUND if their height is within both the slope and non-ground thresholds,
 *   and updates ground statistics for the cell.
 * - Optionally rechecks ground points in the cell if enabled and conditions are satisfied.
 */
__device__ void SegmentInitializedCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const uint32_t cell_idx_in_sector)
{
  auto & current_cell = sector_cells_list_dev[cell_idx_in_sector];  // Use reference, not copy
  // auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;

  for (int i = 0; i < num_points_of_cell; ++i) {
    size_t point_idx = static_cast<size_t>(i);
    auto & point = cell_classify_points_dev[point_idx];

    // 1. height is out-of-range
    if (
      point.z > filter_parameters_dev->detection_range_z_max ||
      point.z < -filter_parameters_dev->non_ground_height_threshold) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }

    // 3. Check global slope ratio
    // float slope_ratio = point.z / point.radius;
    float global_height_threshold = point.radius * filter_parameters_dev->global_slope_max_ratio;
    if (
      point.z > global_height_threshold &&
      point.z > filter_parameters_dev->non_ground_height_threshold) {
      point.type = PointType::NON_GROUND;
      continue;
    }

    // 4. Check if point meets ground criteria
    if (
      abs(point.z) < global_height_threshold &&
      abs(point.z) < filter_parameters_dev->non_ground_height_threshold) {
      point.type = PointType::GROUND;
      updateGndPointInCell(current_cell, point);
    }
  }

  if (
    filter_parameters_dev->use_recheck_ground_cluster == 1 && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}

/**
 * @brief Segments and classifies points in a cell as ground, non-ground, or out-of-range based on
 * geometric and slope criteria.
 *
 * This device function processes all points within a given cell, comparing each point's height and
 * position relative to the previous ground cell and estimated ground gradient. The classification
 * is performed using several thresholds defined in the filter parameters, including global and
 * local slope ratios, detection range, and non-ground height threshold. Points are classified as
 * GROUND, NON_GROUND, or OUT_OF_RANGE.
 *
 * If enabled, the function also performs a recheck of ground points in the cell for further
 * refinement.
 *
 * @param sector_cells_list_dev         Pointer to the array of cell centroids for the current
 * sector (device memory).
 * @param cell_classify_points_dev      Pointer to the array of points to classify within the
 * current cell (device memory).
 * @param filter_parameters_dev         Pointer to the filter parameters structure (device memory).
 * @param cell_idx_in_sector            Index of the current cell within the sector.
 * @param last_gnd_cells_indices_dev    Pointer to the array of indices of the most recent ground
 * cells (device memory).
 * @param num_latest_gnd_cells          Number of latest ground cells to use for ground estimation.
 */

__device__ void SegmentContinuousCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const uint32_t cell_idx_in_sector,
  const uint32_t * last_gnd_cells_indices_dev, const uint32_t num_latest_gnd_cells)
{
  // compare point of current cell with previous cell center by local slope angle
  // auto gnd_gradient = calcLocalGndGradient(
  //   centroid_cells, filter_parameters_dev->gnd_cell_buffer_size, sector_start_cell_index,
  //   cell_idx_in_sector, filter_parameters_dev->global_slope_max_ratio);

  auto gnd_gradient = fitLineFromGndCell(
    sector_cells_list_dev, last_gnd_cells_indices_dev, num_latest_gnd_cells, filter_parameters_dev);
  uint32_t cell_id = cell_idx_in_sector;
  auto & current_cell = sector_cells_list_dev[cell_id];  // Use reference, not copy
  // auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;
  auto & prev_gnd_cell = sector_cells_list_dev[last_gnd_cells_indices_dev[0]];
  auto const prev_cell_gnd_height = prev_gnd_cell.gnd_height_avg;

  for (size_t i = 0; i < num_points_of_cell; ++i) {
    auto & point = cell_classify_points_dev[i];
    // 1. height is out-of-range compared to previous cell gnd
    if (point.z - prev_cell_gnd_height > filter_parameters_dev->detection_range_z_max) {
      point.type = PointType::OUT_OF_RANGE;
      return;
    }

    auto d_radius =
      point.radius - prev_gnd_cell.gnd_radius_avg + filter_parameters_dev->cell_divider_size_m;
    auto dz = point.z - prev_gnd_cell.gnd_height_avg;

    // 2. the angle is exceed the global slope threshold
    if (point.z > filter_parameters_dev->global_slope_max_ratio * point.radius) {
      point.type = PointType::NON_GROUND;
      continue;
    }

    // 2. the angle is exceed the local slope threshold
    if (dz > filter_parameters_dev->local_slope_max_ratio * d_radius) {
      point.type = PointType::NON_GROUND;
      continue;
    }

    // 3. height from the estimated ground center estimated by local gradient
    float estimated_ground_z =
      prev_gnd_cell.gnd_height_avg + gnd_gradient * filter_parameters_dev->cell_divider_size_m;
    if (point.z > estimated_ground_z + filter_parameters_dev->non_ground_height_threshold) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    // if (abs(point.z - estimated_ground_z) <= filter_parameters_dev->non_ground_height_threshold)
    // {
    //   continue;  // Mark as ground point
    // }

    if (
      point.z < estimated_ground_z - filter_parameters_dev->non_ground_height_threshold ||
      dz < -filter_parameters_dev->local_slope_max_ratio * d_radius ||
      point.z < -filter_parameters_dev->global_slope_max_ratio * point.radius) {
      // If the point is below the estimated ground height, classify it as non-ground
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    // If the point is close to the estimated ground height, classify it as ground
    point.type = PointType::GROUND;
    updateGndPointInCell(current_cell, point);
  }

  if (
    filter_parameters_dev->use_recheck_ground_cluster == 1 && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}

/**
 * @brief Segments a discontinuous cell by classifying its points as ground, non-ground, or
 * out-of-range.
 *
 * This device function processes all points within a given cell, classifying each point based on
 * its height and radius relative to the previous ground cell and configurable filter parameters.
 * The classification considers global and local slope thresholds, as well as detection range
 * limits. Points are marked as GROUND, NON_GROUND, or OUT_OF_RANGE accordingly. If enabled and
 * applicable, the function also performs a recheck on the ground points within the cell to refine
 * the classification.
 *
 * @param sector_cells_list_dev         Device pointer to the array of cell centroids for the
 * sector.
 * @param cell_classify_points_dev      Device pointer to the array of points to classify within the
 * cell.
 * @param filter_parameters_dev         Device pointer to the filter parameters structure.
 * @param cell_idx_in_sector            Index of the current cell within the sector.
 * @param last_gnd_cells_indices_dev    Device pointer to the array of indices for the latest ground
 * cells.
 * @param num_latest_gnd_cells          Number of latest ground cells in the array.
 */

__device__ void SegmentDiscontinuousCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const uint32_t cell_idx_in_sector,
  const uint32_t * last_gnd_cells_indices_dev, const uint32_t num_latest_gnd_cells)
{
  auto cell_id = cell_idx_in_sector;
  auto & current_cell = sector_cells_list_dev[cell_id];  // Use reference, not copy
  // auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;
  auto & prev_gnd_cell = sector_cells_list_dev[last_gnd_cells_indices_dev[0]];

  for (uint32_t i = 0; i < num_points_of_cell; ++i) {
    size_t point_idx = static_cast<size_t>(i);
    auto & point = cell_classify_points_dev[point_idx];
    // 1. height is out-of-range
    if (point.z - prev_gnd_cell.gnd_height_avg > filter_parameters_dev->detection_range_z_max) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    // 2. the angle is exceed the global slope threshold
    auto dz = point.z - prev_gnd_cell.gnd_height_avg;
    auto d_radius =
      point.radius - prev_gnd_cell.gnd_radius_avg + filter_parameters_dev->cell_divider_size_m;
    float global_height_threshold = point.radius * filter_parameters_dev->global_slope_max_ratio;
    float local_height_threshold = filter_parameters_dev->local_slope_max_ratio * d_radius;
    if (point.z > global_height_threshold) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    if (dz > local_height_threshold) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    // 3. local slope
    if (dz < -local_height_threshold) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    if (point.z < -global_height_threshold) {
      // If the point is below the estimated ground height, classify it as non-ground
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    point.type = PointType::GROUND;  // Mark as ground point
    updateGndPointInCell(current_cell, point);
  }

  if (
    filter_parameters_dev->use_recheck_ground_cluster == 1 && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}

/**
 * @brief Segments and classifies points in a cell that is not continuous with the previous cell.
 *
 * This device function processes a cell within a sector, classifying each point as GROUND,
 * NON_GROUND, or OUT_OF_RANGE based on height and slope thresholds relative to the previous ground
 * cell. It updates ground statistics for the cell and optionally rechecks ground points if certain
 * conditions are met.
 *
 * @param sector_cells_list_dev         Pointer to the array of cell centroids for the sector
 * (device memory).
 * @param cell_classify_points_dev      Pointer to the array of points to classify within the cell
 * (device memory).
 * @param filter_parameters_dev         Pointer to the filter parameters structure (device memory).
 * @param cell_idx_in_sector            Index of the current cell within the sector.
 * @param last_gnd_cells_indices_dev    Pointer to the array containing indices of the latest ground
 * cells (device memory).
 * @param num_latest_gnd_cells          Number of latest ground cells in the array.
 */

__device__ void SegmentBreakCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const uint32_t cell_idx_in_sector,
  const uint32_t * last_gnd_cells_indices_dev, const uint32_t num_latest_gnd_cells)
{
  // This function is called when the cell is not continuous with the previous cell
  auto cell_id = cell_idx_in_sector;
  auto & current_cell = sector_cells_list_dev[cell_id];  // Use reference, not copy
  // auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;
  auto & prev_gnd_cell = sector_cells_list_dev[last_gnd_cells_indices_dev[0]];

  for (uint32_t i = 0; i < num_points_of_cell; ++i) {
    auto & point = cell_classify_points_dev[i];

    // 1. height is out-of-range
    if (point.z - prev_gnd_cell.gnd_height_avg > filter_parameters_dev->detection_range_z_max) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }

    auto dz = point.z - prev_gnd_cell.gnd_height_avg;
    auto d_radius = point.radius - prev_gnd_cell.gnd_radius_avg;
    float global_height_threshold = d_radius * filter_parameters_dev->global_slope_max_ratio;
    // 3. Global slope check
    if (dz > global_height_threshold) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    if (dz < -global_height_threshold) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    point.type = PointType::GROUND;
    updateGndPointInCell(current_cell, point);
  }
  if (
    filter_parameters_dev->use_recheck_ground_cluster == 1 && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}
/**
 * @brief CUDA kernel for ground reference point extraction per sector in scan-based ground
 * segmentation.
 *
 * This kernel processes each sector in parallel, scanning through its cells to identify ground
 * reference points. For each sector, it iterates over all cells, classifying them based on the
 * number of points and their spatial relationship to previously identified ground cells. The kernel
 * supports different segmentation modes (UNINITIALIZED, CONTINUOUS, DISCONTINUOUS, BREAK) and
 * applies the appropriate segmentation logic for each cell.
 *
 * - If a cell contains no points, it is skipped.
 * - For the first cell in a sector, initialization is performed.
 * - For subsequent cells, the kernel searches for the latest ground cells in the sector and
 * determines the segmentation mode based on continuity and buffer thresholds.
 * - Depending on the mode, the cell is segmented using the corresponding segmentation function.
 *
 * @param[in,out] classified_points_dev      Device pointer to the array of classified points.
 * @param[in,out] centroid_cells_list_dev    Device pointer to the array of cell centroids for all
 * sectors.
 * @param[in]     filter_parameters_dev      Device pointer to the filter parameters structure.
 *
 * @note This kernel assumes that the number of threads launched is at least equal to the number of
 * sectors. Each thread processes one sector independently.
 */

__global__ void scanPerSectorGroundReferenceKernel(
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev,
  CellCentroid * __restrict__ centroid_cells_list_dev,
  const FilterParameters * __restrict__ filter_parameters_dev)
{
  // Implementation of the kernel
  // scan in each sector from cell_index_in_sector = 0 to max_num_cells_per_sector
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= filter_parameters_dev->num_sectors) {
    return;  // Out of bounds
  }
  // For each sector, find the ground reference points if points exist
  // otherwise, use the previous sector ground reference points
  // initialize the previous cell centroid

  // Process the first cell of the sector
  SegmentationMode mode = SegmentationMode::UNINITIALIZED;
  auto sector_start_cell_index = idx * filter_parameters_dev->max_num_cells_per_sector;
  CellCentroid * sector_cells_list_dev = &centroid_cells_list_dev[sector_start_cell_index];
  for (int cell_index_in_sector = 0;
       cell_index_in_sector < filter_parameters_dev->max_num_cells_per_sector;
       ++cell_index_in_sector) {
    if (sector_cells_list_dev[cell_index_in_sector].num_points == 0) {
      // if no points in the cell, continue
      continue;
    }

    // declare the points to stogare the gnd cells indexes in the sector
    // this is used to store the ground cells in the sector for line fitting
    // the size of the array is gnd_cell_buffer_size
    uint32_t num_latest_gnd_cells = 0;
    const uint32_t BUFFER_SIZE = 5;
    // declare fixed size array to store the latest gnd cells index in the sector
    // the size of the array is gnd_cell_buffer_size
    uint32_t indices_latest_gnd_cells_array[BUFFER_SIZE] = {0};
    // get the latest gnd cells in the sector
    size_t cell_first_classify_point_index =
      sector_cells_list_dev[cell_index_in_sector].start_point_index;
    ClassifiedPointTypeStruct * cell_classify_points_dev =
      &classified_points_dev[cell_first_classify_point_index];

    if (cell_index_in_sector == 0) {
      mode = SegmentationMode::UNINITIALIZED;
      SegmentInitializedCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector);
      continue;
    }
    RecursiveGndCellSearch(
      sector_cells_list_dev, filter_parameters_dev->gnd_cell_buffer_size, cell_index_in_sector - 1,
      indices_latest_gnd_cells_array, num_latest_gnd_cells);

    // check the segmentation Mode based on prevoius gnd cells
    checkSegmentMode(
      cell_index_in_sector, indices_latest_gnd_cells_array, num_latest_gnd_cells,
      filter_parameters_dev->gnd_grid_continual_thresh, filter_parameters_dev->gnd_cell_buffer_size,
      mode);

    if (mode == SegmentationMode::UNINITIALIZED) {
      SegmentInitializedCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector);
      continue;
    }
    if (mode == SegmentationMode::CONTINUOUS) {
      SegmentContinuousCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector, indices_latest_gnd_cells_array, num_latest_gnd_cells);
      continue;
    }
    if (mode == SegmentationMode::DISCONTINUOUS) {
      SegmentDiscontinuousCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector, indices_latest_gnd_cells_array, num_latest_gnd_cells);
      continue;
    }
    if (mode == SegmentationMode::BREAK) {
      SegmentBreakCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector, indices_latest_gnd_cells_array, num_latest_gnd_cells);
      continue;
    }
    // if the first round of scan
  }
}

__global__ void CellsCentroidInitializeKernel(
  CellCentroid * __restrict__ centroid_cells_list_dev, const uint32_t max_num_cells)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= max_num_cells) {
    return;  // Out of bounds
  }
  centroid_cells_list_dev[idx].gnd_radius_avg = 0.0f;
  centroid_cells_list_dev[idx].gnd_height_avg = 0.0f;
  centroid_cells_list_dev[idx].gnd_height_min = 1e6f;
  centroid_cells_list_dev[idx].num_points = 0;
  centroid_cells_list_dev[idx].cell_id = 0;  // Initialize cell_id to -1
  centroid_cells_list_dev[idx].num_ground_points = 0;
  centroid_cells_list_dev[idx].start_point_index = 0;
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
__global__ void calcCellPointNumberKernel(
  const PointTypeStruct * __restrict__ input_points, const uint32_t num_input_points,
  const FilterParameters * __restrict__ filter_parameters_dev,
  CellCentroid * __restrict__ centroid_cells_list_dev)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_input_points) {
    return;
  }

  const auto & inv_sector_angle_rad = filter_parameters_dev->inv_sector_angle_rad;

  // Calculate the angle and distance from the center
  const float dx = input_points[idx].x - filter_parameters_dev->center_x;
  const float dy = input_points[idx].y - filter_parameters_dev->center_y;
  const float radius = sqrtf(dx * dx + dy * dy);
  const float angle = fastAtan2_0_2Pi(dy, dx);  // replace with approximate atan

  // Determine the radial division index
  auto sector_index = static_cast<uint32_t>(angle * inv_sector_angle_rad);
  auto cell_index_in_sector =
    static_cast<uint32_t>(radius / filter_parameters_dev->cell_divider_size_m);
  auto cell_id =
    sector_index * filter_parameters_dev->max_num_cells_per_sector + cell_index_in_sector;

  if (cell_id >= filter_parameters_dev->max_num_cells) {
    return;  // Out of bounds
  }
  auto & cell = centroid_cells_list_dev[cell_id];
  uint32_t current_cell_points_num = atomicAdd(&cell.num_points, 1);
}

// Mark obstacle points for point in classified_points_dev
__global__ void markObstaclePointsKernel(
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev,
  const uint32_t max_num_classified_points, uint32_t * __restrict__ flags,
  const PointType pointtype)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= max_num_classified_points) {
    return;
  }
  // check if the classified_points_dev[idx] is existing?
  if (classified_points_dev[idx].radius < 0.0f) {
    return;
  }

  // extract origin index of point
  auto origin_index = classified_points_dev[idx].origin_index;
  auto point_type = classified_points_dev[idx].type;
  if (origin_index >= static_cast<size_t>(max_num_classified_points)) {
    return;
  }

  // Mark obstacle points for point in classified_points_dev

  flags[origin_index] = (point_type == pointtype) ? 1 : 0;
}

/**
 * @brief CUDA kernel to scatter selected input points into an output array based on flags and
 * prefix sum indices.
 *
 * This kernel iterates over each input point and, if the corresponding flag is set (non-zero),
 * copies the point to the output array at the position specified by the exclusive prefix sum in
 * `indices`.
 *
 * @param[in] input_points   Pointer to the array of input points.
 * @param[in] flags         Pointer to the array of flags indicating valid points (1 = valid, 0 =
 * invalid).
 * @param[in] indices       Pointer to the array of exclusive prefix sum indices for output
 * positions.
 * @param[in] num_points    Total number of input points.
 * @param[out] output_points Pointer to the array where selected points are written.
 *
 * @note This kernel assumes that `indices` is the result of an exclusive prefix sum over `flags`.
 *       Only threads corresponding to valid points (flags[idx] != 0) will write to the output
 * array.
 */
// input point idx:     0 1 2 3 4 5 6 7 8 9
// flags:               0 1 0 1 1 0 0 1 0 0
// indices:             0 0 1 1 2 3 3 3 4 4  <-- EXCLUSIVE PREFIX SUM
// output point idx:    0 1 2 3
// output points:       1 3 4 7
__global__ void scatterKernel(
  const PointTypeStruct * __restrict__ input_points, const uint32_t * __restrict__ flags,
  const uint32_t * __restrict__ indices, const uint32_t num_points,
  PointTypeStruct * __restrict__ output_points)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }
  // If the point is valid, copy it to the output points using the indices
  if (flags[idx]) {
    const uint32_t output_index = indices[idx];
    output_points[output_index] = input_points[idx];
  }
}

/**
 * @brief CUDA kernel to update the start point index for each cell centroid.
 *
 * This kernel assigns the corresponding start point index from the input array
 * `cell_first_point_indices_dev` to each element in the `centroid_cells_list_dev` array.
 * Each thread processes one cell, identified by its global thread index.
 *
 * @param[out] centroid_cells_list_dev  Device pointer to the array of cell centroids to update.
 * @param[in]  cell_first_point_indices_dev  Device pointer to the array containing the start point
 * indices for each cell.
 * @param[in]  max_num_cells  The total number of cells to process.
 */
__global__ void updateCellStartPointIndexKernel(
  CellCentroid * __restrict__ centroid_cells_list_dev,
  const size_t * __restrict__ cell_first_point_indices_dev, const uint32_t max_num_cells)
{
  uint32_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= max_num_cells) {
    return;  // Out of bounds
  }
  // Update the start point index for each cell
  centroid_cells_list_dev[idx].start_point_index = cell_first_point_indices_dev[idx];
}

}  // namespace

CudaScanGroundSegmentationFilter::CudaScanGroundSegmentationFilter(
  const FilterParameters & filter_parameters, const int64_t max_mem_pool_size_in_byte)
: filter_parameters_(filter_parameters)
{
  CHECK_CUDA_ERROR(cudaStreamCreate(&ground_segment_stream_));

  {
    int current_device_id = 0;
    CHECK_CUDA_ERROR(cudaGetDevice(&current_device_id));
    cudaMemPoolProps pool_props = {};
    pool_props.allocType = cudaMemAllocationTypePinned;
    pool_props.location.id = current_device_id;
    pool_props.location.type = cudaMemLocationTypeDevice;

    CHECK_CUDA_ERROR(cudaMemPoolCreate(&mem_pool_, &pool_props));

    uint64_t pool_release_threshold = max_mem_pool_size_in_byte;
    CHECK_CUDA_ERROR(cudaMemPoolSetAttribute(
      mem_pool_, cudaMemPoolAttrReleaseThreshold, static_cast<void *>(&pool_release_threshold)));
  }
}

template <typename T>
T * CudaScanGroundSegmentationFilter::allocateBufferFromPool(size_t num_elements)
{
  T * buffer{};
  CHECK_CUDA_ERROR(
    cudaMallocFromPoolAsync(&buffer, num_elements * sizeof(T), mem_pool_, ground_segment_stream_));
  CHECK_CUDA_ERROR(cudaMemsetAsync(buffer, 0, num_elements * sizeof(T), ground_segment_stream_));

  return buffer;
}

template <typename T>
void CudaScanGroundSegmentationFilter::returnBufferToPool(T * buffer)
{
  CHECK_CUDA_ERROR(cudaFreeAsync(buffer, ground_segment_stream_));
}

void CudaScanGroundSegmentationFilter::scanObstaclePoints(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
  PointTypeStruct * output_points_dev, size_t * num_output_points,
  CellCentroid * centroid_cells_list_dev)
{
  // Implementation of the function to scan obstacle points
  if (number_input_points_ == 0) {
    *num_output_points = 0;
    return;
  }
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
  ClassifiedPointTypeStruct * classified_points_dev, CellCentroid * centroid_cells_list_dev,
  const FilterParameters * filter_parameters_dev)
{
  const uint32_t num_sectors = filter_parameters_.num_sectors;
  if (num_sectors == 0) {
    return;
  }

  dim3 block_dim(1);
  dim3 grid_dim((num_sectors + block_dim.x - 1) / block_dim.x);

  // Launch the kernel to scan for ground points in each sector
  scanPerSectorGroundReferenceKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    classified_points_dev, centroid_cells_list_dev, filter_parameters_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

// =========== looping all input pointcloud and update cells ==================
void CudaScanGroundSegmentationFilter::countCellPointNum(
  const PointTypeStruct * input_points_dev, CellCentroid * centroid_cells_list_dev,
  const FilterParameters * filter_parameters_dev)
{
  // Implementation of the function to divide the point cloud into radial divisions
  // Sort the points in each radial division by distance from the center
  // return the indices of the points in each radial division
  if (number_input_points_ == 0) {
    return;
  }

  if (filter_parameters_.max_num_cells == 0) {
    return;
  }

  dim3 block_dim(512);

  dim3 cells_grid_dim((filter_parameters_.max_num_cells + block_dim.x - 1) / block_dim.x);

  dim3 points_grid_dim((number_input_points_ + block_dim.x - 1) / block_dim.x);

  // initialize the list of cells centroid
  CellsCentroidInitializeKernel<<<cells_grid_dim, block_dim, 0, ground_segment_stream_>>>(
    centroid_cells_list_dev, filter_parameters_.max_num_cells);
  CHECK_CUDA_ERROR(cudaGetLastError());

  calcCellPointNumberKernel<<<points_grid_dim, block_dim, 0, ground_segment_stream_>>>(
    input_points_dev, number_input_points_, filter_parameters_dev, centroid_cells_list_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

// ========== Assign each pointcloud to specific cell =========================
void CudaScanGroundSegmentationFilter::assignPointToClassifyPoint(
  const PointTypeStruct * input_points_dev, const CellCentroid * centroid_cells_list_dev,
  const FilterParameters * filter_parameters_dev, uint32_t * cell_counts_dev,
  ClassifiedPointTypeStruct * classified_points_dev)
{
  // implementation of the function to split point cloud into cells
  if (number_input_points_ == 0) {
    return;
  }
  dim3 block_dim(512);
  dim3 grid_dim((number_input_points_ + block_dim.x - 1) / block_dim.x);

  initPoints<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    classified_points_dev, number_input_points_);
  CHECK_CUDA_ERROR(cudaGetLastError());

  assignPointToClassifyPointKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    input_points_dev, number_input_points_, centroid_cells_list_dev, cell_counts_dev,
    filter_parameters_dev, classified_points_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
}

// ============= Extract non-ground points =============
void CudaScanGroundSegmentationFilter::extractNonGroundPoints(
  const PointTypeStruct * input_points_dev, ClassifiedPointTypeStruct * classified_points_dev,
  PointTypeStruct * output_points_dev, uint32_t & num_output_points_host, const PointType pointtype)
{
  if (number_input_points_ == 0) {
    num_output_points_host = 0;
    return;
  }
  auto * flag_dev = allocateBufferFromPool<uint32_t>(number_input_points_);

  dim3 block_dim(512);
  dim3 grid_dim((number_input_points_ + block_dim.x - 1) / block_dim.x);
  setFlagsKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    flag_dev, number_input_points_, 0);
  CHECK_CUDA_ERROR(cudaGetLastError());

  // auto * flag_dev = allocateBufferFromPool<uint32_t>(number_input_points_);
  auto * indices_dev = allocateBufferFromPool<uint32_t>(number_input_points_);
  void * temp_storage = nullptr;
  size_t temp_storage_bytes = 0;

  markObstaclePointsKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    classified_points_dev, number_input_points_, flag_dev, pointtype);

  cub::DeviceScan::ExclusiveSum(
    nullptr, temp_storage_bytes, flag_dev, indices_dev, static_cast<int>(number_input_points_),
    ground_segment_stream_);
  CHECK_CUDA_ERROR(
    cudaMallocFromPoolAsync(&temp_storage, temp_storage_bytes, mem_pool_, ground_segment_stream_));

  cub::DeviceScan::ExclusiveSum(
    temp_storage, temp_storage_bytes, flag_dev, indices_dev, static_cast<int>(number_input_points_),
    ground_segment_stream_);
  CHECK_CUDA_ERROR(cudaGetLastError());

  scatterKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    input_points_dev, flag_dev, indices_dev, number_input_points_, output_points_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
  // Count the number of valid points
  uint32_t last_index = 0;
  uint32_t last_flag = 0;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &last_index, indices_dev + number_input_points_ - 1, sizeof(uint32_t), cudaMemcpyDeviceToHost,
    ground_segment_stream_));

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &last_flag, flag_dev + number_input_points_ - 1, sizeof(uint32_t), cudaMemcpyDeviceToHost,
    ground_segment_stream_));

  num_output_points_host = last_flag + last_index;

  if (temp_storage) {
    CHECK_CUDA_ERROR(cudaFreeAsync(temp_storage, ground_segment_stream_));
  }
  returnBufferToPool(flag_dev);
  returnBufferToPool(indices_dev);
}

void CudaScanGroundSegmentationFilter::getCellFirstPointIndex(
  CellCentroid * centroid_cells_list_dev, uint32_t * num_points_per_cell_dev,
  size_t * cell_first_point_indices_dev)
{
  // Validate parameters to prevent invalid kernel launch configurations
  if (filter_parameters_.max_num_cells == 0 || filter_parameters_.num_sectors == 0) {
    return;
  }

  void * d_temp_storage = nullptr;

  size_t temp_storage_bytes = 0;
  uint32_t threads = filter_parameters_.num_sectors;
  uint32_t blocks = (filter_parameters_.max_num_cells + threads - 1) / threads;
  getCellNumPointsKernel<<<blocks, threads, 0, ground_segment_stream_>>>(
    centroid_cells_list_dev, filter_parameters_.max_num_cells, num_points_per_cell_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());

  cub::DeviceScan::ExclusiveSum(
    d_temp_storage, temp_storage_bytes, num_points_per_cell_dev, cell_first_point_indices_dev,
    static_cast<int>(filter_parameters_.max_num_cells), ground_segment_stream_);
  CHECK_CUDA_ERROR(cudaMalloc(&d_temp_storage, temp_storage_bytes));
  cub::DeviceScan::ExclusiveSum(
    d_temp_storage, temp_storage_bytes, num_points_per_cell_dev, cell_first_point_indices_dev,
    static_cast<int>(filter_parameters_.max_num_cells), ground_segment_stream_);

  // update start point index in centroid_cells_list_dev
  updateCellStartPointIndexKernel<<<blocks, threads, 0, ground_segment_stream_>>>(
    centroid_cells_list_dev, cell_first_point_indices_dev, filter_parameters_.max_num_cells);
  CHECK_CUDA_ERROR(cudaGetLastError());

  CHECK_CUDA_ERROR(cudaFree(d_temp_storage));
}

void CudaScanGroundSegmentationFilter::classifyPointcloud(
  const cuda_blackboard::CudaPointCloud2::ConstSharedPtr & input_points,
  cuda_blackboard::CudaPointCloud2::SharedPtr output_points,
  cuda_blackboard::CudaPointCloud2::SharedPtr ground_points)
{
  number_input_points_ = input_points->width * input_points->height;
  input_pointcloud_step_ = input_points->point_step;
  const size_t max_bytes = number_input_points_ * sizeof(PointTypeStruct);

  // auto output_points = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output_points->data = cuda_blackboard::make_unique<std::uint8_t[]>(max_bytes);
  ground_points->data = cuda_blackboard::make_unique<std::uint8_t[]>(max_bytes);

  auto * output_points_dev = reinterpret_cast<PointTypeStruct *>(output_points->data.get());
  auto * ground_points_dev = reinterpret_cast<PointTypeStruct *>(ground_points->data.get());
  uint32_t num_output_points = 0;

  output_points->header = input_points->header;
  output_points->height = 1;
  output_points->is_bigendian = input_points->is_bigendian;
  output_points->point_step = input_points->point_step;
  output_points->is_dense = input_points->is_dense;
  output_points->fields = input_points->fields;

  // ground points for debugging
  ground_points->header = input_points->header;
  ground_points->height = 1;
  ground_points->is_bigendian = input_points->is_bigendian;
  ground_points->point_step = input_points->point_step;
  ground_points->is_dense = input_points->is_dense;
  ground_points->fields = input_points->fields;

  if (number_input_points_ == 0) {
    output_points->width = static_cast<uint32_t>(num_output_points);
    output_points->row_step = static_cast<uint32_t>(num_output_points * sizeof(PointTypeStruct));

    ground_points->width = static_cast<uint32_t>(num_output_points);
    ground_points->row_step = static_cast<uint32_t>(num_output_points * sizeof(PointTypeStruct));
    return;
  }
  const auto * input_points_dev =
    reinterpret_cast<const PointTypeStruct *>(input_points->data.get());

  // Allocate and copy filter parameters to device
  auto * filter_parameters_dev = allocateBufferFromPool<FilterParameters>(1);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    filter_parameters_dev, &filter_parameters_, sizeof(FilterParameters), cudaMemcpyHostToDevice,
    ground_segment_stream_));

  const auto & max_num_cells = filter_parameters_.max_num_cells;
  auto * centroid_cells_list_dev = allocateBufferFromPool<CellCentroid>(max_num_cells);
  auto * num_points_per_cell_dev = allocateBufferFromPool<uint32_t>(max_num_cells);
  auto * cell_first_point_indices_dev = allocateBufferFromPool<size_t>(max_num_cells);
  auto * classified_points_dev =
    allocateBufferFromPool<ClassifiedPointTypeStruct>(number_input_points_);
  auto * cell_counts_dev = allocateBufferFromPool<uint32_t>(max_num_cells);

  countCellPointNum(input_points_dev, centroid_cells_list_dev, filter_parameters_dev);
  // calculate the index of the start point in each cell
  // update start point index into cell_first_point_indices_dev.start_point_index
  getCellFirstPointIndex(
    centroid_cells_list_dev, num_points_per_cell_dev, cell_first_point_indices_dev);

  assignPointToClassifyPoint(
    input_points_dev, centroid_cells_list_dev, filter_parameters_dev, cell_counts_dev,
    classified_points_dev);

  scanPerSectorGroundReference(
    classified_points_dev, centroid_cells_list_dev, filter_parameters_dev);

  // Extract obstacle points from classified_points_dev
  extractNonGroundPoints(
    input_points_dev, classified_points_dev, output_points_dev, num_output_points,
    PointType::NON_GROUND);

  // Extract ground points from classified_points_dev for debugging
  uint32_t num_ground_points = 0;
  extractNonGroundPoints(
    input_points_dev, classified_points_dev, ground_points_dev, num_ground_points,
    PointType::GROUND);

  returnBufferToPool(cell_counts_dev);
  returnBufferToPool(num_points_per_cell_dev);
  returnBufferToPool(cell_first_point_indices_dev);
  returnBufferToPool(classified_points_dev);
  returnBufferToPool(filter_parameters_dev);
  returnBufferToPool(centroid_cells_list_dev);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));

  output_points->width = static_cast<uint32_t>(num_output_points);
  output_points->row_step = static_cast<uint32_t>(num_output_points * sizeof(PointTypeStruct));

  ground_points->width = static_cast<uint32_t>(num_ground_points);
  ground_points->row_step = static_cast<uint32_t>(num_ground_points * sizeof(PointTypeStruct));
}

}  // namespace autoware::cuda_ground_segmentation
