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
  // const float PI = 3.14159265358979323846f;
  // const float PI2 = 6.28318530717958647692f;
  // const float PI_2 = 1.57079632679489661923f;

  // // Avoid divide-by-zero
  // float abs_y = fabsf(y) + 1e-10f;

  // float r, angle;

  // if (x >= 0.0f) {
  //   // First and fourth quadrants
  //   r = (x - abs_y) / (x + abs_y);
  //   angle = PI_2 - r * (0.2447f + 0.0663f * fabsf(r));  // polynomial approx
  //   if (y < 0.0f) {
  //     angle = PI2 - angle;  // 4th quadrant
  //   }
  // } else {
  //   // Second and third quadrants
  //   r = (x + abs_y) / (abs_y - x);
  //   angle = 3.0f * PI_2 - r * (0.2447f + 0.0663f * fabsf(r));  // poly approx
  //   if (y < 0.0f) {
  //     angle = angle - PI2;  // wrap into [0, 2π]
  //   }
  // }
  float angle = atan2f(y, x);  // Returns [-π, π]
  if (angle < 0.0f) {
    angle += 2.0f * M_PI;  // Convert to [0, 2π]
  }
  return angle;
}
__device__ inline int getCellID(
  const PointTypeStruct & point, const float center_x, const float center_y,
  const float inv_sector_angle_rad, const float cell_size_m, const int max_num_cells_per_sector,
  const int max_num_cells)
{
  const float dx = point.x - center_x;
  const float dy = point.y - center_y;
  const float radius = sqrtf(dx * dx + dy * dy);
  const float angle = fastAtan2_0_2Pi(dy, dx);
  // Determine the sector index
  int division_sector_index = static_cast<int>(angle * inv_sector_angle_rad);

  // Determine the radial cell index

  int cell_index_in_sector = static_cast<int>(radius / cell_size_m);

  // combine to get unique cell ID

  int cell_id = division_sector_index * max_num_cells_per_sector + cell_index_in_sector;

  // clamp invalid values

  if (cell_id < 0 || cell_id >= max_num_cells) {
    return -1;
  }
  return cell_id;
}

__global__ void initPoints(ClassifiedPointTypeStruct * arr, int N)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= N) return;
  arr[idx].z = 0.0f;
  arr[idx].type = PointType::INIT;
  arr[idx].radius = -1.0f;
  arr[idx].origin_index = 0;
}

__global__ void setFlagsKernel(int * flags, int n, const int value)
{
  size_t i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i < n) flags[i] = value;  // write real int 0 or 1
}

__global__ void getCellNumPointsKernel(
  const CellCentroid * __restrict__ centroid_cells_list_dev, const size_t num_cells,
  int * __restrict__ num_points_per_cell)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_cells) {
    return;  // Out of bounds
  }
  num_points_per_cell[idx] = centroid_cells_list_dev[idx].num_points;
}

__global__ void assignPointToClassifyPointKernel(
  const PointTypeStruct * __restrict__ input_points, const size_t num_points,
  const CellCentroid * __restrict__ cells_centroid_dev, int * __restrict__ cell_counts_dev,
  const FilterParameters * __restrict__ filter_parameters_dev,
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev)
{
  // This kernel split pointcloud into sectors and cells
  // Each point is allocated to a cell
  // The number points in each cells is set as num_points_per_cell_dev
  // The point is allocated to a cell based on its angle and distance from the center
  // This is a placeholder for the actual implementation
  // memory index for classified_points_dev is calculated as

  size_t idx = static_cast<size_t>(blockIdx.x * blockDim.x + threadIdx.x);
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
  auto division_sector_index = static_cast<int>(angle * inv_sector_angle_rad);
  auto cell_index_in_sector = static_cast<int>(radius / filter_parameters_dev->cell_divider_size_m);
  auto cell_id =
    division_sector_index * filter_parameters_dev->max_num_cells_per_sector + cell_index_in_sector;
  if (cell_id < 0 || cell_id >= filter_parameters_dev->max_num_cells) {
    return;  // Out of bounds
  }

  // calc index of point in classified_point_dev
  // the last index of point in cell is located in cell_counts_dev
  // attomically get slot index in the current cell
  int slot_idx = atomicAdd(&cell_counts_dev[cell_id], 1);
  // check local bounds for slot_idx
  if (slot_idx >= cells_centroid_dev[cell_id].num_points) {
    return;  // Out of bounds
  }
  auto classify_point_idx =
    static_cast<size_t>(cells_centroid_dev[cell_id].start_point_index + slot_idx);
  // Check overall bounds for classified_points_dev
  if (classify_point_idx >= num_points) {
    return;  // Out of bounds
  }
  // add pointcloud to output grid list
  auto & assign_classified_point_dev = classified_points_dev[classify_point_idx];
  assign_classified_point_dev.z = input_points[idx].z;
  assign_classified_point_dev.type = PointType::INIT;

  assign_classified_point_dev.radius = radius;
  assign_classified_point_dev.origin_index = idx;  // index in the original point cloud
  // Update the cell centroid
}

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
__device__ void removeGndPointInCell(CellCentroid & cell, const ClassifiedPointTypeStruct & point)
{
  cell.gnd_height_avg =
    (cell.gnd_height_avg * cell.num_ground_points - point.z) / (cell.num_ground_points - 1);
  cell.gnd_radius_avg =
    (cell.gnd_radius_avg * cell.num_ground_points - point.radius) / (cell.num_ground_points - 1);
  cell.num_ground_points--;
}

__device__ void checkSegmentMode(
  const int cell_index_in_sector, const int * last_gnd_cells_dev, const int num_latest_gnd_cells,
  const int gnd_cell_continual_thresh, const int gnd_cell_buffer_size, SegmentationMode & mode)
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
    mode = SegmentationMode::CONTINUOUS;  // If the latest ground cell is within threshold, set mode
                                          // to CONTINUOUS
    return;
  }
  mode = SegmentationMode::DISCONTINUOUS;  // If the latest ground cell is not within threshold, set
                                           // mode to DISCONTINUOUS
  return;
}

// __device__ void checkSegmentMode(
//   const CellCentroid * centroid_cells, const int cell_idx_in_sector, const int
//   sector_start_index, const int continues_checking_cell_num, SegmentationMode & mode)
// {
//   mode = SegmentationMode::UNINITIALIZED;
//   if (cell_idx_in_sector == 0) {
//     // If this is the first cell in the sector, we need to check the previous cells
//     return;
//   }
//   // UNITIALIZED if all previous cell in the same sector has no ground points
//   int prev_cell_id_in_sector = cell_idx_in_sector - 1;
//   for (; prev_cell_id_in_sector >= 0; --prev_cell_id_in_sector) {
//     // find the latest cell with ground points
//     auto prev_cell_in_sector = centroid_cells[sector_start_index + prev_cell_id_in_sector];
//     if (prev_cell_in_sector.num_ground_points > 0) {
//       break;
//     }
//   }
//   if (prev_cell_id_in_sector < 0) {
//     // If no previous cell has ground points, set mode to UNINITIALIZED
//     mode = SegmentationMode::UNINITIALIZED;
//     return;
//   }

//   // If previous cell has no points (there's a gap), set mode to BREAK
//   if (prev_cell_id_in_sector < cell_idx_in_sector - 1) {
//     mode = SegmentationMode::BREAK;
//     return;
//   }

//   // Check if all continuous checking previous cells have points for CONTINUOUS mode
//   mode = SegmentationMode::CONTINUOUS;
//   for (int i = cell_idx_in_sector - 1;
//        i > cell_idx_in_sector - continues_checking_cell_num && i >= 0; --i) {
//     auto & check_cell = centroid_cells[sector_start_index + i];
//     if (check_cell.num_ground_points == 0) {
//       mode = SegmentationMode::DISCONTINUOUS;
//       return;  // If any previous cell has no ground points, set mode to DISCONTINUOUS
//     }
//   }
// }

__device__ void RecursiveGndCellSearch(
  const CellCentroid * __restrict__ sector_cells_list_dev, const int last_gnd_cells_num_threshold,
  const int cell_index_in_sector, int * last_gnd_cells_dev, int & num_latest_gnd_cells)
{
  // Recursively search for ground cells in the sector
  if (cell_index_in_sector < 0) {
    return;  // Base case: no more cells to check
  }
  if (num_latest_gnd_cells >= last_gnd_cells_num_threshold) {  // TODO: check the condition is
                                                               // correct
    return;  // Stop if we have enough ground cells
  }
  const auto & current_cell = sector_cells_list_dev[cell_index_in_sector];

  if (current_cell.num_ground_points > 0) {
    // If the cell has enough ground points, add it to the list
    last_gnd_cells_dev[num_latest_gnd_cells++] = cell_index_in_sector;
  }

  // Continue searching in the previous cell
  RecursiveGndCellSearch(
    sector_cells_list_dev, last_gnd_cells_num_threshold, cell_index_in_sector - 1,
    last_gnd_cells_dev, num_latest_gnd_cells);
}

__device__ float calcLocalGndGradient(
  const CellCentroid * __restrict__ centroid_cells, const int continues_checking_cell_num,
  const int sector_start_index, const int cell_idx_in_sector, const float gradient_threshold)
{
  // Calculate the local ground gradient based on the previous cells
  if (continues_checking_cell_num < 2) {
    return 0.0f;  // Not enough data to calculate gradient
  }
  auto cell_id = sector_start_index + cell_idx_in_sector;

  float start_gnd_cell_avg_height =
    centroid_cells[cell_id - continues_checking_cell_num].gnd_height_avg;
  float start_gnd_cell_avg_radius =
    centroid_cells[cell_id - continues_checking_cell_num].gnd_radius_avg;
  float gradient = 0.0f;
  int valid_gradients = 0;

  // Calculate gradients from reference to each valid previous cell
  for (int i = 1; i < continues_checking_cell_num; ++i) {
    const auto & prev_cell = centroid_cells[cell_id - i];
    if (prev_cell.num_ground_points > 0) {
      float dz = prev_cell.gnd_height_avg - start_gnd_cell_avg_height;
      float dr = prev_cell.gnd_radius_avg - start_gnd_cell_avg_radius;

      // Avoid division by zero
      if (fabsf(dr) > 1e-6f) {
        gradient += dz / dr;
        valid_gradients++;
      }
    }
  }

  if (valid_gradients == 0) {
    return 0.0f;  // No valid gradients found
  }
  gradient /= valid_gradients;
  gradient = gradient > gradient_threshold ? gradient_threshold : gradient;    // Clamp to threshold
  gradient = gradient < -gradient_threshold ? -gradient_threshold : gradient;  // Clamp to threshold
  return gradient;  // Return average gradient
}

__device__ float fitLineFromGndCell(
  const CellCentroid * __restrict__ sector_cells_list_dev, const int * last_gnd_cells_dev,
  const int num_fitting_cells, const FilterParameters * __restrict__ filter_parameters_dev)
{
  float a = 0.0f;
  float b = 0.0f;

  if (num_fitting_cells == 0) {
    a = 0.0f;  // No fitting cells, set to zero
    b = 0.0f;  // No fitting cells, set to zero
    return a;  // No fitting cells, return zero slope
  }

  if (num_fitting_cells == 1) {
    auto cell_idx_in_sector = last_gnd_cells_dev[0];
    const auto & cell = sector_cells_list_dev[cell_idx_in_sector];
    a = cell.gnd_height_avg / cell.gnd_radius_avg;
    b = 0.0f;  // Only one point, no line fitting needed
    return a;  // Return the slope based on the single point
  }

  // calculate the line by least squares method
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xy = 0.0f;
  float sum_xx = 0.0f;
  for (int i = 0; i < num_fitting_cells; ++i) {
    const auto & cell_idx_in_sector = last_gnd_cells_dev[i];
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
    const auto & cell_idx_in_sector = last_gnd_cells_dev[0];
    const auto & cell = sector_cells_list_dev[cell_idx_in_sector];
    a = cell.gnd_height_avg / cell.gnd_radius_avg;
    b = 0.0f;
    return a;  // If denominator is zero, return slope based on the first cell
  } else {
    a = (num_fitting_cells * sum_xy - sum_x * sum_y) / denominator;  // slope
    a = a > filter_parameters_dev->global_slope_max_ratio
          ? filter_parameters_dev->global_slope_max_ratio
          : a;  // Clamp to threshold
    a = a < -filter_parameters_dev->global_slope_max_ratio
          ? -filter_parameters_dev->global_slope_max_ratio
          : a;                                            // Clamp to threshold
    b = (sum_y * sum_xx - sum_x * sum_xy) / denominator;  // intercept
  }
  return a;
}
__device__ void recheckCell(
  CellCentroid & cell, ClassifiedPointTypeStruct * __restrict__ classify_points,
  const FilterParameters * __restrict__ filter_parameters_dev)
{
  // This function is called to recheck the current cell
  // It should be implemented based on the specific requirements of the segmentation algorithm
  auto const idx_start_point_of_cell = cell.start_point_index;
  if (cell.num_ground_points < 2) {
    // If the cell has less than 2 ground points, we can skip rechecking
    return;
  }
  for (int i = 0; i < cell.num_points; i++) {
    // Recheck the point
    auto point_idx = static_cast<size_t>(idx_start_point_of_cell + i);
    auto & point = classify_points[point_idx];
    if (point.type != PointType::GROUND) {
      continue;
    }
    // Apply the rechecking logic
    if (point.z > cell.gnd_height_min + filter_parameters_dev->non_ground_height_threshold) {
      point.type = PointType::NON_GROUND;
      removeGndPointInCell(cell, point);
    }
  }
}

__device__ void SegmentInitializedCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const int cell_idx_in_sector)
{
  auto & current_cell = sector_cells_list_dev[cell_idx_in_sector];  // Use reference, not copy
  auto const idx_start_point_of_cell = current_cell.start_point_index;
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
    float slope_ratio = point.z / point.radius;
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
    filter_parameters_dev->use_recheck_ground_cluster && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}
/*
This function segments all points in continuous cells
This cell has a continual previous gnd cells
*/
__device__ void SegmentContinuousCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const int cell_idx_in_sector,
  const int * last_gnd_cells_dev, const int num_latest_gnd_cells)
{
  // compare point of current cell with previous cell center by local slope angle
  // auto gnd_gradient = calcLocalGndGradient(
  //   centroid_cells, filter_parameters_dev->gnd_cell_buffer_size, sector_start_cell_index,
  //   cell_idx_in_sector, filter_parameters_dev->global_slope_max_ratio);

  auto gnd_gradient = fitLineFromGndCell(
    sector_cells_list_dev, last_gnd_cells_dev, num_latest_gnd_cells, filter_parameters_dev);
  int cell_id = cell_idx_in_sector;
  auto & current_cell = sector_cells_list_dev[cell_id];  // Use reference, not copy
  auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;
  auto & prev_gnd_cell = sector_cells_list_dev[last_gnd_cells_dev[0]];
  auto const prev_cell_gnd_height = prev_gnd_cell.gnd_height_avg;

  for (size_t i = 0; i < num_points_of_cell; ++i) {
    auto & point = cell_classify_points_dev[i];
    // 1. height is out-of-range compared to previous cell gnd
    if (point.z - prev_cell_gnd_height > filter_parameters_dev->detection_range_z_max) {
      point.type = PointType::OUT_OF_RANGE;
      return;
    }

    auto d_radius = point.radius - prev_gnd_cell.gnd_radius_avg;
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
    filter_parameters_dev->use_recheck_ground_cluster && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}
/*
This function is to classify point in a discontinuous cell
Which has few discontinual gnd cells before
*/

__device__ void SegmentDiscontinuousCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const int cell_idx_in_sector,
  const int * last_gnd_cells_dev, const int num_latest_gnd_cells)
{
  auto cell_id = cell_idx_in_sector;
  auto & current_cell = sector_cells_list_dev[cell_id];  // Use reference, not copy
  auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;
  auto & prev_gnd_cell = sector_cells_list_dev[last_gnd_cells_dev[0]];

  for (int i = 0; i < num_points_of_cell; ++i) {
    size_t point_idx = static_cast<size_t>(i);
    auto & point = cell_classify_points_dev[point_idx];
    // 1. height is out-of-range
    if (point.z - prev_gnd_cell.gnd_height_avg > filter_parameters_dev->detection_range_z_max) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    // 2. the angle is exceed the global slope threshold
    auto dz = point.z - prev_gnd_cell.gnd_height_avg;
    auto d_radius = point.radius - prev_gnd_cell.gnd_radius_avg;

    if (point.z > filter_parameters_dev->global_slope_max_ratio * point.radius) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    if (dz > filter_parameters_dev->local_slope_max_ratio * d_radius) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    // 3. local slope
    if (dz < -filter_parameters_dev->local_slope_max_ratio * d_radius) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    if (point.z < -filter_parameters_dev->global_slope_max_ratio * point.radius) {
      // If the point is below the estimated ground height, classify it as non-ground
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    point.type = PointType::GROUND;  // Mark as ground point
    updateGndPointInCell(current_cell, point);
  }

  if (
    filter_parameters_dev->use_recheck_ground_cluster && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}

/*
This function is called when the prev gnd cell is far from current cell
*/

__device__ void SegmentBreakCell(
  CellCentroid * __restrict__ sector_cells_list_dev,
  ClassifiedPointTypeStruct * __restrict__ cell_classify_points_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, const int cell_idx_in_sector,
  const int * last_gnd_cells_dev, const int num_latest_gnd_cells)
{
  // This function is called when the cell is not continuous with the previous cell
  auto cell_id = cell_idx_in_sector;
  auto & current_cell = sector_cells_list_dev[cell_id];  // Use reference, not copy
  auto const idx_start_point_of_cell = current_cell.start_point_index;
  auto const num_points_of_cell = current_cell.num_points;
  auto & prev_gnd_cell = sector_cells_list_dev[last_gnd_cells_dev[0]];

  for (int i = 0; i < num_points_of_cell; ++i) {
    auto & point = cell_classify_points_dev[i];

    // 1. height is out-of-range
    if (point.z - prev_gnd_cell.gnd_height_avg > filter_parameters_dev->detection_range_z_max) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }

    auto dz = point.z - prev_gnd_cell.gnd_height_avg;
    auto d_radius = point.radius - prev_gnd_cell.gnd_radius_avg;

    // 3. Global slope check
    if (dz > filter_parameters_dev->global_slope_max_ratio * d_radius) {
      point.type = PointType::NON_GROUND;
      continue;
    }
    if (dz < -filter_parameters_dev->global_slope_max_ratio * d_radius) {
      point.type = PointType::OUT_OF_RANGE;
      continue;
    }
    point.type = PointType::GROUND;
    updateGndPointInCell(current_cell, point);
  }
  if (
    filter_parameters_dev->use_recheck_ground_cluster && current_cell.num_ground_points > 1 &&
    current_cell.gnd_radius_avg > filter_parameters_dev->recheck_start_distance) {
    // Recheck the ground points in the cell
    recheckCell(current_cell, cell_classify_points_dev, filter_parameters_dev);
  }
}

__global__ void scanPerSectorGroundReferenceKernel(
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev,
  CellCentroid * __restrict__ centroid_cells_list_dev,
  const FilterParameters * __restrict__ filter_parameters_dev, int * last_gnd_cells_dev)
{
  // Implementation of the kernel
  // scan in each sector from cell_index_in_sector = 0 to max_num_cells_per_sector
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
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
  auto * sector_last_gnd_cells_dev =
    &last_gnd_cells_dev[sector_start_cell_index * filter_parameters_dev->gnd_cell_buffer_size];
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
    int num_latest_gnd_cells = 0;
    // get the latest gnd cells in the sector
    size_t first_sector_classified_point_index =
      sector_cells_list_dev[cell_index_in_sector].start_point_index;
    ClassifiedPointTypeStruct * cell_classify_points_dev =
      &classified_points_dev[first_sector_classified_point_index];
    auto * last_gnd_cells_dev =
      &sector_last_gnd_cells_dev
        [cell_index_in_sector * filter_parameters_dev->gnd_cell_buffer_size];

    RecursiveGndCellSearch(
      sector_cells_list_dev, filter_parameters_dev->gnd_cell_buffer_size, cell_index_in_sector - 1,
      last_gnd_cells_dev, num_latest_gnd_cells);

    // check the segmentation Mode based on prevoius gnd cells
    checkSegmentMode(
      cell_index_in_sector, last_gnd_cells_dev, num_latest_gnd_cells,
      filter_parameters_dev->gnd_grid_continual_thresh, filter_parameters_dev->gnd_cell_buffer_size,
      mode);

    if (mode == SegmentationMode::UNINITIALIZED) {
      SegmentInitializedCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector);
    } else if (mode == SegmentationMode::CONTINUOUS) {
      SegmentContinuousCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector, last_gnd_cells_dev, num_latest_gnd_cells);
    } else if (mode == SegmentationMode::DISCONTINUOUS) {
      SegmentDiscontinuousCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector, last_gnd_cells_dev, num_latest_gnd_cells);
    } else if (mode == SegmentationMode::BREAK) {
      SegmentBreakCell(
        sector_cells_list_dev, cell_classify_points_dev, filter_parameters_dev,
        cell_index_in_sector, last_gnd_cells_dev, num_latest_gnd_cells);
    }
    // if the first round of scan
  }
}

__global__ void sortPointsInCellsKernel(
  const int * __restrict__ num_points_per_cell_dev,
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev, const int max_num_cells,
  const int max_num_points_per_cell)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= max_num_cells) {
    return;  // Out of bounds
  }

  auto * cell_points = classified_points_dev + idx * max_num_points_per_cell;
  int num_points_in_cell = num_points_per_cell_dev[idx];
  if (num_points_in_cell <= 1) {
    return;  // No need to sort if there is 0 or 1 point in the cell
  }
  // Sort the points in the cell by radius using a cub::DeviceRadixSort
  // the points are located in cell_points, and the number of points is num_points_in_cell
}

__global__ void CellsCentroidInitializeKernel(
  CellCentroid * __restrict__ centroid_cells_list_dev, const int max_num_cells)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= max_num_cells) {
    return;  // Out of bounds
  }
  centroid_cells_list_dev[idx].gnd_radius_avg = 0.0f;
  centroid_cells_list_dev[idx].gnd_height_avg = 0.0f;
  centroid_cells_list_dev[idx].gnd_height_min = 1e6f;
  centroid_cells_list_dev[idx].num_points = 0;
  centroid_cells_list_dev[idx].cell_id = -1;  // Initialize cell_id to -1
  centroid_cells_list_dev[idx].num_ground_points = 0;
  centroid_cells_list_dev[idx].start_point_index = 0;
}

__global__ void calcCellPointNumberKernel(
  const PointTypeStruct * __restrict__ input_points, const size_t num_input_points,
  const FilterParameters * __restrict__ filter_parameters_dev,
  CellCentroid * __restrict__ centroid_cells_list_dev)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
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
  auto sector_index = static_cast<int>(angle * inv_sector_angle_rad);
  auto cell_index_in_sector = static_cast<int>(radius / filter_parameters_dev->cell_divider_size_m);
  auto cell_id =
    sector_index * filter_parameters_dev->max_num_cells_per_sector + cell_index_in_sector;

  // const auto cell_id = getCellID(
  //   input_points[idx], center_x, center_y, inv_sector_angle_rad, cell_size_m,
  //   max_number_cels_per_sector, max_num_cells);

  if (cell_id < 0 || cell_id >= filter_parameters_dev->max_num_cells) {
    return;  // Out of bounds
  }
  // add pointcloud to output grid list
  // Update the existing grid
  auto & cell = centroid_cells_list_dev[cell_id];
  int current_cell_points_num = atomicAdd(&cell.num_points, 1);
}

// Mark obstacle points for point in classified_points_dev
__global__ void markObstaclePointsKernel(
  ClassifiedPointTypeStruct * __restrict__ classified_points_dev,
  const int max_num_classified_points, const size_t num_points, int * __restrict__ flags,
  const PointType pointtype)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= static_cast<size_t>(max_num_classified_points)) {
    return;
  }
  // check if the classified_points_dev[idx] is existing?
  if (classified_points_dev[idx].radius < 0.0f) {
    return;
  }

  // extract origin index of point
  auto origin_index = classified_points_dev[idx].origin_index;
  auto point_type = classified_points_dev[idx].type;
  if (origin_index >= static_cast<size_t>(num_points)) {
    return;
  }

  // Mark obstacle points for point in classified_points_dev

  flags[origin_index] = (point_type == pointtype) ? 1 : 0;
}

__global__ void markValidKernel(
  const PointTypeStruct * __restrict__ input_points, const size_t num_points, float z_threshold,
  int * __restrict__ flags)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }
  // Mark the point as valid if its z value is above the height threshold
  flags[idx] = (input_points[idx].z > z_threshold) ? 1 : 0;
}

__global__ void scatterKernel(
  const PointTypeStruct * __restrict__ input_points, const int * __restrict__ flags,
  const int * __restrict__ indices, size_t num_points, PointTypeStruct * __restrict__ output_points)
{
  size_t idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_points) {
    return;
  }
  // If the point is valid, copy it to the output points using the indices
  if (flags[idx]) {
    const int output_index = indices[idx];
    output_points[output_index] = input_points[idx];
  }
}

__global__ void updateCellStartPointIndexKernel(
  CellCentroid * __restrict__ centroid_cells_list_dev,
  const size_t * __restrict__ cell_first_point_indices_dev, const int max_num_cells)
{
  int idx = blockIdx.x * blockDim.x + threadIdx.x;
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
  // Return (but not actual) working buffer to the pool
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
    return;  // No points to process
  }
}

// ============= Sort points in each cell by radius =============
void CudaScanGroundSegmentationFilter::sortPointsInCells(
  const int * num_points_per_cell_dev, ClassifiedPointTypeStruct * classified_points_dev)
{
  (void)num_points_per_cell_dev;
  (void)classified_points_dev;
}

// ============ Scan per sector to get ground reference and Non-Ground points =============
void CudaScanGroundSegmentationFilter::scanPerSectorGroundReference(
  ClassifiedPointTypeStruct * classified_points_dev, CellCentroid * centroid_cells_list_dev,
  const FilterParameters * filter_parameters_dev, int * last_gnd_cells_dev)
{
  const int num_sectors = filter_parameters_.num_sectors;

  // Validate parameters to prevent invalid kernel launch configurations
  if (num_sectors == 0) {
    return;  // No sectors to process
  }

  // Ensure block size doesn't exceed CUDA limits (max 1024 threads per block)
  dim3 block_dim(1);
  dim3 grid_dim((num_sectors + block_dim.x - 1) / block_dim.x);

  // Launch the kernel to scan for ground points in each sector
  scanPerSectorGroundReferenceKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    classified_points_dev, centroid_cells_list_dev, filter_parameters_dev, last_gnd_cells_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));
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
    return;  // No points to process
  }

  // Additional validation for kernel parameters
  if (filter_parameters_.max_num_cells == 0) {
    return;  // No cells to initialize
  }

  // Launch the kernel to divide the point cloud into radial divisions
  // Each thread will process one point and calculate its angle and distance from the center

  dim3 block_dim(512);

  // For CellsCentroidInitializeKernel: grid size based on max_num_cells
  dim3 cells_grid_dim((filter_parameters_.max_num_cells + block_dim.x - 1) / block_dim.x);

  // For calcCellPointNumberKernel: grid size based on number_input_points_
  dim3 points_grid_dim((number_input_points_ + block_dim.x - 1) / block_dim.x);

  // initialize the list of cells centroid
  CellsCentroidInitializeKernel<<<cells_grid_dim, block_dim, 0, ground_segment_stream_>>>(
    centroid_cells_list_dev, filter_parameters_.max_num_cells);
  CHECK_CUDA_ERROR(cudaGetLastError());

  auto max_cells_num = filter_parameters_.max_num_cells;
  calcCellPointNumberKernel<<<points_grid_dim, block_dim, 0, ground_segment_stream_>>>(
    input_points_dev, number_input_points_, filter_parameters_dev, centroid_cells_list_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());

  // CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));
}

// ========== Assign each pointcloud to specific cell =========================
void CudaScanGroundSegmentationFilter::assignPointToClassifyPoint(
  const PointTypeStruct * input_points_dev, const CellCentroid * centroid_cells_list_dev,
  const FilterParameters * filter_parameters_dev, int * cell_counts_dev,
  ClassifiedPointTypeStruct * classified_points_dev)
{
  // implementation of the function to split point cloud into cells
  if (number_input_points_ == 0) {
    return;  // No points to process
  }
  // Initialize the centroid_cells_list_dev value
  dim3 block_dim(512);
  dim3 grid_dim((number_input_points_ + block_dim.x - 1) / block_dim.x);

  initPoints<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    classified_points_dev, number_input_points_);
  CHECK_CUDA_ERROR(cudaGetLastError());

  assignPointToClassifyPointKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    input_points_dev, number_input_points_, centroid_cells_list_dev, cell_counts_dev,
    filter_parameters_dev, classified_points_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));
}

// ============= Extract non-ground points =============
void CudaScanGroundSegmentationFilter::extractNonGroundPoints(
  const PointTypeStruct * input_points_dev, ClassifiedPointTypeStruct * classified_points_dev,
  PointTypeStruct * output_points_dev, size_t & num_output_points_host, const PointType pointtype)
{
  if (number_input_points_ == 0) {
    num_output_points_host = 0;
    return;  // No points to process
  }
  int * flag_dev = allocateBufferFromPool<int>(
    number_input_points_);  // list flag of Non-Groud pointcloud related to classified_points_dev

  dim3 block_dim(512);
  dim3 grid_dim((number_input_points_ + block_dim.x - 1) / block_dim.x);
  setFlagsKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    flag_dev, number_input_points_, 1);
  CHECK_CUDA_ERROR(cudaGetLastError());

  // auto * flag_dev = allocateBufferFromPool<int>(number_input_points_);
  auto * indices_dev = allocateBufferFromPool<int>(number_input_points_);
  void * temp_storage = nullptr;
  size_t temp_storage_bytes = 0;

  markObstaclePointsKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    classified_points_dev, number_input_points_, number_input_points_, flag_dev, pointtype);
  // CHECK_CUDA_ERROR(cudaMemset(&flag_dev,1,number_input_points_));
  // CHECK_CUDA_ERROR(cudaGetLastError());

  cub::DeviceScan::ExclusiveSum(
    nullptr, temp_storage_bytes, flag_dev, indices_dev, static_cast<int>(number_input_points_),
    ground_segment_stream_);
  CHECK_CUDA_ERROR(
    cudaMallocFromPoolAsync(&temp_storage, temp_storage_bytes, mem_pool_, ground_segment_stream_));

  cub::DeviceScan::ExclusiveSum(
    temp_storage, temp_storage_bytes, flag_dev, indices_dev, static_cast<int>(number_input_points_),
    ground_segment_stream_);
  CHECK_CUDA_ERROR(
    cudaMallocFromPoolAsync(&temp_storage, temp_storage_bytes, mem_pool_, ground_segment_stream_));

  CHECK_CUDA_ERROR(cudaGetLastError());

  scatterKernel<<<grid_dim, block_dim, 0, ground_segment_stream_>>>(
    input_points_dev, flag_dev, indices_dev, number_input_points_, output_points_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());
  // Count the number of valid points
  int last_index = 0;
  int last_flag = 0;
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &last_index, indices_dev + number_input_points_ - 1, sizeof(int), cudaMemcpyDeviceToHost,
    ground_segment_stream_));

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    &last_flag, flag_dev + number_input_points_ - 1, sizeof(int), cudaMemcpyDeviceToHost,
    ground_segment_stream_));
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));

  const size_t num_output_points = static_cast<size_t>(last_flag + last_index);
  num_output_points_host = num_output_points;

  if (temp_storage) {
    CHECK_CUDA_ERROR(cudaFreeAsync(temp_storage, ground_segment_stream_));
  }
  returnBufferToPool(flag_dev);
  returnBufferToPool(indices_dev);
}

void CudaScanGroundSegmentationFilter::getCellFirstPointIndex(
  const FilterParameters * filter_parameters_dev, CellCentroid * centroid_cells_list_dev,
  int * num_points_per_cell_dev, size_t * cell_first_point_indices_dev)
{
  // Validate parameters to prevent invalid kernel launch configurations
  if (filter_parameters_.max_num_cells == 0 || filter_parameters_.num_sectors == 0) {
    return;  // No cells or sectors to process
  }

  void * d_temp_storage = nullptr;

  size_t temp_storage_bytes = 0;
  int threads = filter_parameters_.num_sectors;
  int blocks = (filter_parameters_.max_num_cells + threads - 1) / threads;
  getCellNumPointsKernel<<<blocks, threads, 0, ground_segment_stream_>>>(
    centroid_cells_list_dev, filter_parameters_.max_num_cells, num_points_per_cell_dev);
  CHECK_CUDA_ERROR(cudaGetLastError());

  // accumulate num_points_per_cell_dev into cell_first_point_indices_dev
  //  Exclusive scan
  // First call: get temporary storage size
  cub::DeviceScan::ExclusiveSum(
    d_temp_storage, temp_storage_bytes, num_points_per_cell_dev, cell_first_point_indices_dev,
    filter_parameters_.max_num_cells, ground_segment_stream_);
  CHECK_CUDA_ERROR(cudaMalloc(&d_temp_storage, temp_storage_bytes));
  cub::DeviceScan::ExclusiveSum(
    d_temp_storage, temp_storage_bytes, num_points_per_cell_dev, cell_first_point_indices_dev,
    filter_parameters_.max_num_cells, ground_segment_stream_);

  // update start point index in centroid_cells_list_dev
  updateCellStartPointIndexKernel<<<blocks, threads, 0, ground_segment_stream_>>>(
    centroid_cells_list_dev, cell_first_point_indices_dev, filter_parameters_.max_num_cells);
  CHECK_CUDA_ERROR(cudaGetLastError());
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));

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
  size_t num_output_points = 0;

  output_points->header = input_points->header;
  output_points->height = 1;  // Set height to 1 for unorganized point cloud
  output_points->is_bigendian = input_points->is_bigendian;
  output_points->point_step = input_points->point_step;
  output_points->is_dense = input_points->is_dense;
  output_points->fields = input_points->fields;

  ground_points->header = input_points->header;
  ground_points->height = 1;  // Set height to 1 for unorganized point cloud
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
  // CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));

  // split pointcloud to radial divisions
  // sort points in each radial division by distance from the center
  const auto & max_num_cells = filter_parameters_.max_num_cells;
  auto * centroid_cells_list_dev = allocateBufferFromPool<CellCentroid>(max_num_cells);
  auto * num_points_per_cell_dev = allocateBufferFromPool<int>(max_num_cells);
  auto * cell_first_point_indices_dev = allocateBufferFromPool<size_t>(max_num_cells);
  auto * classified_points_dev =
    allocateBufferFromPool<ClassifiedPointTypeStruct>(number_input_points_);
  auto * cell_counts_dev = allocateBufferFromPool<int>(max_num_cells);
  auto * last_gnd_cells_dev =
    allocateBufferFromPool<int>(max_num_cells * filter_parameters_.gnd_cell_buffer_size);

  // calculate the centroid of each cell

  countCellPointNum(input_points_dev, centroid_cells_list_dev, filter_parameters_dev);
  // calculate the index of the start point in each cell
  // update start point index into cell_first_point_indices_dev.start_point_index
  getCellFirstPointIndex(
    filter_parameters_dev, centroid_cells_list_dev, num_points_per_cell_dev,
    cell_first_point_indices_dev);

  assignPointToClassifyPoint(
    input_points_dev, centroid_cells_list_dev, filter_parameters_dev, cell_counts_dev,
    classified_points_dev);

  scanPerSectorGroundReference(
    classified_points_dev, centroid_cells_list_dev, filter_parameters_dev, last_gnd_cells_dev);

  // Extract obstacle points from classified_points_dev
  extractNonGroundPoints(
    input_points_dev, classified_points_dev, output_points_dev, num_output_points,
    PointType::NON_GROUND);

  size_t num_ground_points = 0;
  extractNonGroundPoints(
    input_points_dev, classified_points_dev, ground_points_dev, num_ground_points,
    PointType::GROUND);

  // // mark valid points based on height threshold
  // Return the device memory to pool

  returnBufferToPool(cell_counts_dev);
  returnBufferToPool(num_points_per_cell_dev);
  returnBufferToPool(cell_first_point_indices_dev);
  returnBufferToPool(classified_points_dev);
  returnBufferToPool(last_gnd_cells_dev);
  returnBufferToPool(filter_parameters_dev);
  returnBufferToPool(centroid_cells_list_dev);

  CHECK_CUDA_ERROR(cudaStreamSynchronize(ground_segment_stream_));

  output_points->width = static_cast<uint32_t>(num_output_points);
  output_points->row_step = static_cast<uint32_t>(num_output_points * sizeof(PointTypeStruct));

  // return output_points;

  ground_points->width = static_cast<uint32_t>(num_ground_points);
  ground_points->row_step = static_cast<uint32_t>(num_ground_points * sizeof(PointTypeStruct));
}

}  // namespace autoware::cuda_ground_segmentation
