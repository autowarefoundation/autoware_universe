// Copyright 2024 TIER IV, Inc.
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

#include "grid_ground_filter.hpp"

#include "data.hpp"

#include <pcl/PointIndices.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <vector>

namespace autoware::ground_segmentation
{

// float GridGroundFilter::getRadialDividerAngleRad(const float radius) const
// {
//   // Find the appropriate angle based on radius using binary search
//   // The map is sorted by radius in ascending order
//   // We find the last entry where entry.radius <= radius
//   if (param_.radial_divider_angle_map.empty()) {
//     throw std::runtime_error("radial_divider_angle_map is empty");
//   }

//   // Use binary search to find the first entry where entry.radius > radius
//   const auto it = std::upper_bound(
//     param_.radial_divider_angle_map.begin(), param_.radial_divider_angle_map.end(), radius,
//     [](const float r, const RadialDividerAngleEntry & entry) { return r < entry.radius; });

//   // If it points to the beginning, use the first entry
//   if (it == param_.radial_divider_angle_map.begin()) {
//     return param_.radial_divider_angle_map[0].angle_rad;
//   }

//   // If it points to the end, use the last entry
//   if (it == param_.radial_divider_angle_map.end()) {
//     return param_.radial_divider_angle_map.back().angle_rad;
//   }

//   // Use the previous entry (last entry where entry.radius <= radius)
//   return (it - 1)->angle_rad;
// }
// assign the pointcloud data to the grid
void GridGroundFilter::convert()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const size_t in_cloud_data_size = in_cloud_->data.size();
  const size_t in_cloud_point_step = in_cloud_->point_step;

  for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
       data_index += in_cloud_point_step) {
    // Get Point
    pcl::PointXYZ input_point;
    data_accessor_.getPoint(in_cloud_, data_index, input_point);
    grid_ptr_->addPoint(input_point.x, input_point.y, input_point.z, data_index);
  }
}

// preprocess the grid data, set the grid connections
void GridGroundFilter::preprocess()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // eliminate empty cells from connection for efficiency
  grid_ptr_->setGridConnections();
}

// iterative search for ground grid cells toward the origin (avoids recursion and stack use)
bool GridGroundFilter::recursiveSearch(
  const int check_idx, const int search_cnt, std::vector<int> & idx) const
{
  constexpr size_t count_limit = 1023;
  idx.clear();
  idx.reserve(static_cast<size_t>(search_cnt));
  int cur = check_idx;
  int remaining = search_cnt;
  size_t iterations = 0;
  while (remaining > 0 && cur >= 0 && iterations < count_limit) {
    ++iterations;
    const Cell & cell = grid_ptr_->getCellUnchecked(cur);
    if (cell.has_ground_) {
      idx.push_back(cur);
      --remaining;
    }
    cur = cell.scan_grid_root_idx_;
  }
  return remaining == 0;
}

// fit the line from the ground grid cells
void GridGroundFilter::fitLineFromGndGrid(const std::vector<int> & idx, float & a, float & b) const
{
  if (idx.empty()) {
    a = 0.0f;
    b = 0.0f;
    return;
  }
  if (idx.size() == 1) {
    const auto & cell = grid_ptr_->getCellUnchecked(idx.front());
    a = cell.avg_height_ / cell.avg_radius_;
    b = 0.0f;
    return;
  }
  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_xy = 0.0f;
  float sum_x2 = 0.0f;
  for (const auto & i : idx) {
    const auto & cell = grid_ptr_->getCellUnchecked(i);
    sum_x += cell.avg_radius_;
    sum_y += cell.avg_height_;
    sum_xy += cell.avg_radius_ * cell.avg_height_;
    sum_x2 += cell.avg_radius_ * cell.avg_radius_;
  }
  const float n = static_cast<float>(idx.size());
  const float denominator = n * sum_x2 - sum_x * sum_x;
  if (denominator != 0.0f) {
    a = (n * sum_xy - sum_x * sum_y) / denominator;
    a = std::clamp(a, -param_.global_slope_max_ratio, param_.global_slope_max_ratio);
    b = (sum_y - a * sum_x) / n;
  } else {
    const auto & cell = grid_ptr_->getCellUnchecked(idx.front());
    a = cell.avg_height_ / cell.avg_radius_;
    b = 0.0f;
  }
}

// process the grid data to initialize the ground cells prior to the ground segmentation
void GridGroundFilter::initializeGround(pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto grid_size = grid_ptr_->getGridSize();
  for (size_t idx = 0; idx < grid_size; ++idx) {
    auto & cell = grid_ptr_->getCellUnchecked(static_cast<int>(idx));
    if (cell.isEmpty()) continue;
    if (cell.is_ground_initialized_) continue;

    if (cell.scan_grid_root_idx_ >= 0) {
      const Cell & prev_cell = grid_ptr_->getCellUnchecked(cell.scan_grid_root_idx_);
      if (prev_cell.is_ground_initialized_) {
        cell.is_ground_initialized_ = true;
        continue;
      }
    }

    // initialize ground in this cell
    bool is_ground_found = false;
    PointsCentroid ground_bin;

    for (const auto & pt : cell.point_list_) {
      const size_t & pt_idx = pt.index;
      const float & radius = pt.distance;
      const float & height = pt.height;

      const float global_slope_threshold = param_.global_slope_max_ratio * radius;
      if (height >= global_slope_threshold && height > param_.non_ground_height_threshold) {
        // this point is obstacle
        out_no_ground_indices.indices.push_back(pt_idx);
      } else if (
        std::abs(height) < global_slope_threshold &&
        std::abs(height) < param_.non_ground_height_threshold) {
        // this point is ground
        ground_bin.addPoint(radius, height, pt_idx);
        is_ground_found = true;
      }
      // else, this point is not classified, not ground nor obstacle
    }
    cell.is_processed_ = true;
    cell.has_ground_ = is_ground_found;
    if (is_ground_found) {
      cell.is_ground_initialized_ = true;
      ground_bin.processAverage();
      cell.avg_height_ = ground_bin.getAverageHeight();
      cell.avg_radius_ = ground_bin.getAverageRadius();
      cell.max_height_ = ground_bin.getMaxHeight();
      cell.min_height_ = ground_bin.getMinHeight();
      cell.gradient_ = std::clamp(
        cell.avg_height_ / cell.avg_radius_, -param_.global_slope_max_ratio,
        param_.global_slope_max_ratio);
      cell.intercept_ = 0.0f;
    } else {
      cell.is_ground_initialized_ = false;
    }
  }
}

// segment the point in the cell, logic for the continuous cell
void GridGroundFilter::SegmentContinuousCell(
  const Cell & cell, const Cell & prev_cell, PointsCentroid & ground_bin,
  pcl::PointIndices & out_no_ground_indices)
{
  const float local_thresh_angle_ratio = std::tan(DEG2RAD(5.0));

  // loop over points in the cell
  for (const auto & pt : cell.point_list_) {
    const size_t & pt_idx = pt.index;
    const float & radius = pt.distance;
    const float & height = pt.height;

    // 1. height is out-of-range
    const float delta_z = height - prev_cell.avg_height_;
    if (delta_z > param_.detection_range_z_max) {
      // this point is out-of-range
      continue;
    }

    // 2. the angle is exceed the global slope threshold
    if (height > param_.global_slope_max_ratio * radius) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }

    // 3. local slope
    const float delta_radius = radius - prev_cell.avg_radius_;
    if (std::abs(delta_z) < param_.local_slope_max_ratio * delta_radius) {
      // this point is ground
      ground_bin.addPoint(radius, height, pt_idx);
      // go to the next point
      continue;
    }

    // 3. height from the estimated ground
    const float next_gnd_z = cell.gradient_ * radius + cell.intercept_;
    const float gnd_z_local_thresh = local_thresh_angle_ratio * delta_radius;
    const float delta_gnd_z = height - next_gnd_z;
    const float gnd_z_threshold = param_.non_ground_height_threshold + gnd_z_local_thresh;
    if (delta_gnd_z > gnd_z_threshold) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
    if (std::abs(delta_gnd_z) <= gnd_z_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, height, pt_idx);
      // go to the next point
      continue;
    }
    // else, this point is not classified, not ground nor obstacle
  }
}

// segment the point in the cell, logic for the discontinuous cell
void GridGroundFilter::SegmentDiscontinuousCell(
  const Cell & cell, const Cell & prev_cell, PointsCentroid & ground_bin,
  pcl::PointIndices & out_no_ground_indices)
{
  // loop over points in the cell
  for (const auto & pt : cell.point_list_) {
    const size_t & pt_idx = pt.index;
    const float & radius = pt.distance;
    const float & height = pt.height;

    // 1. height is out-of-range
    const float delta_avg_z = height - prev_cell.avg_height_;
    if (delta_avg_z > param_.detection_range_z_max) {
      // this point is out-of-range
      continue;
    }

    // 2. the angle is exceed the global slope threshold
    if (height > param_.global_slope_max_ratio * radius) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
    // 3. local slope
    const float delta_radius = radius - prev_cell.avg_radius_;
    const float local_slope_threshold = param_.local_slope_max_ratio * delta_radius;
    if (std::abs(delta_avg_z) < local_slope_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, height, pt_idx);
      // go to the next point
      continue;
    }
    // 4. height from the estimated ground
    if (std::abs(delta_avg_z) < param_.non_ground_height_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, height, pt_idx);
      // go to the next point
      continue;
    }
    const float delta_max_z = height - prev_cell.max_height_;
    if (std::abs(delta_max_z) < param_.non_ground_height_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, height, pt_idx);
      // go to the next point
      continue;
    }
    // 5. obstacle from local slope
    if (delta_avg_z >= local_slope_threshold) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
    // else, this point is not classified, not ground nor obstacle
  }
}

// segment the point in the cell, logic for the break cell
void GridGroundFilter::SegmentBreakCell(
  const Cell & cell, const Cell & prev_cell, PointsCentroid & ground_bin,
  pcl::PointIndices & out_no_ground_indices)
{
  // loop over points in the cell
  for (const auto & pt : cell.point_list_) {
    const size_t & pt_idx = pt.index;
    const float & radius = pt.distance;
    const float & height = pt.height;

    // 1. height is out-of-range
    const float delta_z = height - prev_cell.avg_height_;
    if (delta_z > param_.detection_range_z_max) {
      // this point is out-of-range
      continue;
    }

    // 2. the angle is exceed the global slope threshold
    if (height > param_.global_slope_max_ratio * radius) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }

    // 3. the point is over discontinuous grid
    const float delta_radius = radius - prev_cell.avg_radius_;
    const float global_slope_threshold = param_.global_slope_max_ratio * delta_radius;
    if (std::abs(delta_z) < global_slope_threshold) {
      // this point is ground
      ground_bin.addPoint(radius, height, pt_idx);
      // go to the next point
      continue;
    }
    if (delta_z >= global_slope_threshold) {
      // this point is obstacle
      out_no_ground_indices.indices.push_back(pt_idx);
      // go to the next point
      continue;
    }
    // else, this point is not classified, not ground nor obstacle
  }
}

// classify the point cloud into ground and non-ground points
void GridGroundFilter::classify(pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto grid_size = grid_ptr_->getGridSize();
  std::vector<int> grid_idcs;
  grid_idcs.reserve(static_cast<size_t>(param_.gnd_grid_buffer_size));
  PointsCentroid ground_bin;

  for (size_t idx = 0; idx < grid_size; ++idx) {
    auto & cell = grid_ptr_->getCellUnchecked(static_cast<int>(idx));
    if (cell.isEmpty()) continue;
    if (cell.is_processed_) continue;

    if (cell.scan_grid_root_idx_ < 0) continue;
    const Cell & prev_cell = grid_ptr_->getCellUnchecked(cell.scan_grid_root_idx_);
    if (!prev_cell.is_ground_initialized_) continue;

    recursiveSearch(cell.scan_grid_root_idx_, param_.gnd_grid_buffer_size, grid_idcs);

    enum SegmentationMode { NONE, CONTINUOUS, DISCONTINUOUS, BREAK };
    SegmentationMode mode = NONE;
    if (!grid_idcs.empty()) {
      const int front_radial_id = grid_ptr_->getCellUnchecked(grid_idcs.back()).radial_idx_ +
                                  static_cast<int>(grid_idcs.size());
      const float radial_diff_between_cells = cell.center_radius_ - prev_cell.center_radius_;

      if (radial_diff_between_cells < param_.gnd_grid_continual_thresh * cell.radial_size_) {
        if (cell.radial_idx_ - front_radial_id < param_.gnd_grid_continual_thresh) {
          mode = SegmentationMode::CONTINUOUS;
        } else {
          mode = SegmentationMode::DISCONTINUOUS;
        }
      } else {
        mode = SegmentationMode::BREAK;
      }
    }

    ground_bin.initialize();
    if (mode == SegmentationMode::CONTINUOUS) {
      float a, b;
      fitLineFromGndGrid(grid_idcs, a, b);
      cell.gradient_ = a;
      cell.intercept_ = b;
      SegmentContinuousCell(cell, prev_cell, ground_bin, out_no_ground_indices);
    } else if (mode == SegmentationMode::DISCONTINUOUS) {
      SegmentDiscontinuousCell(cell, prev_cell, ground_bin, out_no_ground_indices);
    } else if (mode == SegmentationMode::BREAK) {
      SegmentBreakCell(cell, prev_cell, ground_bin, out_no_ground_indices);
    }

    if (
      param_.use_recheck_ground_cluster && cell.avg_radius_ > param_.recheck_start_distance &&
      ground_bin.getGroundPointNum() > 0) {
      float reference_height = param_.use_lowest_point
                                 ? ground_bin.getMinHeightOnly()
                                 : (ground_bin.processAverage(), ground_bin.getAverageHeight());
      const float threshold = reference_height + param_.non_ground_height_threshold;
      const std::vector<size_t> & gnd_indices = ground_bin.getIndicesRef();
      const std::vector<float> & height_list = ground_bin.getHeightListRef();
      const size_t n_pts = height_list.size();
      for (size_t j = 0; j < n_pts; ++j) {
        if (height_list[j] >= threshold) {
          out_no_ground_indices.indices.push_back(gnd_indices[j]);
          ground_bin.is_ground_list[j] = false;
        }
      }
    }

    // finalize current cell, update the cell ground information
    if (ground_bin.getGroundPointNum() > 0) {
      ground_bin.processAverage();
      cell.avg_height_ = ground_bin.getAverageHeight();
      cell.avg_radius_ = ground_bin.getAverageRadius();
      cell.max_height_ = ground_bin.getMaxHeight();
      cell.min_height_ = ground_bin.getMinHeight();
      cell.has_ground_ = true;
    } else {
      // copy previous cell
      cell.avg_radius_ = prev_cell.avg_radius_;
      cell.avg_height_ = prev_cell.avg_height_;
      cell.max_height_ = prev_cell.max_height_;
      cell.min_height_ = prev_cell.min_height_;
      cell.has_ground_ = false;
    }

    cell.is_processed_ = true;
  }
}

// process the point cloud to segment the ground points
void GridGroundFilter::process(
  const PointCloud2ConstPtr & in_cloud, pcl::PointIndices & out_no_ground_indices)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  in_cloud_ = in_cloud;

  out_no_ground_indices.indices.clear();
  out_no_ground_indices.indices.reserve(in_cloud->width * in_cloud->height);

  // reset grid cells
  grid_ptr_->resetCells();

  // 1. assign points to grid cells
  convert();

  // 2. cell preprocess
  preprocess();

  // 3. initialize ground
  initializeGround(out_no_ground_indices);

  // 4. classify point cloud
  classify(out_no_ground_indices);
}

}  // namespace autoware::ground_segmentation
