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

#ifndef SCAN_GROUND_FILTER__GRID_HPP_
#define SCAN_GROUND_FILTER__GRID_HPP_

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

namespace
{

float pseudoArcTan2(const float y, const float x)
{
  // lightweight arc tangent

  // avoid divide-by-zero
  if (y == 0.0f) {
    if (x >= 0.0f) return 0.0f;
    return M_PIf;
  }
  if (x == 0.0f) {
    if (y >= 0.0f) return M_PI_2f;
    return -M_PI_2f;
  }

  const float x_abs = std::abs(x);
  const float y_abs = std::abs(y);

  // divide to 8 zones
  constexpr float M_2PIf = 2.0f * M_PIf;
  constexpr float M_3PI_2f = 3.0f * M_PI_2f;
  if (x_abs > y_abs) {
    const float ratio = y_abs / x_abs;
    const float angle = ratio * M_PI_4f;
    if (y >= 0.0f) {
      if (x >= 0.0f) return angle;  // 1st zone
      return M_PIf - angle;         // 2nd zone
    } else {
      if (x >= 0.0f) return M_2PIf - angle;  // 4th zone
      return M_PIf + angle;                  // 3rd zone
    }
  } else {
    const float ratio = x_abs / y_abs;
    const float angle = ratio * M_PI_4f;
    if (y >= 0.0f) {
      if (x >= 0.0f) return M_PI_2f - angle;  // 1st zone
      return M_PI_2f + angle;                 // 2nd zone
    } else {
      if (x >= 0.0f) return M_3PI_2f + angle;  // 4th zone
      return M_3PI_2f - angle;                 // 3rd zone
    }
  }
}

}  // namespace

namespace autoware::ground_segmentation
{
using autoware_utils::ScopedTimeTrack;

struct Point
{
  size_t index;
  float distance;
  float height;
};

// Radial divider angle map: stores radius thresholds and corresponding divider angles
struct RadialDividerAngleEntry
{
  float radius;     // radius threshold in meters
  float angle_rad;  // angle in radians
};

// Concentric Zone Model (CZM) based polar grid
class Cell
{
public:
  // list of point indices
  std::vector<Point> point_list_;  // point index and distance

  // method to check if the cell is empty
  inline bool isEmpty() const { return point_list_.empty(); }

  // index of the cell
  int grid_idx_;
  int radial_idx_;
  int azimuth_idx_;
  int prev_grid_idx_;

  int scan_grid_root_idx_;

  // geometric properties of the cell
  float center_radius_;
  float center_azimuth_;
  float radial_size_;
  float azimuth_size_;

  // ground statistics of the points in the cell
  float avg_height_;
  float max_height_;
  float min_height_;
  float avg_radius_;
  float gradient_;
  float intercept_;

  // process flags
  bool is_processed_ = false;
  bool is_ground_initialized_ = false;
  bool has_ground_ = false;
};

class Grid
{
public:
  Grid(const float origin_x, const float origin_y) : origin_x_(origin_x), origin_y_(origin_y) {}
  ~Grid() = default;

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
  {
    time_keeper_ = std::move(time_keeper_ptr);
  }

  void initialize(
    const float grid_dist_size,
    const std::vector<RadialDividerAngleEntry> & radial_divider_angle_map,
    const float grid_radial_limit,
    const float default_angle_rad = 0.0f,
    const float front_azimuth_half_span_rad = 0.0f)
  {
    grid_dist_size_ = grid_dist_size;
    radial_divider_angle_map_ = radial_divider_angle_map;
    grid_radial_limit_ = grid_radial_limit;
    default_angle_rad_ = default_angle_rad;
    front_azimuth_half_span_rad_ = front_azimuth_half_span_rad;
    use_front_region_ = (front_azimuth_half_span_rad_ > 0.0f && default_angle_rad_ > 0.0f);
    grid_radial_max_num_ = std::ceil(grid_radial_limit / grid_dist_size_);
    grid_dist_size_inv_ = 1.0f / grid_dist_size_;

    // generate grid geometry
    setGridBoundaries();

    // initialize and resize cells
    cells_.clear();
    cells_.resize(radial_idx_offsets_.back() + azimuth_grids_per_radial_.back());

    // set cell geometry
    setCellGeometry();

    // set initialized flag
    is_initialized_ = true;
  }

  // method to add a point to the grid
  void addPoint(const float x, const float y, const float z, const size_t point_idx)
  {
    const float x_fixed = x - origin_x_;
    const float y_fixed = y - origin_y_;
    const float radius = std::sqrt(x_fixed * x_fixed + y_fixed * y_fixed);
    const float azimuth = pseudoArcTan2(y_fixed, x_fixed);

    // calculate the grid id
    const int grid_idx = getGridIdx(radius, azimuth);

    // check if the point is within the grid
    if (grid_idx < 0) {
      return;
    }
    const size_t grid_idx_idx = static_cast<size_t>(grid_idx);

    // add the point to the cell
    cells_[grid_idx_idx].point_list_.emplace_back(Point{point_idx, radius, z});
  }

  size_t getGridSize() const { return cells_.size(); }

  // method to get the cell
  inline Cell & getCell(const int grid_idx)
  {
    const size_t idx = static_cast<size_t>(grid_idx);
    if (idx >= cells_.size()) {
      throw std::out_of_range("Invalid grid index");
    }
    return cells_[idx];
  }

  /** Same as getCell but no bounds check; use only when grid_idx is known valid (faster hot path). */
  inline Cell & getCellUnchecked(const int grid_idx)
  {
    return cells_[static_cast<size_t>(grid_idx)];
  }
  inline const Cell & getCellUnchecked(const int grid_idx) const
  {
    return cells_[static_cast<size_t>(grid_idx)];
  }

  void resetCells()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

    for (auto & cell : cells_) {
      cell.point_list_.clear();
      cell.is_processed_ = false;
      cell.is_ground_initialized_ = false;
      cell.has_ground_ = false;
    }
  }

  void setGridConnections()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

    // iterate over grid cells
    for (Cell & cell : cells_) {
      // find and link the scan-grid root cell
      cell.scan_grid_root_idx_ = cell.prev_grid_idx_;
      while (cell.scan_grid_root_idx_ >= 0) {
        const auto & prev_cell = cells_[cell.scan_grid_root_idx_];
        // if the previous cell has point, set the previous cell as the root cell
        if (!prev_cell.isEmpty()) break;
        // keep searching the previous cell
        cell.scan_grid_root_idx_ = prev_cell.scan_grid_root_idx_;
      }
      // if the grid root idx reaches -1, finish the search
    }
  }

private:
  // given parameters
  float origin_x_;
  float origin_y_;
  float grid_dist_size_ = 1.0f;  // meters
  std::vector<RadialDividerAngleEntry> radial_divider_angle_map_;

  // calculated parameters
  float grid_dist_size_inv_ = 0.0f;  // inverse of the grid size in meters
  bool is_initialized_ = false;

  // configured parameters
  float grid_radial_limit_ = 200.0f;  // meters
  int grid_radial_max_num_ = 0;
  float default_angle_rad_ = 0.0f;       // for non-front region when use_front_region_
  float front_azimuth_half_span_rad_ = 0.0f;  // front sector: +/- this angle (rad)
  bool use_front_region_ = false;        // if true, use map only in front sector

  // array of grid boundaries
  std::vector<float> grid_radial_boundaries_;
  std::vector<int> azimuth_grids_per_radial_;
  std::vector<float> azimuth_interval_per_radial_;
  std::vector<int> radial_idx_offsets_;
  // front-region: per-radial front/rest split (only used when use_front_region_)
  std::vector<int> front_azimuth_cells_per_radial_;
  std::vector<float> front_azimuth_interval_per_radial_;
  std::vector<int> rest_azimuth_cells_per_radial_;
  std::vector<float> rest_azimuth_interval_per_radial_;

  // list of cells
  std::vector<Cell> cells_;

  // debug information
  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  /*!
   * Get radial divider angle based on radius using binary search
   * @param[in] radius Radial distance in meters
   * @return Radial divider angle in radians
   */
  float getRadialDividerAngleRad(const float radius) const
  {
    if (radial_divider_angle_map_.empty()) {
      throw std::runtime_error("radial_divider_angle_map is empty");
    }

    // Use binary search to find the first entry where entry.radius > radius
    // The map is sorted by radius in ascending order
    const auto it = std::upper_bound(
      radial_divider_angle_map_.begin(), radial_divider_angle_map_.end(), radius,
      [](const float r, const RadialDividerAngleEntry & entry) { return r < entry.radius; });

    // If it points to the beginning, use the first entry
    if (it == radial_divider_angle_map_.begin()) {
      return radial_divider_angle_map_[0].angle_rad;
    }

    // If it points to the end, use the last entry
    if (it == radial_divider_angle_map_.end()) {
      return radial_divider_angle_map_.back().angle_rad;
    }

    // Use the previous entry (last entry where entry.radius <= radius)
    return (it - 1)->angle_rad;
  }

  // Generate grid geometry
  // the grid is cylindrical mesh grid
  // azimuth interval: dynamic angle based on radial_divider_angle_map (angle_rad varies by radius)
  // radial interval: constant distance
  void setGridBoundaries()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

    // radial boundaries
    {
      // constant distance
      for (int i = 0; i < grid_radial_max_num_; i++) {
        grid_radial_boundaries_.push_back(i * grid_dist_size_);
      }
    }

    const size_t radial_grid_num = grid_radial_boundaries_.size();
    constexpr float two_pi = 2.0f * M_PIf;

    // azimuth boundaries - dynamic based on radial_divider_angle_map (or front region + default)
    azimuth_grids_per_radial_.resize(radial_grid_num);
    azimuth_interval_per_radial_.resize(radial_grid_num);
    if (use_front_region_) {
      front_azimuth_cells_per_radial_.resize(radial_grid_num);
      front_azimuth_interval_per_radial_.resize(radial_grid_num);
      rest_azimuth_cells_per_radial_.resize(radial_grid_num);
      rest_azimuth_interval_per_radial_.resize(radial_grid_num);
      const float front_span = 2.0f * front_azimuth_half_span_rad_;
      const float rest_span = two_pi - front_span;
      for (size_t i = 0; i < radial_grid_num; ++i) {
        const float cell_radius = grid_radial_boundaries_[i];
        const float front_angle_rad = getRadialDividerAngleRad(cell_radius);
        const int front_cells = std::max(
          1, static_cast<int>(std::ceil(front_span / front_angle_rad)));
        const float front_interval = front_span / static_cast<float>(front_cells);
        const int rest_cells =
          std::max(1, static_cast<int>(std::ceil(rest_span / default_angle_rad_)));
        const float rest_interval = rest_span / static_cast<float>(rest_cells);
        front_azimuth_cells_per_radial_[i] = front_cells;
        front_azimuth_interval_per_radial_[i] = front_interval;
        rest_azimuth_cells_per_radial_[i] = rest_cells;
        rest_azimuth_interval_per_radial_[i] = rest_interval;
        azimuth_grids_per_radial_[i] = front_cells + rest_cells;
        azimuth_interval_per_radial_[i] = two_pi / static_cast<float>(front_cells + rest_cells);
      }
    } else {
      // full circle uses radial_divider_angle_map
      struct CachedAngleConfig
      {
        float angle_rad;
        int azimuth_grid_num;
        float azimuth_interval;
      };
      std::vector<std::pair<float, CachedAngleConfig>> angle_cache;

      if (!radial_divider_angle_map_.empty()) {
        for (const auto & entry : radial_divider_angle_map_) {
          const float angle_rad = entry.angle_rad;
          const int azimuth_grid_num = static_cast<int>(std::ceil(two_pi / angle_rad));
          const float azimuth_interval = two_pi / static_cast<float>(azimuth_grid_num);
          angle_cache.emplace_back(
            entry.radius, CachedAngleConfig{angle_rad, azimuth_grid_num, azimuth_interval});
        }
        if (angle_cache.size() > 0) {
          const auto & last_config = angle_cache.back().second;
          angle_cache.emplace_back(
            grid_radial_limit_ + 1.0f,
            CachedAngleConfig{last_config.angle_rad, last_config.azimuth_grid_num,
                              last_config.azimuth_interval});
        }
      }

      if (angle_cache.empty()) {
        for (size_t i = 0; i < radial_grid_num; ++i) {
          const float cell_radius = grid_radial_boundaries_[i];
          const float angle_rad = getRadialDividerAngleRad(cell_radius);
          const int azimuth_grid_num = static_cast<int>(std::ceil(two_pi / angle_rad));
          const float azimuth_interval = two_pi / static_cast<float>(azimuth_grid_num);
          azimuth_grids_per_radial_[i] = azimuth_grid_num;
          azimuth_interval_per_radial_[i] = azimuth_interval;
        }
      } else {
        size_t cache_idx = 0;
        for (size_t i = 0; i < radial_grid_num; ++i) {
          const float cell_radius = grid_radial_boundaries_[i];
          while (
            cache_idx < angle_cache.size() - 2 &&
            cell_radius >= angle_cache[cache_idx + 1].first) {
            ++cache_idx;
          }
          const auto & config = angle_cache[cache_idx].second;
          azimuth_grids_per_radial_[i] = config.azimuth_grid_num;
          azimuth_interval_per_radial_[i] = config.azimuth_interval;
        }
      }
    }

    // accumulate the number of azimuth grids per radial grid, set offset for each radial grid
    radial_idx_offsets_.resize(radial_grid_num);
    radial_idx_offsets_[0] = 0;
    for (size_t i = 1; i < radial_grid_num; ++i) {
      radial_idx_offsets_[i] = radial_idx_offsets_[i - 1] + azimuth_grids_per_radial_[i - 1];
    }
  }

  int getAzimuthGridIdx(const int & radial_idx, const float & azimuth) const
  {
    constexpr float two_pi = 2.0f * M_PIf;
    const float az =
      (azimuth >= 0.0f && azimuth < two_pi) ? azimuth : (azimuth < 0.0f ? azimuth + two_pi : azimuth - two_pi);

    if (use_front_region_) {
      const int front_cells = front_azimuth_cells_per_radial_[radial_idx];
      const int rest_cells = rest_azimuth_cells_per_radial_[radial_idx];
      const float front_interval = front_azimuth_interval_per_radial_[radial_idx];
      const float rest_interval = rest_azimuth_interval_per_radial_[radial_idx];
      const float half = front_azimuth_half_span_rad_;
      const float front_span = 2.0f * half;
      if (az <= half || az >= two_pi - half) {
        float offset = (az <= half) ? az : (az - (two_pi - front_span));
        int front_idx = static_cast<int>(std::floor(offset / front_interval));
        if (front_idx >= front_cells) front_idx = front_cells - 1;
        if (front_idx < 0) front_idx = 0;
        return front_idx;
      }
      int rest_idx =
        static_cast<int>(std::floor((az - half) / rest_interval));
      if (rest_idx >= rest_cells) rest_idx = rest_cells - 1;
      if (rest_idx < 0) rest_idx = 0;
      return front_cells + rest_idx;
    }

    const int azimuth_grid_num = azimuth_grids_per_radial_[radial_idx];
    int azimuth_grid_idx =
      static_cast<int>(std::floor(az / azimuth_interval_per_radial_[radial_idx]));
    if (azimuth_grid_idx == azimuth_grid_num) {
      azimuth_grid_idx = 0;
    }
    return azimuth_grid_idx;
  }

  int getRadialIdx(const float & radius) const
  {
    // check if the point is within the grid
    if (radius > grid_radial_limit_) {
      return -1;
    }
    if (radius < 0) {
      return -1;
    }
    return static_cast<int>(radius * grid_dist_size_inv_);
  }

  int getGridIdx(const int & radial_idx, const int & azimuth_idx) const
  {
    return radial_idx_offsets_[radial_idx] + azimuth_idx;
  }

  // method to determine the grid id of a point
  // -1 means out of range
  // range limit is horizon angle
  int getGridIdx(const float & radius, const float & azimuth) const
  {
    const int grid_rad_idx = getRadialIdx(radius);
    if (grid_rad_idx < 0) {
      return -1;
    }

    // azimuth grid id
    const int grid_az_idx = getAzimuthGridIdx(grid_rad_idx, azimuth);
    if (grid_az_idx < 0) {
      return -1;
    }

    return getGridIdx(grid_rad_idx, grid_az_idx);
  }

  void getRadialAzimuthIdxFromCellIdx(const int cell_id, int & radial_idx, int & azimuth_idx) const
  {
    radial_idx = -1;
    azimuth_idx = -1;
    for (size_t i = 0; i < radial_idx_offsets_.size(); ++i) {
      if (cell_id < radial_idx_offsets_[i]) {
        radial_idx = i - 1;
        azimuth_idx = cell_id - radial_idx_offsets_[i - 1];
        break;
      }
    }
    if (cell_id >= radial_idx_offsets_.back()) {
      radial_idx = radial_idx_offsets_.size() - 1;
      azimuth_idx = cell_id - radial_idx_offsets_.back();
    }
  }

  void setCellGeometry()
  {
    std::unique_ptr<ScopedTimeTrack> st_ptr;
    if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

    constexpr float two_pi = 2.0f * M_PIf;

    for (size_t idx = 0; idx < cells_.size(); ++idx) {
      Cell & cell = cells_[idx];

      int radial_idx = 0;
      int azimuth_idx = 0;
      getRadialAzimuthIdxFromCellIdx(static_cast<int>(idx), radial_idx, azimuth_idx);

      cell.grid_idx_ = static_cast<int>(idx);
      cell.radial_idx_ = radial_idx;
      cell.azimuth_idx_ = azimuth_idx;

      const auto radial_grid_num = static_cast<int>(grid_radial_boundaries_.size() - 1);
      if (radial_idx < radial_grid_num) {
        cell.radial_size_ =
          grid_radial_boundaries_[radial_idx + 1] - grid_radial_boundaries_[radial_idx];
      } else {
        cell.radial_size_ = grid_radial_limit_ - grid_radial_boundaries_[radial_idx];
      }

      if (use_front_region_) {
        const int front_cells = front_azimuth_cells_per_radial_[radial_idx];
        const float front_interval = front_azimuth_interval_per_radial_[radial_idx];
        const float rest_interval = rest_azimuth_interval_per_radial_[radial_idx];
        const float half = front_azimuth_half_span_rad_;
        const int front_cells_in_first_half = std::min(
          front_cells,
          static_cast<int>(std::floor(half / front_interval)) + 1);
        if (azimuth_idx < front_cells) {
          cell.azimuth_size_ = front_interval;
          if (azimuth_idx < front_cells_in_first_half) {
            cell.center_azimuth_ =
              (static_cast<float>(azimuth_idx) + 0.5f) * front_interval;
          } else {
            cell.center_azimuth_ = two_pi - half +
              (static_cast<float>(azimuth_idx - front_cells_in_first_half) + 0.5f) * front_interval;
          }
        } else {
          const int rest_idx = azimuth_idx - front_cells;
          cell.azimuth_size_ = rest_interval;
          cell.center_azimuth_ =
            half + (static_cast<float>(rest_idx) + 0.5f) * rest_interval;
        }
      } else {
        cell.azimuth_size_ = azimuth_interval_per_radial_[radial_idx];
        cell.center_azimuth_ =
          (static_cast<float>(azimuth_idx) + 0.5f) * cell.azimuth_size_;
      }

      cell.center_radius_ = grid_radial_boundaries_[radial_idx] + cell.radial_size_ * 0.5f;

      int prev_grid_idx = -1;
      if (radial_idx > 0) {
        const float azimuth = cell.center_azimuth_;
        const int azimuth_idx_prev =
          getAzimuthGridIdx(radial_idx - 1, azimuth);
        prev_grid_idx = getGridIdx(radial_idx - 1, azimuth_idx_prev);
      }
      cell.prev_grid_idx_ = prev_grid_idx;
      cell.scan_grid_root_idx_ = -1;
    }
  }
};

}  // namespace autoware::ground_segmentation

#endif  // SCAN_GROUND_FILTER__GRID_HPP_
