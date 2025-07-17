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

#ifndef AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_

#include <array>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <vector>

namespace autoware::diffusion_planner
{
constexpr size_t POINT_STATE_DIM = 7;

enum PolylineLabel { LANE = 0, ROAD_LINE = 1, ROAD_EDGE = 2, CROSSWALK = 3 };

// Normalize a 3D direction vector (shared utility)
inline void normalize_direction(float & dx, float & dy, float & dz)
{
  const float magnitude = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (magnitude > 1e-6f) {
    dx /= magnitude;
    dy /= magnitude;
    dz /= magnitude;
  }
}

struct LanePoint
{
  // Construct a new instance filling all elements by `0.0f`.
  LanePoint() : data_({0.0f}) {}

  /**
   * @brief Construct a new instance with specified values.
   *
   * @param x X position.
   * @param y Y position.
   * @param z Z position.
   * @param dx Normalized delta x.
   * @param dy Normalized delta y.
   * @param dz Normalized delta z.
   * @param label Label.
   */
  LanePoint(
    const float x, const float y, const float z, const float dx, const float dy, const float dz,
    const float label)
  : data_({x, y, z, dx, dy, dz, label}), x_(x), y_(y), z_(z), dx_(x), dy_(y), dz_(z), label_(label)
  {
  }

  // Construct a new instance filling all elements by `0.0f`.
  static LanePoint empty() noexcept { return {}; }

  // Return the point state dimensions `D`.
  static size_t dim() { return POINT_STATE_DIM; }

  // Return the x position of the point.
  [[nodiscard]] float x() const { return x_; }

  // Return the y position of the point.
  [[nodiscard]] float y() const { return y_; }

  // Return the z position of the point.
  [[nodiscard]] float z() const { return z_; }

  // Return the x direction of the point.
  [[nodiscard]] float dx() const { return dx_; }

  // Return the y direction of the point.
  [[nodiscard]] float dy() const { return dy_; }

  // Return the z direction of the point.
  [[nodiscard]] float dz() const { return dz_; }

  // Return the label of the point.
  [[nodiscard]] float label() const { return label_; }

  /**
   * @brief Return the distance between myself and another one.
   *
   * @param other Another point.
   * @return float Distance between myself and another one.
   */
  [[nodiscard]] float distance(const LanePoint & other) const
  {
    return std::hypot(x_ - other.x(), y_ - other.y(), z_ - other.z());
  }

  // Return the address pointer of data array.
  [[nodiscard]] const float * data_ptr() const noexcept { return data_.data(); }

  [[nodiscard]] LanePoint lerp(const LanePoint & other, float t) const
  {
    // Interpolate position
    float new_x = x_ + t * (other.x_ - x_);
    float new_y = y_ + t * (other.y_ - y_);
    float new_z = z_ + t * (other.z_ - z_);

    // Calculate direction vector from interpolated positions
    float new_dx = other.x_ - x_;
    float new_dy = other.y_ - y_;
    float new_dz = other.z_ - z_;

    // Check if points are too close
    const float magnitude_sq = new_dx * new_dx + new_dy * new_dy + new_dz * new_dz;
    if (magnitude_sq < 1e-12f) {
      // If points are too close, use the first point's direction
      new_dx = dx_;
      new_dy = dy_;
      new_dz = dz_;
    } else {
      normalize_direction(new_dx, new_dy, new_dz);
    }

    // Interpolate label
    float new_label = label_ + t * (other.label_ - label_);

    return LanePoint{new_x, new_y, new_z, new_dx, new_dy, new_dz, new_label};
  }

private:
  std::array<float, POINT_STATE_DIM> data_;
  float x_{0.0f}, y_{0.0f}, z_{0.0f}, dx_{0.0f}, dy_{0.0f}, dz_{0.0f}, label_{0.0f};
};

enum class MapType {
  Lane,
  Crosswalk,
  Unused,
  // ...
};

class Polyline
{
public:
  static constexpr std::array<int, 3> XYZ_IDX = {0, 1, 2};
  static constexpr std::array<int, 2> XY_IDX = {0, 1};
  static constexpr int FULL_DIM3D = 7;
  static constexpr int FULL_DIM2D = 5;

  Polyline() = default;

  explicit Polyline(MapType type) : polyline_type_(type) {}

  Polyline(MapType type, const std::vector<LanePoint> & points)
  : polyline_type_(type), waypoints_(points)
  {
  }

  void assign_waypoints(const std::vector<LanePoint> & points) { waypoints_ = points; }

  void clear() { waypoints_.clear(); }

  [[nodiscard]] bool is_empty() const { return waypoints_.empty(); }

  [[nodiscard]] size_t size() const { return waypoints_.size(); }

  [[nodiscard]] const std::vector<LanePoint> & waypoints() const { return waypoints_; }

  [[nodiscard]] const MapType & polyline_type() const { return polyline_type_; }

  [[nodiscard]] std::vector<std::array<float, 3>> xyz() const
  {
    std::vector<std::array<float, 3>> coords;
    coords.reserve(waypoints_.size());
    for (const auto & pt : waypoints_) {
      coords.push_back({pt.x(), pt.y(), pt.z()});
    }
    return coords;
  }

  [[nodiscard]] std::vector<std::array<float, 2>> xy() const
  {
    std::vector<std::array<float, 2>> coords;
    coords.reserve(waypoints_.size());
    for (const auto & pt : waypoints_) {
      coords.push_back({pt.x(), pt.y()});
    }
    return coords;
  }

  [[nodiscard]] std::vector<std::array<float, 3>> dxyz() const
  {
    if (waypoints_.empty()) return {};

    std::vector<std::array<float, 3>> directions(waypoints_.size(), {0.0f, 0.0f, 0.0f});
    for (size_t i = 1; i < waypoints_.size(); ++i) {
      float dx = waypoints_[i].x() - waypoints_[i - 1].x();
      float dy = waypoints_[i].y() - waypoints_[i - 1].y();
      float dz = waypoints_[i].z() - waypoints_[i - 1].z();
      float norm = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (norm > 1e-6f) {
        directions[i] = {dx / norm, dy / norm, dz / norm};
      }
    }
    return directions;
  }

  [[nodiscard]] std::vector<std::array<float, 2>> dxy() const
  {
    if (waypoints_.empty()) return {};

    std::vector<std::array<float, 2>> directions(waypoints_.size(), {0.0f, 0.0f});
    for (size_t i = 1; i < waypoints_.size(); ++i) {
      float dx = waypoints_[i].x() - waypoints_[i - 1].x();
      float dy = waypoints_[i].y() - waypoints_[i - 1].y();
      float norm = std::sqrt(dx * dx + dy * dy);
      if (norm > 1e-6f) {
        directions[i] = {dx / norm, dy / norm};
      }
    }
    return directions;
  }

  [[nodiscard]] std::vector<std::array<float, FULL_DIM3D>> as_full_array() const
  {
    std::vector<std::array<float, FULL_DIM3D>> result;
    result.reserve(waypoints_.size());

    auto dirs3d = dxyz();
    for (size_t i = 0; i < waypoints_.size(); ++i) {
      const auto & pt = waypoints_[i];
      std::array<float, FULL_DIM3D> entry = {
        pt.x(),
        pt.y(),
        pt.z(),
        dirs3d[i][0],
        dirs3d[i][1],
        dirs3d[i][2],
        static_cast<float>(polyline_type_)};
      result.push_back(entry);
    }
    return result;
  }

private:
  MapType polyline_type_{MapType::Unused};
  std::vector<LanePoint> waypoints_;
};

using BoundarySegment = Polyline;

struct PolylineData
{
  /**
   * @brief Construct a new PolylineData instance.
   *
   * @param points Source points vector.
   * @param min_num_polyline The minimum number of polylines should be generated. If the number of
   * polylines, resulting in separating input points, is less than this value, empty polylines will
   * be added.
   * @param max_num_point The maximum number of points that each polyline can include. If the
   * polyline contains fewer points than this value, empty points will be added.
   * @param distance_threshold The distance threshold to separate polylines.
   */
  PolylineData(
    const std::vector<LanePoint> & points, const size_t min_num_polyline,
    const size_t max_num_point, const float distance_threshold)
  : num_point_(max_num_point), distance_threshold_(distance_threshold)
  {
    std::size_t point_cnt = 0;

    // point_cnt > PointNum at a to a new polyline group
    // distance > threshold -> add to a new polyline group
    for (std::size_t i = 0; i < points.size(); ++i) {
      auto & cur_point = points.at(i);

      if (i == 0) {
        add_new_polyline(cur_point, point_cnt);
        continue;
      }

      if (point_cnt >= num_point_) {
        add_new_polyline(cur_point, point_cnt);
      } else if (const auto & prev_point = points.at(i - 1);
                 cur_point.distance(prev_point) >= distance_threshold_ ||
                 cur_point.label() != prev_point.label()) {
        if (point_cnt < num_point_) {
          add_empty_points(point_cnt);
        }
        add_new_polyline(cur_point, point_cnt);
      } else {
        add_point(cur_point, point_cnt);
      }
    }
    add_empty_points(point_cnt);

    if (num_polyline_ < min_num_polyline) {
      add_empty_polyline(min_num_polyline - num_polyline_);
    }
  }

  // Return the number of polylines `K`.
  [[nodiscard]] size_t num_polyline() const { return num_polyline_; }

  // Return the number of points contained in each polyline `P`.
  [[nodiscard]] size_t num_point() const { return num_point_; }

  // Return the number of point dimensions `D`.
  static size_t state_dim() { return POINT_STATE_DIM; }

  // Return the number of all elements `K*P*D`.
  [[nodiscard]] size_t size() const { return num_polyline_ * num_point_ * state_dim(); }

  // Return the data shape ordering in `(K, P, D)`.
  [[nodiscard]] std::tuple<size_t, size_t, size_t> shape() const
  {
    return {num_polyline_, num_point_, state_dim()};
  }

  // Return the address pointer of data array.
  [[nodiscard]] const float * data_ptr() const noexcept { return data_.data(); }

private:
  /**
   * @brief Add a new polyline group filled by empty points. This member function increments
   * `PolylineNum` by `num_polyline` internally.
   *
   * @param num_polyline The number of polylines to add.
   */
  void add_empty_polyline(size_t num_polyline)
  {
    for (size_t i = 0; i < num_polyline; ++i) {
      size_t point_cnt = 0;
      auto empty_point = LanePoint::empty();
      add_new_polyline(empty_point, point_cnt);
      add_empty_points(point_cnt);
    }
  }

  /**
   * @brief Add a new polyline group with the specified point. This member function increments
   * `PolylineNum` by `1` internally.
   *
   * @param point LanePoint instance.
   * @param point_cnt The current count of points, which will be reset to `1`.
   */
  void add_new_polyline(const LanePoint & point, size_t & point_cnt)
  {
    const auto s = point.data_ptr();
    for (size_t d = 0; d < state_dim(); ++d) {
      data_.push_back(*(s + d));
    }
    ++num_polyline_;
    point_cnt = 1;
  }

  /**
   * @brief Add `(PointNum - point_cnt)` empty points filled by `0.0`.
   *
   * @param point_cnt The number of current count of points, which will be reset to `PointNum`.
   */
  void add_empty_points(size_t & point_cnt)
  {
    const auto s = LanePoint::empty().data_ptr();
    for (std::size_t n = point_cnt; n < num_point_; ++n) {
      for (std::size_t d = 0; d < state_dim(); ++d) {
        data_.push_back(*(s + d));
      }
    }
    point_cnt = num_point_;
  }

  /**
   * @brief Add the specified point and increment `point_cnt` by `1`.
   *
   * @param point
   * @param point_cnt
   */
  void add_point(const LanePoint & point, std::size_t & point_cnt)
  {
    const auto s = point.data_ptr();
    for (size_t d = 0; d < state_dim(); ++d) {
      data_.push_back(*(s + d));
    }
    ++point_cnt;
  }

  size_t num_polyline_{0};
  size_t num_point_{0};
  std::vector<float> data_;
  const float distance_threshold_;
};
}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__POLYLINE_HPP_
