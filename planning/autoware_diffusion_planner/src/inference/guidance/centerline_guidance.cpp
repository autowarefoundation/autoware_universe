// Copyright 2026 TIER IV, Inc.
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

#include "autoware/diffusion_planner/inference/guidance/centerline_guidance.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "utils.hpp"

#include <Eigen/Core>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

using BoostPoint = bg::model::point<double, 2, bg::cs::cartesian>;
using BoostSegment = bg::model::segment<BoostPoint>;
using IndexedSegment = std::pair<BoostSegment, size_t>;

size_t route_lane_index(const int64_t segment, const int64_t point, const int64_t dim)
{
  return (static_cast<size_t>(segment) * POINTS_PER_SEGMENT + point) * SEGMENT_POINT_DIM + dim;
}

bool is_valid_route_point(
  const std::vector<float> & route_lanes, const int64_t segment, const int64_t point)
{
  for (int64_t dim = 0; dim < SEGMENT_POINT_DIM; ++dim) {
    const float value = route_lanes[route_lane_index(segment, point, dim)];
    if (std::abs(value) > std::numeric_limits<float>::epsilon()) {
      return true;
    }
  }
  return false;
}

std::vector<Eigen::Vector2d> extract_centerline_points(const std::vector<float> & route_lanes)
{
  const size_t single_batch_size =
    static_cast<size_t>(NUM_SEGMENTS_IN_ROUTE) * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM;
  if (route_lanes.size() < single_batch_size) {
    return {};
  }

  std::vector<Eigen::Vector2d> points;
  points.reserve(static_cast<size_t>(NUM_SEGMENTS_IN_ROUTE) * POINTS_PER_SEGMENT);
  for (int64_t segment = 0; segment < NUM_SEGMENTS_IN_ROUTE; ++segment) {
    for (int64_t point = 0; point < POINTS_PER_SEGMENT; ++point) {
      if (!is_valid_route_point(route_lanes, segment, point)) {
        continue;
      }
      points.emplace_back(
        route_lanes[route_lane_index(segment, point, X)],
        route_lanes[route_lane_index(segment, point, Y)]);
    }
  }
  return points;
}

Eigen::Vector2d project_to_segment(
  const Eigen::Vector2d & point, const Eigen::Vector2d & segment_start,
  const Eigen::Vector2d & segment_end)
{
  const Eigen::Vector2d segment = segment_end - segment_start;
  const double squared_length = segment.squaredNorm();
  if (squared_length <= std::numeric_limits<double>::epsilon()) {
    return segment_start;
  }

  const double ratio = std::clamp((point - segment_start).dot(segment) / squared_length, 0.0, 1.0);
  return segment_start + ratio * segment;
}

std::vector<Eigen::Vector2d> snap_trajectory_to_centerline(
  const std::vector<Eigen::Vector2d> & trajectory,
  const std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> & centerline_segments,
  const bgi::rtree<IndexedSegment, bgi::quadratic<16>> & rtree, const int64_t start_step)
{
  std::vector<Eigen::Vector2d> guided_trajectory = trajectory;
  std::vector<IndexedSegment> nearest_segments;
  nearest_segments.reserve(1);

  for (int64_t t = start_step; t <= OUTPUT_T; ++t) {
    nearest_segments.clear();
    const BoostPoint query(trajectory[t].x(), trajectory[t].y());
    rtree.query(bgi::nearest(query, 1), std::back_inserter(nearest_segments));
    if (nearest_segments.empty()) {
      continue;
    }
    const auto segment_index = nearest_segments.front().second;
    const auto & [segment_start, segment_end] = centerline_segments[segment_index];
    guided_trajectory[t] = project_to_segment(trajectory[t], segment_start, segment_end);
  }

  return guided_trajectory;
}
}  // namespace

CenterlineGuidance::CenterlineGuidance()
{
  config_ = CenterlineGuidanceConfig{};
}

CenterlineGuidance::CenterlineGuidance(const CenterlineGuidanceConfig & config)
{
  config_ = config;
}

void CenterlineGuidance::set_config(const CenterlineGuidanceConfig & config)
{
  config_ = config;
}

void CenterlineGuidance::set_route_lanes(const std::vector<float> & route_lanes)
{
  route_lanes_ = route_lanes;
}

GuidanceResult CenterlineGuidance::compute_delta(
  [[maybe_unused]] const GuidanceContext & context, const std::vector<float> & model_output) const
{
  GuidanceResult result;
  if (!is_enabled()) {
    return result;
  }

  const auto centerline_points = extract_centerline_points(route_lanes_);
  if (centerline_points.empty()) {
    return result;
  }

  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> centerline_segments;
  centerline_segments.reserve(centerline_points.size() - 1);
  std::vector<IndexedSegment> indexed_segments;
  indexed_segments.reserve(centerline_points.size() - 1);
  for (size_t i = 0; i + 1 < centerline_points.size(); ++i) {
    if (
      (centerline_points[i + 1] - centerline_points[i]).squaredNorm() <=
      std::numeric_limits<double>::epsilon()) {
      continue;
    }

    const size_t segment_index = centerline_segments.size();
    centerline_segments.emplace_back(centerline_points[i], centerline_points[i + 1]);
    indexed_segments.emplace_back(
      BoostSegment(
        BoostPoint(centerline_points[i].x(), centerline_points[i].y()),
        BoostPoint(centerline_points[i + 1].x(), centerline_points[i + 1].y())),
      segment_index);
  }
  if (indexed_segments.empty()) {
    return result;
  }
  const bgi::rtree<IndexedSegment, bgi::quadratic<16>> rtree(indexed_segments);

  const float x_std = config_.x_std;
  const float y_std = config_.y_std;
  const std::vector<std::vector<Eigen::Vector2d>> trajectories =
    extract_denormalized_trajectories_from_model_output(
      model_output, config_.x_mean, config_.y_mean, x_std, y_std);
  if (trajectories.empty()) {
    return result;
  }

  const int64_t start_step = std::max<int64_t>(
    static_cast<int64_t>(std::ceil(config_.start_time_s / constants::PREDICTION_TIME_STEP_S)), 1);
  if (start_step > OUTPUT_T) {
    return result;
  }
  std::vector<std::vector<Eigen::Vector2d>> guided_trajectories;
  guided_trajectories.reserve(trajectories.size());
  result.triggered.assign(trajectories.size(), false);

  for (const auto & trajectory : trajectories) {
    guided_trajectories.push_back(
      snap_trajectory_to_centerline(trajectory, centerline_segments, rtree, start_step));
    result.triggered[guided_trajectories.size() - 1] = true;
  }

  result.delta =
    create_delta_from_denormalized_trajectories(trajectories, guided_trajectories, x_std, y_std);
  return result;
}

}  // namespace autoware::diffusion_planner
