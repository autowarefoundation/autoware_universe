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

#ifndef AUTOWARE__TRAJECTORY__UTILS__CURVATURE_UTILS_HPP_
#define AUTOWARE__TRAJECTORY__UTILS__CURVATURE_UTILS_HPP_

#include "autoware/trajectory/forward.hpp"

#include <algorithm>

namespace autoware::trajectory
{
template <class PointType>
double max_curvature(const Trajectory<PointType> & trajectory)
{
  double max_curvature = 0.0;
  for (const auto & base : trajectory.get_internal_bases()) {
    const auto curvature = trajectory.compute_curvature(base);
    max_curvature = std::max(max_curvature, curvature);
  }
  return max_curvature;
}

template <class PointType>
double min_curvature(const Trajectory<PointType> & trajectory)
{
  double min_curvature = 0.0;
  for (const auto & base : trajectory.get_internal_bases()) {
    const auto curvature = trajectory.compute_curvature(base);
    min_curvature = std::min(min_curvature, curvature);
  }
  return min_curvature;
}

}  // namespace autoware::trajectory

#endif  // AUTOWARE__TRAJECTORY__UTILS__CURVATURE_UTILS_HPP_
