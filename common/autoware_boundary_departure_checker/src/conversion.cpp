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

#include "autoware/boundary_departure_checker/conversion.hpp"

namespace autoware::boundary_departure_checker::utils
{
Point2d to_point_2d(const Eigen::Matrix<double, 3, 1> & ll_pt)
{
  return {ll_pt.x(), ll_pt.y()};
}

Segment2d to_segment_2d(
  const Eigen::Matrix<double, 3, 1> & ll_pt1, const Eigen::Matrix<double, 3, 1> & ll_pt2)
{
  return {to_point_2d(ll_pt1), to_point_2d(ll_pt2)};
}

Segment2d to_segment_2d(const Segment3d & segment)
{
  return {to_point_2d(segment.first), to_point_2d(segment.second)};
}
}  // namespace autoware::boundary_departure_checker::utils
