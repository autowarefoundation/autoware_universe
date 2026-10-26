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

#ifndef TRAJECTORY_SANITIZER_HPP_
#define TRAJECTORY_SANITIZER_HPP_

#include "type_alias.hpp"

#include <cstddef>

namespace autoware::in_lane_mrm_planner
{

/// Remove consecutive points closer than min_interval (2D distance).
///
/// Downstream consumers (e.g. MPC spline resampling) require strictly increasing arc length;
/// (near-)duplicate points violate that. The first point of an overlap run is kept, and its
/// longitudinal velocity is replaced by the minimum over the run so an overlapping stop point
/// (v=0) is not lost.
///
/// @return number of removed points
size_t remove_overlap_points(TrajectoryPoints & points, double min_interval);

}  // namespace autoware::in_lane_mrm_planner

#endif  // TRAJECTORY_SANITIZER_HPP_
