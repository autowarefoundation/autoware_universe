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

#include "autoware/perception_online_evaluator/metrics/deviation_metrics.hpp"

#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/geometry/pose_deviation.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <vector>

namespace autoware::perception_diagnostics
{
namespace metrics
{

double calcLateralDeviation(const std::vector<Pose> & ref_path, const Pose & target_pose)
{
  if (ref_path.empty()) {
    return 0.0;
  }

  const size_t nearest_index =
    autoware::motion_utils::findNearestIndex(ref_path, target_pose.position);
  return std::abs(
    autoware_utils::calc_lateral_deviation(ref_path[nearest_index], target_pose.position));
}

double calcYawDeviation(const std::vector<Pose> & ref_path, const Pose & target_pose)
{
  if (ref_path.empty()) {
    return 0.0;
  }

  const size_t nearest_index =
    autoware::motion_utils::findNearestIndex(ref_path, target_pose.position);
  return std::abs(autoware_utils::calc_yaw_deviation(ref_path[nearest_index], target_pose));
}

}  // namespace metrics
}  // namespace autoware::perception_diagnostics
