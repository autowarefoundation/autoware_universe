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

#ifndef TRAJECTORY_LATERAL_OFFSET_CALCULATOR_HPP_
#define TRAJECTORY_LATERAL_OFFSET_CALCULATOR_HPP_

#include <autoware/motion_utils/trajectory/trajectory.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <cmath>
#include <vector>

namespace autoware::motion_velocity_planner
{
// Calculates the lateral offset from a trajectory to a point. The result is the same as
// autoware::motion_utils::calcLateralOffset(traj_points, point).
//
// That function copies the whole trajectory to remove the overlapping points every time it is
// called, which is costly when it is called for each of thousands of points on the same
// trajectory. This class removes them only once, in the constructor, and calls the overload that
// takes the segment index.
class TrajectoryLateralOffsetCalculator
{
public:
  explicit TrajectoryLateralOffsetCalculator(
    const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & traj_points)
  : points_without_overlap_(autoware::motion_utils::removeOverlapPoints(traj_points))
  {
  }

  // NaN if the trajectory has fewer than two points that do not overlap, like calcLateralOffset
  [[nodiscard]] double calc(const geometry_msgs::msg::Point & point) const
  {
    if (points_without_overlap_.size() < 2) {
      return std::nan("");
    }
    const size_t seg_idx =
      autoware::motion_utils::findNearestSegmentIndex(points_without_overlap_, point);
    return autoware::motion_utils::calcLateralOffset(points_without_overlap_, point, seg_idx);
  }

private:
  std::vector<autoware_planning_msgs::msg::TrajectoryPoint> points_without_overlap_;
};
}  // namespace autoware::motion_velocity_planner

#endif  // TRAJECTORY_LATERAL_OFFSET_CALCULATOR_HPP_
