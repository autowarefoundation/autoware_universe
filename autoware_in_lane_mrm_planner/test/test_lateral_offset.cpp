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

#include "in_lane_mrm_trajectory_planner.hpp"

#include <autoware_utils/geometry/geometry.hpp>

#include <gtest/gtest.h>

namespace autoware::in_lane_mrm_planner
{
namespace
{

PathWithLaneId make_straight_path(const size_t num_points, const double spacing)
{
  PathWithLaneId path;
  path.header.frame_id = "map";
  for (size_t i = 0; i < num_points; ++i) {
    PathPointWithLaneId point;
    point.point.pose.position.x = spacing * static_cast<double>(i);
    point.point.pose.position.y = 0.0;
    point.point.pose.position.z = 0.0;
    point.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);
    path.points.push_back(point);
  }
  return path;
}

}  // namespace

TEST(LateralOffsetTest, ApplyConstantLateralOffsetPreservesPointCount)
{
  const auto path = make_straight_path(3, 1.0);
  const auto offset_path = InLaneMrmTrajectoryPlanner::apply_lateral_offset(path, 1.0);

  ASSERT_EQ(offset_path.points.size(), 3U);
  EXPECT_NEAR(offset_path.points.at(1).point.pose.position.x, 1.0, 1e-6);
  EXPECT_NEAR(offset_path.points.at(1).point.pose.position.y, 1.0, 1e-6);
}

TEST(LateralOffsetTest, ComputeLateralOffsetMatchesAppliedShift)
{
  const auto path = make_straight_path(3, 1.0);
  geometry_msgs::msg::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 1.0;
  pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);

  const double lateral_offset = InLaneMrmTrajectoryPlanner::compute_lateral_offset(path, pose);
  EXPECT_NEAR(lateral_offset, 1.0, 1e-6);
}

}  // namespace autoware::in_lane_mrm_planner
