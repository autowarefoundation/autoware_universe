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

#include "autoware/control_validator/utils.hpp"

#include <tf2/LinearMath/Quaternion.hpp>

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <utility>
#include <vector>

using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

namespace autoware::control_validator::detail
{

static TrajectoryPoints make_trajectory(const std::vector<std::pair<double, double>> & xys)
{
  TrajectoryPoints points;
  points.reserve(xys.size());
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  auto quat = tf2::toMsg(q);

  for (const auto & [x, y] : xys) {
    TrajectoryPoint p;
    p.pose.position.x = static_cast<float>(x);
    p.pose.position.y = static_cast<float>(y);
    p.pose.orientation = quat;
    p.longitudinal_velocity_mps = 0.0f;
    points.emplace_back(p);
  }
  return points;
}

static TrajectoryPoints make_trajectory_range(
  std::pair<double, double> start, std::pair<double, double> end, size_t n)
{
  TrajectoryPoints points;
  points.reserve(n);
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  auto quat = tf2::toMsg(q);
  if (n < 2u) return points;
  for (size_t i = 0; i < n; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(n - 1);
    const double x = start.first + ratio * (end.first - start.first);
    const double y = start.second + ratio * (end.second - start.second);
    TrajectoryPoint p;
    p.pose.position.x = static_cast<float>(x);
    p.pose.position.y = static_cast<float>(y);
    p.pose.orientation = quat;
    p.longitudinal_velocity_mps = 0.0f;
    points.emplace_back(p);
  }
  return points;
}

TEST(align_trajectory_with_reference_trajectory, EmptyReferenceOrPredicted)
{
  // reference with fewer than 2 points -> empty
  auto ref1 = make_trajectory({{0.0, 0.0}});
  auto pred1 = make_trajectory_range({0.0, 0.0}, {10.0, 0.0}, 11);
  auto out1 = align_trajectory_with_reference_trajectory(ref1, pred1);
  EXPECT_TRUE(out1.empty());

  // predicted empty -> empty
  auto ref2 = make_trajectory_range({0.0, 0.0}, {10.0, 0.0}, 11);
  auto pred2 = make_trajectory({});
  auto out2 = align_trajectory_with_reference_trajectory(ref2, pred2);
  EXPECT_TRUE(out2.empty());
}

TEST(align_trajectory_with_reference_trajectory, IdenticalTrajectories_ReturnsSame)
{
  auto ref = make_trajectory_range({0.0, 0.0}, {10.0, 0.0}, 11);
  auto pred = make_trajectory_range({0.0, 0.0}, {10.0, 0.0}, 11);
  auto out = align_trajectory_with_reference_trajectory(ref, pred);
  // identical inputs should produce identical aligned sequence
  ASSERT_EQ(out.size(), pred.size());
  for (size_t i = 0; i < out.size(); ++i) {
    EXPECT_NEAR(out[i].pose.position.x, pred[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out[i].pose.position.y, pred[i].pose.position.y, 1e-6);
  }
}

TEST(align_trajectory_with_reference_trajectory, NonOverlappingBeforeAfter_ReturnsEmpty)
{
  // predicted entirely before reference
  auto ref = make_trajectory_range({0.0, 0.0}, {10.0, 0.0}, 11);
  auto pred_before = make_trajectory_range({-20.0, 0.0}, {-10.0, 0.0}, 11);
  auto out_before = align_trajectory_with_reference_trajectory(ref, pred_before);
  EXPECT_TRUE(out_before.empty());

  // predicted entirely after reference
  auto pred_after = make_trajectory_range({20.0, 0.0}, {30.0, 0.0}, 11);
  auto out_after = align_trajectory_with_reference_trajectory(ref, pred_after);
  EXPECT_TRUE(out_after.empty());
}

TEST(align_trajectory_with_reference_trajectory, LargePredicted_IsClamped)
{
  // predicted trajectory larger than MAX_TRAJECTORY_SIZE (5000), expect result size <= 5000
  constexpr size_t large_n = 6000u;

  auto ref = make_trajectory_range({0.0, 0.0}, {100.0, 0.0}, 101);
  auto pred = make_trajectory_range({0.0, 0.0}, {60.0, 0.0}, large_n);

  auto out = align_trajectory_with_reference_trajectory(ref, pred);
  EXPECT_LE(out.size(), 5000u);
}

TEST(align_trajectory_with_reference_trajectory, Trajectories1D)
{
  auto ref = make_trajectory_range({0.0, 0.0}, {10.0, 0.0}, 11);

  // trim front
  auto pred1 = make_trajectory({{-2.0, 0.0}, {-1.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  auto out1 = align_trajectory_with_reference_trajectory(ref, pred1);
  auto expected1 = make_trajectory({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  ASSERT_EQ(out1.size(), expected1.size());
  for (size_t i = 0; i < out1.size(); ++i) {
    EXPECT_NEAR(out1[i].pose.position.x, expected1[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out1[i].pose.position.y, expected1[i].pose.position.y, 1e-6);
  }

  // trim back
  auto pred2 = make_trajectory({{8.0, 0.0}, {9.0, 0.0}, {11.0, 0.0}, {12.0, 0.0}});
  auto out2 = align_trajectory_with_reference_trajectory(ref, pred2);
  auto expected2 = make_trajectory({{8.0, 0.0}, {9.0, 0.0}, {10.0, 0.0}});
  ASSERT_EQ(out2.size(), expected2.size());
  for (size_t i = 0; i < out2.size(); ++i) {
    EXPECT_NEAR(out2[i].pose.position.x, expected2[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out2[i].pose.position.y, expected2[i].pose.position.y, 1e-6);
  }

  // trim both
  auto pred3 = make_trajectory({{-5.0, 0.0}, {15.0, 0.0}});
  auto out3 = align_trajectory_with_reference_trajectory(ref, pred3);
  auto expected3 = make_trajectory({{0.0, 0.0}, {10.0, 0.0}});
  ASSERT_EQ(out3.size(), expected3.size());
  for (size_t i = 0; i < out3.size(); ++i) {
    EXPECT_NEAR(out3[i].pose.position.x, expected3[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out3[i].pose.position.y, expected3[i].pose.position.y, 1e-6);
  }

  // no trim
  auto pred4 = make_trajectory({{2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0}, {8.0, 0.0}});
  auto out4 = align_trajectory_with_reference_trajectory(ref, pred4);
  ASSERT_EQ(out4.size(), pred4.size());
  for (size_t i = 0; i < out4.size(); ++i) {
    EXPECT_NEAR(out4[i].pose.position.x, pred4[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out4[i].pose.position.y, pred4[i].pose.position.y, 1e-6);
  }
}

TEST(align_trajectory_with_reference_trajectory, Trajectories2D)
{
  auto ref = make_trajectory({{3.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}, {0.0, 3.0}});

  auto pred = make_trajectory({{5.0, 0.0}, {0.0, 5.0}});
  auto out = align_trajectory_with_reference_trajectory(ref, pred);
  auto expected = make_trajectory({{4.0, 1.0}, {1.0, 4.0}});
  ASSERT_EQ(out.size(), expected.size());
  for (size_t i = 0; i < out.size(); ++i) {
    EXPECT_NEAR(out[i].pose.position.x, expected[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out[i].pose.position.y, expected[i].pose.position.y, 1e-6);
  }
}

TEST(align_trajectory_with_reference_trajectory, DISABLED_Trajectories2D_Pathological)
{
  auto ref = make_trajectory({{3.0, 0.0}, {2.0, 0.0}, {0.0, 2.0}, {0.0, 3.0}});

  auto pred = make_trajectory({{5.0, 4.0}, {4.0, 5.0}});
  auto out = align_trajectory_with_reference_trajectory(ref, pred);

  // Disabled test case:
  //
  // even though the middle reference segment overlaps with predicted trajectory,
  // current algorithm only considers the first and last segments for overlap checking,
  // so the result is empty.

  ASSERT_EQ(out.size(), pred.size());
  for (size_t i = 0; i < out.size(); ++i) {
    EXPECT_NEAR(out[i].pose.position.x, pred[i].pose.position.x, 1e-6);
    EXPECT_NEAR(out[i].pose.position.y, pred[i].pose.position.y, 1e-6);
  }
}

}  // namespace autoware::control_validator::detail
