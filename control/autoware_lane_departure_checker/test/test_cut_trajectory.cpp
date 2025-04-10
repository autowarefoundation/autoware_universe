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

#include "autoware/lane_departure_checker/utils.hpp"

#include <Eigen/Core>

#include <gtest/gtest.h>

using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;

namespace
{
TrajectoryPoints create_trajectory_points(const std::vector<Eigen::Vector3d> & points)
{
  TrajectoryPoints trajectory_points;
  for (const auto & point : points) {
    TrajectoryPoint p;
    p.pose.position.x = point.x();
    p.pose.position.y = point.y();
    p.pose.position.z = point.z();
    trajectory_points.push_back(p);
  }
  return trajectory_points;
}
}  // namespace

struct CutTrajectoryTestParam
{
  std::string description;
  std::vector<Eigen::Vector3d> trajectory_points;
  double length;
  std::vector<Eigen::Vector3d> expected_points;
};

std::ostream & operator<<(std::ostream & os, const CutTrajectoryTestParam & p)
{
  return os << p.description;
}

class CutTrajectoryTest : public ::testing::TestWithParam<CutTrajectoryTestParam>
{
};

TEST_P(CutTrajectoryTest, test_cut_trajectory)
{
  const auto p = GetParam();
  const auto trajectory = create_trajectory_points(p.trajectory_points);
  const auto cut = autoware::lane_departure_checker::utils::cutTrajectory(trajectory, p.length);

  ASSERT_EQ(cut.size(), p.expected_points.size());

  for (size_t i = 0; i < p.expected_points.size(); ++i) {
    EXPECT_DOUBLE_EQ(cut[i].pose.position.x, p.expected_points[i].x());
    EXPECT_DOUBLE_EQ(cut[i].pose.position.y, p.expected_points[i].y());
    EXPECT_DOUBLE_EQ(cut[i].pose.position.z, p.expected_points[i].z());
  }
}

INSTANTIATE_TEST_SUITE_P(
  LaneDepartureCheckerTest, CutTrajectoryTest,
  ::testing::Values(
    CutTrajectoryTestParam{
      "EmptyTrajectory", std::vector<Eigen::Vector3d>{}, 1.0, std::vector<Eigen::Vector3d>{}},
    CutTrajectoryTestParam{
      "SinglePointTrajectory", std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}}, 1.0,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}}},
    CutTrajectoryTestParam{
      "InterpolationAtViaPoint",
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, 1.0,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}}},
    CutTrajectoryTestParam{
      "InterpolationInMiddle",
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, 1.5,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {1.5, 0.0, 0.0}}},
    CutTrajectoryTestParam{
      "NoCut", std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}, 3.0,
      std::vector<Eigen::Vector3d>{{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}}}),
  ::testing::PrintToStringParamName());
