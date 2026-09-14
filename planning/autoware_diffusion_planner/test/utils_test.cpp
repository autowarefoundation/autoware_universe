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

#include "autoware/diffusion_planner/utils/utils.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner::test
{

class UtilsTest : public ::testing::Test
{
protected:
  void SetUp() override {}
};

TEST_F(UtilsTest, CreateFloatDataDefaultFill)
{
  std::vector<int64_t> shape{2, 3};
  auto data = utils::create_float_data(shape);
  ASSERT_EQ(data.size(), 6u);
  for (auto v : data) {
    EXPECT_FLOAT_EQ(v, 1.0f);
  }
}

TEST_F(UtilsTest, CreateFloatDataCustomFill)
{
  std::vector<int64_t> shape{4};
  auto data = utils::create_float_data(shape, 7.5f);
  ASSERT_EQ(data.size(), 4u);
  for (auto v : data) {
    EXPECT_FLOAT_EQ(v, 7.5f);
  }
}

TEST_F(UtilsTest, CreateFloatDataEmptyShape)
{
  std::vector<int64_t> shape{};
  auto data = utils::create_float_data(shape, 2.0f);
  // By convention, empty shape means one element
  ASSERT_EQ(data.size(), 1u);
  EXPECT_FLOAT_EQ(data[0], 2.0f);
}

TEST_F(UtilsTest, CreateFloatDataZeroDim)
{
  std::vector<int64_t> shape{0, 5};
  auto data = utils::create_float_data(shape, 3.0f);
  ASSERT_EQ(data.size(), 0u);
}

TEST_F(UtilsTest, GetTransformMatrixIdentity)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  const Eigen::Matrix4d bl2map = utils::pose_to_matrix4d(odom.pose.pose);
  const Eigen::Matrix4d map2bl = utils::inverse(bl2map);

  Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(bl2map(i, j), I(i, j), 1e-6);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) EXPECT_NEAR(map2bl(i, j), I(i, j), 1e-6);
}

TEST_F(UtilsTest, GetTransformMatrixTranslation)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 1.0;
  odom.pose.pose.position.y = 2.0;
  odom.pose.pose.position.z = 3.0;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = 0.0;
  odom.pose.pose.orientation.w = 1.0;

  const Eigen::Matrix4d bl2map = utils::pose_to_matrix4d(odom.pose.pose);
  const Eigen::Matrix4d map2bl = utils::inverse(bl2map);

  EXPECT_FLOAT_EQ(bl2map(0, 3), 1.0f);
  EXPECT_FLOAT_EQ(bl2map(1, 3), 2.0f);
  EXPECT_FLOAT_EQ(bl2map(2, 3), 3.0f);

  Eigen::Vector3f t(1.0f, 2.0f, 3.0f);
  Eigen::Vector3f inv_t = -t;
  EXPECT_FLOAT_EQ(map2bl(0, 3), inv_t.x());
  EXPECT_FLOAT_EQ(map2bl(1, 3), inv_t.y());
  EXPECT_FLOAT_EQ(map2bl(2, 3), inv_t.z());
}

TEST_F(UtilsTest, GetTransformMatrixRotation)
{
  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  // 90 degree rotation around Z axis
  double angle = M_PI_2;
  odom.pose.pose.orientation.x = 0.0;
  odom.pose.pose.orientation.y = 0.0;
  odom.pose.pose.orientation.z = std::sin(angle / 2);
  odom.pose.pose.orientation.w = std::cos(angle / 2);

  const Eigen::Matrix4d bl2map = utils::pose_to_matrix4d(odom.pose.pose);
  const Eigen::Matrix4d map2bl = utils::inverse(bl2map);

  // The rotation part should be a 90 degree rotation matrix
  Eigen::Matrix3f R;
  R << 0, -1, 0, 1, 0, 0, 0, 0, 1;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(bl2map(i, j), R(i, j), 1e-6);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) EXPECT_NEAR(map2bl(i, j), R.transpose()(i, j), 1e-6);
}

TEST_F(UtilsTest, CheckInputMapValid)
{
  std::unordered_map<std::string, std::vector<float>> input_map;
  input_map["a"] = {1.0f, 2.0f, 3.0f};
  input_map["b"] = {0.0f, -1.0f, 42.0f};
  EXPECT_TRUE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, CheckInputMapWithInf)
{
  std::unordered_map<std::string, std::vector<float>> input_map;
  input_map["a"] = {1.0f, std::numeric_limits<float>::infinity()};
  EXPECT_FALSE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, CheckInputMapWithNaN)
{
  std::unordered_map<std::string, std::vector<float>> input_map;
  input_map["a"] = {1.0f, std::nanf("")};
  EXPECT_FALSE(utils::check_input_map(input_map));
}

TEST_F(UtilsTest, CheckInputMapEmpty)
{
  std::unordered_map<std::string, std::vector<float>> input_map;
  EXPECT_TRUE(utils::check_input_map(input_map));
}

namespace
{
Eigen::Matrix4d make_pose(const double x, const double y, const double yaw)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  pose(0, 0) = std::cos(yaw);
  pose(0, 1) = -std::sin(yaw);
  pose(1, 0) = std::sin(yaw);
  pose(1, 1) = std::cos(yaw);
  pose(0, 3) = x;
  pose(1, 3) = y;
  return pose;
}

double yaw_of(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}
}  // namespace

// A query point lying exactly on a vertex of the polyline must be a no-op: the returned pose
// equals that vertex (position and heading). This mirrors the Perfect-Tracker invariant where the
// ego lands exactly on the previous prediction.
TEST_F(UtilsTest, ProjectPoseOntoPolylineOnVertexIsNoOp)
{
  const std::vector<Eigen::Matrix4d> polyline{
    make_pose(0.0, 0.0, 0.0), make_pose(1.0, 0.0, 0.0), make_pose(2.0, 0.0, 0.0)};

  const Eigen::Matrix4d projected = utils::project_pose_onto_polyline(1.0, 0.0, polyline, 5).pose;

  EXPECT_NEAR(projected(0, 3), 1.0, 1e-9);
  EXPECT_NEAR(projected(1, 3), 0.0, 1e-9);
  EXPECT_NEAR(yaw_of(projected), 0.0, 1e-9);
}

// A query point offset laterally from a straight polyline snaps to the foot of the perpendicular.
TEST_F(UtilsTest, ProjectPoseOntoPolylineLateralOffset)
{
  const std::vector<Eigen::Matrix4d> polyline{make_pose(0.0, 0.0, 0.0), make_pose(10.0, 0.0, 0.0)};

  const Eigen::Matrix4d projected = utils::project_pose_onto_polyline(3.0, 2.0, polyline, 5).pose;

  EXPECT_NEAR(projected(0, 3), 3.0, 1e-9);
  EXPECT_NEAR(projected(1, 3), 0.0, 1e-9);
  EXPECT_NEAR(yaw_of(projected), 0.0, 1e-9);
}

// A query point past the end of the polyline is clamped to the closest endpoint.
TEST_F(UtilsTest, ProjectPoseOntoPolylineClampsToEndpoint)
{
  const std::vector<Eigen::Matrix4d> polyline{make_pose(0.0, 0.0, 0.0), make_pose(10.0, 0.0, 0.0)};

  const Eigen::Matrix4d projected = utils::project_pose_onto_polyline(15.0, 5.0, polyline, 5).pose;

  EXPECT_NEAR(projected(0, 3), 10.0, 1e-9);
  EXPECT_NEAR(projected(1, 3), 0.0, 1e-9);
}

// The heading is slerp-interpolated between the endpoints of the closest segment. Projecting the
// midpoint of a segment whose endpoints face 0 and pi/2 yields a heading of pi/4.
TEST_F(UtilsTest, ProjectPoseOntoPolylineInterpolatesHeading)
{
  const std::vector<Eigen::Matrix4d> polyline{
    make_pose(0.0, 0.0, 0.0), make_pose(2.0, 0.0, M_PI_2)};

  const Eigen::Matrix4d projected = utils::project_pose_onto_polyline(1.0, 0.0, polyline, 5).pose;

  EXPECT_NEAR(projected(0, 3), 1.0, 1e-9);
  EXPECT_NEAR(projected(1, 3), 0.0, 1e-9);
  EXPECT_NEAR(yaw_of(projected), M_PI_4, 1e-9);
}

// The closest segment is selected among the searched segments of the polyline.
TEST_F(UtilsTest, ProjectPoseOntoPolylineSelectsClosestSegment)
{
  const std::vector<Eigen::Matrix4d> polyline{
    make_pose(0.0, 0.0, 0.0), make_pose(1.0, 0.0, 0.0), make_pose(1.0, 5.0, M_PI_2)};

  // Closest to the second (vertical) segment, at ratio 3.0 / 5.0 along it.
  const Eigen::Matrix4d projected = utils::project_pose_onto_polyline(1.3, 3.0, polyline, 5).pose;

  EXPECT_NEAR(projected(0, 3), 1.0, 1e-9);
  EXPECT_NEAR(projected(1, 3), 3.0, 1e-9);
  // The heading is slerp-interpolated between the segment endpoints (0 and pi/2) by that same
  // ratio, not taken from the end vertex.
  EXPECT_NEAR(yaw_of(projected), 0.6 * M_PI_2, 1e-9);
}

// Segments beyond max_search_segment_count are never selected, even when one of them is much
// closer to the query point than anything inside the window.
TEST_F(UtilsTest, ProjectPoseOntoPolylineIgnoresSegmentsBeyondSearchWindow)
{
  // A straight run along +x, then a vertex that comes back right next to the query point.
  std::vector<Eigen::Matrix4d> polyline;
  for (size_t i = 0; i < 6; ++i) {
    polyline.push_back(make_pose(static_cast<double>(i), 0.0, 0.0));
  }
  polyline.push_back(make_pose(5.0, 20.0, M_PI_2));
  polyline.push_back(make_pose(5.0, 21.0, M_PI_2));

  // The query sits on the last segment, which is the 7th one and therefore outside a window of 5.
  const auto windowed = utils::project_pose_onto_polyline(5.0, 20.5, polyline, 5);
  EXPECT_NEAR(windowed.pose(0, 3), 5.0, 1e-9);
  EXPECT_NEAR(windowed.pose(1, 3), 0.0, 1e-9);
  EXPECT_NEAR(windowed.interpolation_index, 5.0, 1e-9);

  // Widening the window lets the same query reach the far segment.
  const auto full = utils::project_pose_onto_polyline(5.0, 20.5, polyline, 7);
  EXPECT_NEAR(full.pose(0, 3), 5.0, 1e-9);
  EXPECT_NEAR(full.pose(1, 3), 20.5, 1e-9);
  EXPECT_NEAR(yaw_of(full.pose), M_PI_2, 1e-9);
}

TEST_F(UtilsTest, ProjectPoseOntoPolylineThrowsOnTooFewPoints)
{
  const std::vector<Eigen::Matrix4d> polyline{make_pose(0.0, 0.0, 0.0)};
  EXPECT_THROW(utils::project_pose_onto_polyline(0.0, 0.0, polyline, 5), std::runtime_error);
}

TEST_F(UtilsTest, ProjectPoseOntoPolylineThrowsOnNonPositiveSearchWindow)
{
  const std::vector<Eigen::Matrix4d> polyline{make_pose(0.0, 0.0, 0.0), make_pose(1.0, 0.0, 0.0)};
  EXPECT_THROW(utils::project_pose_onto_polyline(0.0, 0.0, polyline, 0), std::runtime_error);
}

}  // namespace autoware::diffusion_planner::test
