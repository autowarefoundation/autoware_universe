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
}  // namespace

namespace
{
// Straight polyline along +x with the given spacing and constant heading 0.
std::vector<Eigen::Matrix4d> straight_polyline(const size_t count, const double spacing)
{
  std::vector<Eigen::Matrix4d> polyline;
  for (size_t i = 0; i < count; ++i) {
    polyline.push_back(make_pose(static_cast<double>(i) * spacing, 0.0, 0.0));
  }
  return polyline;
}

constexpr utils::TrajectorySnapOptions default_options{0, 5, 1.0, 0.2};
}  // namespace

// A query point lying exactly on a vertex of a straight polyline is a no-op: same position, same
// heading (both the interpolated vertex heading and the geometric tangent), integer interpolation
// index. This mirrors the Perfect-Tracker invariant where the ego lands on the previous prediction.
TEST_F(UtilsTest, SnapPointToTrajectoryOnVertexIsNoOp)
{
  const auto polyline = straight_polyline(10, 1.0);

  const auto snap = utils::snap_point_to_trajectory(3.0, 0.0, polyline, default_options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_NEAR(snap->position.x(), 3.0, 1e-6);
  EXPECT_NEAR(snap->position.y(), 0.0, 1e-6);
  EXPECT_NEAR(snap->interpolation_index, 3.0, 1e-6);
  EXPECT_NEAR(snap->heading_yaw, 0.0, 1e-6);
  ASSERT_TRUE(snap->tangent_yaw.has_value());
  EXPECT_NEAR(*snap->tangent_yaw, 0.0, 1e-6);
}

// A query point offset laterally from a straight polyline snaps to the foot of the perpendicular,
// and the interpolation index reflects the fraction along the segment.
TEST_F(UtilsTest, SnapPointToTrajectoryLateralOffset)
{
  const auto polyline = straight_polyline(10, 1.0);

  const auto snap = utils::snap_point_to_trajectory(2.5, 0.3, polyline, default_options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_NEAR(snap->position.x(), 2.5, 1e-6);
  EXPECT_NEAR(snap->position.y(), 0.0, 1e-6);
  EXPECT_NEAR(snap->interpolation_index, 2.5, 1e-6);
}

// The closest point is only searched within the leading max_search_segment_count segments.
TEST_F(UtilsTest, SnapPointToTrajectoryRespectsSearchWindow)
{
  const auto polyline = straight_polyline(10, 1.0);
  const utils::TrajectorySnapOptions options{0, 3, 1.0, 0.2};

  const auto snap = utils::snap_point_to_trajectory(8.0, 0.0, polyline, options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_NEAR(snap->position.x(), 3.0, 1e-6);
  EXPECT_NEAR(snap->interpolation_index, 3.0, 1e-6);
}

// The vertex headings of a predicted trajectory can be noisy while its positions trace a clean
// path. The interpolated vertex heading inherits that noise; the geometric tangent does not.
TEST_F(UtilsTest, SnapPointToTrajectoryTangentIgnoresNoisyVertexHeadings)
{
  std::vector<Eigen::Matrix4d> polyline;
  for (size_t i = 0; i < 20; ++i) {
    const double noisy_yaw = (i % 2 == 0) ? 0.2 : -0.2;
    polyline.push_back(make_pose(static_cast<double>(i) * 0.3, 0.0, noisy_yaw));
  }

  // Midway between two vertices, where the slerp of +-0.2 gives exactly 0 by symmetry, so query a
  // point closer to one vertex instead.
  const auto snap = utils::snap_point_to_trajectory(0.9 + 0.06, 0.0, polyline, default_options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_GT(std::abs(snap->heading_yaw), 0.05);
  ASSERT_TRUE(snap->tangent_yaw.has_value());
  EXPECT_NEAR(*snap->tangent_yaw, 0.0, 1e-3);
}

// Small position jitter of the vertices (as produced by the first, very short prediction steps)
// barely moves the averaged tangent, while a single segment's direction would swing by tens of
// degrees.
TEST_F(UtilsTest, SnapPointToTrajectoryTangentIsRobustToPositionJitter)
{
  std::vector<Eigen::Matrix4d> polyline;
  for (size_t i = 0; i < 30; ++i) {
    const double jitter_y = (i % 2 == 0) ? 0.01 : -0.01;  // +-1 cm on 10 cm segments (~11 deg)
    polyline.push_back(make_pose(static_cast<double>(i) * 0.1, jitter_y, 0.0));
  }

  // 1.35 m from the start so the symmetric +-1 m window is not clipped.
  const auto snap = utils::snap_point_to_trajectory(
    1.35, 0.0, polyline, utils::TrajectorySnapOptions{0, 20, 1.0, 0.2});

  ASSERT_TRUE(snap.has_value());
  ASSERT_TRUE(snap->tangent_yaw.has_value());
  EXPECT_NEAR(*snap->tangent_yaw, 0.0, 0.02);
}

// On a circular arc the averaged tangent matches the analytic tangent at the snapped point.
TEST_F(UtilsTest, SnapPointToTrajectoryTangentOnArc)
{
  constexpr double radius = 20.0;
  std::vector<Eigen::Matrix4d> polyline;
  for (size_t i = 0; i < 40; ++i) {
    const double theta = static_cast<double>(i) * 0.5 / radius;  // 0.5 m arc steps
    polyline.push_back(
      make_pose(radius * std::sin(theta), radius * (1.0 - std::cos(theta)), theta));
  }

  const double query_theta = 1.25 / radius;  // between vertices 2 and 3
  const auto snap = utils::snap_point_to_trajectory(
    radius * std::sin(query_theta), radius * (1.0 - std::cos(query_theta)), polyline,
    default_options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_NEAR(snap->heading_yaw, query_theta, 1e-3);
  ASSERT_TRUE(snap->tangent_yaw.has_value());
  EXPECT_NEAR(*snap->tangent_yaw, query_theta, 1e-3);
}

// When the trajectory around the snapped point is shorter than yaw_fit_min_length_m the geometry
// carries no reliable heading, so no tangent heading is reported (the caller falls back).
TEST_F(UtilsTest, SnapPointToTrajectoryNoTangentOnTooShortWindow)
{
  const auto polyline = straight_polyline(3, 0.05);  // 10 cm in total
  const utils::TrajectorySnapOptions options{0, 5, 1.0, 0.2};

  const auto snap = utils::snap_point_to_trajectory(0.05, 0.0, polyline, options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_FALSE(snap->tangent_yaw.has_value());
}

// A prediction that comes to a stop repeats its last position. Those coincident vertices must not
// break the spline: the leading distinct vertices are used and the snap still succeeds.
TEST_F(UtilsTest, SnapPointToTrajectoryIgnoresCoincidentTail)
{
  auto polyline = straight_polyline(6, 1.0);
  for (size_t i = 0; i < 10; ++i) {
    polyline.push_back(make_pose(5.0, 0.0, 0.0));
  }

  const auto snap = utils::snap_point_to_trajectory(2.0, 0.1, polyline, default_options);

  ASSERT_TRUE(snap.has_value());
  EXPECT_NEAR(snap->position.x(), 2.0, 1e-6);
  EXPECT_NEAR(snap->position.y(), 0.0, 1e-6);
}

// Fewer than two distinct vertices (e.g. a fully stopped prediction) cannot be snapped onto.
TEST_F(UtilsTest, SnapPointToTrajectoryReturnsNulloptOnDegeneratePolyline)
{
  const std::vector<Eigen::Matrix4d> polyline(5, make_pose(1.0, 1.0, 0.0));

  EXPECT_FALSE(utils::snap_point_to_trajectory(1.0, 1.0, polyline, default_options).has_value());
  EXPECT_FALSE(utils::snap_point_to_trajectory(0.0, 0.0, {}, default_options).has_value());
}

// The tangent window is symmetric around the snapped point: close to the start of the polyline it
// shrinks instead of extending forward only, so a curve does not bias the heading into the turn.
TEST_F(UtilsTest, SnapPointToTrajectoryTangentWindowIsSymmetricNearStart)
{
  constexpr double radius = 10.0;
  std::vector<Eigen::Matrix4d> polyline;
  for (size_t i = 0; i < 40; ++i) {
    const double theta = static_cast<double>(i) * 0.25 / radius;  // 0.25 m arc steps
    polyline.push_back(
      make_pose(radius * std::sin(theta), radius * (1.0 - std::cos(theta)), theta));
  }
  const double query_theta = 0.5 / radius;  // 0.5 m from the start; a +-1 m window would clip
  const auto snap = utils::snap_point_to_trajectory(
    radius * std::sin(query_theta), radius * (1.0 - std::cos(query_theta)), polyline,
    utils::TrajectorySnapOptions{0, 5, 1.0, 0.2});

  ASSERT_TRUE(snap.has_value());
  ASSERT_TRUE(snap->tangent_yaw.has_value());
  // A forward-only window [0, 1.5] would average to ~0.075 rad; symmetric [0, 1.0] gives 0.05.
  EXPECT_NEAR(*snap->tangent_yaw, query_theta, 2e-3);
}

// Prefix vertices extend the spline behind the trajectory start: the closest point is still
// searched from the trajectory start, interpolation_index stays relative to it, and the tangent
// window can now reach behind the snapped point instead of shrinking.
TEST_F(UtilsTest, SnapPointToTrajectoryPrefixExtendsWindowBackwards)
{
  std::vector<Eigen::Matrix4d> polyline;
  for (int i = -10; i < 20; ++i) {  // 10 prefix vertices at x<0, trajectory from x=0
    const double jitter_y = (i % 2 == 0) ? 0.01 : -0.01;
    polyline.push_back(make_pose(static_cast<double>(i) * 0.1, jitter_y, 0.0));
  }
  const utils::TrajectorySnapOptions options{10, 5, 1.0, 0.2};

  // Query behind the trajectory start: must clamp onto the start, not onto the prefix.
  const auto behind = utils::snap_point_to_trajectory(-0.5, 0.0, polyline, options);
  ASSERT_TRUE(behind.has_value());
  EXPECT_NEAR(behind->position.x(), 0.0, 1e-6);
  EXPECT_NEAR(behind->interpolation_index, 0.0, 1e-6);

  // Query 0.35 m in: a symmetric +-1 m window is available thanks to the prefix, so the jittered
  // headings average out where a prefix-less polyline would only offer +-0.35 m.
  const auto snap = utils::snap_point_to_trajectory(0.35, 0.0, polyline, options);
  ASSERT_TRUE(snap.has_value());
  EXPECT_NEAR(snap->interpolation_index, 3.5, 0.2);
  ASSERT_TRUE(snap->tangent_yaw.has_value());
  EXPECT_NEAR(*snap->tangent_yaw, 0.0, 0.02);
}

TEST_F(UtilsTest, BoundSnappedPoseKeepsSnappedPoseWhenStrengthIsOne)
{
  const auto b = utils::bound_snapped_pose(
    Eigen::Vector2d(0.1, 0.2), 0.05, Eigen::Vector2d(0.0, 0.0), 0.0, 1.0, 0.3, 0.1);
  EXPECT_NEAR(b.position.x(), 0.0, 1e-9);
  EXPECT_NEAR(b.position.y(), 0.0, 1e-9);
  EXPECT_NEAR(b.yaw, 0.0, 1e-9);
}

TEST_F(UtilsTest, BoundSnappedPoseBlendsByStrength)
{
  const auto b = utils::bound_snapped_pose(
    Eigen::Vector2d(0.2, 0.0), 0.04, Eigen::Vector2d(0.0, 0.0), 0.0, 0.75, 0.3, 0.1);
  // virtual = real - strength * (real - snapped)
  EXPECT_NEAR(b.position.x(), 0.05, 1e-9);
  EXPECT_NEAR(b.yaw, 0.01, 1e-9);
}

// Beyond the limits the pose is saturated at the limit from the real pose instead of being
// rejected, so the output is continuous in the residual.
TEST_F(UtilsTest, BoundSnappedPoseSaturatesAtLimits)
{
  const auto b = utils::bound_snapped_pose(
    Eigen::Vector2d(0.0, 1.0), 0.5, Eigen::Vector2d(0.0, 0.0), 0.0, 1.0, 0.3, 0.1);
  EXPECT_NEAR(b.position.x(), 0.0, 1e-9);
  EXPECT_NEAR(b.position.y(), 0.7, 1e-9);  // 0.3 m from the real pose toward the snapped one
  EXPECT_NEAR(b.yaw, 0.4, 1e-9);           // 0.1 rad from the real yaw toward the snapped one
}

TEST_F(UtilsTest, BoundSnappedPoseWrapsYaw)
{
  const auto b = utils::bound_snapped_pose(
    Eigen::Vector2d::Zero(), M_PI - 0.01, Eigen::Vector2d::Zero(), -M_PI + 0.01, 1.0, 0.3, 0.1);
  // The residual across the +-pi seam is 0.02 rad, not 2 pi - 0.02.
  EXPECT_NEAR(std::abs(b.yaw), M_PI - 0.01, 1e-9);
}

TEST_F(UtilsTest, BoundSnappedPoseThrowsOnBadArguments)
{
  EXPECT_THROW(
    utils::bound_snapped_pose(
      Eigen::Vector2d::Zero(), 0.0, Eigen::Vector2d::Zero(), 0.0, 1.5, 0.3, 0.1),
    std::runtime_error);
  EXPECT_THROW(
    utils::bound_snapped_pose(
      Eigen::Vector2d::Zero(), 0.0, Eigen::Vector2d::Zero(), 0.0, 0.1, 0.0, 0.1),
    std::runtime_error);
}

TEST_F(UtilsTest, SnapPointToTrajectoryThrowsOnNonPositiveSearchWindow)
{
  const auto polyline = straight_polyline(3, 1.0);
  const utils::TrajectorySnapOptions options{0, 0, 1.0, 0.2};

  EXPECT_THROW(utils::snap_point_to_trajectory(0.0, 0.0, polyline, options), std::runtime_error);
}

}  // namespace autoware::diffusion_planner::test
