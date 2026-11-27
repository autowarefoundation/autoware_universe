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

#include "autoware/pointcloud_preprocessor/distortion_corrector/motion_table.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace autoware::pointcloud_preprocessor::motion_table
{
namespace
{

constexpr float tolerance = 1e-5F;

constexpr std::int64_t operator""_ms(unsigned long long milliseconds)  // NOLINT(runtime/int)
{
  return static_cast<std::int64_t>(milliseconds) * 1'000'000;
}

void undistort_point(const Bin2D & bin, float remainder, Eigen::Vector3f & point)
{
  undistort(bin, remainder, point.x(), point.y());
}

void undistort_point(const Bin3D & bin, float remainder, Eigen::Vector3f & point)
{
  undistort(bin, remainder, point.x(), point.y(), point.z());
}

/// Where the table moves `point`, stamped `time_ns` after the header stamp.
template <typename BinT>
Eigen::Vector3f undistorted(
  const std::vector<BinT> & bins, std::int64_t time_ns, Eigen::Vector3f point,
  std::uint32_t * flags = nullptr)
{
  float remainder = 0.F;
  const BinT & bin = lookup(
    bins.data(), static_cast<std::uint32_t>(bins.size() - 1), static_cast<std::uint32_t>(time_ns),
    remainder);
  if (flags != nullptr) {
    *flags = bin.flags;
  }
  undistort_point(bin, remainder, point);
  return point;
}

/// The pose after moving with the constant planar velocity (`v`, `w`) for `t` seconds.
Eigen::Isometry3d planar_arc(double v, double w, double t)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(v / w * std::sin(w * t), v / w * (1.0 - std::cos(w * t)), 0);
  pose.linear() = Eigen::AngleAxisd(w * t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return pose;
}

void expect_near(const Eigen::Vector3f & actual, const Eigen::Vector3d & expected)
{
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

// Times at the start, in the middle and at the end of bins, and across many bins.
const std::vector<std::int64_t> probe_times_ns = {
  0, 1, 500'000, 1'048'575, 1'048'576, 1'048'577, 7'000'000, 33'333'333, 60'000'000, 89'999'999};

TEST(MotionTable, ConstantPlanarTwistFollowsTheArc)
{
  const double v = 20.0;
  const double w = 0.5;
  const Eigen::Vector3f linear(v, 0, 0);
  const Eigen::Vector3f angular(0, 0, w);
  const std::vector<TwistSample> twists = {
    TwistSample{0, linear, angular}, TwistSample{100_ms, linear, angular}};
  const Eigen::Vector3f point(3.0F, -2.0F, 1.0F);

  std::vector<Bin2D> bins_2d;
  std::vector<Bin3D> bins_3d;
  build<Integrator2D>(twists, {}, false, bins_2d);
  build<Integrator3D>(twists, {}, false, bins_3d);

  for (const auto time_ns : probe_times_ns) {
    SCOPED_TRACE(time_ns);
    const Eigen::Vector3d expected = planar_arc(v, w, 1e-9 * time_ns) * point.cast<double>();
    std::uint32_t flags = ~0U;
    expect_near(undistorted(bins_2d, time_ns, point, &flags), expected);
    EXPECT_EQ(flags, 0U);
    expect_near(undistorted(bins_3d, time_ns, point, &flags), expected);
    EXPECT_EQ(flags, 0U);
  }
}

// The reference composes a microsecond of translation and of rotation at a time, which shares no
// formula with the integrator.
TEST(MotionTable, ConstantSpatialTwistMatchesFineIntegration)
{
  const Eigen::Vector3f linear(15.0F, -1.0F, 0.5F);
  const Eigen::Vector3f angular(0.2F, -0.3F, 0.6F);
  const std::vector<TwistSample> twists = {
    TwistSample{0, linear, angular}, TwistSample{100_ms, linear, angular}};
  const Eigen::Vector3f point(3.0F, -2.0F, 1.0F);

  std::vector<Bin3D> bins;
  build<Integrator3D>(twists, {}, false, bins);

  constexpr double step_sec = 1e-6;
  const Eigen::AngleAxisd step_rotation(
    angular.cast<double>().norm() * step_sec, angular.cast<double>().normalized());
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  for (int step = 1; step <= 90'000; ++step) {
    pose.translate(linear.cast<double>() * step_sec);
    pose.rotate(step_rotation);
    if (step % 9'000 == 0) {
      SCOPED_TRACE(step);
      expect_near(undistorted(bins, step * 1'000LL, point), pose * point.cast<double>());
    }
  }
}

TEST(MotionTable, TimeTakesItsVelocityFromTheFirstSampleAtOrAfterIt)
{
  const Eigen::Vector3f zero = Eigen::Vector3f::Zero();
  const std::vector<TwistSample> twists = {
    TwistSample{-5_ms, Eigen::Vector3f(100.0F, 0, 0), zero},
    TwistSample{10_ms, Eigen::Vector3f(1.0F, 0, 0), zero},
    TwistSample{30_ms, Eigen::Vector3f(3.0F, 0, 0), zero}};

  std::vector<Bin2D> bins;
  build<Integrator2D>(twists, {}, false, bins);

  const auto x_at = [&bins](std::int64_t time_ns) {
    return undistorted(bins, time_ns, Eigen::Vector3f::Zero()).x();
  };
  EXPECT_NEAR(x_at(5_ms), 0.005F, tolerance);
  EXPECT_NEAR(x_at(9_ms), 0.009F, tolerance);
  EXPECT_NEAR(x_at(11_ms), 0.013F, tolerance);
  EXPECT_NEAR(x_at(20_ms), 0.040F, tolerance);
  EXPECT_NEAR(x_at(30_ms), 0.070F, tolerance);
  // The pose is linear within a bin, which spreads a velocity step of dv inside the bin over the
  // whole bin. That misplaces the times in that bin by up to dv * bin_width_sec / 4.
  EXPECT_NEAR(x_at(10_ms), 0.010F, 2.0F * bin_width_sec / 4.0F);
  // The last sample stays in effect after its stamp
  EXPECT_NEAR(x_at(40_ms), 0.100F, tolerance);
}

TEST(MotionTable, AngularVelocitySamplesTakePrecedenceOverTheTwists)
{
  const std::vector<TwistSample> twists = {
    TwistSample{0, Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 0, 1.0F)},
    TwistSample{100_ms, Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 0, 1.0F)}};
  const std::vector<AngularVelocitySample> angular_velocities = {
    AngularVelocitySample{0, Eigen::Vector3f(0, 0, 2.0F)},
    AngularVelocitySample{100_ms, Eigen::Vector3f(0, 0, 2.0F)}};
  const Eigen::Vector3f point(10.0F, 0, 0);

  std::vector<Bin2D> bins;
  build<Integrator2D>(twists, angular_velocities, true, bins);
  EXPECT_NEAR(undistorted(bins, 50_ms, point).y(), 10.0 * std::sin(0.1), tolerance);

  build<Integrator2D>(twists, angular_velocities, false, bins);
  EXPECT_NEAR(undistorted(bins, 50_ms, point).y(), 10.0 * std::sin(0.05), tolerance);
}

TEST(MotionTable, TimesWithoutASampleNearbyAreFlaggedAndDoNotMove)
{
  const std::vector<TwistSample> twists = {
    TwistSample{0, Eigen::Vector3f(1.0F, 0, 0), Eigen::Vector3f::Zero()}};
  const std::vector<AngularVelocitySample> angular_velocities = {
    AngularVelocitySample{0, Eigen::Vector3f::Zero()}};

  std::vector<Bin2D> bins;
  build<Integrator2D>(twists, angular_velocities, true, bins);

  std::uint32_t flags = ~0U;
  EXPECT_NEAR(undistorted(bins, 50_ms, Eigen::Vector3f::Zero(), &flags).x(), 0.05F, tolerance);
  EXPECT_EQ(flags, 0U);

  // The sample is `max_sample_distance_ns` old at 100 ms. The pose stops within a bin of that.
  const float bin_travel = 1.0F * bin_width_sec;
  for (const auto time_ns : {150_ms, 1'000_ms, 4'000_ms}) {
    SCOPED_TRACE(time_ns);
    EXPECT_NEAR(undistorted(bins, time_ns, Eigen::Vector3f::Zero(), &flags).x(), 0.1F, bin_travel);
    EXPECT_EQ(flags, flag_twist_missing | flag_imu_missing | flag_mismatch);
  }
}

TEST(MotionTable, MissingAngularVelocityFallsBackToTheTwistAndCountsAsMismatch)
{
  const std::vector<TwistSample> twists = {
    TwistSample{0, Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 0, 1.0F)},
    TwistSample{100_ms, Eigen::Vector3f::Zero(), Eigen::Vector3f(0, 0, 1.0F)}};

  std::vector<Bin2D> bins;
  build<Integrator2D>(twists, {}, true, bins);

  std::uint32_t flags = 0;
  const Eigen::Vector3f point(10.0F, 0, 0);
  EXPECT_NEAR(undistorted(bins, 50_ms, point, &flags).y(), 10.0 * std::sin(0.05), tolerance);
  EXPECT_EQ(flags, flag_mismatch);
}

TEST(MotionTable, TableWithoutSamplesLeavesPointsInPlace)
{
  std::vector<Bin3D> bins;
  build<Integrator3D>({}, {}, false, bins);

  ASSERT_EQ(bins.size(), 1U);
  const Eigen::Vector3f point(3.0F, -2.0F, 1.0F);
  std::uint32_t flags = 0;
  EXPECT_EQ(undistorted(bins, 50_ms, point, &flags), point);
  EXPECT_EQ(flags, flag_twist_missing | flag_mismatch);
}

TEST(MotionTable, SamplesLongBeforeTheHeaderStampAreMissing)
{
  const std::vector<TwistSample> twists = {
    TwistSample{-500_ms, Eigen::Vector3f(1.0F, 0, 0), Eigen::Vector3f::Zero()}};

  std::vector<Bin2D> bins;
  build<Integrator2D>(twists, {}, false, bins);

  ASSERT_EQ(bins.size(), 1U);
  EXPECT_EQ(bins.front().flags, flag_twist_missing | flag_mismatch);
}

TEST(MotionTable, PoseIsContinuousAcrossBins)
{
  const std::vector<TwistSample> twists = {
    TwistSample{0, Eigen::Vector3f(30.0F, 0, 0), Eigen::Vector3f(0, 0, 1.0F)},
    TwistSample{100_ms, Eigen::Vector3f(30.0F, 0, 0), Eigen::Vector3f(0, 0, 1.0F)}};
  const Eigen::Vector3f point(50.0F, 20.0F, 0);

  std::vector<Bin2D> bins;
  build<Integrator2D>(twists, {}, false, bins);

  // A nanosecond of this motion moves the point by well under a micrometre
  for (std::int64_t bin = 1; bin < 80; ++bin) {
    const std::int64_t boundary_ns = bin * bin_width_ns;
    const Eigen::Vector3f before = undistorted(bins, boundary_ns - 1, point);
    const Eigen::Vector3f after = undistorted(bins, boundary_ns, point);
    EXPECT_LT((after - before).norm(), tolerance) << bin;
  }
}

}  // namespace
}  // namespace autoware::pointcloud_preprocessor::motion_table
