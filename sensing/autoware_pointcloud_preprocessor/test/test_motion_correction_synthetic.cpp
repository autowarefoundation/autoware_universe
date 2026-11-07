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

#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"
#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace
{

using autoware::pointcloud_preprocessor::CombineCloudHandler;
using autoware::pointcloud_preprocessor::DistortionCorrector3D;
using autoware::pointcloud_preprocessor::PointCloud2Traits;

constexpr int32_t kStampSec = 20;
constexpr uint32_t kStampNsec = 100'000'000;
constexpr double kNsecPerSec = 1.0e9;
constexpr double kExactTolerance = 2.0e-4;

struct TimedPoint
{
  double relative_time_s{};
  Eigen::Vector3d measured_sensor;
  Eigen::Vector3d expected_start_sensor;
};

struct Plane
{
  Eigen::Vector3d normal;
  double offset{};
};

enum class ScanPattern { LeftToRight, TopToBottom, Diagonal };
enum class PointOrder { IncreasingTime, DecreasingTime, EvenOddInterleaved };
enum class ExpectedResult { Exact, ExpectedFailure };

struct Twist
{
  Eigen::Vector3d linear{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular{Eigen::Vector3d::Zero()};
};

struct MotionProfile
{
  std::string name;
  Twist (*twist_at)(double);
};

struct DistortionCase
{
  std::string name;
  MotionProfile motion;
  ScanPattern scan_pattern{};
  PointOrder point_order{};
  double scan_duration_s{};
  double twist_sample_period_s{};
  ExpectedResult expected{};
  double failure_residual_threshold{};
};

struct ConcatenateCase
{
  std::string name;
  Twist twist;
  double dt_s{};
  ExpectedResult expected{};
  double failure_norm_threshold{};
};

Sophus::SE3d exp_twist(const Twist & twist, const double dt)
{
  Sophus::SE3d::Tangent tangent;
  tangent << twist.linear.x(), twist.linear.y(), twist.linear.z(), twist.angular.x(),
    twist.angular.y(), twist.angular.z();
  return Sophus::SE3d::exp(tangent * dt);
}

Twist static_twist(const double)
{
  return {};
}

Twist pure_x_translation(const double)
{
  return {Eigen::Vector3d{3.0, 0.0, 0.0}, Eigen::Vector3d::Zero()};
}

Twist pure_xyz_translation(const double)
{
  return {Eigen::Vector3d{1.0, -0.7, 0.4}, Eigen::Vector3d::Zero()};
}

Twist constant_yaw_arc(const double)
{
  return {Eigen::Vector3d{4.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.8}};
}

Twist constant_twist(const double)
{
  return {Eigen::Vector3d{3.0, -0.4, 0.2}, Eigen::Vector3d{0.2, -0.15, 0.35}};
}

Twist linear_changing_twist(const double t)
{
  return {
    Eigen::Vector3d{2.0 + 6.0 * t, -0.5 + 2.0 * t, 0.2 - 1.0 * t},
    Eigen::Vector3d{0.05 + 0.8 * t, -0.2 + 0.6 * t, 0.15 - 0.4 * t}};
}

Twist complex_changing_twist(const double t)
{
  return {
    Eigen::Vector3d{2.0 + std::sin(7.0 * t), -0.3 + 0.5 * std::cos(5.0 * t), 0.2 * t},
    Eigen::Vector3d{0.15 + 0.5 * std::sin(3.0 * t), -0.15 * std::cos(4.0 * t), 0.25 + 0.2 * t}};
}

std::string to_string(const ScanPattern pattern)
{
  switch (pattern) {
    case ScanPattern::LeftToRight:
      return "LeftToRight";
    case ScanPattern::TopToBottom:
      return "TopToBottom";
    case ScanPattern::Diagonal:
      return "Diagonal";
  }
  return "UnknownScanPattern";
}

std::string to_string(const PointOrder order)
{
  switch (order) {
    case PointOrder::IncreasingTime:
      return "IncreasingTime";
    case PointOrder::DecreasingTime:
      return "DecreasingTime";
    case PointOrder::EvenOddInterleaved:
      return "EvenOddInterleaved";
  }
  return "UnknownPointOrder";
}

std::string to_string(const ExpectedResult expected)
{
  return expected == ExpectedResult::Exact ? "Exact" : "ExpectedFailure";
}

std::ostream & operator<<(std::ostream & os, const DistortionCase & param)
{
  return os << param.name << " motion=" << param.motion.name
            << " scan=" << to_string(param.scan_pattern)
            << " order=" << to_string(param.point_order)
            << " expected=" << to_string(param.expected);
}

std::ostream & operator<<(std::ostream & os, const ConcatenateCase & param)
{
  return os << param.name << " expected=" << to_string(param.expected);
}

template <typename TwistFunction>
Sophus::SE3d integrate_pose(const TwistFunction & twist_function, const double target_time_s)
{
  Sophus::SE3d pose;
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(target_time_s) / 0.0002)));
  const double dt = target_time_s / static_cast<double>(steps);
  for (int i = 0; i < steps; ++i) {
    const double sample_time = (static_cast<double>(i) + 0.5) * dt;
    pose = pose * exp_twist(twist_function(sample_time), dt);
  }
  return pose;
}

std::vector<Eigen::Vector3d> generate_scan_directions(
  const ScanPattern pattern, const std::size_t count)
{
  std::vector<Eigen::Vector3d> directions;
  directions.reserve(count);

  for (std::size_t i = 0; i < count; ++i) {
    const double u = count == 1 ? 0.0 : -1.0 + 2.0 * static_cast<double>(i) / (count - 1);
    double yaw = 0.0;
    double pitch = 0.0;
    if (pattern == ScanPattern::LeftToRight) {
      yaw = 0.45 * u;
      pitch = 0.18 * std::sin(2.0 * u);
    } else if (pattern == ScanPattern::TopToBottom) {
      yaw = 0.25 * std::sin(2.5 * u);
      pitch = 0.35 * u;
    } else {
      yaw = 0.45 * u;
      pitch = 0.35 * u;
    }

    Eigen::Vector3d direction{
      std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)};
    directions.push_back(direction.normalized());
  }
  return directions;
}

std::pair<Eigen::Vector3d, bool> intersect_world(
  const Eigen::Vector3d & origin, const Eigen::Vector3d & direction)
{
  const std::array<Plane, 6> planes{{
    {Eigen::Vector3d{1.0, 0.0, 0.0}, 18.0},
    {Eigen::Vector3d{0.0, 1.0, 0.0}, 14.0},
    {Eigen::Vector3d{0.0, -1.0, 0.0}, 14.0},
    {Eigen::Vector3d{0.0, 0.0, 1.0}, 5.0},
    {Eigen::Vector3d{0.0, 0.0, -1.0}, 3.0},
    {Eigen::Vector3d{1.0, 1.0, 1.0}.normalized(), 24.0},
  }};

  double best_distance = std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_point = Eigen::Vector3d::Zero();
  for (const auto & plane : planes) {
    const double denominator = plane.normal.dot(direction);
    if (std::abs(denominator) < 1.0e-9) {
      continue;
    }
    const double distance = (plane.offset - plane.normal.dot(origin)) / denominator;
    if (distance > 0.5 && distance < best_distance) {
      best_distance = distance;
      best_point = origin + distance * direction;
    }
  }

  return {best_point, std::isfinite(best_distance)};
}

template <typename TwistFunction>
std::vector<TimedPoint> simulate_scan(
  const TwistFunction & twist_function, const ScanPattern pattern, const PointOrder point_order,
  const double scan_duration_s, const std::size_t point_count)
{
  std::vector<TimedPoint> scan;
  scan.reserve(point_count);
  const auto directions = generate_scan_directions(pattern, point_count);

  for (std::size_t i = 0; i < point_count; ++i) {
    const double ratio = point_count == 1 ? 0.0 : static_cast<double>(i) / (point_count - 1);
    const double time_s = ratio * scan_duration_s;
    const auto pose = integrate_pose(twist_function, time_s);
    const Eigen::Vector3d origin = pose.translation();
    const Eigen::Vector3d world_direction = pose.so3().matrix() * directions[i];
    const auto [world_point, hit] = intersect_world(origin, world_direction);
    if (!hit) {
      throw std::runtime_error("synthetic ray did not hit any world plane");
    }

    scan.push_back({time_s, pose.inverse() * world_point, world_point});
  }

  if (point_order == PointOrder::IncreasingTime) {
    std::sort(scan.begin(), scan.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.relative_time_s < rhs.relative_time_s;
    });
  } else if (point_order == PointOrder::DecreasingTime) {
    std::sort(scan.begin(), scan.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.relative_time_s > rhs.relative_time_s;
    });
  } else {
    std::sort(scan.begin(), scan.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.relative_time_s < rhs.relative_time_s;
    });
    std::vector<TimedPoint> reordered;
    reordered.reserve(scan.size());
    for (std::size_t i = 0; i < scan.size(); i += 2) {
      reordered.push_back(scan[i]);
    }
    for (std::size_t i = 1; i < scan.size(); i += 2) {
      reordered.push_back(scan[i]);
    }
    scan = std::move(reordered);
  }
  return scan;
}

sensor_msgs::msg::PointCloud2 make_pointcloud(const std::vector<TimedPoint> & points)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.stamp = rclcpp::Time(kStampSec, kStampNsec, RCL_ROS_TIME);
  cloud.header.frame_id = "base_link";
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    10, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
    sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::UINT8, "return_type", 1,
    sensor_msgs::msg::PointField::UINT8, "channel", 1, sensor_msgs::msg::PointField::UINT16,
    "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32, "elevation", 1,
    sensor_msgs::msg::PointField::FLOAT32, "distance", 1, sensor_msgs::msg::PointField::FLOAT32,
    "time_stamp", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_azimuth(cloud, "azimuth");
  sensor_msgs::PointCloud2Iterator<float> it_elevation(cloud, "elevation");
  sensor_msgs::PointCloud2Iterator<float> it_distance(cloud, "distance");
  sensor_msgs::PointCloud2Iterator<std::uint32_t> it_time_stamp(cloud, "time_stamp");

  for (const auto & point : points) {
    *it_x = static_cast<float>(point.measured_sensor.x());
    *it_y = static_cast<float>(point.measured_sensor.y());
    *it_z = static_cast<float>(point.measured_sensor.z());
    *it_azimuth =
      static_cast<float>(std::atan2(point.measured_sensor.y(), point.measured_sensor.x()));
    *it_elevation = static_cast<float>(
      std::atan2(point.measured_sensor.z(), point.measured_sensor.head<2>().norm()));
    *it_distance = static_cast<float>(point.measured_sensor.norm());
    *it_time_stamp = static_cast<std::uint32_t>(std::llround(point.relative_time_s * kNsecPerSec));

    ++it_x;
    ++it_y;
    ++it_z;
    ++it_azimuth;
    ++it_elevation;
    ++it_distance;
    ++it_time_stamp;
  }

  return cloud;
}

std::shared_ptr<geometry_msgs::msg::TwistWithCovarianceStamped> make_twist_msg(
  const Twist & twist, const double relative_time_s)
{
  auto msg = std::make_shared<geometry_msgs::msg::TwistWithCovarianceStamped>();
  const int64_t stamp_nsec = static_cast<int64_t>(kStampSec) * 1'000'000'000LL +
                             static_cast<int64_t>(kStampNsec) +
                             static_cast<int64_t>(std::llround(relative_time_s * kNsecPerSec));
  msg->header.stamp = rclcpp::Time(stamp_nsec, RCL_ROS_TIME);
  msg->header.frame_id = "base_link";
  msg->twist.twist.linear.x = twist.linear.x();
  msg->twist.twist.linear.y = twist.linear.y();
  msg->twist.twist.linear.z = twist.linear.z();
  msg->twist.twist.angular.x = twist.angular.x();
  msg->twist.twist.angular.y = twist.angular.y();
  msg->twist.twist.angular.z = twist.angular.z();
  return msg;
}

template <typename TwistFunction>
std::shared_ptr<DistortionCorrector3D> make_corrector_with_twists(
  rclcpp::Node & node, const TwistFunction & twist_function, const double scan_duration_s,
  const double sample_period_s)
{
  auto corrector = std::make_shared<DistortionCorrector3D>(node);
  for (double t = 0.0; t <= scan_duration_s + 1.0e-9; t += sample_period_s) {
    corrector->process_twist_message(make_twist_msg(twist_function(t), t));
  }
  corrector->initialize();
  corrector->set_pointcloud_transform("base_link", "base_link");
  return corrector;
}

std::vector<Eigen::Vector3d> read_xyz(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<Eigen::Vector3d> points;
  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    points.emplace_back(*it_x, *it_y, *it_z);
  }
  return points;
}

double max_residual(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::vector<TimedPoint> & expected)
{
  const auto corrected = read_xyz(cloud);
  EXPECT_EQ(corrected.size(), expected.size());

  double max_error = 0.0;
  for (std::size_t i = 0; i < corrected.size(); ++i) {
    max_error = std::max(max_error, (corrected[i] - expected[i].expected_start_sensor).norm());
  }
  return max_error;
}

std::shared_ptr<CombineCloudHandler<PointCloud2Traits>> make_combine_handler(rclcpp::Node & node)
{
  return std::make_shared<CombineCloudHandler<PointCloud2Traits>>(
    node, std::vector<std::string>{"lidar_a", "lidar_b"}, "base_link", true, false, false);
}

class SyntheticMotionCorrectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides({{"matching_strategy.type", "naive"}});
    node_ = std::make_shared<rclcpp::Node>("synthetic_motion_correction_test", options);
  }

  std::shared_ptr<rclcpp::Node> node_;
};

class SyntheticDistortionCorrectionTest : public SyntheticMotionCorrectionTest,
                                          public ::testing::WithParamInterface<DistortionCase>
{
};

TEST_P(SyntheticDistortionCorrectionTest, CorrectsSyntheticStaticWorld)
{
  const auto param = GetParam();
  SCOPED_TRACE(param);

  auto scan = simulate_scan(
    param.motion.twist_at, param.scan_pattern, param.point_order, param.scan_duration_s, 128);
  auto cloud = make_pointcloud(scan);
  auto corrector = make_corrector_with_twists(
    *node_, param.motion.twist_at, param.scan_duration_s, param.twist_sample_period_s);

  corrector->undistort_pointcloud(false, std::nullopt, cloud);

  const double residual = max_residual(cloud, scan);
  if (param.expected == ExpectedResult::Exact) {
    EXPECT_LT(residual, kExactTolerance);
    EXPECT_EQ(corrector->get_timestamp_mismatch_count(), 0);
  } else {
    EXPECT_GT(residual, param.failure_residual_threshold)
      << "Expected failure: current CPU 3D distortion correction assumes monotonic point order "
         "and piecewise-constant twist samples.";
  }
}

class SyntheticConcatenateMotionCorrectionTest
: public SyntheticMotionCorrectionTest,
  public ::testing::WithParamInterface<ConcatenateCase>
{
};

TEST_P(SyntheticConcatenateMotionCorrectionTest, ComputesWholeCloudTimestampTransform)
{
  const auto param = GetParam();
  SCOPED_TRACE(param);

  auto handler = make_combine_handler(*node_);
  handler->process_twist(make_twist_msg(param.twist, 0.0));
  handler->process_twist(make_twist_msg(param.twist, param.dt_s));

  const Eigen::Matrix4f transform = handler->compute_transform_to_adjust_for_old_timestamp(
    rclcpp::Time(kStampSec, kStampNsec, RCL_ROS_TIME),
    rclcpp::Time(
      static_cast<int64_t>(kStampSec) * 1'000'000'000LL + static_cast<int64_t>(kStampNsec) +
        static_cast<int64_t>(std::llround(param.dt_s * kNsecPerSec)),
      RCL_ROS_TIME));
  const auto expected = exp_twist(param.twist, param.dt_s).matrix().cast<float>();
  const auto error_norm = (transform.block<3, 4>(0, 0) - expected.block<3, 4>(0, 0)).norm();

  if (param.expected == ExpectedResult::Exact) {
    EXPECT_LT(error_norm, kExactTolerance);
  } else {
    EXPECT_GT(error_norm, param.failure_norm_threshold)
      << "Expected failure: current concatenate motion compensation is SE(2), uses linear.x and "
         "yaw only, and is not exact for general SE(3) cloud-to-cloud motion.";
  }
}

std::vector<DistortionCase> distortion_cases()
{
  const MotionProfile static_motion{"Static", static_twist};
  const MotionProfile pure_x{"PureXTranslation", pure_x_translation};
  const MotionProfile pure_xyz{"PureXYZTranslation", pure_xyz_translation};
  const MotionProfile yaw_arc{"ConstantYawArc", constant_yaw_arc};
  const MotionProfile se3{"ConstantSE3Twist", constant_twist};
  const MotionProfile linear{"LinearChangingSE3Twist", linear_changing_twist};
  const MotionProfile complex{"ComplexChangingSE3Twist", complex_changing_twist};

  return {
    {"static_left_to_right", static_motion, ScanPattern::LeftToRight, PointOrder::IncreasingTime,
     0.1, 0.01, ExpectedResult::Exact, 0.0},
    {"pure_x_top_to_bottom", pure_x, ScanPattern::TopToBottom, PointOrder::IncreasingTime, 0.1,
     0.01, ExpectedResult::Exact, 0.0},
    {"pure_xyz_diagonal", pure_xyz, ScanPattern::Diagonal, PointOrder::IncreasingTime, 0.1, 0.01,
     ExpectedResult::Exact, 0.0},
    {"yaw_arc_left_to_right", yaw_arc, ScanPattern::LeftToRight, PointOrder::IncreasingTime, 0.1,
     0.01, ExpectedResult::Exact, 0.0},
    {"constant_se3_top_to_bottom_slow_scan", se3, ScanPattern::TopToBottom,
     PointOrder::IncreasingTime, 0.1, 0.01, ExpectedResult::Exact, 0.0},
    {"constant_se3_left_to_right_fast_scan", se3, ScanPattern::LeftToRight,
     PointOrder::IncreasingTime, 0.03, 0.01, ExpectedResult::Exact, 0.0},
    {"constant_se3_decreasing_time_order", se3, ScanPattern::LeftToRight,
     PointOrder::DecreasingTime, 0.1, 0.01, ExpectedResult::ExpectedFailure, 0.5},
    {"linear_se3_interleaved_time_order", linear, ScanPattern::TopToBottom,
     PointOrder::EvenOddInterleaved, 0.1, 0.01, ExpectedResult::ExpectedFailure, 1.0e-3},
    {"linear_changing_se3", linear, ScanPattern::LeftToRight, PointOrder::IncreasingTime, 0.1, 0.01,
     ExpectedResult::ExpectedFailure, 1.0e-3},
    {"complex_changing_se3_sparse_twist", complex, ScanPattern::TopToBottom,
     PointOrder::IncreasingTime, 0.1, 0.02, ExpectedResult::ExpectedFailure, 1.0e-3},
    {"complex_changing_se3_dense_twist", complex, ScanPattern::Diagonal, PointOrder::IncreasingTime,
     0.1, 0.002, ExpectedResult::ExpectedFailure, kExactTolerance},
  };
}

std::vector<ConcatenateCase> concatenate_cases()
{
  return {
    {"static", {}, 0.1, ExpectedResult::Exact, 0.0},
    {"pure_x_translation",
     {Eigen::Vector3d{2.0, 0.0, 0.0}, Eigen::Vector3d::Zero()},
     0.1,
     ExpectedResult::Exact,
     0.0},
    {"pure_y_translation",
     {Eigen::Vector3d{0.0, 2.0, 0.0}, Eigen::Vector3d::Zero()},
     0.1,
     ExpectedResult::ExpectedFailure,
     0.1},
    {"pure_z_translation",
     {Eigen::Vector3d{0.0, 0.0, 2.0}, Eigen::Vector3d::Zero()},
     0.1,
     ExpectedResult::ExpectedFailure,
     0.1},
    {"forward_yaw_arc",
     {Eigen::Vector3d{5.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 2.0}},
     0.1,
     ExpectedResult::ExpectedFailure,
     0.04},
    {"lateral_and_yaw",
     {Eigen::Vector3d{1.0, 1.5, 0.0}, Eigen::Vector3d{0.0, 0.0, 1.0}},
     0.1,
     ExpectedResult::ExpectedFailure,
     0.1},
    {"roll_pitch_motion",
     {Eigen::Vector3d{0.0, 0.0, 1.5}, Eigen::Vector3d{0.4, -0.2, 0.0}},
     0.1,
     ExpectedResult::ExpectedFailure,
     0.1},
    {"full_se3_twist",
     {Eigen::Vector3d{1.0, -0.5, 0.4}, Eigen::Vector3d{0.3, -0.2, 0.7}},
     0.1,
     ExpectedResult::ExpectedFailure,
     0.05},
  };
}

std::string sanitize_test_name(std::string name)
{
  for (auto & c : name) {
    if (!std::isalnum(static_cast<unsigned char>(c))) {
      c = '_';
    }
  }
  return name;
}

INSTANTIATE_TEST_SUITE_P(
  RealWorldCases, SyntheticDistortionCorrectionTest, ::testing::ValuesIn(distortion_cases()),
  [](const auto & info) { return sanitize_test_name(info.param.name); });

INSTANTIATE_TEST_SUITE_P(
  RealWorldCases, SyntheticConcatenateMotionCorrectionTest,
  ::testing::ValuesIn(concatenate_cases()),
  [](const auto & info) { return sanitize_test_name(info.param.name); });

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
