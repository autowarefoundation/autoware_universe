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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_

#include "autoware/pointcloud_preprocessor/distortion_corrector/motion_table.hpp"

#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tl/expected.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

struct AngleConversion
{
  // Equation for the conversion between sensor azimuth coordinates and Cartesian coordinates:
  // sensor azimuth coordinates = offset_rad + sign * cartesian coordinates;
  // offset_rad is restricted to be a multiple of 90, and sign is restricted to be 1 or -1.
  float offset_rad{0};
  float sign{1};
  static constexpr float offset_rad_threshold{(5.0f / 180.0f) * M_PI};

  static constexpr float sign_threshold{0.1f};
};

// Reason why an input pointcloud was rejected. Reported to the caller instead of being logged, so
// that the core stays free of ROS logging.
enum class PointcloudValidity {
  kValid,
  kEmpty,
  kMissingTimeStampField,
  kIncompatibleLayout,
};

// Outcome of undistort_pointcloud(). The core reports these conditions to its caller instead of
// logging them.
struct UndistortionResult
{
  PointcloudValidity validity{PointcloudValidity::kValid};
  bool twist_queue_empty{false};
  bool twist_timestamp_too_late{false};
  bool imu_timestamp_too_late{false};
};

class DistortionCorrectorBase
{
protected:
  geometry_msgs::msg::TransformStamped::SharedPtr geometry_imu_to_base_link_ptr_;
  bool pointcloud_transform_needed_{false};

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  int timestamp_mismatch_count_{0};
  double timestamp_mismatch_fraction_{0.0};

  // Motion samples of the current pointcloud, stamped relative to its header stamp.
  std::vector<motion_table::TwistSample> twist_samples_;
  std::vector<motion_table::AngularVelocitySample> angular_velocity_samples_;

  // Transforms between the pointcloud's frame and the base frame, in which the motion is given.
  Eigen::Matrix4f lidar_to_base_link_{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f base_link_to_lidar_{Eigen::Matrix4f::Identity()};

  void enqueue_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  // Collects the queued motion samples that can affect a pointcloud stamped `pointcloud_stamp`.
  void collect_motion_samples(const builtin_interfaces::msg::Time & pointcloud_stamp, bool use_imu);
  // The lidar-to-base-frame transform (header.frame_id = base frame, child_frame_id = lidar frame)
  // is supplied by the caller; TF lookup remains outside the core.
  void set_pointcloud_transform(const geometry_msgs::msg::TransformStamped & lidar_to_base_link);

public:
  DistortionCorrectorBase() = default;
  virtual ~DistortionCorrectorBase() = default;

  std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue();
  std::deque<geometry_msgs::msg::Vector3Stamped> get_angular_velocity_queue();
  // Samples are expected in timestamp order. A sample older than the queue front clears the queue
  // (e.g. a rosbag restart); samples older than the newest one by more than 1 s are dropped.
  void insert_twist(const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg);

  // The IMU-to-base-frame transform must be provided (via set_imu_transform) before processing IMU
  // messages, so that angular velocities can be rotated into the base frame.
  void set_imu_transform(const geometry_msgs::msg::TransformStamped & imu_to_base_link);

  // Same ordering and retention rules as insert_twist().
  void insert_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

  tl::expected<AngleConversion, std::string> try_compute_angle_conversion(
    sensor_msgs::msg::PointCloud2 & pointcloud);

  PointcloudValidity check_pointcloud_validity(sensor_msgs::msg::PointCloud2 & pointcloud);

  [[nodiscard]] int get_timestamp_mismatch_count() const { return timestamp_mismatch_count_; }
  [[nodiscard]] double get_timestamp_mismatch_fraction() const
  {
    return timestamp_mismatch_fraction_;
  }

  virtual UndistortionResult undistort_pointcloud(
    bool use_imu, std::optional<AngleConversion> angle_conversion_opt,
    const geometry_msgs::msg::TransformStamped & lidar_to_base_link,
    sensor_msgs::msg::PointCloud2 & pointcloud) = 0;
};

// Every point is corrected by its own `time_stamp` field alone, relative to the header stamp of
// the pointcloud, which no point precedes. The points may be stored in any order.
template <class IntegratorT>
class DistortionCorrector : public DistortionCorrectorBase
{
public:
  UndistortionResult undistort_pointcloud(
    bool use_imu, std::optional<AngleConversion> angle_conversion_opt,
    const geometry_msgs::msg::TransformStamped & lidar_to_base_link,
    sensor_msgs::msg::PointCloud2 & pointcloud) override;

private:
  std::vector<typename IntegratorT::Bin> motion_bins_;
};

// Corrects for planar motion: linear velocity along x and angular velocity around z.
using DistortionCorrector2D = DistortionCorrector<motion_table::Integrator2D>;
// Corrects for rigid motion in space.
using DistortionCorrector3D = DistortionCorrector<motion_table::Integrator3D>;

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
