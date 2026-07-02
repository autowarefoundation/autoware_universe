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

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tf2/LinearMath/Transform.hpp>

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

// Outcome of undistort_pointcloud(). Conditions that the node used to log directly are surfaced
// here so the (ROS-free) core can report them to its caller.
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
  bool pointcloud_transform_exists_{false};

  std::deque<geometry_msgs::msg::TwistStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  int timestamp_mismatch_count_{0};
  double timestamp_mismatch_fraction_{0.0};

  void enqueue_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);
  void get_twist_and_imu_iterator(
    bool use_imu, double first_point_time_stamp_sec,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu);

public:
  DistortionCorrectorBase() = default;
  virtual ~DistortionCorrectorBase() = default;

  [[nodiscard]] bool pointcloud_transform_exists() const;
  [[nodiscard]] bool pointcloud_transform_needed() const;
  std::deque<geometry_msgs::msg::TwistStamped> get_twist_queue();
  std::deque<geometry_msgs::msg::Vector3Stamped> get_angular_velocity_queue();
  void process_twist_message(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg);

  // The IMU-to-base-frame transform must be provided (via set_imu_transform) before processing IMU
  // messages, so that angular velocities can be rotated into the base frame.
  void set_imu_transform(const geometry_msgs::msg::TransformStamped & imu_to_base_link);

  void process_imu_message(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

  std::optional<AngleConversion> try_compute_angle_conversion(
    sensor_msgs::msg::PointCloud2 & pointcloud);

  PointcloudValidity check_pointcloud_validity(sensor_msgs::msg::PointCloud2 & pointcloud);

  [[nodiscard]] int get_timestamp_mismatch_count() const { return timestamp_mismatch_count_; }
  [[nodiscard]] double get_timestamp_mismatch_fraction() const
  {
    return timestamp_mismatch_fraction_;
  }

  // The lidar-to-base-frame transform (header.frame_id = base frame, child_frame_id = lidar frame)
  // must be provided before undistorting a pointcloud expressed in the lidar frame.
  virtual void set_pointcloud_transform(
    const geometry_msgs::msg::TransformStamped & lidar_to_base_link) = 0;
  virtual void initialize() = 0;
  virtual UndistortionResult undistort_pointcloud(
    bool use_imu, std::optional<AngleConversion> angle_conversion_opt,
    sensor_msgs::msg::PointCloud2 & pointcloud) = 0;
};

template <class T>
class DistortionCorrector : public DistortionCorrectorBase
{
public:
  UndistortionResult undistort_pointcloud(
    bool use_imu, std::optional<AngleConversion> angle_conversion_opt,
    sensor_msgs::msg::PointCloud2 & pointcloud) override;

  void undistort_point(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, float const & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid)
  {
    static_cast<T *>(this)->undistort_point_implementation(
      it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);
  };
};

class DistortionCorrector2D : public DistortionCorrector<DistortionCorrector2D>
{
private:
  // defined outside of for loop for performance reasons.
  tf2::Quaternion baselink_quat_;
  tf2::Transform baselink_tf_odom_;
  tf2::Vector3 point_tf_;
  tf2::Vector3 undistorted_point_tf_;
  float theta_;
  float x_;
  float y_;

  // TF
  tf2::Transform tf2_lidar_to_base_link_;
  tf2::Transform tf2_base_link_to_lidar_;

public:
  void initialize() override;
  void set_pointcloud_transform(
    const geometry_msgs::msg::TransformStamped & lidar_to_base_link) override;
  void undistort_point_implementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid);
};

class DistortionCorrector3D : public DistortionCorrector<DistortionCorrector3D>
{
private:
  // defined outside of for loop for performance reasons.
  Eigen::Vector4f point_eigen_;
  Eigen::Vector4f undistorted_point_eigen_;
  Eigen::Matrix4f transformation_matrix_;
  Eigen::Matrix4f prev_transformation_matrix_;

  // TF
  Eigen::Matrix4f eigen_lidar_to_base_link_;
  Eigen::Matrix4f eigen_base_link_to_lidar_;

public:
  void initialize() override;
  void set_pointcloud_transform(
    const geometry_msgs::msg::TransformStamped & lidar_to_base_link) override;
  void undistort_point_implementation(
    sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
    sensor_msgs::PointCloud2Iterator<float> & it_z,
    std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
    std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
    const bool & is_twist_valid, const bool & is_imu_valid);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_HPP_
