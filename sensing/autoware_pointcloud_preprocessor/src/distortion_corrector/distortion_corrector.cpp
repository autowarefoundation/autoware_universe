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

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include "autoware/pointcloud_preprocessor/concatenate_data/time_utils.hpp"
#include "autoware/pointcloud_preprocessor/utility/memory.hpp"
#include "autoware_utils/math/constants.hpp"

#include <Eigen/Geometry>
#include <autoware/point_types/types.hpp>
#include <autoware_utils/math/trigonometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::pointcloud_preprocessor
{
namespace
{
Eigen::Matrix4f to_eigen_matrix(const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Quaternionf rotation(
    static_cast<float>(transform.rotation.w), static_cast<float>(transform.rotation.x),
    static_cast<float>(transform.rotation.y), static_cast<float>(transform.rotation.z));
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = rotation.toRotationMatrix();
  matrix.block<3, 1>(0, 3) = Eigen::Vector3f(
    static_cast<float>(transform.translation.x), static_cast<float>(transform.translation.y),
    static_cast<float>(transform.translation.z));
  return matrix;
}

std::int64_t to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
}

// A rigid transform, applied to points in place.
struct Transform3f
{
  explicit Transform3f(const Eigen::Matrix4f & matrix)
  {
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 4; ++col) {
        m[row][col] = matrix(row, col);
      }
    }
  }

  void apply(float & x, float & y, float & z) const
  {
    const float px = x;
    const float py = y;
    const float pz = z;
    x = m[0][0] * px + m[0][1] * py + m[0][2] * pz + m[0][3];
    y = m[1][0] * px + m[1][1] * py + m[1][2] * pz + m[1][3];
    z = m[2][0] * px + m[2][1] * py + m[2][2] * pz + m[2][3];
  }

  float m[3][4];
};

inline void undistort_point(
  const motion_table::Bin2D & bin, float remainder, float & x, float & y, float & /*z*/)
{
  motion_table::undistort(bin, remainder, x, y);
}

inline void undistort_point(
  const motion_table::Bin3D & bin, float remainder, float & x, float & y, float & z)
{
  motion_table::undistort(bin, remainder, x, y, z);
}
}  // namespace

std::deque<geometry_msgs::msg::TwistStamped> DistortionCorrectorBase::get_twist_queue()
{
  return twist_queue_;
}

std::deque<geometry_msgs::msg::Vector3Stamped> DistortionCorrectorBase::get_angular_velocity_queue()
{
  return angular_velocity_queue_;
}

void DistortionCorrectorBase::insert_twist(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr twist_msg)
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header = twist_msg->header;
  msg.twist = twist_msg->twist.twist;

  // If time jumps backwards (e.g. when a rosbag restarts), clear buffer
  if (!twist_queue_.empty()) {
    if (is_earlier(msg.header.stamp, twist_queue_.front().header.stamp)) {
      twist_queue_.clear();
    }
  }

  // Twist data in the queue that is older than the current twist by 1 second will be cleared.
  const auto cutoff_time = add(msg.header.stamp, -std::chrono::seconds(1));

  while (!twist_queue_.empty()) {
    if (is_earlier(cutoff_time, twist_queue_.front().header.stamp)) {
      break;
    }
    twist_queue_.pop_front();
  }

  twist_queue_.push_back(msg);
}

void DistortionCorrectorBase::set_imu_transform(
  const geometry_msgs::msg::TransformStamped & imu_to_base_link)
{
  // Only the rotation is needed to rotate angular velocities into the base frame.
  geometry_imu_to_base_link_ptr_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
  geometry_imu_to_base_link_ptr_->transform.rotation = imu_to_base_link.transform.rotation;
}

void DistortionCorrectorBase::insert_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  if (!geometry_imu_to_base_link_ptr_) {
    return;  // set_imu_transform() must be called before processing IMU messages.
  }
  enqueue_imu(imu_msg);
}

void DistortionCorrectorBase::enqueue_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg)
{
  // Rotate the angular velocity into the base frame (rotation only; the stored transform carries no
  // translation). Equivalent to tf2::doTransform on a Vector3, without depending on
  // tf2_geometry_msgs.
  const auto & rotation = geometry_imu_to_base_link_ptr_->transform.rotation;
  const tf2::Vector3 rotated_angular_velocity = tf2::quatRotate(
    tf2::Quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
    tf2::Vector3(
      imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z));

  geometry_msgs::msg::Vector3Stamped transformed_angular_velocity;
  transformed_angular_velocity.vector.x = rotated_angular_velocity.x();
  transformed_angular_velocity.vector.y = rotated_angular_velocity.y();
  transformed_angular_velocity.vector.z = rotated_angular_velocity.z();
  transformed_angular_velocity.header = imu_msg->header;

  // If time jumps backwards (e.g. when a rosbag restarts), clear buffer
  if (!angular_velocity_queue_.empty()) {
    if (is_earlier(imu_msg->header.stamp, angular_velocity_queue_.front().header.stamp)) {
      angular_velocity_queue_.clear();
    }
  }

  // IMU data in the queue that is older than the current imu msg by 1 second will be cleared.
  const auto cutoff_time = add(imu_msg->header.stamp, -std::chrono::seconds(1));

  while (!angular_velocity_queue_.empty()) {
    if (is_earlier(cutoff_time, angular_velocity_queue_.front().header.stamp)) {
      break;
    }
    angular_velocity_queue_.pop_front();
  }

  angular_velocity_queue_.push_back(transformed_angular_velocity);
}

void DistortionCorrectorBase::collect_motion_samples(
  const builtin_interfaces::msg::Time & pointcloud_stamp, bool use_imu)
{
  const std::int64_t pointcloud_stamp_ns = to_nanoseconds(pointcloud_stamp);

  // No point precedes the header stamp, and a time takes its velocity from the first sample at or
  // after it, so older samples have no effect. The newest sample is kept in any case.
  const auto first_relevant = [pointcloud_stamp_ns](const auto & queue) {
    auto it = std::lower_bound(
      std::begin(queue), std::end(queue), pointcloud_stamp_ns,
      [](const auto & sample, const std::int64_t stamp_ns) {
        return to_nanoseconds(sample.header.stamp) < stamp_ns;
      });
    return it == std::end(queue) && !queue.empty() ? std::end(queue) - 1 : it;
  };

  twist_samples_.clear();
  for (auto it = first_relevant(twist_queue_); it != std::end(twist_queue_); ++it) {
    const auto & linear = it->twist.linear;
    const auto & angular = it->twist.angular;
    twist_samples_.push_back(
      {to_nanoseconds(it->header.stamp) - pointcloud_stamp_ns,
       Eigen::Vector3d(linear.x, linear.y, linear.z).cast<float>(),
       Eigen::Vector3d(angular.x, angular.y, angular.z).cast<float>()});
  }

  angular_velocity_samples_.clear();
  if (!use_imu) return;
  for (auto it = first_relevant(angular_velocity_queue_); it != std::end(angular_velocity_queue_);
       ++it) {
    angular_velocity_samples_.push_back(
      {to_nanoseconds(it->header.stamp) - pointcloud_stamp_ns,
       Eigen::Vector3d(it->vector.x, it->vector.y, it->vector.z).cast<float>()});
  }
}

void DistortionCorrectorBase::set_pointcloud_transform(
  const geometry_msgs::msg::TransformStamped & lidar_to_base_link)
{
  lidar_to_base_link_ = to_eigen_matrix(lidar_to_base_link.transform);
  base_link_to_lidar_ = lidar_to_base_link_.inverse();
  pointcloud_transform_needed_ =
    lidar_to_base_link.header.frame_id != lidar_to_base_link.child_frame_id;
}

PointcloudValidity DistortionCorrectorBase::check_pointcloud_validity(
  sensor_msgs::msg::PointCloud2 & pointcloud)
{
  if (pointcloud.data.empty()) {
    return PointcloudValidity::kEmpty;
  }

  auto time_stamp_field_it = std::find_if(
    std::cbegin(pointcloud.fields), std::cend(pointcloud.fields),
    [](const sensor_msgs::msg::PointField & field) { return field.name == "time_stamp"; });
  if (time_stamp_field_it == pointcloud.fields.cend()) {
    return PointcloudValidity::kMissingTimeStampField;
  }

  if (!utils::is_data_layout_compatible_with_point_xyzircaedt(pointcloud)) {
    return PointcloudValidity::kIncompatibleLayout;
  }

  return PointcloudValidity::kValid;
}

tl::expected<AngleConversion, std::string> DistortionCorrectorBase::try_compute_angle_conversion(
  sensor_msgs::msg::PointCloud2 & pointcloud)
{
  // This function tries to compute the angle conversion from Cartesian coordinates to LiDAR azimuth
  // coordinates system

  if (check_pointcloud_validity(pointcloud) != PointcloudValidity::kValid) {
    return tl::make_unexpected(
      "Input pointcloud is invalid. Could not calculate the angle conversion.");
  }

  AngleConversion angle_conversion;

  sensor_msgs::PointCloud2Iterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_azimuth(pointcloud, "azimuth");

  auto next_it_x = it_x;
  auto next_it_y = it_y;
  auto next_it_azimuth = it_azimuth;

  if (it_x != it_x.end() && it_x + 1 != it_x.end()) {
    next_it_x = it_x + 1;
    next_it_y = it_y + 1;
    next_it_azimuth = it_azimuth + 1;
  } else {
    return tl::make_unexpected(
      "Current point cloud only has a single point. Could not calculate the angle conversion.");
  }

  for (; next_it_x != it_x.end();
       ++it_x, ++it_y, ++it_azimuth, ++next_it_x, ++next_it_y, ++next_it_azimuth) {
    auto current_cartesian_rad = autoware_utils::opencv_fast_atan2(*it_y, *it_x);
    auto next_cartesian_rad = autoware_utils::opencv_fast_atan2(*next_it_y, *next_it_x);

    // If the angle exceeds 180 degrees, it may cross the 0-degree axis,
    // which could disrupt the calculation of the formula.
    if (
      std::abs(*next_it_azimuth - *it_azimuth) == 0 ||
      std::abs(next_cartesian_rad - current_cartesian_rad) == 0) {
      // Angle between two points is 0 degrees. Iterate to next point.
      continue;
    }

    // restrict the angle difference between [-180, 180] (degrees)
    float azimuth_diff = std::abs(*next_it_azimuth - *it_azimuth) > autoware_utils::pi
                           ? std::abs(*next_it_azimuth - *it_azimuth) - 2 * autoware_utils::pi
                           : *next_it_azimuth - *it_azimuth;
    float cartesian_rad_diff =
      std::abs(next_cartesian_rad - current_cartesian_rad) > autoware_utils::pi
        ? std::abs(next_cartesian_rad - current_cartesian_rad) - 2 * autoware_utils::pi
        : next_cartesian_rad - current_cartesian_rad;

    float sign = azimuth_diff / cartesian_rad_diff;

    // Check if 'sign' can be adjusted to 1 or -1
    if (std::abs(sign - 1.0f) <= angle_conversion.sign_threshold) {
      angle_conversion.sign = 1.0f;
    } else if (std::abs(sign + 1.0f) <= angle_conversion.sign_threshold) {
      angle_conversion.sign = -1.0f;
    } else {
      // Value of sign is not close to 1 or -1. Iterate to next point.
      continue;
    }

    float offset_rad = *it_azimuth - sign * current_cartesian_rad;
    // Check if 'offset_rad' can be adjusted to offset_rad multiple of π/2
    int multiple_of_90_degrees = std::round(offset_rad / (autoware_utils::pi / 2));
    if (
      std::abs(offset_rad - multiple_of_90_degrees * (autoware_utils::pi / 2)) >
      angle_conversion.offset_rad_threshold) {
      // Value of offset_rad is not close to a multiple of 90 degrees. Iterate to next point.
      continue;
    }

    // Limit the range of offset_rad in [0, 360)
    multiple_of_90_degrees = (multiple_of_90_degrees % 4 + 4) % 4;

    angle_conversion.offset_rad = multiple_of_90_degrees * (autoware_utils::pi / 2);

    return angle_conversion;
  }
  return tl::make_unexpected(
    "No pair of consecutive points matches a valid angle conversion. Could not calculate the angle "
    "conversion.");
}

template <class IntegratorT>
UndistortionResult DistortionCorrector<IntegratorT>::undistort_pointcloud(
  bool use_imu, std::optional<AngleConversion> angle_conversion_opt,
  const geometry_msgs::msg::TransformStamped & lidar_to_base_link,
  sensor_msgs::msg::PointCloud2 & pointcloud)
{
  UndistortionResult result;

  timestamp_mismatch_count_ = 0;
  timestamp_mismatch_fraction_ = 0.0;

  result.validity = check_pointcloud_validity(pointcloud);
  if (result.validity != PointcloudValidity::kValid) return result;
  if (twist_queue_.empty()) {
    result.twist_queue_empty = true;
    return result;
  }

  set_pointcloud_transform(lidar_to_base_link);
  if (angle_conversion_opt.has_value() && !pointcloud_transform_needed_) {
    throw std::runtime_error(
      "The pointcloud is not in the sensor's frame and thus azimuth and distance cannot be "
      "updated. "
      "Please change the input pointcloud or set update_azimuth_and_distance to false.");
  }

  collect_motion_samples(pointcloud.header.stamp, use_imu);
  motion_table::build<IntegratorT>(
    twist_samples_, angular_velocity_samples_, use_imu, motion_bins_);

  const auto * const bins = motion_bins_.data();
  const auto last_bin_index = static_cast<std::uint32_t>(motion_bins_.size() - 1);
  const Transform3f lidar_to_base(lidar_to_base_link_);
  const Transform3f base_to_lidar(base_link_to_lidar_);
  const bool transform_needed = pointcloud_transform_needed_;

  const std::size_t point_step = pointcloud.point_step;
  const std::size_t num_points = std::min(
    static_cast<std::size_t>(pointcloud.width) * pointcloud.height,
    pointcloud.data.size() / point_step);
  std::uint8_t * const data = pointcloud.data.data();

  std::uint32_t flags = 0;
  std::uint32_t mismatch_count = 0;

  for (std::size_t i = 0; i < num_points; ++i) {
    auto & point =
      *reinterpret_cast<autoware::point_types::PointXYZIRCAEDT *>(data + i * point_step);

    float remainder = 0.f;
    const auto & bin = motion_table::lookup(bins, last_bin_index, point.time_stamp, remainder);
    flags |= bin.flags;
    mismatch_count += (bin.flags / motion_table::flag_mismatch) & 1U;

    float x = point.x;
    float y = point.y;
    float z = point.z;
    if (transform_needed) lidar_to_base.apply(x, y, z);
    undistort_point(bin, remainder, x, y, z);
    if (transform_needed) base_to_lidar.apply(x, y, z);
    point.x = x;
    point.y = y;
    point.z = z;

    if (angle_conversion_opt.has_value()) {
      float updated_azimuth = angle_conversion_opt->offset_rad +
                              angle_conversion_opt->sign * autoware_utils::opencv_fast_atan2(y, x);
      if (updated_azimuth < 0) {
        updated_azimuth += autoware_utils::pi * 2;
      } else if (updated_azimuth > 2 * autoware_utils::pi) {
        updated_azimuth -= autoware_utils::pi * 2;
      }

      point.azimuth = updated_azimuth;
      point.distance = std::sqrt(x * x + y * y + z * z);
    }
  }

  timestamp_mismatch_count_ = static_cast<int>(mismatch_count);
  timestamp_mismatch_fraction_ =
    num_points > 0 ? static_cast<float>(mismatch_count) / static_cast<float>(num_points) : 0.0f;

  result.twist_timestamp_too_late = (flags & motion_table::flag_twist_missing) != 0U;
  result.imu_timestamp_too_late = (flags & motion_table::flag_imu_missing) != 0U;
  return result;
}

template class DistortionCorrector<motion_table::Integrator2D>;
template class DistortionCorrector<motion_table::Integrator3D>;

}  // namespace autoware::pointcloud_preprocessor
