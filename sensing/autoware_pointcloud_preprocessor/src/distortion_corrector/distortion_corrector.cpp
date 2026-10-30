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

#include "autoware/pointcloud_preprocessor/utility/memory.hpp"
#include "autoware_utils/math/constants.hpp"

#include <Eigen/Geometry>
#include <autoware_utils/math/trigonometry.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

namespace autoware::pointcloud_preprocessor
{
namespace
{
constexpr int64_t kNanosecondsPerSecond = 1'000'000'000LL;

template <typename Stamp>
int64_t to_nanoseconds(const Stamp & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond + stamp.nanosec;
}

template <typename Stamp>
double to_seconds(const Stamp & stamp)
{
  return static_cast<double>(to_nanoseconds(stamp)) * 1e-9;
}

tf2::Transform to_tf2_transform(const geometry_msgs::msg::Transform & transform)
{
  const tf2::Vector3 origin(
    transform.translation.x, transform.translation.y, transform.translation.z);
  const tf2::Quaternion rotation(
    transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w);

  tf2::Transform out;
  out.setOrigin(origin);
  out.setRotation(rotation);
  return out;
}

Eigen::Matrix4f to_eigen_matrix(const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Translation3d translation(
    transform.translation.x, transform.translation.y, transform.translation.z);
  const Eigen::Quaterniond rotation(
    transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z);
  const Eigen::Isometry3d isometry = translation * rotation;
  return isometry.matrix().cast<float>();
}

template <class Queue>
void prune_before_latest_before_stamp(Queue & queue, const int64_t stamp_nanoseconds)
{
  if (queue.size() < 2) {
    return;
  }

  auto keep_it = std::lower_bound(
    std::begin(queue), std::end(queue), stamp_nanoseconds,
    [](const auto & sample, const int64_t stamp) {
      return to_nanoseconds(sample.header.stamp) < stamp;
    });

  if (keep_it == std::end(queue)) {
    keep_it = std::prev(std::end(queue));
  } else if (to_nanoseconds(keep_it->header.stamp) > stamp_nanoseconds) {
    if (keep_it == std::begin(queue)) {
      return;
    }
    --keep_it;
  }

  queue.erase(std::begin(queue), keep_it);
}
}  // namespace

bool DistortionCorrectorBase::pointcloud_transform_exists() const
{
  return pointcloud_transform_exists_;
}

bool DistortionCorrectorBase::pointcloud_transform_needed() const
{
  return pointcloud_transform_needed_;
}

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

  angular_velocity_queue_.push_back(transformed_angular_velocity);
}

void DistortionCorrectorBase::prune_twist_queue(const int64_t first_point_stamp_nanoseconds)
{
  prune_before_latest_before_stamp(twist_queue_, first_point_stamp_nanoseconds);
}

void DistortionCorrectorBase::prune_angular_velocity_queue(
  const int64_t first_point_stamp_nanoseconds)
{
  prune_before_latest_before_stamp(angular_velocity_queue_, first_point_stamp_nanoseconds);
}

void DistortionCorrectorBase::get_twist_and_imu_iterator(
  bool use_imu, double first_point_time_stamp_sec,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu)
{
  it_twist = std::lower_bound(
    std::begin(twist_queue_), std::end(twist_queue_), first_point_time_stamp_sec,
    [](const geometry_msgs::msg::TwistStamped & x, const double t) {
      return to_seconds(x.header.stamp) < t;
    });
  it_twist = it_twist == std::end(twist_queue_) ? std::end(twist_queue_) - 1 : it_twist;

  if (use_imu && !angular_velocity_queue_.empty()) {
    it_imu = std::lower_bound(
      std::begin(angular_velocity_queue_), std::end(angular_velocity_queue_),
      first_point_time_stamp_sec, [](const geometry_msgs::msg::Vector3Stamped & x, const double t) {
        return to_seconds(x.header.stamp) < t;
      });
    it_imu =
      it_imu == std::end(angular_velocity_queue_) ? std::end(angular_velocity_queue_) - 1 : it_imu;
  }
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

std::optional<AngleConversion> DistortionCorrectorBase::try_compute_angle_conversion(
  sensor_msgs::msg::PointCloud2 & pointcloud)
{
  // This function tries to compute the angle conversion from Cartesian coordinates to LiDAR azimuth
  // coordinates system

  if (check_pointcloud_validity(pointcloud) != PointcloudValidity::kValid) return std::nullopt;

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
    // Current point cloud only has a single point; cannot calculate the angle conversion.
    return std::nullopt;
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
  return std::nullopt;
}

template <class T>
UndistortionResult DistortionCorrector<T>::undistort_pointcloud(
  bool use_imu, std::optional<AngleConversion> angle_conversion_opt,
  const geometry_msgs::msg::TransformStamped & lidar_to_base_link,
  sensor_msgs::msg::PointCloud2 & pointcloud)
{
  UndistortionResult result;

  timestamp_mismatch_count_ = 0;
  timestamp_mismatch_fraction_ = 0.0;

  // Reset the per-cloud undistortion state so callers don't need to call initialize() themselves.
  this->initialize();

  result.validity = check_pointcloud_validity(pointcloud);
  if (result.validity != PointcloudValidity::kValid) return result;
  if (twist_queue_.empty()) {
    result.twist_queue_empty = true;
    return result;
  }

  sensor_msgs::PointCloud2Iterator<float> it_x(pointcloud, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(pointcloud, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(pointcloud, "z");
  sensor_msgs::PointCloud2Iterator<float> it_azimuth(pointcloud, "azimuth");
  sensor_msgs::PointCloud2Iterator<float> it_distance(pointcloud, "distance");
  sensor_msgs::PointCloud2ConstIterator<std::uint32_t> it_time_stamp(pointcloud, "time_stamp");

  const int64_t pointcloud_stamp_nanoseconds = to_nanoseconds(pointcloud.header.stamp);
  const int64_t first_point_stamp_nanoseconds = pointcloud_stamp_nanoseconds + *it_time_stamp;
  prune_twist_queue(first_point_stamp_nanoseconds);
  if (use_imu) {
    prune_angular_velocity_queue(first_point_stamp_nanoseconds);
  }

  this->set_pointcloud_transform(lidar_to_base_link);

  double prev_time_stamp_sec{
    pointcloud.header.stamp.sec + 1e-9 * (pointcloud.header.stamp.nanosec + *it_time_stamp)};
  const double first_point_time_stamp_sec{
    pointcloud.header.stamp.sec + 1e-9 * (pointcloud.header.stamp.nanosec + *it_time_stamp)};

  std::deque<geometry_msgs::msg::TwistStamped>::iterator it_twist;
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator it_imu;
  get_twist_and_imu_iterator(use_imu, first_point_time_stamp_sec, it_twist, it_imu);

  // For performance, do not recompute the stamp seconds inside of the for-loop
  double twist_stamp = to_seconds(it_twist->header.stamp);
  double imu_stamp{0.0};
  if (use_imu && !angular_velocity_queue_.empty()) {
    imu_stamp = to_seconds(it_imu->header.stamp);
  }

  // If there is a point in a pointcloud that cannot be associated, record it to issue a warning
  bool is_twist_time_stamp_too_late = false;
  bool is_imu_time_stamp_too_late = false;
  constexpr double time_diff = 0.1;

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_time_stamp) {
    bool is_twist_valid = true;
    bool is_imu_valid = true;

    const double current_point_stamp =
      pointcloud.header.stamp.sec + 1e-9 * (pointcloud.header.stamp.nanosec + *it_time_stamp);

    // Get closest twist information
    while (it_twist != std::end(twist_queue_) - 1 && current_point_stamp > twist_stamp) {
      ++it_twist;
      twist_stamp = to_seconds(it_twist->header.stamp);
    }
    if (std::abs(current_point_stamp - twist_stamp) > time_diff) {
      is_twist_time_stamp_too_late = true;
      is_twist_valid = false;
    }

    // Get closest IMU information
    if (use_imu && !angular_velocity_queue_.empty()) {
      while (it_imu != std::end(angular_velocity_queue_) - 1 && current_point_stamp > imu_stamp) {
        ++it_imu;
        imu_stamp = to_seconds(it_imu->header.stamp);
      }

      if (std::abs(current_point_stamp - imu_stamp) > time_diff) {
        is_imu_time_stamp_too_late = true;
        is_imu_valid = false;
      }
    } else {
      is_imu_valid = false;
    }

    if (!is_twist_valid || (use_imu && !is_imu_valid)) ++timestamp_mismatch_count_;

    auto time_offset = static_cast<float>(current_point_stamp - prev_time_stamp_sec);

    // Undistort a single point based on the strategy
    undistort_point(it_x, it_y, it_z, it_twist, it_imu, time_offset, is_twist_valid, is_imu_valid);

    if (angle_conversion_opt.has_value()) {
      if (!pointcloud_transform_needed_) {
        throw std::runtime_error(
          "The pointcloud is not in the sensor's frame and thus azimuth and distance cannot be "
          "updated. "
          "Please change the input pointcloud or set update_azimuth_and_distance to false.");
      }
      float cartesian_coordinate_azimuth = autoware_utils::opencv_fast_atan2(*it_y, *it_x);
      float updated_azimuth = angle_conversion_opt->offset_rad +
                              angle_conversion_opt->sign * cartesian_coordinate_azimuth;
      if (updated_azimuth < 0) {
        updated_azimuth += autoware_utils::pi * 2;
      } else if (updated_azimuth > 2 * autoware_utils::pi) {
        updated_azimuth -= autoware_utils::pi * 2;
      }

      *it_azimuth = updated_azimuth;
      *it_distance = sqrt(*it_x * *it_x + *it_y * *it_y + *it_z * *it_z);

      ++it_azimuth;
      ++it_distance;
    }

    prev_time_stamp_sec = current_point_stamp;
  }

  const auto total_points = pointcloud.width * pointcloud.height;
  timestamp_mismatch_fraction_ = total_points > 0 ? static_cast<float>(timestamp_mismatch_count_) /
                                                      static_cast<float>(total_points)
                                                  : 0.0f;

  result.twist_timestamp_too_late = is_twist_time_stamp_too_late;
  result.imu_timestamp_too_late = is_imu_time_stamp_too_late;
  return result;
}

///////////////////////// Functions for different undistortion strategies /////////////////////////

void DistortionCorrector2D::initialize()
{
  x_ = 0.0f;
  y_ = 0.0f;
  theta_ = 0.0f;
}

void DistortionCorrector3D::initialize()
{
  prev_transformation_matrix_ = Eigen::Matrix4f::Identity();
}

void DistortionCorrector2D::set_pointcloud_transform(
  const geometry_msgs::msg::TransformStamped & lidar_to_base_link)
{
  tf2_lidar_to_base_link_ = to_tf2_transform(lidar_to_base_link.transform);
  tf2_base_link_to_lidar_ = tf2_lidar_to_base_link_.inverse();
  pointcloud_transform_exists_ = true;
  pointcloud_transform_needed_ =
    lidar_to_base_link.header.frame_id != lidar_to_base_link.child_frame_id;
}

void DistortionCorrector3D::set_pointcloud_transform(
  const geometry_msgs::msg::TransformStamped & lidar_to_base_link)
{
  eigen_lidar_to_base_link_ = to_eigen_matrix(lidar_to_base_link.transform);
  eigen_base_link_to_lidar_ = eigen_lidar_to_base_link_.inverse();
  pointcloud_transform_exists_ = true;
  pointcloud_transform_needed_ =
    lidar_to_base_link.header.frame_id != lidar_to_base_link.child_frame_id;
}

inline void DistortionCorrector2D::undistort_point_implementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
  const bool & is_twist_valid, const bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v{0.0f};
  float w{0.0f};
  if (is_twist_valid) {
    v = static_cast<float>(it_twist->twist.linear.x);
    w = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_tf_.setValue(*it_x, *it_y, *it_z);

  if (pointcloud_transform_needed_) {
    point_tf_ = tf2_lidar_to_base_link_ * point_tf_;
  }
  theta_ += w * time_offset;
  auto [sin_half_theta, cos_half_theta] = autoware_utils::sin_and_cos(theta_ * 0.5f);
  auto [sin_theta, cos_theta] = autoware_utils::sin_and_cos(theta_);

  baselink_quat_.setValue(
    0, 0, sin_half_theta, cos_half_theta);  // baselink_quat.setRPY(0.0, 0.0, theta); (Note that the
                                            // value is slightly different)
  const float dis = v * time_offset;
  x_ += dis * cos_theta;
  y_ += dis * sin_theta;

  baselink_tf_odom_.setOrigin(tf2::Vector3(x_, y_, 0.0));
  baselink_tf_odom_.setRotation(baselink_quat_);

  undistorted_point_tf_ = baselink_tf_odom_ * point_tf_;

  if (pointcloud_transform_needed_) {
    undistorted_point_tf_ = tf2_base_link_to_lidar_ * undistorted_point_tf_;
  }

  *it_x = static_cast<float>(undistorted_point_tf_.getX());
  *it_y = static_cast<float>(undistorted_point_tf_.getY());
  *it_z = static_cast<float>(undistorted_point_tf_.getZ());
}

inline void DistortionCorrector3D::undistort_point_implementation(
  sensor_msgs::PointCloud2Iterator<float> & it_x, sensor_msgs::PointCloud2Iterator<float> & it_y,
  sensor_msgs::PointCloud2Iterator<float> & it_z,
  std::deque<geometry_msgs::msg::TwistStamped>::iterator & it_twist,
  std::deque<geometry_msgs::msg::Vector3Stamped>::iterator & it_imu, const float & time_offset,
  const bool & is_twist_valid, const bool & is_imu_valid)
{
  // Initialize linear velocity and angular velocity
  float v_x{0.0f};
  float v_y{0.0f};
  float v_z{0.0f};
  float w_x{0.0f};
  float w_y{0.0f};
  float w_z{0.0f};
  if (is_twist_valid) {
    v_x = static_cast<float>(it_twist->twist.linear.x);
    v_y = static_cast<float>(it_twist->twist.linear.y);
    v_z = static_cast<float>(it_twist->twist.linear.z);
    w_x = static_cast<float>(it_twist->twist.angular.x);
    w_y = static_cast<float>(it_twist->twist.angular.y);
    w_z = static_cast<float>(it_twist->twist.angular.z);
  }
  if (is_imu_valid) {
    w_x = static_cast<float>(it_imu->vector.x);
    w_y = static_cast<float>(it_imu->vector.y);
    w_z = static_cast<float>(it_imu->vector.z);
  }

  // Undistort point
  point_eigen_ << *it_x, *it_y, *it_z, 1.0;
  if (pointcloud_transform_needed_) {
    point_eigen_ = eigen_lidar_to_base_link_ * point_eigen_;
  }

  Sophus::SE3f::Tangent twist(v_x, v_y, v_z, w_x, w_y, w_z);
  twist = twist * time_offset;
  transformation_matrix_ = Sophus::SE3f::exp(twist).matrix();
  transformation_matrix_ = transformation_matrix_ * prev_transformation_matrix_;
  undistorted_point_eigen_ = transformation_matrix_ * point_eigen_;

  if (pointcloud_transform_needed_) {
    undistorted_point_eigen_ = eigen_base_link_to_lidar_ * undistorted_point_eigen_;
  }
  *it_x = undistorted_point_eigen_[0];
  *it_y = undistorted_point_eigen_[1];
  *it_z = undistorted_point_eigen_[2];

  prev_transformation_matrix_ = transformation_matrix_;
}

template class DistortionCorrector<DistortionCorrector2D>;
template class DistortionCorrector<DistortionCorrector3D>;

}  // namespace autoware::pointcloud_preprocessor
