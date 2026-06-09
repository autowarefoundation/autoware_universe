// Copyright 2020 TIER IV, Inc.
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

#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/tracker/trackers/vehicle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>

namespace autoware::multi_object_tracker
{

namespace
{

types::DynamicObject normalizeYaw(const types::DynamicObject & object, const double reference_yaw)
{
  types::DynamicObject corrected = object;
  const double obs_yaw = tf2::getYaw(corrected.pose.orientation);
  const double yaw_diff = autoware_utils_math::normalize_radian(obs_yaw - reference_yaw);
  if (std::abs(yaw_diff) > M_PI_2) {
    tf2::Quaternion q;
    q.setRPY(0, 0, obs_yaw + M_PI);
    corrected.pose.orientation = tf2::toMsg(q);
    for (auto & pt : corrected.shape.footprint.points) {
      pt.x = -pt.x;
      pt.y = -pt.y;
    }
  }
  return corrected;
}

}  // namespace

VehicleTracker::VehicleTracker(
  const object_model::ObjectModel & object_model, const rclcpp::Time & time,
  const types::DynamicObject & object)
: Tracker(time, object),
  logger_(rclcpp::get_logger("VehicleTracker")),
  object_model_(object_model),
  shape_model_(object_model),
  shape_update_anchor_(BicycleMotionModel::LengthUpdateAnchor::CENTER)
{
  // set tracker type based on object model
  switch (object_model.type) {
    case object_model::ObjectModelType::GeneralVehicle:
      tracker_type_ = TrackerType::GENERAL_VEHICLE;
      break;
    case object_model::ObjectModelType::NormalVehicle:
      tracker_type_ = TrackerType::NORMAL_VEHICLE;
      break;
    case object_model::ObjectModelType::BigVehicle:
      tracker_type_ = TrackerType::BIG_VEHICLE;
      break;
    case object_model::ObjectModelType::Bicycle:
      tracker_type_ = TrackerType::BICYCLE;
      break;
    default:
      RCLCPP_ERROR(
        logger_, "VehicleTracker: Unsupported object model type: %d",
        static_cast<int>(object_model.type));
      break;
  }

  // velocity deviation threshold
  velocity_deviation_threshold_ = autoware_utils_math::kmph2mps(10);  // [m/s]

  // Initialize shape manager (forces BBOX, clears footprint, clamps)
  shape_model_.init(object);

  // Seed footprint before the motion model is ready — no tracker pose yet, direct copy
  shape_model_.updateFootprint(object, time);

  // Determine initial vehicle length for motion model initialization
  const double initial_length =
    (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX &&
     object.shape.dimensions.x > 0.0)
      ? std::clamp(
          object.shape.dimensions.x, object_model_.size_limit.length_min,
          object_model_.size_limit.length_max)
      : object_model_.init_size.length;

  // Set motion model parameters
  motion_model_.setMotionParams(
    object_model_.process_noise, object_model_.bicycle_state, object_model_.process_limit);

  // Set initial state
  {
    using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    const double x = object.pose.position.x;
    const double y = object.pose.position.y;
    const double yaw = tf2::getYaw(object.pose.orientation);

    auto pose_cov = object.pose_covariance;
    if (!object.kinematics.has_position_covariance) {
      const auto & p0_cov_x = object_model_.initial_covariance.pos_x;
      const auto & p0_cov_y = object_model_.initial_covariance.pos_y;
      const auto & p0_cov_yaw = object_model_.initial_covariance.yaw;

      const double cos_yaw = std::cos(yaw);
      const double sin_yaw = std::sin(yaw);
      const double sin_2yaw = std::sin(2.0 * yaw);
      pose_cov[XYZRPY_COV_IDX::X_X] = p0_cov_x * cos_yaw * cos_yaw + p0_cov_y * sin_yaw * sin_yaw;
      pose_cov[XYZRPY_COV_IDX::X_Y] = 0.5 * (p0_cov_x - p0_cov_y) * sin_2yaw;
      pose_cov[XYZRPY_COV_IDX::Y_Y] = p0_cov_x * sin_yaw * sin_yaw + p0_cov_y * cos_yaw * cos_yaw;
      pose_cov[XYZRPY_COV_IDX::Y_X] = pose_cov[XYZRPY_COV_IDX::X_Y];
      pose_cov[XYZRPY_COV_IDX::YAW_YAW] = p0_cov_yaw;
    }

    double vel_x = 0.0;
    double vel_y = 0.0;
    double vel_x_cov = object_model_.initial_covariance.vel_long;
    double vel_y_cov = object_model_.bicycle_state.init_slip_angle_cov;
    if (object.kinematics.has_twist) {
      vel_x = object.twist.linear.x;
      vel_y = object.twist.linear.y;
    }
    if (object.kinematics.has_twist_covariance) {
      vel_x_cov = object.twist_covariance[XYZRPY_COV_IDX::X_X];
      vel_y_cov = object.twist_covariance[XYZRPY_COV_IDX::Y_Y];
    }

    motion_model_.initialize(
      time, x, y, yaw, pose_cov, vel_x, vel_x_cov, vel_y, vel_y_cov, initial_length);
  }
}

bool VehicleTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

bool VehicleTracker::updateKinematics(
  const types::DynamicObject & object, const types::InputChannel & channel_info)
{
  // Use measurement length only when the channel and shape are trustworthy.
  const bool is_bbox = (object.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  const bool can_update_shape = channel_info.trust_extension && is_bbox;
  constexpr double min_length = 1.0;
  const double length = can_update_shape ? std::max(object.shape.dimensions.x, min_length)
                                         : std::max(motion_model_.getLength(), min_length);

  const bool is_yaw_available =
    object.kinematics.orientation_availability != types::OrientationAvailability::UNAVAILABLE &&
    channel_info.trust_orientation;
  const bool is_velocity_available = object.kinematics.has_twist;

  bool is_updated = false;
  {
    const double & x = object.pose.position.x;
    const double & y = object.pose.position.y;
    const double & yaw = tf2::getYaw(object.pose.orientation);
    const double & vel_x = object.twist.linear.x;
    const double & vel_y = object.twist.linear.y;

    if (is_yaw_available && is_velocity_available) {
      is_updated = motion_model_.updateStatePoseHeadVel(
        x, y, yaw, object.pose_covariance, vel_x, vel_y, object.twist_covariance, length);
    } else if (is_yaw_available && !is_velocity_available) {
      is_updated = motion_model_.updateStatePoseHead(x, y, yaw, object.pose_covariance, length);
    } else if (!is_yaw_available && is_velocity_available) {
      is_updated = motion_model_.updateStatePoseVel(
        x, y, object.pose_covariance, yaw, vel_x, vel_y, object.twist_covariance, length);
    } else {
      is_updated = motion_model_.updateStatePose(x, y, object.pose_covariance, length);
    }
    const double pre_limit_yaw = motion_model_.getYawState();
    motion_model_.limitStates();
    // Flip stored footprint when yaw-limit correction reverses heading by 180°
    if (shape_model_.isFootprintValid()) {
      const double yaw_diff =
        autoware_utils_math::normalize_radian(motion_model_.getYawState() - pre_limit_yaw);
      if (std::abs(yaw_diff) > M_PI_2) {
        shape_model_.flipFootprintXY();
      }
    }
  }

  // Low-pass filter on z position (2D motion model does not track z).
  {
    constexpr double gain = 0.1;
    object_.pose.position.z =
      (1.0 - gain) * object_.pose.position.z + gain * object.pose.position.z;
  }

  return is_updated;
}

bool VehicleTracker::updateWheelKinematics(
  const UpdateStrategy & strategy, const types::DynamicObject & measurement)
{
  std::array<double, 36> pose_cov = measurement.pose_covariance;
  bool is_updated = false;
  if (strategy.type == UpdateStrategyType::FRONT_WHEEL_UPDATE) {
    shape_update_anchor_ = BicycleMotionModel::LengthUpdateAnchor::FRONT;
    is_updated = motion_model_.updateStatePoseFront(
      strategy.anchor_point.x, strategy.anchor_point.y, pose_cov);
  } else {
    shape_update_anchor_ = BicycleMotionModel::LengthUpdateAnchor::REAR;
    is_updated =
      motion_model_.updateStatePoseRear(strategy.anchor_point.x, strategy.anchor_point.y, pose_cov);
  }
  // Wheel-anchor EKF only updates x/y; z position and height are applied here.
  constexpr double z_gain = 0.4;
  object_.pose.position.z =
    (1.0 - z_gain) * object_.pose.position.z + z_gain * measurement.pose.position.z;
  shape_model_.updateHeight(measurement.shape.dimensions.z);
  return is_updated;
}

bool VehicleTracker::measure(
  const types::DynamicObject & in_object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "VehicleTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  const types::DynamicObject corrected = normalizeYaw(in_object, motion_model_.getYawState());

  const bool is_bbox = (corrected.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  updateKinematics(corrected, channel_info);
  if (channel_info.trust_extension && is_bbox) {
    shape_model_.updateShape(corrected);
  }

  // Get current tracker pose for footprint transform
  geometry_msgs::msg::Pose tracker_pose;
  std::array<double, 36> dummy_cov{};
  geometry_msgs::msg::Twist dummy_twist;
  const bool has_pose =
    motion_model_.getPredictedState(time, tracker_pose, dummy_cov, dummy_twist, dummy_cov);
  shape_model_.updateFootprint(
    corrected, time, has_pose ? std::make_optional(tracker_pose) : std::nullopt);

  shape_update_anchor_ = BicycleMotionModel::LengthUpdateAnchor::CENTER;
  removeCache();
  return true;
}

bool VehicleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  if (!getCachedObject(time, object)) {
    object = object_;
    object.time = time;

    auto & pose = object.pose;
    auto & pose_cov = object.pose_covariance;
    auto & twist = object.twist;
    auto & twist_cov = object.twist_covariance;
    if (!motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov)) {
      RCLCPP_WARN(logger_, "VehicleTracker::getTrackedObject: Failed to get predicted state.");
      return false;
    }

    updateCache(object, time);
  }
  // Compose bbox dimensions and stored footprint into the output object.
  shape_model_.exportTo(object, motion_model_.getLength());

  if (to_publish) {
    using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    auto & twist = object.twist;
    constexpr double vel_cov_buffer = 0.7;
    constexpr double vel_too_low_ignore = 0.25;
    const double vel_long = std::abs(twist.linear.x);
    if (vel_long > vel_too_low_ignore) {
      const double vel_limit =
        std::max(std::sqrt(object.twist_covariance[XYZRPY_COV_IDX::X_X]) - vel_cov_buffer, 0.0);

      if (vel_long < vel_limit) {
        twist.linear.x = twist.linear.x > 0 ? vel_too_low_ignore : -vel_too_low_ignore;
      } else {
        double vel_suppressed = vel_long - vel_limit;
        vel_suppressed = std::max(vel_suppressed, vel_too_low_ignore);
        twist.linear.x = twist.linear.x > 0 ? vel_suppressed : -vel_suppressed;
      }
    }
  }

  return true;
}

bool VehicleTracker::conditionedUpdate(
  const types::DynamicObject & measurement, const types::DynamicObject & prediction,
  const autoware_perception_msgs::msg::Shape & tracker_shape, const rclcpp::Time & measurement_time,
  const types::InputChannel & channel_info)
{
  const auto aligned = shapes::alignClusterToOrientation(measurement, motion_model_.getYawState());
  const types::DynamicObject & meas = aligned ? *aligned : measurement;

  UpdateStrategy strategy = determineUpdateStrategy(meas, prediction);

  if (strategy.type == UpdateStrategyType::WEAK_UPDATE) {
    types::DynamicObject pseudo_measurement = prediction;
    createPseudoMeasurement(measurement, pseudo_measurement, tracker_shape, true);

    const types::DynamicObject pseudo_corrected =
      normalizeYaw(pseudo_measurement, motion_model_.getYawState());
    updateKinematics(pseudo_corrected, channel_info);

    // Only update height from real measurement (z-span of polygon cluster is reliable)
    shape_model_.updateHeight(measurement.shape.dimensions.z);

    // Store footprint from the real measurement using the post-update tracker pose
    geometry_msgs::msg::Pose tracker_pose;
    std::array<double, 36> dummy_cov{};
    geometry_msgs::msg::Twist dummy_twist;
    const bool has_pose = motion_model_.getPredictedState(
      measurement_time, tracker_pose, dummy_cov, dummy_twist, dummy_cov);
    shape_model_.updateFootprint(
      measurement, measurement_time, has_pose ? std::make_optional(tracker_pose) : std::nullopt);

    shape_update_anchor_ = BicycleMotionModel::LengthUpdateAnchor::CENTER;
    removeCache();
    return true;
  }

  const bool is_updated = updateWheelKinematics(strategy, measurement);

  geometry_msgs::msg::Pose tracker_pose;
  std::array<double, 36> dummy_cov{};
  geometry_msgs::msg::Twist dummy_twist;
  const bool has_pose = motion_model_.getPredictedState(
    measurement_time, tracker_pose, dummy_cov, dummy_twist, dummy_cov);
  shape_model_.updateFootprint(
    measurement, measurement_time, has_pose ? std::make_optional(tracker_pose) : std::nullopt);

  removeCache();
  return is_updated;
}

void VehicleTracker::setObjectShape(const autoware_perception_msgs::msg::Shape & shape)
{
  const auto new_len = shape_model_.setShape(shape, getLatestMeasurementTime());
  if (new_len) {
    motion_model_.updateStateLength(*new_len, shape_update_anchor_);
  }
  shape_update_anchor_ = BicycleMotionModel::LengthUpdateAnchor::CENTER;
}

void VehicleTracker::mergeFootprintFrom(
  const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose)
{
  shape_model_.mergeFrom(footprint, src_pose, object_.pose, getLatestMeasurementTime());
}

}  // namespace autoware::multi_object_tracker
