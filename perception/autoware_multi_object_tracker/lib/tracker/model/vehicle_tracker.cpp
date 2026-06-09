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

#include "autoware/multi_object_tracker/tracker/model/vehicle_tracker.hpp"

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils_geometry/boost_polygon_utils.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <limits>

namespace autoware::multi_object_tracker
{

namespace
{

constexpr double ALIGNMENT_RATIO_THRESHOLD = 0.2;     // 20% of the larger object's length
constexpr double ALIGNMENT_ABSOLUTE_THRESHOLD = 1.0;  // [m] minimum tolerance for small objects

struct EdgePositions
{
  double front_x, front_y;
  double rear_x, rear_y;
};

enum class Edge { FRONT, REAR };

struct EdgeAlignment
{
  double min_alignment_distance;
  Edge aligned_pred_edge;
  Edge aligned_meas_edge;
};

EdgePositions calculateEdgeCenters(const types::DynamicObject & obj)
{
  const double yaw = tf2::getYaw(obj.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double half_length = obj.shape.dimensions.x * 0.5;

  return {
    obj.pose.position.x + half_length * cos_yaw,  // front_x
    obj.pose.position.y + half_length * sin_yaw,  // front_y
    obj.pose.position.x - half_length * cos_yaw,  // rear_x
    obj.pose.position.y - half_length * sin_yaw   // rear_y
  };
}

EdgeAlignment findAlignedEdges(
  const EdgePositions & meas_edges, const types::DynamicObject & prediction)
{
  const double pred_yaw = tf2::getYaw(prediction.pose.orientation);
  const double pred_cos_yaw = std::cos(pred_yaw);
  const double pred_sin_yaw = std::sin(pred_yaw);

  const auto project_to_axis = [pred_cos_yaw, pred_sin_yaw](double x, double y) {
    return x * pred_cos_yaw + y * pred_sin_yaw;
  };

  const double meas_front_axis = project_to_axis(meas_edges.front_x, meas_edges.front_y);
  const double meas_rear_axis = project_to_axis(meas_edges.rear_x, meas_edges.rear_y);

  const double pred_center_axis =
    prediction.pose.position.x * pred_cos_yaw + prediction.pose.position.y * pred_sin_yaw;
  const double predicted_half_length = prediction.shape.dimensions.x * 0.5;
  const double pred_front_axis = pred_center_axis + predicted_half_length;
  const double pred_rear_axis = pred_center_axis - predicted_half_length;

  struct Candidate
  {
    double distance;
    Edge pred_edge;
    Edge meas_edge;
  };
  const std::array<Candidate, 4> candidates = {
    {{std::abs(meas_front_axis - pred_front_axis), Edge::FRONT, Edge::FRONT},
     {std::abs(meas_rear_axis - pred_front_axis), Edge::FRONT, Edge::REAR},
     {std::abs(meas_front_axis - pred_rear_axis), Edge::REAR, Edge::FRONT},
     {std::abs(meas_rear_axis - pred_rear_axis), Edge::REAR, Edge::REAR}}};

  const auto best = std::min_element(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.distance < b.distance; });

  return {best->distance, best->pred_edge, best->meas_edge};
}

geometry_msgs::msg::Point calculateAnchorPoint(
  const EdgeAlignment & alignment, const types::DynamicObject & measurement)
{
  geometry_msgs::msg::Point anchor_point;

  const double meas_yaw = tf2::getYaw(measurement.pose.orientation);
  const double meas_cos_yaw = std::cos(meas_yaw);
  const double meas_sin_yaw = std::sin(meas_yaw);
  const double meas_half_length = measurement.shape.dimensions.x * 0.5;

  if (alignment.aligned_meas_edge == Edge::FRONT) {
    anchor_point.x = measurement.pose.position.x + meas_half_length * meas_cos_yaw;
    anchor_point.y = measurement.pose.position.y + meas_half_length * meas_sin_yaw;
  } else {
    anchor_point.x = measurement.pose.position.x - meas_half_length * meas_cos_yaw;
    anchor_point.y = measurement.pose.position.y - meas_half_length * meas_sin_yaw;
  }

  return anchor_point;
}

UpdateStrategy determineUpdateStrategy(
  const types::DynamicObject & measurement, const types::DynamicObject & prediction)
{
  UpdateStrategy strategy;

  const EdgePositions meas_edges = calculateEdgeCenters(measurement);
  const EdgeAlignment alignment = findAlignedEdges(meas_edges, prediction);

  const double predicted_length = prediction.shape.dimensions.x;
  const double measured_length = measurement.shape.dimensions.x;
  const double max_length = std::max(predicted_length, measured_length);
  const double alignment_threshold =
    std::max(ALIGNMENT_RATIO_THRESHOLD * max_length, ALIGNMENT_ABSOLUTE_THRESHOLD);
  const bool is_edge_aligned = alignment.min_alignment_distance < alignment_threshold;

  if (!is_edge_aligned) {
    strategy.type = UpdateStrategyType::WEAK_UPDATE;
    return strategy;
  }

  strategy.type = (alignment.aligned_pred_edge == Edge::FRONT)
                    ? UpdateStrategyType::FRONT_WHEEL_UPDATE
                    : UpdateStrategyType::REAR_WHEEL_UPDATE;
  strategy.anchor_point = calculateAnchorPoint(alignment, measurement);

  return strategy;
}

}  // namespace

VehicleTracker::VehicleTracker(
  const object_model::ObjectModel & object_model, const rclcpp::Time & time,
  const types::DynamicObject & object)
: Tracker(time, object), logger_(rclcpp::get_logger("VehicleTracker")), object_model_(object_model)
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
  //   if the predicted velocity is close to the observed velocity,
  //   the observed velocity is used as the measurement.
  velocity_deviation_threshold_ = autoware_utils_math::kmph2mps(10);  // [m/s]

  if (object.shape.type != autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    // set default initial size
    auto & object_extension = object_.shape.dimensions;
    object_extension.x = object_model_.init_size.length;
    object_extension.y = object_model_.init_size.width;
    object_extension.z = object_model_.init_size.height;
  }
  object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object_.shape.footprint.points.clear();

  // set maximum and minimum size
  limitObjectExtension(object_model_);

  // Seed footprint before the motion model is ready — getPredictedState is unavailable, so
  // updateFootprint takes the direct-copy path (detection pose == initial tracker pose).
  footprint_valid_ = false;
  last_footprint_update_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  updateFootprint(object, time);

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
      // initial state covariance
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

    const double & length = object_.shape.dimensions.x;

    // initialize motion model
    motion_model_.initialize(time, x, y, yaw, pose_cov, vel_x, vel_x_cov, vel_y, vel_y_cov, length);
  }
}

bool VehicleTracker::predict(const rclcpp::Time & time)
{
  return motion_model_.predictState(time);
}

types::DynamicObject VehicleTracker::normalizeYaw(
  const types::DynamicObject & object, const double reference_yaw) const
{
  types::DynamicObject corrected = object;
  const double obs_yaw = tf2::getYaw(corrected.pose.orientation);
  const double yaw_diff = autoware_utils_math::normalize_radian(obs_yaw - reference_yaw);
  if (std::abs(yaw_diff) > M_PI_2) {
    tf2::Quaternion q;
    q.setRPY(0, 0, obs_yaw + M_PI);
    corrected.pose.orientation = tf2::toMsg(q);
  }
  return corrected;
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

  // Select and run EKF update variant based on available measurement data.
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
    motion_model_.limitStates();
  }

  // Low-pass filter on z position (2D motion model does not track z).
  {
    constexpr double gain = 0.1;
    object_.pose.position.z =
      (1.0 - gain) * object_.pose.position.z + gain * object.pose.position.z;
  }

  return is_updated;
}

void VehicleTracker::updateShapeSize(const types::DynamicObject & object, const bool can_update)
{
  if (!can_update) return;

  // Reject measurements whose size is implausibly large or small.
  constexpr double size_max_multiplier = 1.5;
  constexpr double size_min_multiplier = 0.25;
  if (
    object.shape.dimensions.x > object_model_.size_limit.length_max * size_max_multiplier ||
    object.shape.dimensions.x < object_model_.size_limit.length_min * size_min_multiplier ||
    object.shape.dimensions.y > object_model_.size_limit.width_max * size_max_multiplier ||
    object.shape.dimensions.y < object_model_.size_limit.width_min * size_min_multiplier) {
    return;
  }

  // IIR(Infinite Impulse Response)-blend width and height; length is authoritative from the motion
  // model.
  {
    constexpr double gain = 0.4;
    constexpr double gain_inv = 1.0 - gain;
    auto & ext = object_.shape.dimensions;
    ext.x = motion_model_.getLength();
    ext.y = gain_inv * ext.y + gain * object.shape.dimensions.y;
    ext.z = gain_inv * ext.z + gain * object.shape.dimensions.z;
  }

  limitObjectExtension(object_model_);
  object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object_.area = types::getArea(object_.shape);
}

bool VehicleTracker::updateWheelKinematics(
  const UpdateStrategy & strategy, const types::DynamicObject & measurement)
{
  std::array<double, 36> pose_cov = measurement.pose_covariance;
  bool is_updated = false;
  if (strategy.type == UpdateStrategyType::FRONT_WHEEL_UPDATE) {
    is_updated = motion_model_.updateStatePoseFront(
      strategy.anchor_point.x, strategy.anchor_point.y, pose_cov);
  } else {
    is_updated =
      motion_model_.updateStatePoseRear(strategy.anchor_point.x, strategy.anchor_point.y, pose_cov);
  }
  // Wheel-anchor EKF only updates x/y; z position and height are applied here.
  constexpr double z_gain = 0.4;
  object_.pose.position.z =
    (1.0 - z_gain) * object_.pose.position.z + z_gain * measurement.pose.position.z;
  if (measurement.shape.dimensions.z > 0.0) {
    constexpr double height_gain = 0.4;
    object_.shape.dimensions.z = (1.0 - height_gain) * object_.shape.dimensions.z +
                                 height_gain * measurement.shape.dimensions.z;
  }
  return is_updated;
}

bool VehicleTracker::measure(
  const types::DynamicObject & in_object, const rclcpp::Time & time,
  const types::InputChannel & channel_info)
{
  // check time gap
  const double dt = motion_model_.getDeltaTime(time);
  if (0.01 /*10msec*/ < dt) {
    RCLCPP_WARN(
      logger_,
      "VehicleTracker::measure There is a large gap between predicted time and measurement "
      "time. (%f)",
      dt);
  }

  // Flip measurement orientation 180° if it points opposite to the tracker heading.
  const types::DynamicObject corrected = normalizeYaw(in_object, motion_model_.getYawState());

  const bool is_bbox = (corrected.shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX);
  updateKinematics(corrected, channel_info);
  updateShapeSize(corrected, channel_info.trust_extension && is_bbox);
  // Store polygon footprint from the original (pre-flip) detection pose.
  updateFootprint(in_object, time);

  removeCache();
  return true;
}

bool VehicleTracker::getTrackedObject(
  const rclcpp::Time & time, types::DynamicObject & object, const bool to_publish) const
{
  // try to return cached object
  if (!getCachedObject(time, object)) {
    // if there is no cached object, predict and update cache
    object = object_;
    object.time = time;

    // predict from motion model
    auto & pose = object.pose;
    auto & pose_cov = object.pose_covariance;
    auto & twist = object.twist;
    auto & twist_cov = object.twist_covariance;
    if (!motion_model_.getPredictedState(time, pose, pose_cov, twist, twist_cov)) {
      RCLCPP_WARN(logger_, "VehicleTracker::getTrackedObject: Failed to get predicted state.");
      return false;
    }

    // cache object
    updateCache(object, time);
  }
  // Compose bbox dimensions and stored footprint into the output object.
  exportShape(object);

  // if the tracker is to be published, check twist uncertainty
  // in case the twist uncertainty is large, lower the twist value
  if (to_publish) {
    using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    // lower the x twist magnitude 1 sigma smaller
    // if the twist is smaller than 1 sigma, the twist is zeroed
    auto & twist = object.twist;
    constexpr double vel_cov_buffer = 0.7;  // [m/s] buffer not to limit certain twist
    constexpr double vel_too_low_ignore =
      0.25;  // [m/s] if the velocity is lower than this, do not limit
    const double vel_long = std::abs(twist.linear.x);
    if (vel_long > vel_too_low_ignore) {
      const double vel_limit = std::max(
        std::sqrt(object.twist_covariance[XYZRPY_COV_IDX::X_X]) - vel_cov_buffer, 0.0);  // [m/s]

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
  // For cluster measurements, re-project the footprint onto the tracker heading before strategy
  // selection. nullopt is returned when there are no footprint points.
  const auto aligned = shapes::alignClusterToOrientation(measurement, motion_model_.getYawState());
  const types::DynamicObject & meas = aligned ? *aligned : measurement;

  // Determine update strategy
  UpdateStrategy strategy = determineUpdateStrategy(meas, prediction);

  // Handle weak update strategy (no edge alignment - use weak update with pseudo measurement)
  if (strategy.type == UpdateStrategyType::WEAK_UPDATE) {
    types::DynamicObject pseudo_measurement = prediction;

    // Create pseudo measurement with enlarged covariance for weak update
    createPseudoMeasurement(measurement, pseudo_measurement, tracker_shape, true);

    // Apply kinematic-only EKF update via the pseudo measurement; align yaw first.
    // Footprint must not be taken from the fake pseudo_measurement.
    const types::DynamicObject pseudo_corrected =
      normalizeYaw(pseudo_measurement, motion_model_.getYawState());
    updateKinematics(pseudo_corrected, channel_info);

    // Update height from real measurement (z span of polygon cluster is reliable).
    // Width is intentionally not updated here — cluster measurements have unreliable bbox width.
    if (measurement.shape.dimensions.z > 0.0) {
      constexpr double gain = 0.4;
      object_.shape.dimensions.z =
        (1.0 - gain) * object_.shape.dimensions.z + gain * measurement.shape.dimensions.z;
    }

    // Store footprint from the real measurement using the post-update tracker pose.
    updateFootprint(measurement, measurement_time);
    removeCache();
    return true;
  }

  // Handle wheel-based update strategies (FRONT_WHEEL_UPDATE or REAR_WHEEL_UPDATE).
  // Width is intentionally not updated here — cluster measurements have unreliable bbox width.
  const bool is_updated = updateWheelKinematics(strategy, measurement);

  // Store measurement footprint after the wheel-anchor kinematic update.
  updateFootprint(measurement, measurement_time);
  removeCache();

  return is_updated;
}

void VehicleTracker::setObjectShape(const autoware_perception_msgs::msg::Shape & shape)
{
  // Update bbox dimensions (kinematic frame authority); internal type is always BOUNDING_BOX
  object_.shape.dimensions = shape.dimensions;
  object_.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;

  // Footprint is stored independently — update only when the new shape carries one; never clear.
  if (!shape.footprint.points.empty()) {
    object_.shape.footprint = shape.footprint;
    footprint_valid_ = true;
    last_footprint_update_time_ = getLatestMeasurementTime();
  }

  object_.area = types::getArea(shape);

  // Only sync the bicycle model length for bounding boxes — POLYGON shapes may carry
  // zero or meaningless dimensions.x.
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    motion_model_.updateStateLength(
      shape.dimensions.x, BicycleMotionModel::LengthUpdateAnchor::CENTER);
  }
}

void VehicleTracker::updateFootprint(const types::DynamicObject & object, const rclcpp::Time & time)
{
  using Shape = autoware_perception_msgs::msg::Shape;
  const bool has_poly =
    !object.shape.footprint.points.empty() &&
    (object.shape.type == Shape::BOUNDING_BOX || object.shape.type == Shape::POLYGON);
  if (!has_poly) return;

  // Get tracker's current pose after kinematic update (dt=0 → EKF state as-is).
  // If the motion model is not yet initialized (constructor path), fall back to direct copy
  // because detection pose == tracker initial pose and no transform is needed.
  geometry_msgs::msg::Pose tracker_pose;
  std::array<double, 36> dummy_cov{};
  geometry_msgs::msg::Twist dummy_twist;
  if (!motion_model_.getPredictedState(time, tracker_pose, dummy_cov, dummy_twist, dummy_cov)) {
    object_.shape.footprint = object.shape.footprint;
    footprint_valid_ = true;
    last_footprint_update_time_ = time;
    return;
  }

  object_.shape.footprint =
    shapes::transformFootprint(object.shape.footprint, object.pose, tracker_pose);
  footprint_valid_ = true;
  last_footprint_update_time_ = time;
}

void VehicleTracker::mergeFootprintFrom(
  const geometry_msgs::msg::Polygon & footprint, const geometry_msgs::msg::Pose & src_pose)
{
  if (footprint.points.empty()) return;
  const auto transformed = shapes::transformFootprint(footprint, src_pose, object_.pose);
  object_.shape.footprint = shapes::unionFootprints(object_.shape.footprint, transformed);
  footprint_valid_ = true;
  last_footprint_update_time_ = getLatestMeasurementTime();
}

void VehicleTracker::exportShape(types::DynamicObject & object) const
{
  object.shape.dimensions.x = motion_model_.getLength();

  const bool footprint_fresh =
    footprint_valid_ && (object.time - last_footprint_update_time_).seconds() < FOOTPRINT_TIMEOUT_S;

  if (footprint_fresh) {
    object.shape.footprint = object_.shape.footprint;
  } else {
    object.shape.footprint.points.clear();
  }
}

}  // namespace autoware::multi_object_tracker
