// Copyright 2022 Tier IV, Inc.
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

#include "autoware/planning_validator/planning_validator.hpp"

#include "autoware/planning_validator/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>

#include <angles/angles/angles.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <utility>

namespace autoware::planning_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

PlanningValidator::PlanningValidator(const rclcpp::NodeOptions & options)
: Node("planning_validator", options)
{
  using std::placeholders::_1;

  sub_traj_ = create_subscription<Trajectory>(
    "~/input/trajectory", 1, std::bind(&PlanningValidator::onTrajectory, this, _1));

  pub_traj_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_status_ = create_publisher<PlanningValidatorStatus>("~/output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);
  pub_processing_time_ms_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  debug_pose_publisher_ = std::make_shared<PlanningValidatorDebugMarkerPublisher>(this);

  setupParameters();

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void PlanningValidator::setupParameters()
{
  auto set_handling_type = [&](auto & type, const std::string & key) {
    const auto value = declare_parameter<int>(key);
    if (value == 0) {
      type = InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS;
    } else if (value == 1) {
      type = InvalidTrajectoryHandlingType::STOP_PUBLISHING;
    } else if (value == 2) {
      type = InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT;
    } else {
      throw std::invalid_argument{
        "unsupported invalid_trajectory_handling_type (" + std::to_string(value) + ")"};
    }
  };

  set_handling_type(params_.inv_traj_handling_type, "handling_type.noncritical");
  set_handling_type(params_.inv_traj_critical_handling_type, "handling_type.critical");

  params_.publish_diag = declare_parameter<bool>("publish_diag");
  params_.diag_error_count_threshold = declare_parameter<int>("diag_error_count_threshold");
  params_.display_on_terminal = declare_parameter<bool>("display_on_terminal");

  {
    auto set_validation_flags = [&](auto & param, const std::string & key) {
      param.enable = declare_parameter<bool>(key + ".enable");
      param.is_critical = declare_parameter<bool>(key + ".is_critical");
    };

    auto set_validation_params = [&](auto & param, const std::string & key) {
      set_validation_flags(param, key);
      param.threshold = declare_parameter<double>(key + ".threshold");
    };

    auto & p = params_.validation_params;
    const std::string t = "validity_checks.";
    set_validation_params(p.interval, t + "interval");
    set_validation_params(p.relative_angle, t + "relative_angle");
    set_validation_params(p.curvature, t + "curvature");
    set_validation_params(p.latency, t + "latency");
    set_validation_params(p.steering, t + "steering");
    set_validation_params(p.steering_rate, t + "steering_rate");

    set_validation_flags(p.acceleration, t + "acceleration");
    p.acceleration.lateral_th = declare_parameter<double>(t + "acceleration.lateral_th");
    p.acceleration.longitudinal_max_th =
      declare_parameter<double>(t + "acceleration.longitudinal_max_th");
    p.acceleration.longitudinal_min_th =
      declare_parameter<double>(t + "acceleration.longitudinal_min_th");

    set_validation_flags(p.deviation, t + "deviation");
    p.deviation.velocity_th = declare_parameter<double>(t + "deviation.velocity_th");
    p.deviation.distance_th = declare_parameter<double>(t + "deviation.distance_th");
    p.deviation.lon_distance_th = declare_parameter<double>(t + "deviation.lon_distance_th");
    p.deviation.yaw_th = declare_parameter<double>(t + "deviation.yaw_th");

    set_validation_flags(p.forward_trajectory_length, t + "forward_trajectory_length");
    p.forward_trajectory_length.acceleration =
      declare_parameter<double>(t + "forward_trajectory_length.acceleration");
    p.forward_trajectory_length.margin =
      declare_parameter<double>(t + "forward_trajectory_length.margin");
  }

  try {
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();
  } catch (...) {
    RCLCPP_ERROR(get_logger(), "failed to get vehicle info. use default value.");
    vehicle_info_.front_overhang_m = 0.5;
    vehicle_info_.wheel_base_m = 4.0;
  }
}

void PlanningValidator::setStatus(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg)
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
  } else if (validation_status_.invalid_count < params_.diag_error_count_threshold) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(validation_status_.invalid_count) + " < " +
                          std::to_string(params_.diag_error_count_threshold) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void PlanningValidator::setupDiag()
{
  diag_updater_ = std::make_shared<Updater>(this);
  auto & d = diag_updater_;
  d->setHardwareID("planning_validator");

  std::string ns = "trajectory_validation_";
  d->add(ns + "size", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_size, "invalid trajectory size is found");
  });
  d->add(ns + "finite", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_finite_value, "infinite value is found");
  });
  d->add(ns + "interval", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_interval, "points interval is too long");
  });
  d->add(ns + "relative_angle", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_relative_angle, "relative angle is too large");
  });
  d->add(ns + "curvature", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_curvature, "curvature is too large");
  });
  d->add(ns + "lateral_acceleration", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_lateral_acc, "lateral acceleration is too large");
  });
  d->add(ns + "acceleration", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_longitudinal_max_acc, "acceleration is too large");
  });
  d->add(ns + "deceleration", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_longitudinal_min_acc, "deceleration is too large");
  });
  d->add(ns + "steering", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_steering, "expected steering is too large");
  });
  d->add(ns + "steering_rate", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_steering_rate, "expected steering rate is too large");
  });
  d->add(ns + "velocity_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_velocity_deviation, "velocity deviation is too large");
  });
  d->add(ns + "distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_distance_deviation, "distance deviation is too large");
  });
  d->add(ns + "longitudinal_distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_longitudinal_distance_deviation,
      "longitudinal distance deviation is too large");
  });
  d->add(ns + "forward_trajectory_length", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_forward_trajectory_length,
      "trajectory length is too short");
  });
  d->add(ns + "latency", [&](auto & stat) {
    setStatus(stat, validation_status_.is_valid_latency, "latency is larger than expected value.");
  });
  d->add(ns + "yaw_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_.is_valid_yaw_deviation,
      "difference between vehicle yaw and closest trajectory yaw is too large.");
  });
}

bool PlanningValidator::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s);
    return false;
  };

  if (!current_kinematics_) {
    return waiting("current_kinematics_");
  }
  if (!current_trajectory_) {
    return waiting("current_trajectory_");
  }
  return true;
}

void PlanningValidator::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  stop_watch_.tic(__func__);

  current_trajectory_ = msg;

  // receive data
  current_kinematics_ = sub_kinematics_.take_data();

  if (!isDataReady()) return;

  if (params_.publish_diag && !diag_updater_) {
    setupDiag();  // run setup after all data is ready.
  }

  debug_pose_publisher_->clearMarkers();

  validate(*current_trajectory_);

  diag_updater_->force_update();

  publishTrajectory();

  // for debug
  publishProcessingTime(stop_watch_.toc(__func__));
  publishDebugInfo();
  displayStatus();
}

void PlanningValidator::publishTrajectory()
{
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(validation_status_)) {
    pub_traj_->publish(*current_trajectory_);
    published_time_publisher_->publish_if_subscribed(pub_traj_, current_trajectory_->header.stamp);
    previous_published_trajectory_ = current_trajectory_;
    return;
  }

  //  ----- invalid factor is found. Publish previous trajectory. -----

  const auto handling_type =
    is_critical_error_ ? params_.inv_traj_critical_handling_type : params_.inv_traj_handling_type;

  if (handling_type == InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS) {
    pub_traj_->publish(*current_trajectory_);
    published_time_publisher_->publish_if_subscribed(pub_traj_, current_trajectory_->header.stamp);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Caution! Invalid Trajectory published.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::STOP_PUBLISHING) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Invalid Trajectory detected. Trajectory is not published.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
    if (previous_published_trajectory_) {
      pub_traj_->publish(*previous_published_trajectory_);
      published_time_publisher_->publish_if_subscribed(
        pub_traj_, previous_published_trajectory_->header.stamp);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 3000, "Invalid Trajectory detected. Use previous trajectory.");
      return;
    }
  }

  // trajectory is not published.
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 3000,
    "Invalid Trajectory detected, no valid trajectory found in the past. Trajectory is not "
    "published.");
  return;
}

void PlanningValidator::publishProcessingTime(const double processing_time_ms)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = processing_time_ms;
  pub_processing_time_ms_->publish(msg);
}

void PlanningValidator::publishDebugInfo()
{
  validation_status_.stamp = get_clock()->now();
  pub_status_->publish(validation_status_);

  if (!isAllValid(validation_status_)) {
    geometry_msgs::msg::Pose front_pose = current_kinematics_->pose.pose;
    shiftPose(front_pose, vehicle_info_.front_overhang_m + vehicle_info_.wheel_base_m);
    debug_pose_publisher_->pushVirtualWall(front_pose);
    debug_pose_publisher_->pushWarningMsg(front_pose, "INVALID PLANNING");
  }
  debug_pose_publisher_->publish();
}

void PlanningValidator::validate(const Trajectory & trajectory)
{
  auto & s = validation_status_;

  const auto terminateValidation = [&](const auto & ss) {
    RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), *get_clock(), 3000, ss);
    s.invalid_count += 1;
  };

  s.is_valid_size = checkValidSize(trajectory);
  if (!s.is_valid_size) {
    return terminateValidation(
      "trajectory has invalid point size (" + std::to_string(trajectory.points.size()) +
      "). Stop validation process, raise an error.");
  }

  s.is_valid_finite_value = checkValidFiniteValue(trajectory);
  if (!s.is_valid_finite_value) {
    return terminateValidation(
      "trajectory has invalid value (NaN, Inf, etc). Stop validation process, raise an error.");
  }

  s.is_valid_interval = checkValidInterval(trajectory);
  s.is_valid_longitudinal_max_acc = checkValidMaxLongitudinalAcceleration(trajectory);
  s.is_valid_longitudinal_min_acc = checkValidMinLongitudinalAcceleration(trajectory);
  s.is_valid_velocity_deviation = checkValidVelocityDeviation(trajectory);
  s.is_valid_distance_deviation = checkValidDistanceDeviation(trajectory);
  s.is_valid_longitudinal_distance_deviation = checkValidLongitudinalDistanceDeviation(trajectory);
  s.is_valid_yaw_deviation = checkValidYawDeviation(trajectory);
  s.is_valid_forward_trajectory_length = checkValidForwardTrajectoryLength(trajectory);
  s.is_valid_latency = checkValidLatency(trajectory);

  // use resampled trajectory because the following metrics can not be evaluated for closed points.
  // Note: do not interpolate to keep original trajectory shape.
  constexpr auto min_interval = 1.0;
  const auto resampled = resampleTrajectory(trajectory, min_interval);

  s.is_valid_relative_angle = checkValidRelativeAngle(resampled);
  s.is_valid_curvature = checkValidCurvature(resampled);
  s.is_valid_lateral_acc = checkValidLateralAcceleration(resampled);
  s.is_valid_steering = checkValidSteering(resampled);
  s.is_valid_steering_rate = checkValidSteeringRate(resampled);

  s.invalid_count = isAllValid(s) ? 0 : s.invalid_count + 1;
}

bool PlanningValidator::checkValidSize(const Trajectory & trajectory)
{
  validation_status_.trajectory_size = trajectory.points.size();
  return trajectory.points.size() >= 2;
}

bool PlanningValidator::checkValidFiniteValue(const Trajectory & trajectory)
{
  for (const auto & p : trajectory.points) {
    if (!checkFinite(p)) return false;
  }
  return true;
}

bool PlanningValidator::checkValidInterval(const Trajectory & trajectory)
{
  if (!params_.validation_params.interval.enable) {
    return true;
  }

  const auto [max_interval_distance, i] = calcMaxIntervalDistance(trajectory);
  validation_status_.max_interval_distance = max_interval_distance;

  if (max_interval_distance > params_.validation_params.interval.threshold) {
    if (i > 0) {
      const auto & p = trajectory.points;
      debug_pose_publisher_->pushPoseMarker(p.at(i - 1), "trajectory_interval");
      debug_pose_publisher_->pushPoseMarker(p.at(i), "trajectory_interval");
    }
    is_critical_error_ |= params_.validation_params.interval.is_critical;
    return false;
  }

  return true;
}

bool PlanningValidator::checkValidRelativeAngle(const Trajectory & trajectory)
{
  if (!params_.validation_params.relative_angle.enable) {
    return true;
  }

  const auto [max_relative_angle, i] = calcMaxRelativeAngles(trajectory);
  validation_status_.max_relative_angle = max_relative_angle;

  if (max_relative_angle > params_.validation_params.relative_angle.threshold) {
    const auto & p = trajectory.points;
    if (i < p.size() - 3) {
      debug_pose_publisher_->pushPoseMarker(p.at(i), "trajectory_relative_angle", 0);
      debug_pose_publisher_->pushPoseMarker(p.at(i + 1), "trajectory_relative_angle", 1);
      debug_pose_publisher_->pushPoseMarker(p.at(i + 2), "trajectory_relative_angle", 2);
    }
    is_critical_error_ |= params_.validation_params.relative_angle.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidCurvature(const Trajectory & trajectory)
{
  if (!params_.validation_params.curvature.enable) {
    return true;
  }

  const auto [max_curvature, i] = calcMaxCurvature(trajectory);
  validation_status_.max_curvature = max_curvature;
  if (max_curvature > params_.validation_params.curvature.threshold) {
    const auto & p = trajectory.points;
    if (i > 0 && i < p.size() - 1) {
      debug_pose_publisher_->pushPoseMarker(p.at(i - 1), "trajectory_curvature");
      debug_pose_publisher_->pushPoseMarker(p.at(i), "trajectory_curvature");
      debug_pose_publisher_->pushPoseMarker(p.at(i + 1), "trajectory_curvature");
    }
    is_critical_error_ |= params_.validation_params.curvature.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidLateralAcceleration(const Trajectory & trajectory)
{
  if (!params_.validation_params.acceleration.enable) {
    return true;
  }

  const auto [max_lateral_acc, i] = calcMaxLateralAcceleration(trajectory);
  validation_status_.max_lateral_acc = max_lateral_acc;
  if (max_lateral_acc > params_.validation_params.acceleration.lateral_th) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i), "lateral_acceleration");
    is_critical_error_ |= params_.validation_params.acceleration.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidMinLongitudinalAcceleration(const Trajectory & trajectory)
{
  if (!params_.validation_params.acceleration.enable) {
    return true;
  }

  const auto [min_longitudinal_acc, i] = getMinLongitudinalAcc(trajectory);
  validation_status_.min_longitudinal_acc = min_longitudinal_acc;

  if (min_longitudinal_acc < params_.validation_params.acceleration.longitudinal_min_th) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "min_longitudinal_acc");
    is_critical_error_ |= params_.validation_params.acceleration.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidMaxLongitudinalAcceleration(const Trajectory & trajectory)
{
  if (!params_.validation_params.acceleration.enable) {
    return true;
  }

  const auto [max_longitudinal_acc, i] = getMaxLongitudinalAcc(trajectory);
  validation_status_.max_longitudinal_acc = max_longitudinal_acc;

  if (max_longitudinal_acc > params_.validation_params.acceleration.longitudinal_max_th) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "max_longitudinal_acc");
    is_critical_error_ |= params_.validation_params.acceleration.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidSteering(const Trajectory & trajectory)
{
  if (!params_.validation_params.steering.enable) {
    return true;
  }

  const auto [max_steering, i] = calcMaxSteeringAngles(trajectory, vehicle_info_.wheel_base_m);
  validation_status_.max_steering = max_steering;

  if (max_steering > params_.validation_params.steering.threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "max_steering");
    is_critical_error_ |= params_.validation_params.steering.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidSteeringRate(const Trajectory & trajectory)
{
  if (!params_.validation_params.steering.enable) {
    return true;
  }

  const auto [max_steering_rate, i] = calcMaxSteeringRates(trajectory, vehicle_info_.wheel_base_m);
  validation_status_.max_steering_rate = max_steering_rate;

  if (max_steering_rate > params_.validation_params.steering_rate.threshold) {
    debug_pose_publisher_->pushPoseMarker(trajectory.points.at(i).pose, "max_steering_rate");
    is_critical_error_ |= params_.validation_params.steering.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidVelocityDeviation(const Trajectory & trajectory)
{
  if (!params_.validation_params.deviation.enable) {
    return true;
  }

  // TODO(horibe): set appropriate thresholds for index search
  const auto idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, current_kinematics_->pose.pose);

  validation_status_.velocity_deviation = std::abs(
    trajectory.points.at(idx).longitudinal_velocity_mps -
    current_kinematics_->twist.twist.linear.x);

  if (validation_status_.velocity_deviation > params_.validation_params.deviation.velocity_th) {
    is_critical_error_ |= params_.validation_params.deviation.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidDistanceDeviation(const Trajectory & trajectory)
{
  if (!params_.validation_params.deviation.enable) {
    return true;
  }

  // TODO(horibe): set appropriate thresholds for index search
  const auto idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
    trajectory.points, current_kinematics_->pose.pose);

  validation_status_.distance_deviation =
    autoware_utils::calc_distance2d(trajectory.points.at(idx), current_kinematics_->pose.pose);

  if (validation_status_.distance_deviation > params_.validation_params.deviation.distance_th) {
    is_critical_error_ |= params_.validation_params.deviation.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidLongitudinalDistanceDeviation(const Trajectory & trajectory)
{
  if (!params_.validation_params.deviation.enable) {
    return true;
  }

  if (trajectory.points.size() < 2) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Trajectory size is invalid to calculate distance deviation.");
    return false;
  }

  const auto ego_pose = current_kinematics_->pose.pose;
  const size_t idx =
    autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(trajectory.points, ego_pose);

  if (0 < idx && idx < trajectory.points.size() - 1) {
    return true;  // ego-nearest point exists between trajectory points.
  }

  // Check if the valid longitudinal deviation for given segment index
  const auto HasValidLongitudinalDeviation = [&](const size_t seg_idx, const bool is_last) {
    auto long_offset = autoware::motion_utils::calcLongitudinalOffsetToSegment(
      trajectory.points, seg_idx, ego_pose.position);

    // for last, need to remove distance for the last segment.
    if (is_last) {
      const auto size = trajectory.points.size();
      long_offset -= autoware_utils::calc_distance2d(
        trajectory.points.at(size - 1), trajectory.points.at(size - 2));
    }

    validation_status_.longitudinal_distance_deviation = long_offset;
    return std::abs(validation_status_.longitudinal_distance_deviation) <
           params_.validation_params.deviation.lon_distance_th;
  };

  // Make sure the trajectory is far AHEAD from ego.
  if (idx == 0) {
    const auto seg_idx = 0;
    if (!HasValidLongitudinalDeviation(seg_idx, false)) {
      is_critical_error_ |= params_.validation_params.deviation.is_critical;
      return false;
    }
    return true;
  }

  // Make sure the trajectory is far BEHIND from ego.
  if (idx == trajectory.points.size() - 1) {
    const auto seg_idx = trajectory.points.size() - 2;
    if (!HasValidLongitudinalDeviation(seg_idx, true)) {
      is_critical_error_ |= params_.validation_params.deviation.is_critical;
      return false;
    }
    return true;
  }

  return true;
}

bool PlanningValidator::checkValidYawDeviation(const Trajectory & trajectory)
{
  if (!params_.validation_params.deviation.enable) {
    return true;
  }

  const auto interpolated_trajectory_point =
    motion_utils::calcInterpolatedPoint(trajectory, current_kinematics_->pose.pose);
  validation_status_.yaw_deviation = std::abs(
    angles::shortest_angular_distance(
      tf2::getYaw(interpolated_trajectory_point.pose.orientation),
      tf2::getYaw(current_kinematics_->pose.pose.orientation)));

  if (validation_status_.yaw_deviation > params_.validation_params.deviation.yaw_th) {
    is_critical_error_ |= params_.validation_params.deviation.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidForwardTrajectoryLength(const Trajectory & trajectory)
{
  if (!params_.validation_params.forward_trajectory_length.enable) {
    return true;
  }

  const auto ego_speed = std::abs(current_kinematics_->twist.twist.linear.x);
  if (ego_speed < 1.0 / 3.6) {
    return true;  // Ego is almost stopped.
  }

  const auto forward_length = autoware::motion_utils::calcSignedArcLength(
    trajectory.points, current_kinematics_->pose.pose.position, trajectory.points.size() - 1);

  const auto acc = params_.validation_params.forward_trajectory_length.acceleration;
  const auto forward_length_required = ego_speed * ego_speed / (2.0 * std::abs(acc)) -
                                       params_.validation_params.forward_trajectory_length.margin;

  validation_status_.forward_trajectory_length_required = forward_length_required;
  validation_status_.forward_trajectory_length_measured = forward_length;

  if (forward_length < forward_length_required) {
    is_critical_error_ |= params_.validation_params.forward_trajectory_length.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::checkValidLatency(const Trajectory & trajectory)
{
  if (!params_.validation_params.latency.enable) {
    return true;
  }

  validation_status_.latency = (this->now() - trajectory.header.stamp).seconds();

  if (validation_status_.latency > params_.validation_params.latency.threshold) {
    is_critical_error_ |= params_.validation_params.latency.is_critical;
    return false;
  }
  return true;
}

bool PlanningValidator::isAllValid(const PlanningValidatorStatus & s) const
{
  return s.is_valid_size && s.is_valid_finite_value && s.is_valid_interval &&
         s.is_valid_relative_angle && s.is_valid_curvature && s.is_valid_lateral_acc &&
         s.is_valid_longitudinal_max_acc && s.is_valid_longitudinal_min_acc &&
         s.is_valid_steering && s.is_valid_steering_rate && s.is_valid_velocity_deviation &&
         s.is_valid_distance_deviation && s.is_valid_longitudinal_distance_deviation &&
         s.is_valid_forward_trajectory_length && s.is_valid_latency && s.is_valid_yaw_deviation;
}

void PlanningValidator::displayStatus()
{
  if (!params_.display_on_terminal) return;

  const auto warn = [this](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(s.is_valid_size, "planning trajectory size is invalid, too small.");
  warn(s.is_valid_curvature, "planning trajectory curvature is too large!!");
  warn(s.is_valid_finite_value, "planning trajectory has invalid value!!");
  warn(s.is_valid_interval, "planning trajectory interval is too long!!");
  warn(s.is_valid_lateral_acc, "planning trajectory lateral acceleration is too high!!");
  warn(s.is_valid_longitudinal_max_acc, "planning trajectory acceleration is too high!!");
  warn(s.is_valid_longitudinal_min_acc, "planning trajectory deceleration is too high!!");
  warn(s.is_valid_relative_angle, "planning trajectory yaw angle varies too fast!!");
  warn(s.is_valid_steering, "planning trajectory expected steering angle is too high!!");
  warn(s.is_valid_steering_rate, "planning trajectory expected steering angle rate is too high!!");
  warn(s.is_valid_velocity_deviation, "planning trajectory velocity deviation is too high!!");
  warn(s.is_valid_distance_deviation, "planning trajectory is too far from ego!!");
  warn(
    s.is_valid_longitudinal_distance_deviation,
    "planning trajectory is too far from ego in longitudinal direction!!");
  warn(s.is_valid_forward_trajectory_length, "planning trajectory forward length is not enough!!");
  warn(s.is_valid_latency, "planning component latency is larger than threshold!!");
  warn(s.is_valid_yaw_deviation, "planning trajectory yaw difference from ego yaw is too large!!");
}

}  // namespace autoware::planning_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_validator::PlanningValidator)
