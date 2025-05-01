// Copyright 2025Tier IV, Inc.
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

#include "autoware/planning_validator/node.hpp"

#include "autoware/planning_validator/utils.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>

#include <angles/angles/angles.h>

#include <memory>
#include <string>

namespace autoware::planning_validator
{
using diagnostic_msgs::msg::DiagnosticStatus;

PlanningValidatorNode::PlanningValidatorNode(const rclcpp::NodeOptions & options)
: Node("planning_validator_node", options)
{
  pub_traj_ = create_publisher<Trajectory>("~/output/trajectory", 1);
  pub_status_ = create_publisher<PlanningValidatorStatus>("~/output/validation_status", 1);
  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", 1);
  pub_processing_time_ms_ = create_publisher<Float64Stamped>("~/debug/processing_time_ms", 1);

  debug_pose_publisher_ = std::make_shared<PlanningValidatorDebugMarkerPublisher>(this);
  data_ = std::make_shared<PlanningValidatorData>(*this);
  validation_status_ = std::make_shared<PlanningValidatorStatus>();

  setupParameters();

  // Start timer
  {
    const auto planning_hz = declare_parameter<double>("planning_hz");
    const auto period_ns = rclcpp::Rate(planning_hz).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&PlanningValidatorNode::onTimer, this));
  }

  // Initialize Manager
  for (const auto & name : declare_parameter<std::vector<std::string>>("launch_modules")) {
    // workaround: Since ROS 2 can't get empty list, launcher set [''] on the parameter.
    if (name.empty()) {
      break;
    }
    manager_.load_plugin(*this, name);
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

void PlanningValidatorNode::setupParameters()
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

  params_.enable_soft_stop_on_prev_traj = declare_parameter<bool>("enable_soft_stop_on_prev_traj");
  params_.soft_stop_deceleration = declare_parameter<double>("soft_stop_deceleration");
  params_.soft_stop_jerk_lim = declare_parameter<double>("soft_stop_jerk_lim");

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
    set_validation_params(p.lateral_jerk, t + "lateral_jerk");

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

    set_validation_flags(p.trajectory_shift, t + "trajectory_shift");
    p.trajectory_shift.lat_shift_th =
      declare_parameter<double>(t + "trajectory_shift.lat_shift_th");
    p.trajectory_shift.forward_shift_th =
      declare_parameter<double>(t + "trajectory_shift.forward_shift_th");
    p.trajectory_shift.backward_shift_th =
      declare_parameter<double>(t + "trajectory_shift.backward_shift_th");

    set_validation_flags(p.forward_trajectory_length, t + "forward_trajectory_length");
    p.forward_trajectory_length.acceleration =
      declare_parameter<double>(t + "forward_trajectory_length.acceleration");
    p.forward_trajectory_length.margin =
      declare_parameter<double>(t + "forward_trajectory_length.margin");
  }
}

void PlanningValidatorNode::setStatus(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg,
  const bool is_critical)
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
    return;
  }

  const bool only_warn = std::invoke([&]() {
    const auto handling_type =
      is_critical ? params_.inv_traj_critical_handling_type : params_.inv_traj_handling_type;
    if (handling_type != InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT) {
      return false;
    }
    return params_.enable_soft_stop_on_prev_traj;
  });

  if (validation_status_->invalid_count < params_.diag_error_count_threshold || only_warn) {
    const auto warn_msg = msg + " (invalid count is less than error threshold: " +
                          std::to_string(validation_status_->invalid_count) + " < " +
                          std::to_string(params_.diag_error_count_threshold) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
  } else {
    stat.summary(DiagnosticStatus::ERROR, msg);
  }
}

void PlanningValidatorNode::setupDiag()
{
  diag_updater_ = std::make_shared<Updater>(this);
  auto & d = diag_updater_;
  d->setHardwareID("planning_validator");

  const auto & p = params_.validation_params;

  std::string ns = "trajectory_validation_";
  d->add(ns + "size", [&](auto & stat) {
    setStatus(stat, validation_status_->is_valid_size, "invalid trajectory size is found");
  });
  d->add(ns + "finite", [&](auto & stat) {
    setStatus(stat, validation_status_->is_valid_finite_value, "infinite value is found");
  });
  d->add(ns + "interval", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_interval, "points interval is too long",
      p.interval.is_critical);
  });
  d->add(ns + "relative_angle", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_relative_angle, "relative angle is too large",
      p.relative_angle.is_critical);
  });
  d->add(ns + "curvature", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_curvature, "curvature is too large",
      p.curvature.is_critical);
  });
  d->add(ns + "lateral_acceleration", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_lateral_acc, "lateral acceleration is too large",
      p.acceleration.is_critical);
  });
  d->add(ns + "acceleration", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_longitudinal_max_acc, "acceleration is too large",
      p.acceleration.is_critical);
  });
  d->add(ns + "deceleration", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_longitudinal_min_acc, "deceleration is too large",
      p.acceleration.is_critical);
  });
  d->add(ns + "steering", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_steering, "expected steering is too large",
      p.steering.is_critical);
  });
  d->add(ns + "steering_rate", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_steering_rate, "expected steering rate is too large",
      p.steering.is_critical);
  });
  d->add(ns + "velocity_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_velocity_deviation, "velocity deviation is too large",
      p.deviation.is_critical);
  });
  d->add(ns + "distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_distance_deviation, "distance deviation is too large",
      p.deviation.is_critical);
  });
  d->add(ns + "longitudinal_distance_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_longitudinal_distance_deviation,
      "longitudinal distance deviation is too large", p.deviation.is_critical);
  });
  d->add(ns + "forward_trajectory_length", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_forward_trajectory_length,
      "trajectory length is too short", p.forward_trajectory_length.is_critical);
  });
  d->add(ns + "latency", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_latency, "latency is larger than expected value.",
      p.latency.is_critical);
  });
  d->add(ns + "yaw_deviation", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_yaw_deviation,
      "difference between vehicle yaw and closest trajectory yaw is too large.",
      p.deviation.is_critical);
  });
  d->add(ns + "trajectory_shift", [&](auto & stat) {
    setStatus(
      stat, validation_status_->is_valid_trajectory_shift, "detected sudden shift in trajectory.",
      p.trajectory_shift.is_critical);
  });
}

bool PlanningValidatorNode::isDataReady()
{
  const auto waiting = [this](const auto s) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for %s", s.c_str());
    return false;
  };

  std::string msg;
  if (!data_->is_ready(msg)) {
    return waiting(msg);
  }
  return true;
}

void PlanningValidatorNode::setData()
{
  data_->current_trajectory = sub_trajectory_.take_data();
  data_->current_kinematics = sub_kinematics_.take_data();
  data_->current_acceleration = sub_acceleration_.take_data();
  if (data_->current_trajectory) {
    constexpr auto min_interval = 1.0;
    data_->resampled_current_trajectory =
      std::make_shared<Trajectory>(resampleTrajectory(*data_->current_trajectory, min_interval));
  }
  data_->nearest_point_index.reset();
  data_->nearest_segment_index.reset();
}

void PlanningValidatorNode::onTimer()
{
  stop_watch_.tic(__func__);

  setData();

  if (!isDataReady()) return;

  data_->set_nearest_trajectory_indices();

  if (params_.publish_diag && !diag_updater_) {
    setupDiag();  // run setup after all data is ready.
  }

  debug_pose_publisher_->clearMarkers();
  is_critical_error_ = false;

  validate(data_);

  diag_updater_->force_update();

  publishTrajectory();

  // for debug
  publishProcessingTime(stop_watch_.toc(__func__));
  publishDebugInfo();
  displayStatus();
}

void PlanningValidatorNode::validate(const std::shared_ptr<const PlanningValidatorData> & data)
{
  auto & s = validation_status_;

  manager_.validate(data, s, is_critical_error_);

  s->invalid_count = isAllValid(*s) ? 0 : s->invalid_count + 1;
}

bool PlanningValidatorNode::isAllValid(const PlanningValidatorStatus & s) const
{
  return s.is_valid_size && s.is_valid_finite_value && s.is_valid_interval &&
         s.is_valid_relative_angle && s.is_valid_curvature && s.is_valid_lateral_acc &&
         s.is_valid_lateral_jerk && s.is_valid_longitudinal_max_acc &&
         s.is_valid_longitudinal_min_acc && s.is_valid_steering && s.is_valid_steering_rate &&
         s.is_valid_velocity_deviation && s.is_valid_distance_deviation &&
         s.is_valid_longitudinal_distance_deviation && s.is_valid_forward_trajectory_length &&
         s.is_valid_latency && s.is_valid_yaw_deviation && s.is_valid_trajectory_shift;
}

void PlanningValidatorNode::publishTrajectory()
{
  // Validation check is all green. Publish the trajectory.
  if (isAllValid(*validation_status_)) {
    pub_traj_->publish(*data_->current_trajectory);
    published_time_publisher_->publish_if_subscribed(
      pub_traj_, data_->current_trajectory->header.stamp);
    data_->last_valid_trajectory = data_->current_trajectory;
    soft_stop_trajectory_ = nullptr;
    return;
  }

  //  ----- invalid factor is found. Publish previous trajectory. -----

  const auto handling_type =
    is_critical_error_ ? params_.inv_traj_critical_handling_type : params_.inv_traj_handling_type;

  if (handling_type == InvalidTrajectoryHandlingType::PUBLISH_AS_IT_IS) {
    pub_traj_->publish(*data_->current_trajectory);
    published_time_publisher_->publish_if_subscribed(
      pub_traj_, data_->current_trajectory->header.stamp);
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 3000, "Caution! Invalid Trajectory published.");
    return;
  }

  if (handling_type == InvalidTrajectoryHandlingType::STOP_PUBLISHING) {
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Trajectory is not published.");
    return;
  }

  if (
    handling_type == InvalidTrajectoryHandlingType::USE_PREVIOUS_RESULT &&
    data_->last_valid_trajectory) {
    if (params_.enable_soft_stop_on_prev_traj && !soft_stop_trajectory_) {
      const auto nearest_idx = autoware::motion_utils::findFirstNearestIndexWithSoftConstraints(
        data_->last_valid_trajectory->points, data_->current_kinematics->pose.pose);
      soft_stop_trajectory_ = std::make_shared<Trajectory>(planning_validator::getStopTrajectory(
        *data_->last_valid_trajectory, nearest_idx, data_->current_kinematics->twist.twist.linear.x,
        data_->current_acceleration->accel.accel.linear.x, params_.soft_stop_deceleration,
        params_.soft_stop_jerk_lim));
    }
    const auto & pub_trajectory = params_.enable_soft_stop_on_prev_traj
                                    ? *soft_stop_trajectory_
                                    : *data_->last_valid_trajectory;
    pub_traj_->publish(pub_trajectory);
    published_time_publisher_->publish_if_subscribed(pub_traj_, pub_trajectory.header.stamp);
    RCLCPP_ERROR(get_logger(), "Invalid Trajectory detected. Use previous trajectory.");
    return;
  }

  // trajectory is not published.
  RCLCPP_ERROR_THROTTLE(
    get_logger(), *get_clock(), 3000,
    "Invalid Trajectory detected, no valid trajectory found in the past. Trajectory is not "
    "published.");
  return;
}

void PlanningValidatorNode::publishProcessingTime(const double processing_time_ms)
{
  Float64Stamped msg{};
  msg.stamp = this->now();
  msg.data = processing_time_ms;
  pub_processing_time_ms_->publish(msg);
}

void PlanningValidatorNode::publishDebugInfo()
{
  validation_status_->stamp = get_clock()->now();
  pub_status_->publish(*validation_status_);

  if (!isAllValid(*validation_status_)) {
    geometry_msgs::msg::Pose front_pose = data_->current_kinematics->pose.pose;
    shiftPose(front_pose, data_->vehicle_info.front_overhang_m + data_->vehicle_info.wheel_base_m);
    auto offset_pose = front_pose;
    shiftPose(offset_pose, 0.25);
    debug_pose_publisher_->pushVirtualWall(front_pose);
    debug_pose_publisher_->pushWarningMsg(offset_pose, "INVALID PLANNING");
  }
  debug_pose_publisher_->publish();
}

void PlanningValidatorNode::displayStatus()
{
  if (!params_.display_on_terminal) return;

  const auto warn = [this](const bool status, const std::string & msg) {
    if (!status) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", msg.c_str());
    }
  };

  const auto & s = validation_status_;

  warn(s->is_valid_size, "planning trajectory size is invalid, too small.");
  warn(s->is_valid_curvature, "planning trajectory curvature is too large!!");
  warn(s->is_valid_finite_value, "planning trajectory has invalid value!!");
  warn(s->is_valid_interval, "planning trajectory interval is too long!!");
  warn(s->is_valid_lateral_acc, "planning trajectory lateral acceleration is too high!!");
  warn(s->is_valid_lateral_jerk, "planning trajectory lateral jerk is too high!!");
  warn(s->is_valid_longitudinal_max_acc, "planning trajectory acceleration is too high!!");
  warn(s->is_valid_longitudinal_min_acc, "planning trajectory deceleration is too high!!");
  warn(s->is_valid_relative_angle, "planning trajectory yaw angle varies too fast!!");
  warn(s->is_valid_steering, "planning trajectory expected steering angle is too high!!");
  warn(s->is_valid_steering_rate, "planning trajectory expected steering angle rate is too high!!");
  warn(s->is_valid_velocity_deviation, "planning trajectory velocity deviation is too high!!");
  warn(s->is_valid_distance_deviation, "planning trajectory is too far from ego!!");
  warn(
    s->is_valid_longitudinal_distance_deviation,
    "planning trajectory is too far from ego in longitudinal direction!!");
  warn(s->is_valid_forward_trajectory_length, "planning trajectory forward length is not enough!!");
  warn(s->is_valid_latency, "planning component latency is larger than threshold!!");
  warn(s->is_valid_yaw_deviation, "planning trajectory yaw difference from ego yaw is too large!!");
  warn(s->is_valid_trajectory_shift, "planning trajectory had sudden shift!!");
}

}  // namespace autoware::planning_validator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::planning_validator::PlanningValidatorNode)
