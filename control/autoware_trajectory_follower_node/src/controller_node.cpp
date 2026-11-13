// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#include "autoware/trajectory_follower_node/controller_node.hpp"

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"
#include "autoware/pid_longitudinal_controller/pid_longitudinal_controller_node.hpp"
#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"

#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::motion::control::trajectory_follower_node
{
Controller::Controller(const rclcpp::NodeOptions & node_options)
: Node("controller", node_options),
  diag_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  using std::placeholders::_1;

  const double ctrl_period = declare_parameter<double>("ctrl_period");
  const auto trajectory_reference_mode =
    declare_parameter<std::string>("trajectory_reference_mode", "spatial");
  if (trajectory_reference_mode != "spatial" && trajectory_reference_mode != "temporal") {
    throw std::invalid_argument(
      "Invalid trajectory_reference_mode. Expected \"spatial\" or \"temporal\".");
  }
  timeout_thr_sec_ = declare_parameter<double>("timeout_thr_sec");
  cyclic_message_timeout_thr_sec_ = declare_parameter<double>("cyclic_message_timeout_thr_sec");
  // NOTE: It is possible that using control_horizon could be expected to enhance performance,
  // but it is not a formal interface topic, only an experimental one.
  // So it is disabled by default.
  enable_control_cmd_horizon_pub_ =
    declare_parameter<bool>("enable_control_cmd_horizon_pub", false);

  diag_updater_->setHardwareID("trajectory_follower_node");
  diag_updater_->add("incoming_message_timeout", this, &Controller::check_cyclic_message_timeout);

  std::unique_ptr<trajectory_follower::LateralControllerInterface> lateral_controller;
  const auto lateral_controller_mode =
    getLateralControllerMode(declare_parameter<std::string>("lateral_controller_mode"));
  switch (lateral_controller_mode) {
    case LateralControllerMode::MPC: {
      lateral_controller =
        std::make_unique<mpc_lateral_controller::MpcLateralControllerNode>(*this, diag_updater_);
      break;
    }
    case LateralControllerMode::PURE_PURSUIT: {
      lateral_controller =
        std::make_unique<autoware::pure_pursuit::PurePursuitLateralControllerNode>(*this);
      break;
    }
    default:
      throw std::domain_error("[LateralController] invalid algorithm");
  }

  std::unique_ptr<trajectory_follower::LongitudinalControllerInterface> longitudinal_controller;
  const auto longitudinal_controller_mode =
    getLongitudinalControllerMode(declare_parameter<std::string>("longitudinal_controller_mode"));
  switch (longitudinal_controller_mode) {
    case LongitudinalControllerMode::PID: {
      longitudinal_controller =
        std::make_unique<pid_longitudinal_controller::PidLongitudinalControllerNode>(
          *this, diag_updater_);
      break;
    }
    default:
      throw std::domain_error("[LongitudinalController] invalid algorithm");
  }

  trajectory_follower_ = std::make_unique<TrajectoryFollower>(
    TrajectoryFollowerConfig{timeout_thr_sec_, enable_control_cmd_horizon_pub_},
    std::move(lateral_controller), std::move(longitudinal_controller));

  control_cmd_pub_ = create_publisher<autoware_control_msgs::msg::Control>(
    "~/output/control_cmd", rclcpp::QoS{1}.transient_local());
  predicted_trajectory_pub_ =
    create_publisher<autoware_planning_msgs::msg::Trajectory>("~/output/predicted_trajectory", 1);
  debug_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/debug_marker", rclcpp::QoS{1});

  sub_steering_offset_update_ =
    create_subscription<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "~/input/steering_offset_update", rclcpp::QoS{1}.transient_local(),
      [this](const autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr msg) {
        steering_offset_ = *msg;
      });

  if (enable_control_cmd_horizon_pub_) {
    control_cmd_horizon_pub_ = create_publisher<autoware_control_msgs::msg::ControlHorizon>(
      "~/debug/control_cmd_horizon", 1);
  }

  // Timer
  {
    const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(ctrl_period));
    timer_control_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&Controller::callbackTimerControl, this));
  }

  logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);

  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
}

Controller::LateralControllerMode Controller::getLateralControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "mpc") return LateralControllerMode::MPC;
  if (controller_mode == "pure_pursuit") return LateralControllerMode::PURE_PURSUIT;

  return LateralControllerMode::INVALID;
}

Controller::LongitudinalControllerMode Controller::getLongitudinalControllerMode(
  const std::string & controller_mode) const
{
  if (controller_mode == "pid") return LongitudinalControllerMode::PID;

  return LongitudinalControllerMode::INVALID;
}

void Controller::check_cyclic_message_timeout(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const auto traj_timestamp = sub_ref_path_.last_taken_data_timestamp();

  if (!traj_timestamp) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "timeout");
    stat.add("trajectory", "no message received");
    return;
  }

  const auto elapsed = (this->now() - traj_timestamp.value()).seconds();
  if (elapsed > cyclic_message_timeout_thr_sec_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "timeout");
    stat.add("trajectory", "timeout (elapsed: " + std::to_string(elapsed) + " sec)");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    stat.add("trajectory", "OK");
  }
}

bool Controller::processData(rclcpp::Clock & clock)
{
  bool is_ready = true;

  const auto & logData = [&clock, this](const std::string & data_type) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), clock, logger_throttle_interval, "Waiting for %s data", data_type.c_str());
  };

  const auto & getData = [&logData](auto & dest, auto & sub, const std::string & data_type = "") {
    const auto temp = sub.take_data();
    if (temp) {
      dest = temp;
      return true;
    }
    if (!data_type.empty()) logData(data_type);
    return false;
  };

  is_ready &= getData(current_accel_ptr_, sub_accel_, "acceleration");
  is_ready &= getData(current_steering_ptr_, sub_steering_, "steering");
  is_ready &= getData(current_trajectory_ptr_, sub_ref_path_, "trajectory");
  is_ready &= getData(current_odometry_ptr_, sub_odometry_, "odometry");
  is_ready &= getData(current_operation_mode_ptr_, sub_operation_mode_, "operation mode");

  return is_ready;
}

boost::optional<trajectory_follower::InputData> Controller::createInputData(rclcpp::Clock & clock)
{
  if (!processData(clock)) {
    return {};
  }

  trajectory_follower::InputData input_data;
  input_data.current_trajectory = *current_trajectory_ptr_;
  input_data.current_odometry = *current_odometry_ptr_;
  input_data.current_steering = *current_steering_ptr_;
  input_data.current_accel = *current_accel_ptr_;
  input_data.current_operation_mode = *current_operation_mode_ptr_;
  input_data.steering_offset = steering_offset_;

  return input_data;
}

void Controller::callbackTimerControl()
{
  // read the time once for this control cycle
  const rclcpp::Time stamp = this->now();

  // 1. create input data
  const auto input_data = createInputData(*get_clock());
  if (!input_data) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Control is skipped since input data is not ready.");
    return;
  }

  // 2. run the lateral and longitudinal controllers
  const auto result = trajectory_follower_->run(*input_data, stamp);
  if (!result) {
    const auto & error = result.error();
    if (error.level == TrajectoryFollowerError::Level::error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.message.c_str());
    } else {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.message.c_str());
    }
    return;
  }

  // 4.5. update diagnostics
  diag_updater_->force_update();

  // 5. publish control command
  control_cmd_pub_->publish(result->control_cmd);
  if (result->predicted_trajectory) {
    predicted_trajectory_pub_->publish(*result->predicted_trajectory);
  }

  // 6. publish debug
  published_time_publisher_->publish_if_subscribed(control_cmd_pub_, result->control_cmd.stamp);
  debug_marker_pub_->publish(result->debug_marker);

  // 7. publish experimental topic
  if (enable_control_cmd_horizon_pub_) {
    const auto & control_horizon = result->control_cmd_horizon;
    if (control_horizon.has_value()) {
      control_cmd_horizon_pub_->publish(control_horizon.value());
    }
  }
}

}  // namespace autoware::motion::control::trajectory_follower_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_node::Controller)
