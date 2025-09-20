// Copyright 2022 Autoware Foundation
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

#ifndef STATE_HPP_
#define STATE_HPP_

#include "autoware_operation_mode_transition_manager/msg/operation_mode_transition_manager_debug.hpp"
#include "data.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <utility>

namespace autoware::operation_mode_transition_manager
{

using Control = autoware_control_msgs::msg::Control;
using Odometry = nav_msgs::msg::Odometry;
using Trajectory = autoware_planning_msgs::msg::Trajectory;

struct InputData
{
  std::optional<Odometry> kinematics;
  std::optional<Trajectory> trajectory;
  std::optional<Control> trajectory_follower_control_cmd;
  std::optional<Control> control_cmd;
  OperationModeState gate_operation_mode;
};

class ModeChangeBase
{
public:
  virtual ~ModeChangeBase() = default;
  virtual void update(bool) {}
  virtual bool isModeChangeCompleted(const InputData & input_data) = 0;
  virtual bool isModeChangeAvailable(const InputData & input_data) = 0;

  using DebugInfo =
    autoware_operation_mode_transition_manager::msg::OperationModeTransitionManagerDebug;
  virtual DebugInfo getDebugInfo() { return DebugInfo{}; }
};

class StopMode : public ModeChangeBase
{
public:
  bool isModeChangeCompleted(const InputData &) override { return true; }
  bool isModeChangeAvailable(const InputData &) override { return true; }
};

class AutonomousMode : public ModeChangeBase
{
public:
  explicit AutonomousMode(rclcpp::Node * node);
  void update(bool transition) override;
  bool isModeChangeCompleted(const InputData & input_data) override;
  bool isModeChangeAvailable(const InputData & input_data) override;
  DebugInfo getDebugInfo() override { return debug_info_; }

private:
  bool hasDangerAcceleration(const Odometry & kinematics, const Control & control_cmd);
  std::pair<bool, bool> hasDangerLateralAcceleration(
    const Odometry & kinematics, const Control & control_cmd);

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  bool check_engage_condition_ = true;       // if false, the vehicle is engaged without any checks.
  bool enable_engage_on_driving_ = false;    // if false, engage is not permited on driving
  double nearest_dist_deviation_threshold_;  // [m] for finding nearest index
  double nearest_yaw_deviation_threshold_;   // [rad] for finding nearest index
  EngageAcceptableParam engage_acceptable_param_;
  StableCheckParam stable_check_param_;
  Trajectory trajectory_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  DebugInfo debug_info_;
  std::shared_ptr<rclcpp::Time> stable_start_time_;  // Reset every transition start.
};

// TODO(Takagi, Isamu): Connect with status from local operation node
class LocalMode : public ModeChangeBase
{
public:
  bool isModeChangeCompleted(const InputData &) override { return true; }
  bool isModeChangeAvailable(const InputData &) override { return true; }
};

// TODO(Takagi, Isamu): Connect with status from remote operation node
class RemoteMode : public ModeChangeBase
{
public:
  bool isModeChangeCompleted(const InputData &) override { return true; }
  bool isModeChangeAvailable(const InputData &) override { return true; }
};

}  // namespace autoware::operation_mode_transition_manager

#endif  // STATE_HPP_
