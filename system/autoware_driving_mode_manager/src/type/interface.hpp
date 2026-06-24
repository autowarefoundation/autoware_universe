// Copyright 2026 The Autoware Contributors
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

#ifndef TYPE__INTERFACE_HPP_
#define TYPE__INTERFACE_HPP_

#include "type/types.hpp"

#include <autoware_driving_mode_manager/types.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::driving_mode_manager
{

class MainLogic;
class Interface
{
public:
  virtual ~Interface() = default;
  virtual void init(MainLogic * logic) = 0;

  virtual rclcpp::Time now() const = 0;
  virtual void change_trajectory_source(const TrajectorySource & source) = 0;
  virtual void change_command_source(const CommandSource & source) = 0;
  virtual void change_command_filter(const CommandFilter & filter) = 0;
  virtual void change_platform_mode(const PlatformMode & mode) = 0;
  virtual void publish_operation_mode(const OperationModeState & state) const = 0;
  virtual void publish_mrm_state(const MrmState & state) const = 0;
  virtual void publish_driving_mode_request(const ModeRequest & request) const = 0;
  virtual void publish_driving_mode_info(const ModeInfo & info) const = 0;
  virtual void publish_debug(const DebugStatus & status) const = 0;
  virtual void publish_debug(const RequestModes & request) const = 0;

  virtual void log_info(const std::string & message) const = 0;
  virtual void log_warn(const std::string & message) const = 0;
  virtual void log_error(const std::string & message) const = 0;
  virtual void log_debug(const std::string & message) const = 0;
};

class MainLogic
{
public:
  virtual ~MainLogic() = default;

  virtual void update() = 0;
  virtual void on_trajectory_source(const TrajectorySource & source) = 0;
  virtual void on_command_source(const CommandSource & source) = 0;
  virtual void on_command_filter(const CommandFilter & filter) = 0;
  virtual void on_vehicle_control_mode(const PlatformMode & mode) = 0;
  virtual void on_available_flag(const AutowareMode & mode, bool flag) = 0;
  virtual void on_active_flag(const AutowareMode & mode, bool flag) = 0;
  virtual void on_stable_flag(const AutowareMode & mode, bool flag) = 0;
  virtual void on_continuable_flag(const AutowareMode & mode, bool flag) = 0;
  virtual void on_mrm_state(const AutowareMode & mode, const MrmState::State & state) = 0;
  virtual ServiceResponse change_operation_mode(const OperationMode & operation_mode) = 0;
  virtual ServiceResponse change_autoware_control(const AutowareControl & autoware_control) = 0;
  virtual ServiceResponse change_mrm_request(const MrmRequest & request) = 0;
};

}  // namespace autoware::driving_mode_manager

#endif  // TYPE__INTERFACE_HPP_
