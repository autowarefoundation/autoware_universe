// Copyright 2025 The Autoware Contributors
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

#ifndef COMMAND__SOURCE_HPP_
#define COMMAND__SOURCE_HPP_

#include "common/timeout_diagnostics.hpp"
#include "interface.hpp"

#include <memory>
#include <string>

namespace autoware::control_command_gate
{

class CommandSource
{
public:
  CommandSource(uint16_t id, const std::string & name);
  virtual ~CommandSource() = default;
  virtual void resend_last_command() = 0;

  uint16_t id() const { return source_id_; }
  void set_output(CommandOutput * output);

  TimeoutDiag * create_diag_task(const TimeoutDiag::Params & params, const rclcpp::Clock & clock);
  bool is_timeout() const { return timeout_->is_error(); }

protected:
  void send_control(const Control & msg);
  void send_gear(const GearCommand & msg);
  void send_turn_indicators(const TurnIndicatorsCommand & msg);
  void send_hazard_lights(const HazardLightsCommand & msg);

private:
  const uint16_t source_id_;
  const std::string source_name_;
  CommandOutput * output_;
  std::unique_ptr<TimeoutDiag> timeout_;
};

}  // namespace autoware::control_command_gate

#endif  // COMMAND__SOURCE_HPP_
