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

#include "timeout_diagnostics.hpp"

#include <string>

namespace autoware::control_command_gate
{

TimeoutDiag::TimeoutDiag(
  const Params & params, const rclcpp::Clock & clock, const std::string & name)
: DiagnosticTask(name), params_(params), clock_(clock)
{
  level_ = DiagnosticStatus::STALE;
}

void TimeoutDiag::update()
{
  last_stamp_ = clock_.now();
}

void TimeoutDiag::run(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (!last_stamp_) {
    stat.summary(DiagnosticStatus::ERROR, "no data");
    return;
  }

  const auto duration = (clock_.now() - *last_stamp_).seconds();

  if (duration < params_.warn_duration_) {
    level_ = DiagnosticStatus::OK;
  } else if (duration < params_.error_duration_) {
    level_ = DiagnosticStatus::WARN;
  } else {
    level_ = DiagnosticStatus::ERROR;
  }

  stat.summary(level_, "");
  stat.add("duration", duration);
}

}  // namespace autoware::control_command_gate
