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

#ifndef CORE__MONITOR_HPP_
#define CORE__MONITOR_HPP_

#include "interface.hpp"

#include <autoware_utils_diagnostics/timeout_diagnostics.hpp>

#include <memory>

namespace autoware::trajectory_gate
{

class TrajectoryMonitor : public TrajectorySender, public TrajectoryReceiver
{
public:
  using TimeoutDiag = autoware_utils_diagnostics::TimeoutDiag;
  explicit TrajectoryMonitor(std::unique_ptr<TimeoutDiag> && timeout);
  void receive(const Trajectory & msg) override;

private:
  std::unique_ptr<TimeoutDiag> timeout_;
};

}  // namespace autoware::trajectory_gate

#endif  // CORE__MONITOR_HPP_
