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

#ifndef CORE__CONFIG_HPP_
#define CORE__CONFIG_HPP_

#include <autoware_driving_mode_manager/config.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::driving_mode_manager
{

class DrivingModeConfig : public DrivingModeConfigInterface
{
public:
  void define_autoware_mode(const AutowareMode & mode, const OperationMode & opmode) override;
  void define_autoware_mode(const AutowareMode & mode, const MrmBehavior & behavior) override;
  void define_trajectory_source(const TrajectorySource & source) override;
  void define_command_source(const CommandSource & source) override;
  void bind_name(const AutowareMode & mode, const std::string & name) override;
  void bind_gates(const AutowareMode & mode, const Gates & gates) override;

  void validate() const;

  std::vector<AutowareMode> autoware_modes() const;
  std::string name(const AutowareMode & mode) const;
  bool exists(const AutowareMode & mode) const;
  Gates gates(const AutowareMode & mode) const;
  AutowareMode to_autoware_mode(const OperationMode & opmode) const;
  std::optional<AutowareMode> to_autoware_mode(const MrmBehavior & behavior) const;
  OperationMode to_operation_mode(const AutowareMode & mode) const;
  std::optional<MrmBehavior> to_mrm_behavior(const AutowareMode & mode) const;

private:
  std::unordered_set<AutowareMode> autoware_modes_;
  std::unordered_set<TrajectorySource> trajectory_sources_;
  std::unordered_set<CommandSource> command_sources_;
  std::unordered_map<AutowareMode, std::string> autoware_mode_names_;
  std::unordered_map<AutowareMode, Gates> gates_mapping_;
  std::unordered_map<OperationMode, AutowareMode> operation_to_autoware_;
  std::unordered_map<AutowareMode, OperationMode> autoware_to_operation_;
  std::unordered_map<MrmBehavior, AutowareMode> mrm_to_autoware_;
  std::unordered_map<AutowareMode, MrmBehavior> autoware_to_mrm_;
};

}  // namespace autoware::driving_mode_manager

#endif  // CORE__CONFIG_HPP_
