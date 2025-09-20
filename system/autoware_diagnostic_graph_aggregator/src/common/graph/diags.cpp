// Copyright 2023 The Autoware Contributors
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

#include "graph/diags.hpp"

#include "config/entity.hpp"
#include "graph/levels.hpp"
#include "graph/links.hpp"
#include "graph/logic.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

DiagUnit::DiagUnit(ConfigYaml yaml, const std::string & name)
{
  timeout_ = std::make_unique<TimeoutLevel>(yaml);
  hysteresis_ = std::make_unique<HysteresisLevel>(yaml);

  struct_.name = name;
  status_.level = DiagnosticStatus::STALE;
}

DiagUnit::~DiagUnit()
{
}

DiagLeafStruct DiagUnit::create_struct()
{
  struct_.parent = parent_units().front()->index();
  return struct_;
}

DiagLeafStatus DiagUnit::create_status()
{
  status_.level = hysteresis_->level();
  status_.input_level = hysteresis_->input_level();
  return status_;
}

DiagnosticLevel DiagUnit::level() const
{
  return hysteresis_->level();
}

std::string DiagUnit::name() const
{
  return struct_.name;
}

void DiagUnit::update(const rclcpp::Time & stamp)
{
  timeout_->update(stamp);
  hysteresis_->update(stamp, timeout_->level());
}

void DiagUnit::update(const rclcpp::Time & stamp, const DiagnosticStatus & status)
{
  // No need to update hysteresis because update function is called before using level.
  timeout_->update(stamp, status.level);

  status_.message = status.message;
  status_.hardware_id = status.hardware_id;
  status_.values = status.values;
}

}  // namespace autoware::diagnostic_graph_aggregator
