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

#ifndef COMMON__GRAPH__DIAGS_HPP_
#define COMMON__GRAPH__DIAGS_HPP_

#include "config/yaml.hpp"
#include "graph/units.hpp"
#include "types/diagnostics.hpp"
#include "types/forward.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class DiagUnit : public BaseUnit
{
public:
  explicit DiagUnit(ConfigYaml yaml, const std::string & name);
  ~DiagUnit();
  DiagnosticLevel level() const override;
  std::vector<LinkPort *> ports() const override { return {}; }
  std::string debug() const override { return "DiagUnit"; }

  DiagLeafStruct create_struct();
  DiagLeafStatus create_status();

  std::string name() const;
  void update(const rclcpp::Time & stamp);
  void update(const rclcpp::Time & stamp, const DiagnosticStatus & status);

private:
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
  std::unique_ptr<TimeoutLevel> timeout_;
  std::unique_ptr<HysteresisLevel> hysteresis_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__DIAGS_HPP_
