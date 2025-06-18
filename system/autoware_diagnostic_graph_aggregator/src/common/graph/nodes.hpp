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

#ifndef COMMON__GRAPH__NODES_HPP_
#define COMMON__GRAPH__NODES_HPP_

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

class NodeUnit : public BaseUnit
{
public:
  explicit NodeUnit(Parser & parser);
  ~NodeUnit();
  DiagnosticLevel level() const override;
  std::vector<LinkPort *> ports() const override;
  std::string debug() const override { return "NodeUnit"; }

  DiagNodeStruct create_struct();
  DiagNodeStatus create_status();

  std::string path() const;
  std::string type() const;
  bool dependency() const;
  void reset();
  void update(const rclcpp::Time & stamp);

private:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
  std::unique_ptr<Logic> logic_;
  std::unique_ptr<LatchLevel> latch_;
  LinkItem * dependency_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__NODES_HPP_
