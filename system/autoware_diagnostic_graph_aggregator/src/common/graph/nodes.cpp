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

#include "graph/nodes.hpp"

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

NodeUnit::NodeUnit(Parser & parser)
{
  const auto dependency = parser.yaml().optional("dependent").text("");
  dependency_ = dependency.empty() ? nullptr : parser.parse_link_node(dependency);

  logic_ = LogicFactory::Create(parser);
  latch_ = std::make_unique<LatchLevel>(parser.yaml());

  struct_.path = parser.yaml().optional("path").text("");
  struct_.type = parser.type();
  status_.level = DiagnosticStatus::STALE;
}

NodeUnit::~NodeUnit()
{
}

DiagNodeStruct NodeUnit::create_struct()
{
  return struct_;
}

DiagNodeStatus NodeUnit::create_status()
{
  status_.level = latch_->level();
  status_.input_level = latch_->input_level();  // Note: Equals logic level.
  status_.latch_level = latch_->latch_level();
  status_.is_dependent = dependency();
  return status_;
}

bool NodeUnit::dependency() const
{
  return dependency_ && dependency_->level() != DiagnosticStatus::OK;
}

void NodeUnit::reset()
{
  latch_->reset();
}

DiagnosticLevel NodeUnit::level() const
{
  return latch_->level();
}

std::vector<LinkPort *> NodeUnit::ports() const
{
  return logic_->ports();
}

std::string NodeUnit::path() const
{
  return struct_.path;
}

std::string NodeUnit::type() const
{
  return struct_.type;
}

void NodeUnit::update(const rclcpp::Time & stamp)
{
  latch_->update(stamp, logic_->level());
}

}  // namespace autoware::diagnostic_graph_aggregator
