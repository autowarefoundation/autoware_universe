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

#include "logics/and.hpp"

#include "config/entity.hpp"
#include "graph/links.hpp"

#include <algorithm>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

AndLogic::AndLogic(Parser & parser)
{
  links_ = parser.parse_node_list(parser.yaml().optional("list").list());
}

std::vector<LinkPort *> AndLogic::ports() const
{
  return {links_};
}

DiagnosticLevel AndLogic::level() const
{
  if (links_->empty()) {
    return DiagnosticStatus::OK;
  }

  DiagnosticLevel result = DiagnosticStatus::OK;
  for (const auto & level : links_->levels()) {
    result = std::max(result, level);
  }
  return result;
}

}  // namespace autoware::diagnostic_graph_aggregator

namespace
{

namespace ns = autoware::diagnostic_graph_aggregator;
ns::RegisterLogic<ns::AndLogic> registration1("and");
ns::RegisterLogic<ns::AndLogic> registration2("short-circuit-and");

}  // namespace
