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

#include "logics/diag.hpp"

#include "config/entity.hpp"
#include "graph/links.hpp"

#include <algorithm>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

DiagLogic::DiagLogic(Parser & parser)
{
  link_ = parser.parse_diag_item(parser.yaml());
}

std::vector<LinkPort *> DiagLogic::ports() const
{
  return {link_};
}

DiagnosticLevel DiagLogic::level() const
{
  // STALE to ERROR
  return std::min(link_->level(), DiagnosticStatus::ERROR);
}

}  // namespace autoware::diagnostic_graph_aggregator

namespace
{

namespace ns = autoware::diagnostic_graph_aggregator;
ns::RegisterLogic<ns::DiagLogic> registration("diag");

}  // namespace
