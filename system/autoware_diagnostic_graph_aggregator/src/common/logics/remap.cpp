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

#include "logics/remap.hpp"

#include "config/entity.hpp"
#include "graph/links.hpp"

#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

RemapLogic::RemapLogic(Parser & parser)
{
  link_ = parser.parse_node_item(parser.yaml().required("item"));
}

std::vector<LinkPort *> RemapLogic::ports() const
{
  return {link_};
}

DiagnosticLevel WarnToOkLogic::level() const
{
  if (link_->level() == DiagnosticStatus::WARN) {
    return DiagnosticStatus::OK;
  } else {
    return link_->level();
  }
}

DiagnosticLevel WarnToErrorLogic::level() const
{
  if (link_->level() == DiagnosticStatus::WARN) {
    return DiagnosticStatus::ERROR;
  } else {
    return link_->level();
  }
}

}  // namespace autoware::diagnostic_graph_aggregator

namespace
{

namespace ns = autoware::diagnostic_graph_aggregator;
ns::RegisterLogic<ns::WarnToOkLogic> registration1("warn-to-ok");
ns::RegisterLogic<ns::WarnToErrorLogic> registration2("warn-to-error");

}  // namespace
