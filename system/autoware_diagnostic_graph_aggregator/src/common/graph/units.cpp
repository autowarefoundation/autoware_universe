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

#include "graph/units.hpp"

#include "config/entity.hpp"
#include "graph/links.hpp"
#include "graph/nodes.hpp"
#include "utils/memory.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

void BaseUnit::finalize(int index, std::vector<BaseUnit *> parents)
{
  index_ = index;
  parents_ = parents;
}

std::vector<BaseUnit *> BaseUnit::child_units() const
{
  std::vector<BaseUnit *> result;
  for (const auto & port : ports()) {
    for (const auto & unit : port->iterate()) {
      result.push_back(unit);
    }
  }
  return result;
}

std::vector<NodeUnit *> BaseUnit::child_nodes() const
{
  return filter<NodeUnit>(child_units());
}

std::vector<BaseUnit *> BaseUnit::parent_units() const
{
  return parents_;
}

LinkUnit::LinkUnit(ConfigYaml yaml)
{
  path_ = yaml.optional("path").text("");
  link_ = yaml.required("link").text("");
}

}  // namespace autoware::diagnostic_graph_aggregator
