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

#include "graph/links.hpp"

#include "graph/units.hpp"
#include "utils/memory.hpp"

#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

bool LinkPort::empty() const
{
  return units_.empty();
}

std::vector<BaseUnit *> LinkPort::iterate() const
{
  return units_;
}

LinkItem::LinkItem(BaseUnit * unit)
{
  units_.push_back(unit);
}

DiagnosticLevel LinkItem::level() const
{
  return units_.front()->level();
}

LinkList::LinkList(const std::vector<BaseUnit *> & units)
{
  units_ = units;
}

std::vector<DiagnosticLevel> LinkList::levels() const
{
  std::vector<DiagnosticLevel> levels;
  for (const auto & unit : units_) levels.push_back(unit->level());
  return levels;
}

}  // namespace autoware::diagnostic_graph_aggregator
