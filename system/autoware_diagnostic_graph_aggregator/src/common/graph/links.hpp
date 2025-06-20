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

#ifndef COMMON__GRAPH__LINKS_HPP_
#define COMMON__GRAPH__LINKS_HPP_

#include "config/yaml.hpp"
#include "types/diagnostics.hpp"
#include "types/forward.hpp"

#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class LinkPort
{
public:
  virtual ~LinkPort() = default;
  bool empty() const;
  std::vector<BaseUnit *> iterate() const;

  // TODO(Takagi, Isamu): protected
public:
  std::vector<BaseUnit *> units_;
};

class LinkItem : public LinkPort
{
public:
  explicit LinkItem(BaseUnit * unit);
  DiagnosticLevel level() const;
};

class LinkList : public LinkPort
{
public:
  explicit LinkList(const std::vector<BaseUnit *> & units);
  std::vector<DiagnosticLevel> levels() const;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LINKS_HPP_
