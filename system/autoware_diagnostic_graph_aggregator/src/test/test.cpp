// Copyright 2025 The Autoware Contributors
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

#include "autoware/diagnostic_graph_aggregator/test/test.hpp"

#include "graph/graph.hpp"
#include "graph/units.hpp"

namespace autoware::diagnostic_graph_aggregator::test
{

TestResult test(const TestData & data)
{
  autoware::diagnostic_graph_aggregator::Graph graph;
  graph.create(data.graph_path);

  for (const auto & diag : data.diags.status) {
    graph.update(data.diags.header.stamp, diag);
  }

  TestResult result;
  for (const auto & unit : graph.units()) {
    TestUnit u;
    u.path = unit->path();
    u.level = unit->level();
    result.units.push_back(u);
  }
  return result;
}

}  // namespace autoware::diagnostic_graph_aggregator::test
