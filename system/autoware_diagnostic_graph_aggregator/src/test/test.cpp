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

#include "autoware/diagnostic_graph_aggregator/tests/tests.hpp"
#include "graph/graph.hpp"
#include "graph/units.hpp"

namespace autoware::diagnostic_graph_aggregator::test
{

TestOutput test(const TestInput & input)
{
  autoware::diagnostic_graph_aggregator::Graph graph;
  graph.create(input.graph_path);

  for (const auto & diag : input.diags.status) {
    graph.update(input.diags.header.stamp, diag);
  }

  TestOutput output;
  for (const auto & unit : graph.units()) {
    TestUnitStatus status;
    status.path = unit->path();
    status.level = unit->level();
    output.units.push_back(status);
  }
  return output;
}

}  // namespace autoware::diagnostic_graph_aggregator::test
