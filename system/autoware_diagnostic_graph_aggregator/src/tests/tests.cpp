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

#include "graph/diags.hpp"
#include "graph/graph.hpp"
#include "graph/nodes.hpp"

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator::test
{

struct TestGraph::Impl
{
  explicit Impl(const std::string & path) : graph(path) {}
  autoware::diagnostic_graph_aggregator::Graph graph;
};

TestGraph::TestGraph(const std::string & path)
{
  impl_ = std::make_unique<Impl>(path);
}

TestGraph::~TestGraph()
{
  impl_.reset();
}

std::vector<TestNodeStatus> TestGraph::test(const DiagnosticArray & diags)
{
  auto & graph = impl_->graph;
  graph.update(diags.header.stamp, diags);
  graph.update(diags.header.stamp);

  std::vector<TestNodeStatus> result;
  for (const auto & node : graph.nodes()) {
    TestNodeStatus status;
    status.path = node->path();
    status.level = node->level();
    result.push_back(status);
  }
  return result;
}

std::vector<std::string> TestGraph::list_diag_names() const
{
  const auto & graph = impl_->graph;
  std::vector<std::string> result;
  for (const auto & diag : graph.diags()) {
    result.push_back(diag->name());
  }
  return result;
}

}  // namespace autoware::diagnostic_graph_aggregator::test
