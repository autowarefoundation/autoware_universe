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

#include "config/errors.hpp"
#include "graph/graph.hpp"
#include "graph/nodes.hpp"
#include "tests/utils.hpp"

#include <gmock/gmock.h>

#include <string>
#include <vector>

using namespace autoware::diagnostic_graph_aggregator;  // NOLINT(build/namespaces)

struct Param
{
  std::string config;
  std::vector<std::string> expected;
};

class RemoveEditTest : public testing::TestWithParam<Param>
{
};

auto static get_node_paths(const Graph & graph)
{
  std::vector<std::string> paths;
  for (auto & node : graph.nodes()) {
    paths.push_back(node->path());
  }
  return paths;
}

TEST_P(RemoveEditTest, ComparePaths)
{
  auto param = GetParam();
  auto graph = Graph(resource(param.config));
  auto received_paths = get_node_paths(graph);
  EXPECT_THAT(received_paths, testing::UnorderedElementsAreArray(param.expected));
}

// clang-format off
INSTANTIATE_TEST_SUITE_P(
  RemoveEditTestcases, RemoveEditTest,
  testing::Values(
    Param{"remove/include-all.yaml", {"/autoware/foo", "/autoware/foobar", "/autoware/bar"}},
    Param{"remove/remove-one-path.yaml", {"/autoware/foo", "/autoware/bar"}},
    Param{"remove/remove-two-paths.yaml", {"/autoware/foobar"}},
    Param{"remove/remove-foo.yaml", {"/autoware/bar"}},
    Param{"remove/remove-all.yaml", {}}
  )
);
// clang-format on
