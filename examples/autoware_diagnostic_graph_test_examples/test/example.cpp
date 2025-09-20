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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/diagnostic_graph_aggregator/tests/tests.hpp>

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

using autoware::diagnostic_graph_aggregator::test::DiagnosticArray;
using autoware::diagnostic_graph_aggregator::test::DiagnosticLevel;
using autoware::diagnostic_graph_aggregator::test::DiagnosticStatus;
using autoware::diagnostic_graph_aggregator::test::TestGraph;
using autoware::diagnostic_graph_aggregator::test::TestNodeStatus;

std::unordered_map<std::string, DiagnosticLevel> test(
  TestGraph & graph, const std::unordered_map<std::string, DiagnosticLevel> & input)
{
  DiagnosticArray diags;
  for (const auto & name : graph.list_diag_names()) {
    DiagnosticStatus status;
    status.name = name;
    status.level = input.count(name) ? input.at(name) : DiagnosticStatus::OK;
    diags.status.push_back(status);
  }

  const auto result = graph.test(diags);
  std::unordered_map<std::string, DiagnosticLevel> output;
  for (const auto & status : result) {
    output[status.path] = status.level;
  }
  return output;
}

TEST(TestDiagGraph, TestLocalGraph)
{
  const auto test_data_path = std::string(TEST_DATA_PATH) + "/";
  TestGraph graph(test_data_path + "graph.yaml");

  std::unordered_map<std::string, DiagnosticLevel> input;
  input["test_node: input_2"] = DiagnosticStatus::ERROR;
  input["test_node: input_4"] = DiagnosticStatus::ERROR;

  const auto output = test(graph, input);
  EXPECT_EQ(output.at("/unit/1"), DiagnosticStatus::ERROR);
  EXPECT_EQ(output.at("/unit/2"), DiagnosticStatus::OK);
}

TEST(TestDiagGraph, TestSharedGraph)
{
  const auto package_name = "autoware_diagnostic_graph_test_examples";
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  const auto test_data_path = package_path + "/data/";
  TestGraph graph(test_data_path + "graph.yaml");

  std::unordered_map<std::string, DiagnosticLevel> input;
  input["test_node: input_2"] = DiagnosticStatus::ERROR;
  input["test_node: input_4"] = DiagnosticStatus::ERROR;

  const auto output = test(graph, input);
  EXPECT_EQ(output.at("/unit/1"), DiagnosticStatus::ERROR);
  EXPECT_EQ(output.at("/unit/2"), DiagnosticStatus::OK);
}
