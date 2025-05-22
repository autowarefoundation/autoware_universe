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

using autoware::diagnostic_graph_aggregator::test::DiagnosticLevel;
using autoware::diagnostic_graph_aggregator::test::DiagnosticStatus;
using autoware::diagnostic_graph_aggregator::test::test;
using autoware::diagnostic_graph_aggregator::test::TestInput;
using autoware::diagnostic_graph_aggregator::test::TestOutput;

DiagnosticStatus create_status(const std::string & name, const DiagnosticLevel level)
{
  DiagnosticStatus status;
  status.name = name;
  status.level = level;
  return status;
}

TEST(TestDiagGraph, TestLocalGraph)
{
  const auto test_data_path = std::string(TEST_DATA_PATH) + "/";

  TestInput input;
  input.graph_path = test_data_path + "graph.yaml";
  input.diags.status.push_back(create_status("test_node: input_1", DiagnosticStatus::OK));
  input.diags.status.push_back(create_status("test_node: input_2", DiagnosticStatus::ERROR));
  input.diags.status.push_back(create_status("test_node: input_3", DiagnosticStatus::OK));
  input.diags.status.push_back(create_status("test_node: input_4", DiagnosticStatus::ERROR));

  TestOutput output = test(input);
  std::unordered_map<std::string, DiagnosticLevel> units;
  for (const auto & unit : output.units) {
    units[unit.path] = unit.level;
  }

  EXPECT_EQ(units.at("/unit/1"), DiagnosticStatus::ERROR);
  EXPECT_EQ(units.at("/unit/2"), DiagnosticStatus::OK);
}

TEST(TestDiagGraph, TestSharedGraph)
{
  const auto package_name = "autoware_diagnostic_graph_test_examples";
  const auto package_path = ament_index_cpp::get_package_share_directory(package_name);
  const auto test_data_path = package_path + "/data/";

  TestInput input;
  input.graph_path = test_data_path + "graph.yaml";
  input.diags.status.push_back(create_status("test_node: input_1", DiagnosticStatus::OK));
  input.diags.status.push_back(create_status("test_node: input_2", DiagnosticStatus::ERROR));
  input.diags.status.push_back(create_status("test_node: input_3", DiagnosticStatus::OK));
  input.diags.status.push_back(create_status("test_node: input_4", DiagnosticStatus::ERROR));

  TestOutput output = test(input);
  std::unordered_map<std::string, DiagnosticLevel> units;
  for (const auto & unit : output.units) {
    units[unit.path] = unit.level;
  }

  EXPECT_EQ(units.at("/unit/1"), DiagnosticStatus::ERROR);
  EXPECT_EQ(units.at("/unit/2"), DiagnosticStatus::OK);
}
