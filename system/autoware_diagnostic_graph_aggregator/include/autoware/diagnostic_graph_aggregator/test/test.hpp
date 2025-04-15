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

#ifndef TEST_HPP_
#define TEST_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator::test
{

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticLevel = diagnostic_msgs::msg::DiagnosticStatus::_level_type;

struct TestData
{
  std::string graph_path;
  DiagnosticArray diags;
};

struct TestUnit
{
  std::string path;
  DiagnosticLevel level;
};

struct TestResult
{
  std::vector<TestUnit> units;
};

TestResult test(const TestData & data);

}  // namespace autoware::diagnostic_graph_aggregator::test

#endif  // TEST_HPP_
