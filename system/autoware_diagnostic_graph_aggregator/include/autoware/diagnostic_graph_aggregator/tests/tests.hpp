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

#ifndef AUTOWARE__DIAGNOSTIC_GRAPH_AGGREGATOR__TESTS__TESTS_HPP_
#define AUTOWARE__DIAGNOSTIC_GRAPH_AGGREGATOR__TESTS__TESTS_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator::test
{

using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticLevel = diagnostic_msgs::msg::DiagnosticStatus::_level_type;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

struct TestNodeStatus
{
  std::string path;
  DiagnosticLevel level;
};

class TestGraph
{
public:
  explicit TestGraph(const std::string & path);
  ~TestGraph();
  std::vector<TestNodeStatus> test(const DiagnosticArray & diags);
  std::vector<std::string> list_diag_names() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::diagnostic_graph_aggregator::test

#endif  // AUTOWARE__DIAGNOSTIC_GRAPH_AGGREGATOR__TESTS__TESTS_HPP_
