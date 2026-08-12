// Copyright 2026 The Autoware Contributors
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

#include "graph/graph.hpp"
#include "graph/nodes.hpp"
#include "tests/utils.hpp"

#include <rclcpp/clock.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_map>

using namespace autoware::diagnostic_graph_aggregator;  // NOLINT(build/namespaces)

DiagnosticStatus create_diag_status(const std::string & name, DiagnosticLevel level)
{
  DiagnosticStatus status;
  status.name = name;
  status.level = level;
  return status;
}

std::unordered_map<std::string, DiagnosticLevel> create_level_mapping(const Graph & graph)
{
  std::unordered_map<std::string, DiagnosticLevel> mapping;
  for (const auto & node : graph.nodes()) {
    mapping[node->path()] = node->level();
  }
  return mapping;
}

TEST(GraphOthers, OverrideOk)
{
  const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(create_diag_status("dummy: name0", DiagnosticStatus::OK));
  array.status.push_back(create_diag_status("dummy: name1", DiagnosticStatus::WARN));
  array.status.push_back(create_diag_status("dummy: name2", DiagnosticStatus::ERROR));

  Graph graph(resource("others/override.yaml"));
  EXPECT_EQ(graph.set_override("path0", DiagnosticStatus::OK), "");
  EXPECT_EQ(graph.set_override("path1", DiagnosticStatus::OK), "");
  EXPECT_EQ(graph.set_override("path2", DiagnosticStatus::OK), "");
  graph.update(stamp, array);
  graph.update(stamp);

  const auto mapping = create_level_mapping(graph);
  EXPECT_EQ(mapping.at("path0"), DiagnosticStatus::OK);
  EXPECT_EQ(mapping.at("path1"), DiagnosticStatus::OK);
  EXPECT_EQ(mapping.at("path2"), DiagnosticStatus::OK);
}

TEST(GraphOthers, OverrideWarn)
{
  const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(create_diag_status("dummy: name0", DiagnosticStatus::OK));
  array.status.push_back(create_diag_status("dummy: name1", DiagnosticStatus::WARN));
  array.status.push_back(create_diag_status("dummy: name2", DiagnosticStatus::ERROR));

  Graph graph(resource("others/override.yaml"));
  EXPECT_EQ(graph.set_override("path0", DiagnosticStatus::WARN), "");
  EXPECT_EQ(graph.set_override("path1", DiagnosticStatus::WARN), "");
  EXPECT_EQ(graph.set_override("path2", DiagnosticStatus::WARN), "");
  graph.update(stamp, array);
  graph.update(stamp);

  const auto mapping = create_level_mapping(graph);
  EXPECT_EQ(mapping.at("path0"), DiagnosticStatus::WARN);
  EXPECT_EQ(mapping.at("path1"), DiagnosticStatus::WARN);
  EXPECT_EQ(mapping.at("path2"), DiagnosticStatus::WARN);
}

TEST(GraphOthers, OverrideError)
{
  const auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  DiagnosticArray array;
  array.header.stamp = stamp;
  array.status.push_back(create_diag_status("dummy: name0", DiagnosticStatus::OK));
  array.status.push_back(create_diag_status("dummy: name1", DiagnosticStatus::WARN));
  array.status.push_back(create_diag_status("dummy: name2", DiagnosticStatus::ERROR));

  Graph graph(resource("others/override.yaml"));
  EXPECT_EQ(graph.set_override("path0", DiagnosticStatus::ERROR), "");
  EXPECT_EQ(graph.set_override("path1", DiagnosticStatus::ERROR), "");
  EXPECT_EQ(graph.set_override("path2", DiagnosticStatus::ERROR), "");
  graph.update(stamp, array);
  graph.update(stamp);

  const auto mapping = create_level_mapping(graph);
  EXPECT_EQ(mapping.at("path0"), DiagnosticStatus::ERROR);
  EXPECT_EQ(mapping.at("path1"), DiagnosticStatus::ERROR);
  EXPECT_EQ(mapping.at("path2"), DiagnosticStatus::ERROR);
}

TEST(GraphOthers, OverrideNotAllowed)
{
  const auto message = "override not allowed for the target path";
  Graph graph(resource("others/override.yaml"));
  EXPECT_EQ(graph.set_override("path3", DiagnosticStatus::OK), message);
}

TEST(GraphOthers, OverrideNotFound)
{
  const auto message = "path not found";
  Graph graph(resource("others/override.yaml"));
  EXPECT_EQ(graph.set_override("path4", DiagnosticStatus::OK), message);
}
