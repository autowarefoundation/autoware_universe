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

#include "graph/graph.hpp"
#include "tests/timeline.hpp"
#include "tests/utils.hpp"
#include "types/diagnostics.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

#include <string>

bool match(const std::string & target, const std::string & expect)
{
  if (target.size() != expect.size()) {
    return false;
  }
  for (size_t i = 0; i < expect.size(); ++i) {
    if (expect[i] == '-') {
      continue;
    }
    if (target[i] != expect[i]) {
      return false;
    }
  }
  return true;
}

TEST(GraphLevel, Timeout)
{
  // clang-format off
  const auto input      = "KKKKKKKKKK--------------------";
  const auto result_0_5 = "KKKKKKKKKKKKKKEEEEEEEEEEEEEEEE";
  const auto result_1_0 = "KKKKKKKKKKKKKKKKKKKEEEEEEEEEEE";
  const auto result_1_5 = "KKKKKKKKKKKKKKKKKKKKKKKKEEEEEE";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/timeout.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result_1_0));
  EXPECT_TRUE(match(test.get("path1"), result_0_5));
  EXPECT_TRUE(match(test.get("path2"), result_1_0));
  EXPECT_TRUE(match(test.get("path3"), result_1_5));
}

TEST(GraphLevel, Latch1)
{
  // clang-format off
  const auto input      = "KKKKKEKKEEEKKEEEEEKKKKK";
  const auto result_off = "KKKKKEKKEEEKKEEEEEKKKKK";
  const auto result_0_0 = "KKKKKEEEEEEEEEEEEEEEEEE";
  const auto result_0_2 = "KKKKKEKKEEEEEEEEEEEEEEE";
  const auto result_0_4 = "KKKKKEKKEEEKKEEEEEEEEEE";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result_off));
  EXPECT_TRUE(match(test.get("path1"), result_0_0));
  EXPECT_TRUE(match(test.get("path2"), result_0_2));
  EXPECT_TRUE(match(test.get("path3"), result_0_4));
}

TEST(GraphLevel, Latch2)
{
  // clang-format off
  const auto input  = "KKKKKEKKKKKKKKKKKKKK";
  const auto result = "KKKKKEEEEEKKKKKKKKKK";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({10});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, Latch3)
{
  // clang-format off
  const auto input  = "KKKKKEKKKKKKKKKKKKKK";
  const auto result = "KKKKKEEEEEEEEEEKKKKK";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({15});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, Latch4)
{
  // clang-format off
  const auto input  = "KKKKKEEEEEEEEEEEEEEE";
  const auto result = "KKKKKEEEEEEEEEEEEEEE";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({10});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, Latch5)
{
  // clang-format off
  const auto input  = "KKKKKEWWWWWWWWWKKKKK";
  const auto result = "KKKKKEEEEEWWWWWWWWWW";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({10});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, Hysteresis1)
{
  // clang-format off
  const auto input      = "KKKKKKKKKKKKKKKEEEEEEEEEE";
  const auto result_0_0 = "----------KKKKKEEEEEEEEEE";
  const auto result_0_2 = "----------KKKKKKKEEEEEEEE";
  const auto result_0_4 = "----------KKKKKKKKKEEEEEE";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/hysteresis.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result_0_0));
  EXPECT_TRUE(match(test.get("path1"), result_0_0));
  EXPECT_TRUE(match(test.get("path2"), result_0_2));
  EXPECT_TRUE(match(test.get("path3"), result_0_4));
}

TEST(GraphLevel, Hysteresis2)
{
  // clang-format off
  const auto input      = "EEEEEEEEEEEEEEEKKKKKKKKKK";
  const auto result_0_0 = "----------EEEEEKKKKKKKKKK";
  const auto result_0_2 = "----------EEEEEEEKKKKKKKK";
  const auto result_0_4 = "----------EEEEEEEEEKKKKKK";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/hysteresis.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result_0_0));
  EXPECT_TRUE(match(test.get("path1"), result_0_0));
  EXPECT_TRUE(match(test.get("path2"), result_0_2));
  EXPECT_TRUE(match(test.get("path3"), result_0_4));
}

TEST(GraphLevel, Hysteresis3)
{
  // clang-format off
  const auto input      = "KKKKKKKKKKKKKKKEKKKKKEEEKKKKKEEEEEKKKKK";
  const auto result_0_0 = "----------KKKKKEKKKKKEEEKKKKKEEEEEKKKKK";
  const auto result_0_2 = "----------KKKKKKKKKKKKKEEEKKKKKEEEEEKKK";
  const auto result_0_4 = "----------KKKKKKKKKKKKKKKKKKKKKKKEEEEEK";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/hysteresis.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result_0_0));
  EXPECT_TRUE(match(test.get("path1"), result_0_0));
  EXPECT_TRUE(match(test.get("path2"), result_0_2));
  EXPECT_TRUE(match(test.get("path3"), result_0_4));
}

TEST(GraphLevel, Hysteresis4)
{
  // clang-format off
  const auto input      = "KKKKKKKKKKKKKKKEWEWEEEWEEEWEEEEEWEEEEEWEEEEE";
  const auto result_0_0 = "----------KKKKKEWEWEEEWEEEWEEEEEWEEEEEWEEEEE";
  const auto result_0_2 = "----------KKKKKKKWWWWEEEEEEEEEEEEEEEEEEEEEEE";
  const auto result_0_4 = "----------KKKKKKKKKWWWWWWWWWWWWEEEEEEEEEEEEE";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/hysteresis.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result_0_0));
  EXPECT_TRUE(match(test.get("path1"), result_0_0));
  EXPECT_TRUE(match(test.get("path2"), result_0_2));
  EXPECT_TRUE(match(test.get("path3"), result_0_4));
}

TEST(GraphLevel, Combination)
{
  // clang-format off
  const auto input  = "KKKKKKKKKKKKKKKEEEEEEKKKKEKKKKEKKKK";
  const auto result = "----------KKKKKKKKKKEEEEEEEEEEEEEEE";
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({5, 23});
  test.set("dummy: name0", input);
  test.execute(resource("levels/combination.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result));
}
