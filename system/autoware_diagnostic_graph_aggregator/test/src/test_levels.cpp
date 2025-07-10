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
  const auto input      = "KKKKKKKKKK--------------------";  // cspell:disable-line
  const auto result_0_5 = "KKKKKKKKKKKKKKEEEEEEEEEEEEEEEE";  // cspell:disable-line
  const auto result_1_0 = "KKKKKKKKKKKKKKKKKKKEEEEEEEEEEE";  // cspell:disable-line
  const auto result_1_5 = "KKKKKKKKKKKKKKKKKKKKKKKKEEEEEE";  // cspell:disable-line
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

TEST(GraphLevel, Latch)
{
  // clang-format off
  const auto input      = "KKKKKEKKEEEKKEEEEEKKKKK";  // cspell:disable-line
  const auto result_off = "KKKKKEKKEEEKKEEEEEKKKKK";  // cspell:disable-line
  const auto result_0_0 = "KKKKKEEEEEEEEEEEEEEEEEE";  // cspell:disable-line
  const auto result_0_2 = "KKKKKEKKEEEEEEEEEEEEEEE";  // cspell:disable-line
  const auto result_0_4 = "KKKKKEKKEEEKKEEEEEEEEEE";  // cspell:disable-line
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

TEST(GraphLevel, LatchInitializing)
{
  // clang-format off
  const auto input  = "KKKKKEKKEEEKKEEEEEKKKKK";  // cspell:disable-line
  const auto result = "KKKKKEKKEEEKKEEEEEKKKKK";  // cspell:disable-line
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_initializing({{0, true}});
  test.set("dummy: name0", input);
  test.set("dummy: name1", input);
  test.set("dummy: name2", input);
  test.set("dummy: name3", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result));
  EXPECT_TRUE(match(test.get("path1"), result));
  EXPECT_TRUE(match(test.get("path2"), result));
  EXPECT_TRUE(match(test.get("path3"), result));
}

TEST(GraphLevel, LatchReset10ms)
{
  // clang-format off
  const auto input  = "KKKKKEKKKKKKKKKKKKKK";  // cspell:disable-line
  const auto result = "KKKKKEEEEEKKKKKKKKKK";  // cspell:disable-line
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({10});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, LatchReset15ms)
{
  // clang-format off
  const auto input  = "KKKKKEKKKKKKKKKKKKKK";  // cspell:disable-line
  const auto result = "KKKKKEEEEEEEEEEKKKKK";  // cspell:disable-line
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({15});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, LatchResetOnError)
{
  // clang-format off
  const auto input  = "KKKKKEEEEEEEEEEEEEEE";  // cspell:disable-line
  const auto result = "KKKKKEEEEEEEEEEEEEEE";  // cspell:disable-line
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({10});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, LatchResetOnWarn)
{
  // clang-format off
  const auto input  = "KKKKKEWWWWWWWWWKKKKK";  // cspell:disable-line
  const auto result = "KKKKKEEEEEWWWWWWWWWW";  // cspell:disable-line
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({10});
  test.set("dummy: name1", input);
  test.execute(resource("levels/latch.yaml"));

  EXPECT_TRUE(match(test.get("path1"), result));
}

TEST(GraphLevel, HysteresisOkeyToError)
{
  // clang-format off
  const auto input      = "KKKKKKKKKKKKKKKEEEEEEEEEE";  // cspell:disable-line
  const auto result_0_0 = "----------KKKKKEEEEEEEEEE";  // cspell:disable-line
  const auto result_0_2 = "----------KKKKKKKEEEEEEEE";  // cspell:disable-line
  const auto result_0_4 = "----------KKKKKKKKKEEEEEE";  // cspell:disable-line
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

TEST(GraphLevel, HysteresisErrorToOkey)
{
  // clang-format off
  const auto input      = "EEEEEEEEEEEEEEEKKKKKKKKKK";  // cspell:disable-line
  const auto result_0_0 = "----------EEEEEKKKKKKKKKK";  // cspell:disable-line
  const auto result_0_2 = "----------EEEEEEEKKKKKKKK";  // cspell:disable-line
  const auto result_0_4 = "----------EEEEEEEEEKKKKKK";  // cspell:disable-line
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

TEST(GraphLevel, HysteresisPulseError)
{
  // clang-format off
  const auto input      = "KKKKKKKKKKKKKKKEKKKKKEEEKKKKKEEEEEKKKKK";  // cspell:disable-line
  const auto result_0_0 = "----------KKKKKEKKKKKEEEKKKKKEEEEEKKKKK";  // cspell:disable-line
  const auto result_0_2 = "----------KKKKKKKKKKKKKEEEKKKKKEEEEEKKK";  // cspell:disable-line
  const auto result_0_4 = "----------KKKKKKKKKKKKKKKKKKKKKKKEEEEEK";  // cspell:disable-line
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

TEST(GraphLevel, HysteresisPulseWarnError)
{
  // clang-format off
  const auto input      = "KKKKKKKKKKKKKKKEWEWEEEWEEEWEEEEEWEEEEEWEEEEE";  // cspell:disable-line
  const auto result_0_0 = "----------KKKKKEWEWEEEWEEEWEEEEEWEEEEEWEEEEE";  // cspell:disable-line
  const auto result_0_2 = "----------KKKKKKKWWWWEEEEEEEEEEEEEEEEEEEEEEE";  // cspell:disable-line
  const auto result_0_4 = "----------KKKKKKKKKWWWWWWWWWWWWEEEEEEEEEEEEE";  // cspell:disable-line
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
  const auto input  = "KKKKKKKKKKKKKKKEEEEEEKKKKEKKKKEKKKK";  // cspell:disable-line
  const auto result = "----------KKKKKKKKKKEEEEEEEEEEEEEEE";  // cspell:disable-line
  // clang-format on

  autoware::diagnostic_graph_aggregator::TimelineTest test;
  test.set_interval(0.1);
  test.set_reset({5, 23});
  test.set("dummy: name0", input);
  test.execute(resource("levels/combination.yaml"));

  EXPECT_TRUE(match(test.get("path0"), result));
}
