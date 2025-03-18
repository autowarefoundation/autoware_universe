// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/datatype.hpp"
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/processing/preprocessor.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <iostream>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace autoware::simpl
{
namespace
{
std::ostream & operator<<(std::ostream & os, const archetype::AgentTensor & tensor)
{
  os << "AgentTensor("
     << "N=" << tensor.num_agent << ", T=" << tensor.num_past << ", Da=" << tensor.num_attribute
     << ") size=" << tensor.size() << '\n';

  const float * data = tensor.data();
  // Indexing: data[(n * T * Da) + (t * Da) + da]
  for (size_t n = 0; n < tensor.num_agent; ++n) {
    os << "  [Agent " << n << "]\n";
    for (size_t t = 0; t < tensor.num_past; ++t) {
      os << "    [Time " << t << "]:";
      for (size_t da = 0; da < tensor.num_attribute; ++da) {
        const size_t index =
          (n * (tensor.num_past * tensor.num_attribute)) + (t * tensor.num_attribute) + da;
        os << ' ' << data[index];
      }
      os << '\n';
    }
  }
  return os;
}

std::ostream & operator<<(std::ostream & os, const archetype::MapTensor & tensor)
{
  // The constructor checks size == K * (P - 1) * Dm,
  // so effectively we treat it as shape (K, P-1, Dm).
  os << "MapTensor("
     << "K=" << tensor.num_polyline << ", P=" << tensor.num_point << ", Dm=" << tensor.num_attribute
     << ") size=" << tensor.size() << '\n';

  const float * data = tensor.data();
  // Indexing: data[(k * ((P - 1) * Dm)) + (p * Dm) + d]
  for (size_t k = 0; k < tensor.num_polyline; ++k) {
    os << "  [Polyline " << k << "]\n";
    for (size_t p = 0; p < (tensor.num_point - 1); ++p) {
      os << "    [Point " << p << "]:";
      for (size_t d = 0; d < tensor.num_attribute; ++d) {
        const size_t index =
          k * ((tensor.num_point - 1) * tensor.num_attribute) + p * tensor.num_attribute + d;
        os << ' ' << data[index];
      }
      os << '\n';
    }
  }
  return os;
}
}  // namespace

TEST(TestPreProcessor, testPreProcessor)
{
  std::vector<size_t> label_ids{0};  // == [VEHICLE]
  constexpr size_t num_past = 3;

  // histories
  std::vector<double> timestamps{1.0, 2.0, 3.0};  // .size() == num_past
  const std::map<std::string, archetype::AgentHistory> histories{
    // history of agent1
    {
      "agent1",
      archetype::AgentHistory(
        num_past, timestamps,
        {archetype::AgentState{
           1.0, 1.0, 1.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true},
         archetype::AgentState{
           2.0, 2.0, 2.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true},
         archetype::AgentState{
           3.0, 3.0, 3.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true}}),
    },
    {
      // history of agent2
      "agent2",
      archetype::AgentHistory(
        num_past, timestamps,
        {archetype::AgentState{
           4.0, 4.0, 4.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true},
         archetype::AgentState{
           5.0, 5.0, 5.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true},
         archetype::AgentState{
           3.0, 3.0, 3.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true}}),
    }};

  // map points
  const archetype::MapPoints map_points{
    archetype::MapPoint(0.0, 1.0, 2.0, 0.0, 0.0, 0.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(1.0, 2.0, 3.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(2.0, 3.0, 4.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(3.0, 4.0, 5.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(4.0, 5.0, 6.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY),
    archetype::MapPoint(5.0, 6.0, 7.0, 1.0, 1.0, 1.0, archetype::MapLabel::ROADWAY)};

  // current ego
  const archetype::AgentState current_ego(
    1.0, 1.0, 1.0, 0.5 * M_PI, 1.0, 1.0, archetype::AgentLabel::VEHICLE, true);

  constexpr size_t max_num_agent = 3;
  constexpr size_t max_num_polyline = 3;
  constexpr size_t max_num_point = 3;
  constexpr double break_distance = 5.0;
  constexpr size_t num_rpe = max_num_agent + max_num_polyline;

  // preprocessor
  processing::PreProcessor processor(
    label_ids, max_num_agent, num_past, max_num_polyline, max_num_point, break_distance);

  const auto [agent_metadata, map_metadata, rpe_tensor] =
    processor.process(histories, map_points, current_ego);

  EXPECT_EQ(
    agent_metadata.tensor.size(), max_num_agent * num_past * agent_metadata.tensor.num_attribute);
  EXPECT_EQ(
    map_metadata.tensor.size(),
    max_num_polyline * (max_num_point - 1) * map_metadata.tensor.num_attribute);
  EXPECT_EQ(rpe_tensor.size(), num_rpe * num_rpe * 5);

  // Debug print
  std::cout << agent_metadata.tensor << std::endl;
  std::cout << map_metadata.tensor << std::endl;
}
}  // namespace autoware::simpl
