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

#include "timeline.hpp"

#include "graph/graph.hpp"
#include "types/diagnostics.hpp"

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

TimelineTest::TimelineTest()
{
  interval_ = 0.1;
}

void TimelineTest::execute(const std::string & path)
{
  // clang-format off
  const auto letter_to_level = [](const char letter) -> DiagnosticLevel {
    switch (letter) {
      case 'K': return DiagnosticStatus::OK;
      case 'W': return DiagnosticStatus::WARN;
      case 'E': return DiagnosticStatus::ERROR;
      case 'S': return DiagnosticStatus::STALE;
    }
    // clang-format on
    throw std::runtime_error("Invalid level letter: " + std::to_string(letter));
  };
  const auto level_to_letter = [](const DiagnosticLevel level) -> char {
    // clang-format off
    switch (level) {
      case DiagnosticStatus::OK:    return 'K';
      case DiagnosticStatus::WARN:  return 'W';
      case DiagnosticStatus::ERROR: return 'E';
      case DiagnosticStatus::STALE: return 'S';
    }
    // clang-format on
    return '?';
  };

  // Create input sequence of diagnostic status.
  std::vector<std::vector<DiagnosticStatus>> diagnostic_array_sequence;
  {
    std::vector<std::unordered_map<std::string, DiagnosticLevel>> diagnostic_map_sequence;
    for (const auto & [name, levels] : input_) {
      if (diagnostic_map_sequence.size() < levels.size()) {
        diagnostic_map_sequence.resize(levels.size());
      }
      for (size_t i = 0; i < levels.size(); ++i) {
        if (levels[i] == '-') continue;
        diagnostic_map_sequence[i][name] = letter_to_level(levels[i]);
      }
    }
    for (const auto & map : diagnostic_map_sequence) {
      std::vector<DiagnosticStatus> array;
      for (const auto & [name, level] : map) {
        DiagnosticStatus status;
        status.name = name;
        status.level = level;
        array.push_back(status);
      }
      diagnostic_array_sequence.push_back(array);
    }
  }

  // Create graph.
  auto stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  auto graph = Graph(path);
  graph.reset();

  // Create result sequence.
  const auto structure = graph.create_struct_msg(stamp);
  std::vector<std::string> result_sequence(structure.nodes.size());
  for (size_t step = 0; step < diagnostic_array_sequence.size(); ++step) {
    if (reset_steps_.count(step)) {
      graph.reset();
    }
    if (initializing_steps_.count(step)) {
      graph.set_initializing(initializing_steps_.at(step));
    }

    DiagnosticArray array;
    array.header.stamp = stamp;
    array.status = diagnostic_array_sequence[step];
    graph.update(stamp, array);
    graph.update(stamp);

    const auto msg = graph.create_status_msg(stamp);
    for (size_t i = 0; i < msg.nodes.size(); ++i) {
      result_sequence[i].push_back(level_to_letter(msg.nodes[i].level));
    }
    stamp += rclcpp::Duration::from_seconds(interval_);
  }
  for (size_t i = 0; i < structure.nodes.size(); ++i) {
    output_[structure.nodes[i].path] = result_sequence[i];
  }
}

void TimelineTest::set_interval(double interval)
{
  interval_ = interval;
}

void TimelineTest::set_reset(const std::unordered_set<size_t> & steps)
{
  reset_steps_ = steps;
}

void TimelineTest::set_initializing(const std::unordered_map<size_t, bool> & steps)
{
  initializing_steps_ = steps;
}

void TimelineTest::set(const std::string & name, const std::string & levels)
{
  input_[name] = levels;
}

std::string TimelineTest::get(const std::string & path)
{
  return output_[path];
}

}  // namespace autoware::diagnostic_graph_aggregator
