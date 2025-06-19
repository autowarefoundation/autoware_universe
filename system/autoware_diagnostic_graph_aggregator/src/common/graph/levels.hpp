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

#ifndef COMMON__GRAPH__LEVELS_HPP_
#define COMMON__GRAPH__LEVELS_HPP_

#include "types/diagnostics.hpp"
#include "types/forward.hpp"

#include <rclcpp/time.hpp>

#include <optional>
#include <unordered_map>

namespace autoware::diagnostic_graph_aggregator
{

class LatchLevel
{
public:
  explicit LatchLevel(ConfigYaml yaml);
  void reset();
  void update(const rclcpp::Time & stamp, DiagnosticLevel level);
  DiagnosticLevel level() const;
  DiagnosticLevel input_level() const;
  DiagnosticLevel latch_level() const;

private:
  void update_latch_status(const rclcpp::Time & stamp, DiagnosticLevel level);

  bool latch_enabled_;
  double latch_duration_;

  DiagnosticLevel input_level_;
  bool warn_latched_;
  bool error_latched_;
  std::optional<rclcpp::Time> warn_stamp_;
  std::optional<rclcpp::Time> error_stamp_;
};

class TimeoutLevel
{
public:
  explicit TimeoutLevel(ConfigYaml yaml);
  void update(const rclcpp::Time & stamp, DiagnosticLevel level);
  void update(const rclcpp::Time & stamp);
  DiagnosticLevel level() const;

private:
  double timeout_duration_;
  std::optional<rclcpp::Time> stamp_;
  DiagnosticLevel level_;
};

class HysteresisLevel
{
public:
  explicit HysteresisLevel(ConfigYaml yaml);
  void update(const rclcpp::Time & stamp, DiagnosticLevel level);
  DiagnosticLevel level() const;
  DiagnosticLevel input_level() const;

private:
  static constexpr DiagnosticLevel upper_limit = DiagnosticStatus::STALE;
  static constexpr DiagnosticLevel lower_limit = DiagnosticStatus::OK;

  void update_stamp(const rclcpp::Time & stamp, DiagnosticLevel level);
  void update_level(const rclcpp::Time & stamp, DiagnosticLevel level);

  bool hysteresis_enabled_;
  double hysteresis_duration_;

  DiagnosticLevel stable_level_;
  DiagnosticLevel input_level_;
  std::unordered_map<DiagnosticLevel, std::optional<rclcpp::Time>> upper_edges_;
  std::unordered_map<DiagnosticLevel, std::optional<rclcpp::Time>> lower_edges_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LEVELS_HPP_
