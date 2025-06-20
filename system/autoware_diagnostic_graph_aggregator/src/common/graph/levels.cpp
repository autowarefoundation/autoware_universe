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

#include "graph/levels.hpp"

#include "config/yaml.hpp"

#include <algorithm>

namespace autoware::diagnostic_graph_aggregator
{

LatchLevel::LatchLevel(ConfigYaml yaml)
{
  const auto latch = yaml.optional("latch");
  latch_enabled_ = latch.exists();
  latch_duration_ = latch_enabled_ ? latch.float64() : 0.0;

  warn_latched_ = false;
  error_latched_ = false;
  warn_stamp_ = std::nullopt;
  error_stamp_ = std::nullopt;
  input_level_ = DiagnosticStatus::STALE;
}

void LatchLevel::reset()
{
  // Note: Keep the current input level.
  warn_latched_ = false;
  error_latched_ = false;
  warn_stamp_ = std::nullopt;
  error_stamp_ = std::nullopt;
}

void LatchLevel::update(const rclcpp::Time & stamp, DiagnosticLevel level)
{
  input_level_ = level;

  if (latch_enabled_) {
    update_latch_status(stamp, level);
  }
}

void LatchLevel::update_latch_status(const rclcpp::Time & stamp, DiagnosticLevel level)
{
  if (level < DiagnosticStatus::WARN) {
    warn_stamp_ = std::nullopt;
  } else {
    warn_stamp_ = warn_stamp_.value_or(stamp);
  }
  if (level < DiagnosticStatus::ERROR) {
    error_stamp_ = std::nullopt;
  } else {
    error_stamp_ = error_stamp_.value_or(stamp);
  }

  if (warn_stamp_ && !warn_latched_) {
    const auto duration = (stamp - *warn_stamp_).seconds();
    warn_latched_ = latch_duration_ <= duration;
  }
  if (error_stamp_ && !error_latched_) {
    const auto duration = (stamp - *error_stamp_).seconds();
    error_latched_ = latch_duration_ <= duration;
  }
}

DiagnosticLevel LatchLevel::level() const
{
  return std::max(input_level(), latch_level());
}

DiagnosticLevel LatchLevel::input_level() const
{
  return input_level_;
}

DiagnosticLevel LatchLevel::latch_level() const
{
  if (error_latched_) {
    return DiagnosticStatus::ERROR;
  }
  if (warn_latched_) {
    return DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::OK;
}

TimeoutLevel::TimeoutLevel(ConfigYaml yaml)
{
  timeout_duration_ = yaml.optional("timeout").float64(1.0);
  level_ = DiagnosticStatus::STALE;
}

void TimeoutLevel::update(const rclcpp::Time & stamp, DiagnosticLevel level)
{
  stamp_ = stamp;
  level_ = level;
}

void TimeoutLevel::update(const rclcpp::Time & stamp)
{
  if (stamp_) {
    const auto duration = (stamp - *stamp_).seconds();
    if (timeout_duration_ <= duration) {
      stamp_ = std::nullopt;
      level_ = DiagnosticStatus::STALE;
    }
  }
}

DiagnosticLevel TimeoutLevel::level() const
{
  return level_;
}

HysteresisLevel::HysteresisLevel(ConfigYaml yaml)
{
  const auto hysteresis = yaml.optional("hysteresis");
  hysteresis_enabled_ = hysteresis.exists();
  hysteresis_duration_ = hysteresis_enabled_ ? hysteresis.float64() : 0.0;

  stable_level_ = DiagnosticStatus::STALE;
  input_level_ = DiagnosticStatus::STALE;
}

void HysteresisLevel::update(const rclcpp::Time & stamp, DiagnosticLevel input_level)
{
  input_level_ = input_level;

  if (hysteresis_enabled_) {
    update_stamp(stamp, input_level);
    update_level(stamp, input_level);
  } else {
    stable_level_ = input_level;
  }
}

void HysteresisLevel::update_stamp(const rclcpp::Time & stamp, DiagnosticLevel input_level)
{
  for (auto level = upper_limit; level > input_level; --level) {
    upper_edges_[level] = std::nullopt;
  }
  for (auto level = input_level; level > stable_level_; --level) {
    upper_edges_[level] = upper_edges_[level].value_or(stamp);
  }

  for (auto level = lower_limit; level < input_level; ++level) {
    lower_edges_[level] = std::nullopt;
  }
  for (auto level = input_level; level < stable_level_; ++level) {
    lower_edges_[level] = lower_edges_[level].value_or(stamp);
  }
}

void HysteresisLevel::update_level(const rclcpp::Time & stamp, DiagnosticLevel input_level)
{
  const auto is_continued = [this](const auto & start, const rclcpp::Time & stamp) {
    if (!start) return false;
    const auto duration = (stamp - *start).seconds();
    return hysteresis_duration_ <= duration;
  };

  for (auto level = input_level; level > stable_level_; --level) {
    if (is_continued(upper_edges_[level], stamp)) {
      stable_level_ = level;
      lower_edges_.clear();  // Clear old stamps in the reverse direction.
      return;
    }
  }

  for (auto level = input_level; level < stable_level_; ++level) {
    if (is_continued(lower_edges_[level], stamp)) {
      stable_level_ = level;
      upper_edges_.clear();  // Clear old stamps in the reverse direction.
      return;
    }
  }
}

DiagnosticLevel HysteresisLevel::level() const
{
  return stable_level_;
}

DiagnosticLevel HysteresisLevel::input_level() const
{
  return input_level_;
}

}  // namespace autoware::diagnostic_graph_aggregator
