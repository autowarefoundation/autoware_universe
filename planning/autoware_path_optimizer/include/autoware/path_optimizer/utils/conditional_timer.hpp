// Copyright 2025 Tier IV, Inc.
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

#pragma once

#include <chrono>
#include <optional>

class ConditionalTimer
{
public:
  void update(bool condition)
  {
    if (condition && !start_time_.has_value()) {
      // Condition met, start the timer
      start_time_ = std::chrono::high_resolution_clock::now();
    } else if (!condition && start_time_.has_value()) {
      // Condition no longer met, stop the timer
      start_time_ = std::nullopt;
    }
  }

  std::chrono::duration<double> getElapsedTime() const
  {
    if (start_time_.has_value()) {
      auto current_time = std::chrono::high_resolution_clock::now();
      return current_time - *start_time_;
    } else {
      return std::chrono::duration<double>(0.0);
      ;
    }
  }

private:
  std::optional<std::chrono::time_point<std::chrono::high_resolution_clock>> start_time_{
    std::nullopt};
};
