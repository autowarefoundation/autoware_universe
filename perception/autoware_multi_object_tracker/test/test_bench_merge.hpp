// Copyright 2025 TIER IV, inc.
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
#ifndef TEST_BENCH_MERGE_HPP_
#define TEST_BENCH_MERGE_HPP_
#include "test_bench.hpp"

#include <gtest/gtest.h>

#include <string>
#include <unordered_map>

class MergeTestBench : public TrackingTestBench
{
public:
  MergeTestBench() = default;

  void initializeObjects(const TrackingScenarioConfig & params) override;
  autoware::multi_object_tracker::types::DynamicObjectList generateDetections(
    const rclcpp::Time & stamp) override;

private:
  void addNewUnknown(
    const std::string & id, const ObjectState & car_state, float size_x, float size_y);

  std::unordered_map<std::string, ObjectState> unknown_states_;
  std::unordered_map<std::string, std::string> car_to_unknown_map_;
};
#endif  // TEST_BENCH_MERGE_HPP_
