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
#ifndef TEST_BENCH_ASSOCIATION_LEMNISCATE_HPP_
#define TEST_BENCH_ASSOCIATION_LEMNISCATE_HPP_
#include "test_bench_association.hpp"

#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
class TestBenchAssociationLemniscate : public TestBenchAssociation
{
public:
  explicit TestBenchAssociationLemniscate(const ScenarioParams & params);

  void initializeObjects() override;

private:
  void addNewCar(
    const std::string & id, float x, float y, float speed_x = 0.0f, float speed_y = 0.0f) override;
  void updateCarStates(float dt) override;

  std::pair<float, float> calculateLemniscateVelocity(float param, float offset);
  std::pair<float, float> calculateLemniscatePosition(float param, float offset);
  std::unordered_map<std::string, float> car_param_;   // Parameter along lemniscate for each car
  std::unordered_map<std::string, float> car_offset_;  // Lateral offset for each car
  float lemniscate_param_a_;                           // Parameter controlling size of "8" shape
};

#endif  // TEST_BENCH_ASSOCIATION_LEMNISCATE_HPP_
