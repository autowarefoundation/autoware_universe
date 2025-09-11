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
#ifndef TEST_BENCH_ASSOCIATION_HPP_
#define TEST_BENCH_ASSOCIATION_HPP_
#include "test_bench.hpp"

#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
class TestBenchAssociation : public TestBench
{
public:
  explicit TestBenchAssociation(const ScenarioParams & params);

  void initializeObjects() override;
  autoware::multi_object_tracker::types::DynamicObjectList generateDetections(
    const rclcpp::Time & stamp) override;

protected:
  void addNewCar(
    const std::string & id, float x, float y, float speed_x = 0.0f, float speed_y = 0.0f) override;
  void addNewUnknownNearCar(const std::string & car_id, const std::string & unk_id);
  void updateCarStates(float dt) override;
  void addNoiseAndOrientation(
    autoware::multi_object_tracker::types::DynamicObject & obj,
    const UnknownObjectState & state) override;

  float angular_velocity_;  // Base angular velocity (rad/s)

  // Offset position relative to car
  // Place unknown slightly behind and to the side of the car
  float unknown_offset_x_ = -5.5f;
  float unknown_offset_y_ = 0.0f;
  std::unordered_map<std::string, float> car_radius_;
  std::unordered_map<std::string, float> car_angle_;
  std::unordered_map<std::string, std::string> unk_id_to_car_;
};

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

#endif  // TEST_BENCH_ASSOCIATION_HPP_
