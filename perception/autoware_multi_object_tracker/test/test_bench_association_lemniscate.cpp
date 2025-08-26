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
#include "test_bench_association_lemniscate.hpp"

#include <string>
#include <utility>
#include <vector>

TestBenchAssociationLemniscate::TestBenchAssociationLemniscate(const ScenarioParams & params)
: TestBenchAssociation(params)
{
  lemniscate_param_a_ = 60.0f;  // Parameter controlling the size of the "8" shape
}

void TestBenchAssociationLemniscate::initializeObjects()
{
  // Define offsets for multiple lanes in the "8" shape
  std::vector<float> lane_offsets = {-20.0f, -10.0f, 0.0f, 10.0f, 20.0f};

  for (int lane = 0; lane < static_cast<int>(lane_offsets.size()); ++lane) {
    float offset = lane_offsets[lane];
    std::string car_id = "car_lane_" + std::to_string(lane);

    // Place car on the lemniscate at initial parameter
    float initial_param = 0.0f;
    auto [x, y] = calculateLemniscatePosition(initial_param, offset);
    auto [speed_x, speed_y] = calculateLemniscateVelocity(initial_param, offset);

    // Place car at start
    addNewCar(car_id, x, y, speed_x, speed_y);

    // Store lemniscate parameters for this car
    car_param_[car_id] = initial_param;
    car_offset_[car_id] = offset;

    // Attach unknown near this car
    std::string unk_id = "unk_lane_" + std::to_string(lane);
    addNewUnknownNearCar(car_id, unk_id);
  }
}

// Calculate position on lemniscate (8-shape)
std::pair<float, float> TestBenchAssociationLemniscate::calculateLemniscatePosition(
  float param, float offset)
{
  // Lemniscate of Bernoulli parametric equations
  float scale = lemniscate_param_a_;
  float denominator = 1.0f + std::sin(param) * std::sin(param);

  float x = scale * std::cos(param) / denominator + offset * std::cos(param + M_PI / 2);
  float y =
    scale * std::sin(param) * std::cos(param) / denominator + offset * std::sin(param + M_PI / 2);

  return {x, y};
}

// Calculate velocity on lemniscate
std::pair<float, float> TestBenchAssociationLemniscate::calculateLemniscateVelocity(
  float param, float offset)
{
  // Derivative of lemniscate equations
  float scale = lemniscate_param_a_;
  float sin_p = std::sin(param);
  float cos_p = std::cos(param);
  float denominator = 1.0f + sin_p * sin_p;
  float denominator_sq = denominator * denominator;

  // dx/dparam
  float dx_dp = scale * (-sin_p * denominator - cos_p * 2.0f * sin_p * cos_p) / denominator_sq -
                offset * std::sin(param + M_PI / 2);

  // dy/dparam
  float dy_dp =
    scale * ((cos_p * cos_p - sin_p * sin_p) * denominator - sin_p * cos_p * 2.0f * sin_p * cos_p) /
      denominator_sq +
    offset * std::cos(param + M_PI / 2);

  // Multiply by angular velocity to get actual velocity
  float speed_x = dx_dp * angular_velocity_;
  float speed_y = dy_dp * angular_velocity_;

  return {speed_x, speed_y};
}
void TestBenchAssociationLemniscate::addNewCar(
  const std::string & id, float x, float y, float speed_x, float speed_y)
{
  ObjectState state;
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.twist.linear.x = speed_x;
  state.twist.linear.y = speed_y;
  state.shape = {3.5f, 2.0f};  // length, width
  car_states_[id] = state;

  // Initialize orientation based on velocity direction
  float yaw = std::atan2(speed_y, speed_x);
  state.pose.orientation.z = std::sin(yaw / 2);
  state.pose.orientation.w = std::cos(yaw / 2);
}

// Update car positions in lemniscate ("8" shape) motion
void TestBenchAssociationLemniscate::updateCarStates(float dt)
{
  for (auto & [car_id, state] : car_states_) {
    if (car_param_.count(car_id)) {
      float offset = car_offset_[car_id];

      // Update parameter based on angular velocity
      car_param_[car_id] += angular_velocity_ * dt;

      // Keep parameter in reasonable range to prevent overflow
      if (car_param_[car_id] > 4 * M_PI) {
        car_param_[car_id] -= 4 * M_PI;
      }

      // Calculate new position
      auto [x, y] = calculateLemniscatePosition(car_param_[car_id], offset);
      state.pose.position.x = x;
      state.pose.position.y = y;

      // Calculate velocity vector
      auto [speed_x, speed_y] = calculateLemniscateVelocity(car_param_[car_id], offset);
      state.twist.linear.x = speed_x;
      state.twist.linear.y = speed_y;

      // Update orientation to face direction of motion
      float yaw = std::atan2(speed_y, speed_x);
      state.pose.orientation.z = std::sin(yaw / 2);
      state.pose.orientation.w = std::cos(yaw / 2);
    }
  }
}
