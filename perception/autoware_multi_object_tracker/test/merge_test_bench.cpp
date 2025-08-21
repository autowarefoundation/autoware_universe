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
#include "merge_test_bench.hpp"

#include <cmath>
#include <string>
#include <vector>

MergeTestBench::MergeTestBench(const TrackingScenarioConfig & params) : TrackingTestBench(params)
{
  angular_velocity_ = 0.2f;  // rad/s - base angular velocity
}

void MergeTestBench::initializeObjects()
{
  // Define radii for concentric circles
  std::vector<float> circle_radii = {10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 35.0f,
                                     40.0f, 45.0f, 50.0f, 55.0f, 60.0f, 65.0f};

  // Calculate speeds based on radius (outer lanes faster)
  std::vector<float> lane_speeds;
  for (float radius : circle_radii) {
    // Speed proportional to radius (v = ω*r)
    lane_speeds.push_back(angular_velocity_ * radius);
  }

  for (int lane = 0; lane < static_cast<int>(lane_speeds.size()); ++lane) {
    float radius = circle_radii[lane];
    std::string car_id = "car_lane_" + std::to_string(lane);
    // Place car on the circle at initial angle (starting from positive x-axis)
    float initial_angle = 0.0f;
    float x = radius * std::cos(initial_angle);
    float y = radius * std::sin(initial_angle);
    // Calculate initial velocity vector (tangent to circle)
    float speed = lane_speeds[lane];
    float speed_x = -speed * std::sin(initial_angle);  // -v*sin(θ)
    float speed_y = speed * std::cos(initial_angle);   // v*cos(θ)
    // Place car at start
    addNewCar(car_id, x, y, speed_x, speed_y);
    // Store circle parameters for this car
    car_radius_[car_id] = radius;
    car_angle_[car_id] = initial_angle;
    // Attach unknown near this car
    std::string unk_id = "unk_lane_" + std::to_string(lane);
    addNewUnknownNearCar(car_id, unk_id);
  }
}

// Keep unknowns stuck near cars
autoware::multi_object_tracker::types::DynamicObjectList MergeTestBench::generateDetections(
  const rclcpp::Time & stamp)
{
  auto detections = TrackingTestBench::generateDetections(stamp);

  // Update unknowns to follow their car (with no speed of their own)
  for (auto & [unk_id, state] : unknown_states_) {
    std::string car_id = unk_id_to_car_[unk_id];
    if (car_states_.count(car_id)) {
      const auto & car_state = car_states_[car_id];
      const auto & car_pose = car_state.pose;

      // Calculate car's yaw from quaternion
      float car_yaw = 2.0f * std::atan2(car_pose.orientation.z, car_pose.orientation.w);

      state.pose.position.x = car_pose.position.x + unknown_offset_x_ * std::cos(car_yaw) -
                              unknown_offset_y_ * std::sin(car_yaw);

      state.pose.position.y = car_pose.position.y + unknown_offset_x_ * std::sin(car_yaw) +
                              unknown_offset_y_ * std::cos(car_yaw);

      // Update orientation to match car's orientation
      state.pose.orientation = car_pose.orientation;
    }
  }
  return detections;
}

// Helper to spawn a car
void MergeTestBench::addNewCar(
  const std::string & id, float x, float y, float speed_x, float speed_y)
{
  ObjectState state;
  state.pose.position.x = x;
  state.pose.position.y = y;
  state.twist.linear.x = speed_x;
  state.twist.linear.y = speed_y;
  state.shape = {3.5f, 2.0f};  // length, width
  car_states_[id] = state;
}

// Helper to spawn unknown near a car
void MergeTestBench::addNewUnknownNearCar(const std::string & car_id, const std::string & unk_id)
{
  const auto & car_state = car_states_[car_id];
  UnknownObjectState state;
  // Transform offset based on car's orientation
  float car_yaw = 2.0f * std::atan2(car_state.pose.orientation.z, car_state.pose.orientation.w);

  state.pose.position.x = car_state.pose.position.x + unknown_offset_x_ * std::cos(car_yaw) -
                          unknown_offset_y_ * std::sin(car_yaw);

  state.pose.position.y = car_state.pose.position.y + unknown_offset_x_ * std::sin(car_yaw) +
                          unknown_offset_y_ * std::cos(car_yaw);

  state.pose.position.z = 0.5f;  // fixed height
  // Set orientation to match car's orientation
  state.pose.orientation = car_state.pose.orientation;

  state.is_moving = false;  // keep unknowns static
  state.twist.linear.x = 0.0f;
  state.twist.linear.y = 0.0f;

  state.z_dimension = 0.5f;  // fixed height
  state.previous_footprint = state.current_footprint;
  state.base_size = 2.0f;                                                 // fixed base size
  state.shape_type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;  // simple shape

  unknown_states_[unk_id] = state;

  // Track relationship car<->unk
  unk_id_to_car_[unk_id] = car_id;
}

// Update car positions in circular motion
void MergeTestBench::updateCarStates(float dt)
{
  for (auto & [car_id, state] : car_states_) {
    if (car_radius_.count(car_id)) {
      float radius = car_radius_[car_id];

      // Update angle based on angular velocity (which depends on radius)
      // For constant angular velocity, all cars complete circle in same time
      car_angle_[car_id] += angular_velocity_ * dt;

      // Keep angle in [0, 2π] range
      if (car_angle_[car_id] > 2 * M_PI) {
        car_angle_[car_id] -= 2 * M_PI;
      }

      // Calculate new position
      state.pose.position.x = radius * std::cos(car_angle_[car_id]);
      state.pose.position.y = radius * std::sin(car_angle_[car_id]);

      // Calculate velocity vector (tangent to circle)
      float speed = angular_velocity_ * radius;
      state.twist.linear.x = -speed * std::sin(car_angle_[car_id]);
      state.twist.linear.y = speed * std::cos(car_angle_[car_id]);

      // Update orientation to face direction of motion
      float yaw = std::atan2(state.twist.linear.y, state.twist.linear.x);
      state.pose.orientation.z = std::sin(yaw / 2);
      state.pose.orientation.w = std::cos(yaw / 2);
    }
  }
}
