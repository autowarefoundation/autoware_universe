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

#include "autoware/behavior_path_bidirectional_traffic_module/oncoming_car.hpp"

#include <fmt/core.h>

#include <vector>

namespace autoware::behavior_path_planner
{

bool is_car_object(const autoware_perception_msgs::msg::PredictedObject & object)
{
  return !std::all_of(
    object.classification.begin(), object.classification.end(), [](const auto & c) {
      return c.label != autoware_perception_msgs::msg::ObjectClassification::CAR;
    });
}

bool prev_oncoming_cars_contains_object(
  const std::vector<OncomingCar> & prev_oncoming_cars,
  const autoware_perception_msgs::msg::PredictedObject & object)
{
  return std::any_of(
    prev_oncoming_cars.begin(), prev_oncoming_cars.end(),
    [&object](const auto & oncoming_car) { return oncoming_car == object; });
}

std::vector<OncomingCar> OncomingCar::update_oncoming_cars_in_bidirectional_lane(
  const std::vector<OncomingCar> & prev_oncoming_cars,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
  const ConnectedBidirectionalLanelets & bidirectional_lanelets,
  const geometry_msgs::msg::Pose & ego_pose, const std::function<void(std::string_view)> &)
{
  std::vector<OncomingCar> oncoming_cars;
  for (const auto & object : predicted_objects.objects) {
    if (!is_car_object(object)) {
      continue;
    }
    const auto opposite_bidirectional_lanelets =
      bidirectional_lanelets.get_opposite_bidirectional_lanes();

    if (!opposite_bidirectional_lanelets.is_object_on_this_lane(object)) {
      continue;
    }
    if (
      object.kinematics.initial_twist_with_covariance.twist.linear.x < 0.1 &&
      !prev_oncoming_cars_contains_object(prev_oncoming_cars, object)) {
      continue;
    }

    OncomingCar oncoming_car(object, opposite_bidirectional_lanelets);

    if (oncoming_car.distance_from(ego_pose) < 0.0) {
      continue;
    }

    oncoming_cars.emplace_back(oncoming_car);
  }

  return oncoming_cars;
}

}  // namespace autoware::behavior_path_planner
