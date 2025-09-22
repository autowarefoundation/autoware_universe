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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__COLLISION_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__COLLISION_FILTER_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <rclcpp/duration.hpp>

#include <autoware_perception_msgs/msg/predicted_object.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

// Parameters for CollisionFilter
struct CollisionParams
{
  double max_check_time = 3.0;  // seconds - max time horizon to check
  double min_ttc = 2.0;         // seconds - minimum acceptable time to collision
};

class CollisionFilter : public SafetyFilterInterface
{
public:
  CollisionFilter() : SafetyFilterInterface("CollisionFilter") {}

  bool is_feasible(const TrajectoryPoints & traj_points, const FilterContext & context) override;

  void set_parameters(const std::unordered_map<std::string, std::any> & params) override;

private:
  CollisionParams params_;

  // Calculate Time To Collision for a trajectory point with predicted objects
  bool check_collision(
    const TrajectoryPoint & traj_point,
    const autoware_perception_msgs::msg::PredictedObjects & object,
    const rclcpp::Duration & duration) const;
};

}  // namespace autoware::trajectory_safety_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__COLLISION_FILTER_HPP_