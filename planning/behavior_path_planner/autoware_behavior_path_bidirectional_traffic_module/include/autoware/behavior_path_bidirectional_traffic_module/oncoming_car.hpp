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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__ONCOMING_CAR_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__ONCOMING_CAR_HPP_

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"

#include <rclcpp/time.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <cstddef>
#include <functional>
#include <map>
#include <set>
#include <string_view>
#include <vector>

namespace autoware::behavior_path_planner
{
struct PredictedObjectsComparator
{
  bool operator()(
    const autoware_perception_msgs::msg::PredictedObject & lhs,
    const autoware_perception_msgs::msg::PredictedObject & rhs) const;
};

class CarObject
{
private:
  geometry_msgs::msg::Pose pose_;
  double speed_;
  unique_identifier_msgs::msg::UUID uuid_;
  ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets_;

public:
  CarObject(
    geometry_msgs::msg::Pose pose, const double & speed, unique_identifier_msgs::msg::UUID uuid,
    ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets);

  [[nodiscard]] const unique_identifier_msgs::msg::UUID & get_uuid() const;

  [[nodiscard]] const geometry_msgs::msg::Pose & get_pose() const;

  [[nodiscard]] const double & get_speed() const;

  void update(const geometry_msgs::msg::Pose & pose, const double & speed);
};

double distance_on_lane(
  const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2,
  const ConnectedBidirectionalLanelets::SharedConstPtr & bidirectional_lanelets);
class OncomingCars
{
private:
  std::vector<CarObject> oncoming_cars_;
  std::map<size_t, rclcpp::Time> oncoming_cars_candidates_;

  ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets_;

  std::set<autoware_perception_msgs::msg::PredictedObject, PredictedObjectsComparator>
    prev_car_objects_;

  double time_to_promote_from_candidate_;

  std::function<void(std::string_view)> logger_;

public:
  explicit OncomingCars(
    ConnectedBidirectionalLanelets::SharedConstPtr bidirectional_lanelets,
    const double & time_to_promote_from_candidate = 0.1,
    const std::function<void(std::string_view)> & logger = [](std::string_view) {});

  void update(
    const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
    const geometry_msgs::msg::Pose & ego_pose, const rclcpp::Time & time);

  [[nodiscard]] const std::vector<CarObject> & get_oncoming_cars() const;

  [[nodiscard]] std::optional<CarObject> get_front_oncoming_car() const;

  [[nodiscard]] bool empty() const { return oncoming_cars_.empty(); }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__ONCOMING_CAR_HPP_
