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

#include "autoware/behavior_path_bidirectional_traffic_module/connected_bidirectional_lanelets.hpp"
#include "autoware/trajectory/utils/closest.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <unique_identifier_msgs/msg/detail/uuid__struct.hpp>

#include <functional>
#include <string_view>
#include <utility>
#include <vector>

namespace autoware::behavior_path_planner
{

class OncomingCar
{
private:
  const autoware_perception_msgs::msg::PredictedObject object_;
  const ConnectedBidirectionalLanelets bidirectional_lanelets_;

public:
  explicit OncomingCar(
    autoware_perception_msgs::msg::PredictedObject object,
    ConnectedBidirectionalLanelets bidirectional_lanelets)
  : object_(std::move(object)), bidirectional_lanelets_(std::move(bidirectional_lanelets))
  {
  }

  [[nodiscard]] bool operator==(const autoware_perception_msgs::msg::PredictedObject & object) const
  {
    return object_.object_id == object.object_id;
  }

  [[nodiscard]] auto get_uuid() const { return object_.object_id.uuid; }

  [[nodiscard]] auto get_pose() const
  {
    return object_.kinematics.initial_pose_with_covariance.pose;
  }

  static std::vector<OncomingCar> update_oncoming_cars_in_bidirectional_lane(
    const std::vector<OncomingCar> & prev_oncoming_cars,
    const autoware_perception_msgs::msg::PredictedObjects & predicted_objects,
    const ConnectedBidirectionalLanelets & bidirectional_lanelets,
    const geometry_msgs::msg::Pose & ego_pose,
    const double & forward_looking_distance,
    const std::function<void(std::string_view)> & logger = [](std::string_view) {});

  [[nodiscard]] double distance_from(const geometry_msgs::msg::Pose & pose) const
  {
    auto center_line = bidirectional_lanelets_.get_center_line();
    return autoware::trajectory::closest(center_line, pose) -
           autoware::trajectory::closest(center_line, get_pose());
  }
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__ONCOMING_CAR_HPP_
