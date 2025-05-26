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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_

#include <autoware_perception_msgs/msg/predicted_object.hpp>

#include <memory>
#include <vector>
namespace autoware::behavior_path_planner
{

class GiveWay;

class GiveWayState
{
};

class NoNeedToGiveWay : public GiveWayState
{
};

class ApproachingShiftStart : public GiveWayState
{
};

class ShiftingRoadside : public GiveWayState
{
};

class WaitingForOncomingTrafficToPass : public GiveWayState
{
};

class BackToNormalLane : public GiveWayState
{
};

class GiveWay
{
private:
  std::shared_ptr<GiveWayState> state_;

public:
  void update_objects(const std::vector<autoware_perception_msgs::msg::PredictedObject> & objects);
};

}  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__GIVE_WAY_HPP_
