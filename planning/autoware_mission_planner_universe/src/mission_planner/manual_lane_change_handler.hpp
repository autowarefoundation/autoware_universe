// Copyright 2025 Autoware Foundation
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

#ifndef MISSION_PLANNER__MANUAL_LANE_CHANGE_HANDLER_HPP_
#define MISSION_PLANNER__MANUAL_LANE_CHANGE_HANDLER_HPP_

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <tier4_planning_msgs/srv/set_preferred_lane.hpp>

#include <optional>
#include <string>

namespace autoware::mission_planner_universe
{

using autoware_planning_msgs::msg::LaneletRoute;
using tier4_planning_msgs::srv::SetPreferredLane;

struct LaneChangeRequestResult
{
  LaneletRoute route;
  bool success;
  std::string message;
};

class ManualLaneChangeHandler
{
public:
  explicit ManualLaneChangeHandler(
    LaneletRoute::ConstSharedPtr * current_route,
    std::function<lanelet::ConstLanelet(const int64_t)> get_lanelet_by_id)
  : current_route_(current_route),
    get_lanelet_by_id_(get_lanelet_by_id),
    original_route_{std::nullopt},
    logger_(rclcpp::get_logger("ManualLaneChangeHandler"))
  {
  }
  LaneChangeRequestResult process_lane_change_request(
    const int64_t ego_lanelet_id, const SetPreferredLane::Request::SharedPtr req);

  void reset() { original_route_ = std::nullopt; }

private:
  LaneletRoute::ConstSharedPtr * current_route_;
  std::function<lanelet::ConstLanelet(const int64_t)> get_lanelet_by_id_;
  std::optional<LaneletRoute::ConstSharedPtr> original_route_;
  rclcpp::Logger logger_;
};

}  // namespace autoware::mission_planner_universe

#endif  // MISSION_PLANNER__MANUAL_LANE_CHANGE_HANDLER_HPP_
