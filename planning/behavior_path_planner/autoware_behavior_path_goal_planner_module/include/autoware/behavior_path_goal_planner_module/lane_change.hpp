// Copyright 2024 Tier IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__LANE_CHANGE_HPP_
#define AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__LANE_CHANGE_HPP_

#include <rclcpp/time.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <optional>
#include <string>
#include <variant>

namespace autoware::route_handler
{
class RouteHandler;
}

namespace autoware::behavior_path_planner
{

class LaneChangeContext
{
public:
  class Started
  {
  public:
    Started(const lanelet::Id complete_lane, const rclcpp::Time & start_time)
    : complete_lane_(complete_lane), start_time_(start_time)
    {
    }
    lanelet::Id complete_lane() const { return complete_lane_; }
    rclcpp::Time start_time() const { return start_time_; }

  private:
    lanelet::Id complete_lane_{};
    rclcpp::Time start_time_{};
  };
  class Executing
  {
  public:
    explicit Executing(const Started & from)
    : complete_lane_(from.complete_lane()), start_time_(from.start_time())
    {
    }
    lanelet::Id complete_lane() const { return complete_lane_; }
    rclcpp::Time start_time() const { return start_time_; }

  private:
    lanelet::Id complete_lane_{};
    rclcpp::Time start_time_{};  //<! inherit this value from previous Started/Executing
  };
  struct Aborted
  {
  };
  struct Completed
  {
    explicit Completed([[maybe_unused]] const Executing & exec) {}
  };
  struct NotLaneChanging
  {
  };
  using State = std::variant<Started, Executing, Aborted, Completed, NotLaneChanging>;

  /**
   * @brief compute next state
   */
  State get_next_state(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path,
    const lanelet::Id ego_lane_id, const autoware::route_handler::RouteHandler & route_handler,
    const rclcpp::Time & now) const;

  State get_current_state() const { return state_; }

  template <typename T>
  bool is_state() const
  {
    return std::holds_alternative<T>(state_);
  }

  template <typename T>
  static bool is_state(const State & state)
  {
    return std::holds_alternative<T>(state);
  }

  /**
   * @brief mutate the state to next transition
   */
  void set_state(const State & state)
  {
    is_in_consistent_transition_ = !is_not_consistent_transition(state_, state);
    state_ = state;
  }

  static bool is_not_consistent_transition(const State & from, const State & to);

  bool is_in_consistent_transition() const { return is_in_consistent_transition_; }

  static std::string state_to_string(const State & state)
  {
    return std::visit(
      [](const auto & s) -> std::string {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, Started>) {
          return "Started";
        } else if constexpr (std::is_same_v<T, Executing>) {
          return "Executing";
        } else if constexpr (std::is_same_v<T, Aborted>) {
          return "Aborted";
        } else if constexpr (std::is_same_v<T, Completed>) {
          return "Completed";
        } else if constexpr (std::is_same_v<T, NotLaneChanging>) {
          return "NotLaneChanging";
        }
        return "Unknown";
      },
      state);
  }

private:
  State state_{LaneChangeContext::NotLaneChanging{}};

  bool is_in_consistent_transition_{};
};

};  // namespace autoware::behavior_path_planner
#endif  // AUTOWARE__BEHAVIOR_PATH_GOAL_PLANNER_MODULE__LANE_CHANGE_HPP_
