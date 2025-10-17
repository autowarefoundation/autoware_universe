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

#include "manual_lane_change_handler.hpp"

#include <string>

namespace autoware::manual_lane_change_handler
{

std::vector<autoware_planning_msgs::msg::LaneletPrimitive>
ManualLaneChangeHandler::sortPrimitivesLeftToRight(
  const route_handler::RouteHandler & route_handler,
  autoware_planning_msgs::msg::LaneletPrimitive preferred_primitive,
  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> primitives)
{
  using Primitive = autoware_planning_msgs::msg::LaneletPrimitive;

  std::deque<Primitive> sorted_primitives;

  auto find_primitive = [&](lanelet::Id id) -> std::optional<Primitive> {
    auto it = std::find_if(
      primitives.begin(), primitives.end(), [&](const Primitive & p) { return p.id == id; });
    if (it != primitives.end()) return *it;
    return std::nullopt;
  };

  lanelet::ConstLanelet current = route_handler.getLaneletsFromId(preferred_primitive.id);
  // Walk left lanes
  for (auto left = route_handler.getLeftLanelet(current, true); left;
       left = route_handler.getLeftLanelet(*left, true)) {
    if (auto match = find_primitive(left->id())) {
      sorted_primitives.push_front(*match);
    }
  }

  sorted_primitives.push_back(preferred_primitive);

  // Walk right lanes
  for (auto right = route_handler.getRightLanelet(current, true); right;
       right = route_handler.getRightLanelet(*right, true, true)) {
    if (auto match = find_primitive(right->id())) {
      sorted_primitives.push_back(*match);
    }
  }

  std::vector<autoware_planning_msgs::msg::LaneletPrimitive> result{
    sorted_primitives.begin(), sorted_primitives.end()};

  return result;
}

LaneChangeRequestResult ManualLaneChangeHandler::process_lane_change_request(
  const int64_t ego_lanelet_id, const SetPreferredLane::Request::SharedPtr req)
{
  const DIRECTION override_direction = req->lane_change_direction == 0   ? DIRECTION::MANUAL_LEFT
                                       : req->lane_change_direction == 1 ? DIRECTION::MANUAL_RIGHT
                                                                         : DIRECTION::AUTO;
  if (override_direction == DIRECTION::AUTO) {
    // Use back-up
    if (!original_route_) {
      return {
        {},
        false,
        "Manual lane selection to AUTO is commanded but canceled due to no original route "
        "available."};
    }

    std::vector<LaneletPrimitive> preferred_primitives;
    std::transform(
      original_route_.value()->segments.begin(), original_route_.value()->segments.end(),
      std::back_inserter(preferred_primitives),
      [](const auto & segment) { return segment.preferred_primitive; });
    original_route_ = std::nullopt;

    return {
      preferred_primitives, true,
      "Manual lane selection to AUTO is commanded and executed successfully."};
  }

  if (!current_route_) {
    return {
      {},
      false,
      "Manual lane selection to " +
        (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
         : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                         : std::string("unknown")) +
        std::string(" is commanded but canceled due to no current route available.")};
  }

  LaneletRoute route = *current_route_;

  if (!original_route_) {
    // Save the original route if not already saved
    original_route_ = current_route_;
  }

  const auto final_iter = std::prev(route.segments.end());
  auto start_iter = final_iter;

  // Find the segment that contains the ego_lanelet_id
  for (auto iter = route.segments.begin(); iter != route.segments.end(); ++iter) {
    if (iter->primitives.empty()) {
      continue;
    }
    auto start_iter_primitive = std::find_if(
      iter->primitives.begin(), iter->primitives.end(),
      [&ego_lanelet_id](const autoware_planning_msgs::msg::LaneletPrimitive & p) {
        return p.id == ego_lanelet_id;
      });
    if (start_iter_primitive != iter->primitives.end()) {
      start_iter = iter;
      break;
    }
  }

  bool route_updated = false;
  for (auto iter = start_iter; iter != final_iter; ++iter) {
    auto & current_segment = *iter;

    RCLCPP_INFO_STREAM(
      logger_,
      "Current segment preferred primitive ID: " << current_segment.preferred_primitive.id);

    // Safely get next_segment iterator
    auto next_iter = std::next(iter);
    const auto & next_segment = *next_iter;

    // Find the index of the current preferred primitive
    auto current_it = std::find_if(
      current_segment.primitives.begin(), current_segment.primitives.end(),
      [&](const autoware_planning_msgs::msg::LaneletPrimitive & p) {
        return p.id == current_segment.preferred_primitive.id;
      });
    if (current_it == current_segment.primitives.end()) {
      throw std::runtime_error(
        "ManualLaneChangeHandler: Preferred primitive not found in current segment.");
    }

    // Find the index of the current preferred primitive
    auto next_it = std::find_if(
      next_segment.primitives.begin(), next_segment.primitives.end(),
      [&next_segment](const autoware_planning_msgs::msg::LaneletPrimitive & p) {
        return p.id == next_segment.preferred_primitive.id;
      });
    if (next_it == next_segment.primitives.end()) {
      throw std::runtime_error(
        "ManualLaneChangeHandler: Preferred primitive not found in next segment. next_it.id: " +
        std::to_string(next_segment.preferred_primitive.id));
    }

    std::size_t current_index = std::distance(current_segment.primitives.begin(), current_it);

    const auto current_lanelet = get_lanelet_by_id_(current_it->id);
    std::string current_turning_dir = current_lanelet.attributeOr("turn_direction", "none");

    const auto next_lanelet = get_lanelet_by_id_(next_it->id);
    std::string next_turning_dir = next_lanelet.attributeOr("turn_direction", "none");

    const bool left_shift_not_available =
      (override_direction == DIRECTION::MANUAL_LEFT && current_index == 0);
    const bool right_shift_not_available =
      (override_direction == DIRECTION::MANUAL_RIGHT &&
       current_index + 1 == current_segment.primitives.size());
    const bool next_segment_is_left_turn = (next_turning_dir == "left");
    const bool next_segment_is_right_turn = (next_turning_dir == "right");

    const bool current_segment_shift_not_available =
      left_shift_not_available || right_shift_not_available || next_segment_is_left_turn ||
      next_segment_is_right_turn;

    if (current_segment_shift_not_available) {
      std::string shift_unavailable_reason =
        left_shift_not_available    ? "left shift not available"
        : right_shift_not_available ? "right shift not available"
        : next_segment_is_left_turn ? "next segment is left turn"
                                    : "next segment is right turn";
      RCLCPP_INFO_STREAM(
        logger_, "Cannot shift on the current segment (ID: "
                   << current_segment.preferred_primitive.id << ")");
      break;
    }

    if (override_direction == DIRECTION::MANUAL_LEFT && current_index > 0) {
      // shift to the primitive on the left
      route_updated = true;
      current_segment.preferred_primitive = current_segment.primitives.at(current_index - 1);
      RCLCPP_INFO_STREAM(
        logger_, "Shifted left from "
                   << current_segment.primitives.at(current_index).id
                   << " to primitive ID: " << current_segment.preferred_primitive.id);
    } else if (
      override_direction == DIRECTION::MANUAL_RIGHT &&
      current_index + 1 < current_segment.primitives.size()) {
      // shift to the primitive on the right
      route_updated = true;
      current_segment.preferred_primitive = current_segment.primitives.at(current_index + 1);
      RCLCPP_INFO_STREAM(
        logger_, "Shifted right from "
                   << current_segment.primitives.at(current_index).id
                   << " to primitive ID: " << current_segment.preferred_primitive.id);
    }
  }

  if (!route_updated) {
    return {
      {},
      false,
      std::string("Manual lane selection to ") +
        (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
         : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                         : std::string("unknown")) +
        " is not possible for the current preferred primitive configuration."};
  }
  std::vector<LaneletPrimitive> preferred_primitives;
  preferred_primitives.reserve(route.segments.size());
  std::transform(
    route.segments.begin(), route.segments.end(), std::back_inserter(preferred_primitives),
    [](const auto & segment) { return segment.preferred_primitive; });

  return {
    preferred_primitives, true,
    std::string("Manual lane selection to ") +
      (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
       : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                       : std::string("unknown")) +
      std::string(" is commanded and executed successfully.")};
}

}  // namespace autoware::manual_lane_change_handler

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::manual_lane_change_handler::ManualLaneChangeHandler)
