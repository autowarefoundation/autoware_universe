#include "manual_lane_change_handler.hpp"

#include "mission_planner.hpp"

namespace autoware::mission_planner_universe
{

LaneChangeRequestResult ManualLaneChangeHandler::process_lane_change_request(
  const int64_t ego_lanelet_id, const SetPreferredLane::Request::SharedPtr req)
{
  const DIRECTION override_direction = req->lane_change_direction == 0   ? DIRECTION::MANUAL_LEFT
                                       : req->lane_change_direction == 1 ? DIRECTION::MANUAL_RIGHT
                                                                         : DIRECTION::AUTO;

  if (override_direction == DIRECTION::AUTO) {
    LaneletRoute route;
    // Use back-up
    if (!original_route_) {
      return {
        route, false,
        "Manual lane selection to AUTO is commanded but canceled due to no original route "
        "available."};
    }

    route = **original_route_;
    original_route_ = std::nullopt;

    return {route, true, "Manual lane selection to AUTO is commanded and executed successfully."};
  }

  if (!*current_route_) {
    return {
      LaneletRoute(), false,
      "Manual lane selection to " +
        (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
         : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                         : std::string("unknown")) +
        std::string(" is commanded but canceled due to no current route available.")};
  }

  LaneletRoute route = **current_route_;

  if (!original_route_) {
    // Save the original route if not already saved
    original_route_ = *current_route_;
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
      [&ego_lanelet_id](const LaneletPrimitive & p) { return p.id == ego_lanelet_id; });
    if (start_iter_primitive != iter->primitives.end()) {
      start_iter = iter;
      break;
    }
  }

  bool route_updated = false;
  for (auto iter = start_iter; iter != final_iter; ++iter) {
    auto & current_segment = iter;
    auto next_segment = iter+1;

    // Find the index of the current preferred primitive
    auto current_it = std::find_if(
      current_segment->primitives.begin(), current_segment->primitives.end(),
      [&current_segment](const LaneletPrimitive & p) {
        return p.id == current_segment->preferred_primitive.id;
      }
    );
    if (current_it == current_segment->primitives.end()) continue;

    // Find the index of the current preferred primitive
    auto next_it = next_segment != final_iter ? std::find_if(
      next_segment->primitives.begin(), next_segment->primitives.end(),
      [&next_segment](const LaneletPrimitive & p) {
        return p.id == next_segment->preferred_primitive.id;
      }
    ) : next_segment->primitives.end();

    std::size_t current_index = std::distance(current_segment->primitives.begin(), current_it);

    const auto current_lanelet = next_it != current_segment->primitives.end() ? get_lanelet_by_id_(current_it->id) : lanelet::ConstLanelet{};
    std::string current_turning_dir = current_lanelet.attributeOr("turn_direction", "none");

    const auto next_lanelet = next_it != next_segment->primitives.end() ? get_lanelet_by_id_(next_it->id) : lanelet::ConstLanelet{};
    std::string next_turning_dir = next_lanelet.attributeOr("turn_direction", "none");

    std::cerr << "current lanelet ID: " << current_it->id << ", dir: " << current_turning_dir << "\n";
    std::cerr << "next lanelet ID: " << next_it->id << ", next dir: " << next_turning_dir << "\n";

    const bool left_shift_not_available = (override_direction == DIRECTION::MANUAL_LEFT && current_index == 0);
    const bool right_shift_not_available = (override_direction == DIRECTION::MANUAL_RIGHT &&
       current_index + 1 == current_segment->primitives.size());
    const bool next_segment_is_left_turn = (next_turning_dir == "left");
    const bool next_segment_is_right_turn = (next_turning_dir == "right");

    const bool current_segment_shift_not_available =
      left_shift_not_available || right_shift_not_available || next_segment_is_left_turn || next_segment_is_right_turn;

    if (current_segment_shift_not_available) {
      RCLCPP_INFO_STREAM(
        logger_, "Cannot shift on the current segment (ID: "
                   << current_segment->preferred_primitive.id << ")");
      break;
    }

    if (override_direction == DIRECTION::MANUAL_LEFT && current_index > 0) {
      // shift to the primitive on the left
      route_updated = true;
      current_segment->preferred_primitive = current_segment->primitives.at(current_index - 1);
      RCLCPP_INFO_STREAM(
        logger_, "Shifted left from "
                   << current_segment->primitives.at(current_index).id
                   << " to primitive ID: " << current_segment->preferred_primitive.id);
    } else if (
      override_direction == DIRECTION::MANUAL_RIGHT &&
      current_index + 1 < current_segment->primitives.size()) {
      // shift to the primitive on the right
      route_updated = true;
      current_segment->preferred_primitive = current_segment->primitives.at(current_index + 1);
      RCLCPP_INFO_STREAM(
        logger_, "Shifted right from "
                   << current_segment->primitives.at(current_index).id
                   << " to primitive ID: " << current_segment->preferred_primitive.id);
    }
  }

  if (!route_updated) {
    reset();
    return {
      LaneletRoute(), false,
      std::string("Manual lane selection to ") +
        (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
         : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                         : std::string("unknown")) +
        " is not possible for the current preferred primitive configuration."};
  }

  return {
    route, true,
    std::string("Manual lane selection to ") +
      (override_direction == DIRECTION::MANUAL_LEFT    ? std::string("left")
       : override_direction == DIRECTION::MANUAL_RIGHT ? std::string("right")
                                                       : std::string("unknown")) +
      std::string(" is commanded and executed successfully.")};
}

}  // namespace autoware::mission_planner_universe
