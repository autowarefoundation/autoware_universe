// Copyright 2025 Tier IV, Inc.
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

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/trajectory/utils/crossed.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <tier4_v2x_msgs/msg/key_value.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
namespace
{
tier4_v2x_msgs::msg::KeyValue createKeyValue(const std::string & key, const std::string & value)
{
  return tier4_v2x_msgs::build<tier4_v2x_msgs::msg::KeyValue>().key(key).value(value);
}

std::optional<double> findLastCollisionBeforeLine(
  const Trajectory & path, const double end_line_s, const lanelet::ConstLineString3d & line)
{
  const auto collisions = [&]() {
    auto cropped_path = path;
    cropped_path.crop(0., std::min(end_line_s, cropped_path.length()));
    return autoware::experimental::trajectory::crossed(cropped_path, line);
  }();

  if (collisions.empty()) {
    return std::nullopt;
  }

  return collisions.back();
}

std::optional<double> findLastCollisionBeforeLine(
  const Trajectory & path, const double end_line_s, const lanelet::ConstLineStrings3d & lines)
{
  for (const auto & line : lines) {
    if (const auto collision = findLastCollisionBeforeLine(path, end_line_s, line); collision) {
      return collision;
    }
  }
  return std::nullopt;
}

template <class T>
std::optional<double> calcArcLengthFromCollision(
  const Trajectory & path, const double end_line_s, const T & line,
  const PlannerData & planner_data)
{
  const auto collision = findLastCollisionBeforeLine(path, end_line_s, line);
  if (!collision) {
    return std::nullopt;
  }

  const auto ego_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, planner_data.current_odometry->pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);
  if (!ego_s) {
    return std::nullopt;
  }

  return *collision - *ego_s - planner_data.vehicle_info_.max_longitudinal_offset_m;
}

bool hasRightOfWay(const tier4_v2x_msgs::msg::VirtualTrafficLightState & state)
{
  return state.approval;
}
}  // namespace

VirtualTrafficLightModule::VirtualTrafficLightModule(
  const lanelet::Id module_id, const lanelet::Id lane_id,
  const lanelet::autoware::VirtualTrafficLight & reg_elem, const lanelet::ConstLanelet & lane,
  const PlannerParam & planner_param, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterface(module_id, logger, clock, time_keeper, planning_factor_interface),
  lane_id_(lane_id),
  lane_(lane),
  planner_param_(planner_param),
  base_logger_(logger)
{
  // Get map data
  const auto instrument = reg_elem.getVirtualTrafficLight();

  // Information from instrument
  map_data_.instrument_type = *instrument.attribute("type").as<std::string>();
  map_data_.instrument_id = std::to_string(instrument.id());
  map_data_.instrument_center =
    (instrument.front().basicPoint() + instrument.back().basicPoint()) / 2;

  // Information from regulatory_element
  map_data_.reg_elem_id = reg_elem.id();
  map_data_.stop_line = reg_elem.getStopLine();
  map_data_.start_line = reg_elem.getStartLine();
  map_data_.end_lines = reg_elem.getEndLines();

  // Set stop line ID for logging (safe to use in log messages)
  map_data_.stop_line_id_for_log =
    map_data_.stop_line ? std::to_string(map_data_.stop_line->id()) : "none";

  // Custom tags
  // Map attributes
  for (const auto & attr : instrument.attributes()) {
    const auto & key = attr.first;
    const auto & value = *attr.second.as<std::string>();

    // Ignore mandatory tags
    if (key == "type") {
      continue;
    }

    map_data_.custom_tags.push_back(createKeyValue(key, value));
  }

  // Lane ID
  map_data_.custom_tags.push_back(createKeyValue("lane_id", std::to_string(lane_.id())));

  // Turn direction
  map_data_.custom_tags.push_back(
    createKeyValue("turn_direction", lane_.attributeOr("turn_direction", "straight")));

  // Set command
  command_.type = map_data_.instrument_type;
  command_.id = map_data_.instrument_id;
  command_.custom_tags = map_data_.custom_tags;

  // Set up base logger with instrument information
  base_logger_ =
    base_logger_.get_child((map_data_.instrument_type + "_" + map_data_.instrument_id).c_str());
  updateLoggerWithState();
}

void VirtualTrafficLightModule::setModuleState(
  const State new_state, const std::optional<lanelet::Id> end_line_id)
{
  // +----------+
  // |   NONE   |
  // +----------+
  //      |
  //      | vehicle is before start line
  //      v
  // +----------+
  // |REQUESTING|
  // +----------+
  //      |
  //      | vehicle has passed stop line
  //      v
  // +----------+
  // | PASSING  |
  // +----------+
  //      |
  //      +-------------------------------------------+
  //      | - vehicle is near end line                | vehicle has passed end line
  //      | - finalization isn't completed            |
  //      v                                           v
  // +----------+                                 +-----------+
  // |FINALIZING|                                 | FINALIZED |
  // +----------+                                 +-----------+

  if (state_ == new_state) {
    return;
  }

  // NONE -> REQUESTING
  if (state_ == State::NONE && new_state == State::REQUESTING) {
    logInfo(
      "State transition: NONE -> REQUESTING as vehicle is before start line (ID: %ld)",
      map_data_.start_line.id());
  }

  // REQUESTING -> PASSING
  if (state_ == State::REQUESTING && new_state == State::PASSING) {
    logInfo(
      "State transition: REQUESTING -> PASSING as vehicle has passed stop line (ID: %s)",
      map_data_.stop_line_id_for_log.c_str());
  }

  // PASSING -> FINALIZING
  if (state_ == State::PASSING && new_state == State::FINALIZING) {
    if (end_line_id.has_value()) {
      logInfo(
        "State transition: PASSING -> FINALIZING as vehicle is near end line (ID: %ld)",
        end_line_id.value());
    } else {
      logInfo("State transition: PASSING -> FINALIZING as vehicle is near end line");
    }
  }

  // PASSING -> FINALIZED
  if (state_ == State::PASSING && new_state == State::FINALIZED) {
    if (end_line_id.has_value()) {
      logInfo(
        "State transition: PASSING -> FINALIZED as vehicle has passed end line (ID: %ld)",
        end_line_id.value());
    } else {
      logInfo("State transition: PASSING -> FINALIZED as vehicle has passed end line");
    }
  }

  state_ = new_state;
  updateLoggerWithState();
}

bool VirtualTrafficLightModule::modifyPathVelocity(
  Trajectory & path, [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & left_bound,
  [[maybe_unused]] const std::vector<geometry_msgs::msg::Point> & right_bound,
  const PlannerData & planner_data)
{
  // Initialize
  setInfrastructureCommand({});

  // Calculate path index of end line
  // NOTE: In order to deal with u-turn or self-crossing path, only start/stop lines before the end
  // line are used when whether the ego is before/after the start/stop/end lines is calculated.
  const auto end_line = getFirstEndLine(path, planner_data);
  if (!end_line) {
    return true;
  }
  const auto & [end_line_s, end_line_id] = *end_line;

  // Do nothing if vehicle is before start line
  if (isBeforeStartLine(path, end_line_s, planner_data)) {
    logDebug("before start_line");
    setModuleState<State::NONE>();
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if state is already FINALIZED
  if (state_ == State::FINALIZED) {
    logInfoThrottle(5000, "state is already FINALIZED");
    updateInfrastructureCommand();
    return true;
  }

  // Do nothing if vehicle is after any end line
  if (isAfterAnyEndLine(path, end_line_s, planner_data)) {
    setModuleState<State::FINALIZED>(end_line_id);
    updateInfrastructureCommand();
    return true;
  }

  // Don't need to stop if there is no stop_line
  if (!map_data_.stop_line) {
    logWarnThrottle(5000, "no stop line is found, do nothing in this module");
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if no message received
  if (!virtual_traffic_light_state_) {
    setModuleState<State::REQUESTING>();
    logInfoThrottle(
      5000, "no message received for instrument (ID: %ld), stop at stop line (ID: %s)",
      map_data_.instrument_id.c_str(), map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(path, end_line_s, planner_data);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if no right is given
  if (!hasRightOfWay(*virtual_traffic_light_state_)) {
    setModuleState<State::REQUESTING>();
    logInfoThrottle(
      5000, "no right of way for instrument (ID: %ld) is given, stop at stop line (ID: %s)",
      map_data_.instrument_id.c_str(), map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(path, end_line_s, planner_data);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at stop_line if state is timeout before stop_line
  if (isBeforeStopLine(path, end_line_s, planner_data)) {
    setModuleState<State::REQUESTING>();
    if (isStateTimeout(*virtual_traffic_light_state_)) {
      logWarnThrottle(
        5000, "virtual traffic light state is timeout, stop at stop line (ID: %s)",
        map_data_.stop_line_id_for_log.c_str());
      insertStopVelocityAtStopLine(path, end_line_s, planner_data);
    }
    updateInfrastructureCommand();
    return true;
  }

  // After stop_line
  if (state_ == State::REQUESTING) {
    setModuleState<State::PASSING>();
  }

  // Check timeout after stop_line if the option is on
  if (
    planner_param_.check_timeout_after_stop_line && isStateTimeout(*virtual_traffic_light_state_)) {
    setModuleState<State::PASSING>();
    logWarnThrottle(
      5000,
      "virtual traffic light state is timeout after stop line, insert stop velocity at stop line "
      "(ID: %s)",
      map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtStopLine(path, end_line_s, planner_data);
    updateInfrastructureCommand();
    return true;
  }

  // Stop at end_line if finalization isn't completed
  if (!virtual_traffic_light_state_->is_finalized) {
    logInfoThrottle(
      5000, "finalization isn't completed, insert stop velocity at end line (ID: %s)",
      map_data_.stop_line_id_for_log.c_str());
    insertStopVelocityAtEndLine(path, end_line_s, planner_data);

    if (isNearAnyEndLine(path, end_line_s, planner_data) && planner_data.isVehicleStopped()) {
      setModuleState<State::FINALIZING>(end_line_id);
    }
  }

  updateInfrastructureCommand();
  return true;
}

void VirtualTrafficLightModule::updateInfrastructureCommand()
{
  command_.stamp = clock_->now();
  command_.state = static_cast<uint8_t>(state_);
  setInfrastructureCommand(command_);
}

std::optional<std::pair<lanelet::Id, double>> VirtualTrafficLightModule::getFirstEndLine(
  const Trajectory & path, const PlannerData & planner_data) const
{
  std::optional<std::pair<lanelet::Id, double>> first_end_line = std::nullopt;

  const auto connected_lane_ids =
    planning_utils::collectConnectedLaneIds(lane_id_, planner_data.route_handler_);

  for (const auto & end_line : map_data_.end_lines) {
    const auto collisions = autoware::experimental::trajectory::crossed_with_constraint(
      path, end_line, [&connected_lane_ids](const PathPointWithLaneId & point) {
        return std::any_of(
          point.lane_ids.begin(), point.lane_ids.end(),
          [&connected_lane_ids](const lanelet::Id lane_id) {
            return std::find(connected_lane_ids.begin(), connected_lane_ids.end(), lane_id) !=
                   connected_lane_ids.end();
          });
      });

    if (collisions.empty()) {
      continue;
    }
    const auto first_collision = collisions.front();

    if (!first_end_line || first_collision < first_end_line->second) {
      first_end_line = std::make_pair(end_line.id(), first_collision);
    }
  }

  return first_end_line;
}

bool VirtualTrafficLightModule::isBeforeStartLine(
  const Trajectory & path, const double end_line_s, const PlannerData & planner_data) const
{
  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  const auto arc_length =
    calcArcLengthFromCollision(path, end_line_s, map_data_.start_line, planner_data);
  return arc_length && *arc_length > 0;
}

bool VirtualTrafficLightModule::isBeforeStopLine(
  const Trajectory & path, const double end_line_s, const PlannerData & planner_data) const
{
  // Since the module is registered, a collision should be detected usually.
  // Therefore if no collision found, vehicle's path is fully after the line.
  const auto arc_length =
    calcArcLengthFromCollision(path, end_line_s, *map_data_.stop_line, planner_data);
  return arc_length && *arc_length > -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isAfterAnyEndLine(
  const Trajectory & path, const double end_line_s, const PlannerData & planner_data) const
{
  // Assume stop line is before end lines
  if (isBeforeStopLine(path, end_line_s, planner_data)) {
    return false;
  }

  // If the goal is set before the end line, collision will not be detected.
  // Therefore if there is no collision, the ego vehicle is assumed to be before the end line.
  const auto arc_length =
    calcArcLengthFromCollision(path, end_line_s, map_data_.end_lines, planner_data);
  return arc_length && *arc_length < -planner_param_.dead_line_margin;
}

bool VirtualTrafficLightModule::isNearAnyEndLine(
  const Trajectory & path, const double end_line_s, const PlannerData & planner_data) const
{
  const auto arc_length =
    calcArcLengthFromCollision(path, end_line_s, map_data_.end_lines, planner_data);
  return arc_length && std::abs(*arc_length) < planner_param_.near_line_distance;
}

bool VirtualTrafficLightModule::isStateTimeout(
  const tier4_v2x_msgs::msg::VirtualTrafficLightState & state) const
{
  const auto delay = (clock_->now() - rclcpp::Time(state.stamp)).seconds();
  if (delay > planner_param_.max_delay_sec) {
    logDebug("delay=%f, max_delay=%f", delay, planner_param_.max_delay_sec);
    return true;
  }

  return false;
}

void VirtualTrafficLightModule::insertStopVelocityAtStopLine(
  Trajectory & path, const double end_line_s, const PlannerData & planner_data)
{
  const auto collision = findLastCollisionBeforeLine(path, end_line_s, *map_data_.stop_line);
  const auto offset = -planner_data.vehicle_info_.max_longitudinal_offset_m;

  const auto ego_s = autoware::experimental::trajectory::find_first_nearest_index(
    path, planner_data.current_odometry->pose, planner_data.ego_nearest_dist_threshold,
    planner_data.ego_nearest_yaw_threshold);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision || !ego_s) {
    path.longitudinal_velocity_mps() = 0.0;
    stop_pose = planner_data.current_odometry->pose;
  } else {
    const auto stop_distance = *collision - *ego_s + offset;
    const auto is_stopped = planner_data.isVehicleStopped();

    if (stop_distance < planner_param_.hold_stop_margin_distance && is_stopped) {
      path.longitudinal_velocity_mps().range(*ego_s, path.length()).set(0.0);
    } else {
      path.longitudinal_velocity_mps()
        .range(std::max(0., *collision + offset), path.length())
        .set(0.0);
    }

    // for virtual wall
    stop_pose = path.compute(std::clamp(*collision + offset, 0.0, path.length())).point.pose;
  }

  // Set StopReason
  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  // Set data for visualization
  module_data_.stop_head_pose_at_stop_line = autoware_utils::calc_offset_pose(
    stop_pose, planner_data.vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
}

void VirtualTrafficLightModule::insertStopVelocityAtEndLine(
  Trajectory & path, const double end_line_s, const PlannerData & planner_data)
{
  const auto collision = findLastCollisionBeforeLine(path, end_line_s, map_data_.end_lines);

  geometry_msgs::msg::Pose stop_pose{};
  if (!collision) {
    // No enough length
    if (isBeforeStopLine(path, end_line_s, planner_data)) {
      return;
    }

    path.longitudinal_velocity_mps() = 0.0;
    stop_pose = planner_data.current_odometry->pose;
  } else {
    const auto offset = -planner_data.vehicle_info_.max_longitudinal_offset_m;
    path.longitudinal_velocity_mps()
      .range(std::max(0., *collision + offset), path.length())
      .set(0.0);
    stop_pose = path.compute(std::clamp(*collision + offset, 0.0, path.length())).point.pose;
  }

  // Set StopReason
  planning_factor_interface_->add(
    path.restore(), planner_data.current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "");

  // Set data for visualization
  module_data_.stop_head_pose_at_end_line = autoware_utils::calc_offset_pose(
    stop_pose, planner_data.vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
}

std::optional<tier4_v2x_msgs::msg::InfrastructureCommand>
VirtualTrafficLightModule::getInfrastructureCommand() const
{
  return infrastructure_command_;
}

void VirtualTrafficLightModule::setInfrastructureCommand(
  const std::optional<tier4_v2x_msgs::msg::InfrastructureCommand> & command)
{
  infrastructure_command_ = command;
}

void VirtualTrafficLightModule::setCorrespondingVirtualTrafficLightState(
  const tier4_v2x_msgs::msg::VirtualTrafficLightStateArray::ConstSharedPtr
    virtual_traffic_light_states)
{
  if (!virtual_traffic_light_states) {
    return;
  }

  for (const auto & state : virtual_traffic_light_states->states) {
    if (state.id != map_data_.instrument_id) {
      continue;
    }

    const auto has_previous_state = virtual_traffic_light_state_.has_value();

    const auto prev_has_right_of_way =
      has_previous_state && hasRightOfWay(*virtual_traffic_light_state_);
    if (!prev_has_right_of_way && hasRightOfWay(state)) {
      logInfo(
        "received message for instrument (ID: %s) is updated : right of way is given, approval: "
        "true, finalized: %s, stamp: %ld.%09ld",
        state.id.c_str(), state.is_finalized ? "true" : "false", state.stamp.sec,
        state.stamp.nanosec);
    }

    const auto prev_finalized = has_previous_state && virtual_traffic_light_state_->is_finalized;
    if (!prev_finalized && state.is_finalized) {
      logInfo(
        "received message for instrument (ID: %s) is updated : finalization is completed, "
        "approval: %s, finalized: true, stamp: %ld.%09ld",
        state.id.c_str(), state.approval ? "true" : "false", state.stamp.sec, state.stamp.nanosec);
    }

    virtual_traffic_light_state_ = state;
    return;
  }
}

void VirtualTrafficLightModule::updateLoggerWithState()
{
  const std::string state_name = [this]() {
    switch (state_) {
      case State::NONE:
        return "NONE";
      case State::REQUESTING:
        return "REQUESTING";
      case State::PASSING:
        return "PASSING";
      case State::FINALIZING:
        return "FINALIZING";
      case State::FINALIZED:
        return "FINALIZED";
      default:
        return "UNKNOWN";
    }
  }();
  logger_ = base_logger_.get_child("state: " + state_name);
}

autoware::motion_utils::VirtualWalls VirtualTrafficLightModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  wall.text = "virtual_traffic_light";
  wall.ns = std::to_string(module_id_) + "_";
  wall.style = autoware::motion_utils::VirtualWallType::stop;
  const auto & d = module_data_;
  // virtual_wall_stop_line
  std::vector<geometry_msgs::msg::Pose> wall_poses;
  if (d.stop_head_pose_at_stop_line) wall_poses.push_back(*d.stop_head_pose_at_stop_line);
  // virtual_wall_end_line
  if (d.stop_head_pose_at_end_line) wall_poses.push_back(*d.stop_head_pose_at_end_line);
  for (const auto & p : wall_poses) {
    wall.pose = p;
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

visualization_msgs::msg::MarkerArray VirtualTrafficLightModule::createDebugMarkerArray()
{
  using autoware::experimental::lanelet2_utils::to_ros;
  using autoware_utils::create_default_marker;
  using autoware_utils::create_marker_color;
  using autoware_utils::create_marker_scale;

  visualization_msgs::msg::MarkerArray debug_marker_array;

  // Common
  const auto & m = map_data_;
  const auto now = clock_->now();

  // instrument_id
  {
    auto marker = create_default_marker(
      "map", now, "instrument_id", module_id_, visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      create_marker_scale(0.0, 0.0, 1.0), create_marker_color(1.0, 1.0, 1.0, 0.999));

    marker.pose.position = to_ros(m.instrument_center);
    marker.text = m.instrument_id;

    debug_marker_array.markers.push_back(marker);
  }

  // instrument_center
  {
    auto marker = create_default_marker(
      "map", now, "instrument_center", module_id_, visualization_msgs::msg::Marker::SPHERE,
      create_marker_scale(0.3, 0.3, 0.3), create_marker_color(1.0, 0.0, 0.0, 0.999));

    marker.pose.position = to_ros(m.instrument_center);

    debug_marker_array.markers.push_back(marker);
  }

  // stop_line
  if (m.stop_line) {
    auto marker = create_default_marker(
      "map", now, "stop_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      create_marker_scale(0.3, 0.0, 0.0), create_marker_color(1.0, 1.0, 1.0, 0.999));

    for (const auto & p : *m.stop_line) {
      marker.points.push_back(to_ros(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // start_line
  {
    auto marker = create_default_marker(
      "map", now, "start_line", module_id_, visualization_msgs::msg::Marker::LINE_STRIP,
      create_marker_scale(0.3, 0.0, 0.0), create_marker_color(0.0, 1.0, 0.0, 0.999));

    for (const auto & p : m.start_line) {
      marker.points.push_back(to_ros(p));
    }

    debug_marker_array.markers.push_back(marker);
  }

  // end_lines
  {
    auto marker = create_default_marker(
      "map", now, "end_lines", module_id_, visualization_msgs::msg::Marker::LINE_LIST,
      create_marker_scale(0.3, 0.0, 0.0), create_marker_color(0.0, 1.0, 1.0, 0.999));

    for (const auto & line : m.end_lines) {
      for (size_t i = 1; i < line.size(); ++i) {
        marker.points.push_back(to_ros(line[i - 1]));
        marker.points.push_back(to_ros(line[i]));
      }
    }

    debug_marker_array.markers.push_back(marker);
  }

  return debug_marker_array;
}
}  // namespace autoware::behavior_velocity_planner::experimental
