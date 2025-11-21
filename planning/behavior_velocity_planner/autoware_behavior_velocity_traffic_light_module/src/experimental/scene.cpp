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

#include "../utils.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware/trajectory/utils/find_nearest.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner::experimental
{
TrafficLightModule::TrafficLightModule(
  const lanelet::Id module_id,
  const lanelet::TrafficLight & traffic_light_reg_elem,  //
  lanelet::ConstLanelet lane,                            //
  const lanelet::ConstLineString3d & initial_stop_line,  //
  const PlannerParam & planner_param, const rclcpp::Logger logger,
  const rclcpp::Clock::SharedPtr clock,
  const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper,
  const std::shared_ptr<planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface)
: SceneModuleInterfaceWithRTC(module_id, logger, clock, time_keeper, planning_factor_interface),
  traffic_light_reg_elem_(traffic_light_reg_elem),
  lane_(lane),
  stop_line_(initial_stop_line),
  state_(State::APPROACH),
  debug_data_(),
  is_prev_state_stop_(false)
{
  planner_param_ = planner_param;
}

bool TrafficLightModule::modifyPathVelocity(
  Trajectory & path, const std::vector<geometry_msgs::msg::Point> & left_bound,
  const std::vector<geometry_msgs::msg::Point> & right_bound, const PlannerData & planner_data)
{
  debug_data_ = DebugData();
  debug_data_.base_link2front = planner_data.vehicle_info_.max_longitudinal_offset_m;
  first_stop_point_s_ = path.length();

  const auto & self_pose = planner_data.current_odometry;

  // Calculate stop point
  const auto stop_point_s = calcStopPoint(
    path, left_bound, right_bound, stop_line_,
    planner_param_.stop_margin + planner_data.vehicle_info_.max_longitudinal_offset_m);

  if (!stop_point_s) {
    RCLCPP_WARN_STREAM_ONCE(
      logger_,
      "Failed to calculate stop point for regulatory element id " << traffic_light_reg_elem_.id());
    setSafe(true);
    setDistance(std::numeric_limits<double>::lowest());
    return false;
  }

  // Calculate dist to stop point
  const auto ego_s =
    autoware::experimental::trajectory::find_nearest_index(path, self_pose->pose.position);
  const auto signed_arc_length_to_stop_point = *stop_point_s - ego_s;
  setDistance(signed_arc_length_to_stop_point);

  // Check state
  if (state_ == State::APPROACH) {
    // Move to go out state if ego vehicle over deadline.
    constexpr double signed_deadline_length = -2.0;
    if (signed_arc_length_to_stop_point < signed_deadline_length) {
      RCLCPP_DEBUG(logger_, "APPROACH -> GO_OUT");
      state_ = State::GO_OUT;
      stop_signal_received_time_ptr_.reset();
      return true;
    }

    first_stop_point_s_ = stop_point_s;

    // Check if stop is coming.
    const bool is_stop_signal = isStopSignal(planner_data);

    // Update stop signal received time
    if (!is_stop_signal) {
      stop_signal_received_time_ptr_.reset();
    } else if (!stop_signal_received_time_ptr_) {
      stop_signal_received_time_ptr_ = std::make_unique<Time>(clock_->now());
    }

    // Check hysteresis
    const double time_diff =
      stop_signal_received_time_ptr_
        ? std::max((clock_->now() - *stop_signal_received_time_ptr_).seconds(), 0.0)
        : 0.0;
    bool to_be_stopped =
      is_stop_signal && (is_prev_state_stop_ || time_diff > planner_param_.stop_time_hysteresis);

    debug_data_.is_remaining_time_used = false;
    if (planner_param_.v2i_use_remaining_time) {
      const bool will_traffic_light_turn_red_before_reaching_stop_line =
        willTrafficLightTurnRedBeforeReachingStopLine(
          signed_arc_length_to_stop_point, planner_data);
      if (will_traffic_light_turn_red_before_reaching_stop_line && !is_stop_signal) {
        debug_data_.is_remaining_time_used = true;
      }

      to_be_stopped = to_be_stopped || will_traffic_light_turn_red_before_reaching_stop_line;
    }

    // Check if the vehicle is stopped and within a certain distance to the stop line
    if (
      planner_data.isVehicleStopped() &&
      planner_param_.min_behind_dist_to_stop_for_restart_suppression <
        signed_arc_length_to_stop_point &&
      signed_arc_length_to_stop_point <
        planner_param_.max_behind_dist_to_stop_for_restart_suppression &&
      is_stop_signal) {
      // Suppress restart
      RCLCPP_DEBUG(logger_, "Suppressing restart due to proximity to stop line.");
      path = insertStopVelocity(path, ego_s, planner_data);
      return true;
    }

    setSafe(!to_be_stopped);
    if (isActivated()) {
      is_prev_state_stop_ = false;
      return true;
    }

    // Decide whether to stop or pass even if a stop signal is received.
    if (!isPassthrough(signed_arc_length_to_stop_point, planner_data)) {
      path = insertStopVelocity(path, *stop_point_s, planner_data);
      is_prev_state_stop_ = true;
    }
    return true;
  } else if (state_ == State::GO_OUT) {
    // Initialize if vehicle is far from stop_line
    constexpr bool use_initialization_after_start = true;
    constexpr double restart_length = 1.0;
    if (use_initialization_after_start && signed_arc_length_to_stop_point > restart_length) {
      RCLCPP_DEBUG(logger_, "GO_OUT(RESTART) -> APPROACH");
      state_ = State::APPROACH;
    }
    stop_signal_received_time_ptr_.reset();
    return true;
  }

  return false;
}

bool TrafficLightModule::isStopSignal(const PlannerData & planner_data)
{
  updateTrafficSignal(planner_data);

  // If there is no upcoming traffic signal information,
  //   SIMULATION: it will PASS to prevent stopping on the planning simulator
  //   or scenario simulator.
  //   REAL ENVIRONMENT: it will STOP for safety in cases such that traffic light
  //   recognition is not working properly or the map is incorrect.
  if (!traffic_signal_stamp_) {
    return !planner_data.is_simulation;
  }

  // Stop if the traffic signal information has timed out
  if (isTrafficSignalTimedOut()) {
    return true;
  }

  // Check if the current traffic signal state requires stopping
  return autoware::traffic_light_utils::isTrafficSignalStop(lane_, looking_tl_state_);
}

bool TrafficLightModule::willTrafficLightTurnRedBeforeReachingStopLine(
  const double & distance_to_stop_line, const PlannerData & planner_data) const
{
  double ego_velocity = planner_data.current_velocity->twist.linear.x;
  double predicted_passing_stop_line_time = ego_velocity > planner_param_.v2i_velocity_threshold
                                              ? distance_to_stop_line / ego_velocity
                                              : planner_param_.v2i_required_time_to_departure;

  double seconds = predicted_passing_stop_line_time + planner_param_.v2i_last_time_allowed_to_pass;

  rclcpp::Time now = clock_->now();
  // find stop signal from looking_tl_state_.predictions by using isTrafficSignalStop
  for (const auto & prediction : looking_tl_state_.predictions) {
    if (isTrafficSignalRedStop(lane_, prediction.simultaneous_elements)) {
      rclcpp::Time predicted_time = prediction.predicted_stamp;
      rclcpp::Duration remaining_time = predicted_time - now;
      if (remaining_time.seconds() < seconds) {
        return true;
      }
    }
  }
  return false;
}

void TrafficLightModule::updateTrafficSignal(const PlannerData & planner_data)
{
  TrafficSignalStamped signal;
  if (!findValidTrafficSignal(signal, planner_data)) {
    // Don't stop if it never receives traffic light topic.
    // Reset looking_tl_state
    looking_tl_state_.elements.clear();
    looking_tl_state_.traffic_light_group_id = 0;
    return;
  }

  traffic_signal_stamp_ = signal.stamp;

  // Found signal associated with the lanelet
  looking_tl_state_ = signal.signal;
}

bool TrafficLightModule::isPassthrough(
  const double & signed_arc_length, const PlannerData & planner_data) const
{
  if (is_prev_state_stop_) {
    return false;
  }

  const double max_acc = planner_data.max_stop_acceleration_threshold;
  const double max_jerk = planner_data.max_stop_jerk_threshold;
  const double delay_response_time = planner_data.delay_response_time;

  const double reachable_distance =
    planner_data.current_velocity->twist.linear.x * planner_param_.yellow_lamp_period;

  // Calculate distance until ego vehicle decide not to stop,
  // taking into account the jerk and acceleration.
  const double pass_judge_line_distance = planning_utils::calcJudgeLineDistWithJerkLimit(
    planner_data.current_velocity->twist.linear.x,
    planner_data.current_acceleration->accel.accel.linear.x, max_acc, max_jerk,
    delay_response_time);

  const bool distance_stoppable = pass_judge_line_distance < signed_arc_length;
  const bool slow_velocity =
    planner_data.current_velocity->twist.linear.x < planner_param_.yellow_light_stop_velocity;
  const bool stoppable = distance_stoppable || slow_velocity;
  const bool reachable = signed_arc_length < reachable_distance;

  const auto & enable_pass_judge = planner_param_.enable_pass_judge;

  if (enable_pass_judge && !stoppable) {
    // Cannot stop under acceleration and jerk limits.
    // However, ego vehicle can't enter the intersection while the light is yellow.
    // It is called dilemma zone. Make a stop decision to be safe.
    if (!reachable) {
      // dilemma zone: emergency stop
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000,
        "[traffic_light] cannot pass through intersection during yellow lamp!");
      return false;
    } else {
      // pass through
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 1000, "[traffic_light] can pass through intersection during yellow lamp");
      return true;
    }
  }

  return false;
}

bool TrafficLightModule::findValidTrafficSignal(
  TrafficSignalStamped & valid_traffic_signal, const PlannerData & planner_data) const
{
  // get traffic signal associated with the regulatory element id
  const auto traffic_signal_stamped_opt = planner_data.getTrafficSignal(
    traffic_light_reg_elem_.id(), false /* traffic light module does not keep last observation */);
  if (!traffic_signal_stamped_opt) {
    RCLCPP_WARN_STREAM_ONCE(
      logger_, "the traffic signal data associated with regulatory element id "
                 << traffic_light_reg_elem_.id() << " is not received");
    return false;
  }
  valid_traffic_signal = traffic_signal_stamped_opt.value();
  return true;
}

bool TrafficLightModule::isTrafficSignalTimedOut() const
{
  if (!traffic_signal_stamp_) {
    return false;
  }

  const auto is_traffic_signal_timeout =
    (clock_->now() - *traffic_signal_stamp_).seconds() > planner_param_.tl_state_timeout;
  if (is_traffic_signal_timeout) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000 /* ms */, "the received traffic signal data is outdated");
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, *clock_, 5000 /* ms */,
      "time diff: " << (clock_->now() - *traffic_signal_stamp_).seconds());
    return true;
  }
  return false;
}

Trajectory TrafficLightModule::insertStopVelocity(
  const Trajectory & input, const double & stop_point_s, const PlannerData & planner_data)
{
  auto modified_path = input;

  const auto stop_pose = modified_path.compute(stop_point_s).point.pose;
  debug_data_.stop_poses.push_back(stop_pose);

  modified_path.longitudinal_velocity_mps().range(stop_point_s, modified_path.length()).set(0.0);

  planning_factor_interface_->add(
    modified_path.restore(), planner_data.current_odometry->pose, stop_pose,
    autoware_internal_planning_msgs::msg::PlanningFactor::STOP,
    autoware_internal_planning_msgs::msg::SafetyFactorArray{}, true /*is_driving_forward*/, 0.0,
    0.0 /*shift distance*/, "traffic_light");

  return modified_path;
}

void TrafficLightModule::updateStopLine(const lanelet::ConstLineString3d & stop_line)
{
  stop_line_ = stop_line;
}

visualization_msgs::msg::MarkerArray TrafficLightModule::createDebugMarkerArray()
{
  visualization_msgs::msg::MarkerArray debug_marker_array;
  return debug_marker_array;
}

autoware::motion_utils::VirtualWalls TrafficLightModule::createVirtualWalls()
{
  autoware::motion_utils::VirtualWalls virtual_walls;
  autoware::motion_utils::VirtualWall wall;
  if (debug_data_.is_remaining_time_used) {
    wall.text = "traffic_light(V2I)";
  } else {
    wall.text = "traffic_light";
  }
  wall.ns = std::to_string(module_id_) + "_";

  wall.style = autoware::motion_utils::VirtualWallType::deadline;
  for (const auto & p : debug_data_.dead_line_poses) {
    wall.pose = autoware_utils::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  wall.style = autoware::motion_utils::VirtualWallType::stop;
  for (const auto & p : debug_data_.stop_poses) {
    wall.pose = autoware_utils::calc_offset_pose(p, debug_data_.base_link2front, 0.0, 0.0);
    virtual_walls.push_back(wall);
  }

  return virtual_walls;
}

}  // namespace autoware::behavior_velocity_planner::experimental
