// Copyright 2024 TIER IV, Inc.
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

#include "autoware/behavior_path_direction_change_module/scene.hpp"

#include "autoware/behavior_path_direction_change_module/utils.hpp"
#include "autoware/behavior_path_planner_common/utils/path_utils.hpp"
#include "autoware/behavior_path_planner_common/utils/utils.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/Forward.h>

#include <tf2/utils.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>

namespace
{
void logDirectionChangeDebugInfo(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & current_reference_path,
  const autoware_internal_planning_msgs::msg::PathWithLaneId & output_path,
  const geometry_msgs::msg::Pose & ego_pose)
{
  using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
  auto print_path = [](const std::string & label,
                       const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
  {
    std::cout << "[MY_DEBUG] [DirectionChange] " << label
              << " size=" << path.points.size() << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    /*
    for (size_t i = 0; i < path.points.size(); ++i) {
      const auto & pt = path.points[i].point;
      const double x   = pt.pose.position.x;
      const double y   = pt.pose.position.y;
      const double yaw = tf2::getYaw(pt.pose.orientation);            // [rad]
      const double yaw_deg = yaw * 180.0 / M_PI;                      // [deg]
      const double v   = pt.longitudinal_velocity_mps;                // [m/s]
      std::cout << "  idx=" << i
                << ", x="   << x
                << ", y="   << y
                << ", yaw=" << yaw_deg << " deg"
                << ", v="   << v << " m/s"
                << std::endl;
    } */
  };
  // ① current_reference_path
  print_path("Current reference path (input)", current_reference_path);
  // ② output.path
  print_path("Output path (DirectionChange output)", output_path);
  // ③ Ego
  {
    std::cout << "[MY_DEBUG] [DirectionChange] Ego state:" << std::endl;
    std::cout << std::fixed << std::setprecision(3);
    const double ex   = ego_pose.position.x;
    const double ey   = ego_pose.position.y;
    const double eyaw = tf2::getYaw(ego_pose.orientation);
    const double eyaw_deg = eyaw * 180.0 / M_PI;
    std::cout << "  x="   << ex
              << ", y="   << ey
              << ", yaw=" << eyaw_deg << " deg"
              << std::endl;
  }
}
}  // namespace

namespace autoware::behavior_path_planner
{
using autoware::motion_utils::calcSignedArcLength;
using autoware::motion_utils::findNearestIndex;
using autoware_utils::calc_distance2d;

DirectionChangeModule::DirectionChangeModule(
  const std::string & name, rclcpp::Node & node,
  const std::shared_ptr<DirectionChangeParameters> & parameters,
  const std::unordered_map<std::string, std::shared_ptr<RTCInterface>> & rtc_interface_ptr_map,
  std::unordered_map<std::string, std::shared_ptr<ObjectsOfInterestMarkerInterface>> &
    objects_of_interest_marker_interface_ptr_map,
  const std::shared_ptr<PlanningFactorInterface> planning_factor_interface)
: SceneModuleInterface{
    name, node, rtc_interface_ptr_map, objects_of_interest_marker_interface_ptr_map,
    planning_factor_interface},
  parameters_{parameters}
{
  // Create publisher for processed path with reversed orientations
  // The ~ expands to the node's namespace (behavior_path_planner)
  // Full topic will be: /planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/output/direction_change/path
  path_publisher_ = node.create_publisher<autoware_internal_planning_msgs::msg::PathWithLaneId>(
    "~/output/direction_change/path", 1);
  std::cout << "[MY_DEBUG] [DirectionChange] Constructor: Created path publisher at topic: " 
            << path_publisher_->get_topic_name() 
            << " (publisher valid: " << (path_publisher_ ? "yes" : "no") << ")" << std::endl;
}

void DirectionChangeModule::initVariables()
{
  reference_path_ = PathWithLaneId();
  modified_path_ = PathWithLaneId();
  cusp_point_indices_.clear();
  current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
  current_segment_index_ = 0;
  odometry_buffer_direction_switch_.clear();
  resetPathCandidate();
  resetPathReference();
}

void DirectionChangeModule::processOnEntry()
{
  std::cout << "[MY_DEBUG] [DirectionChange] Module entry - initializing variables" << std::endl;
  initVariables();
  updateData();
}

void DirectionChangeModule::processOnExit()
{
  std::cout << "[MY_DEBUG] [DirectionChange] Module exit - resetting variables" << std::endl;
  initVariables();
}

void DirectionChangeModule::setParameters(
  const std::shared_ptr<DirectionChangeParameters> & parameters)
{
  parameters_ = parameters;
}

bool DirectionChangeModule::isExecutionRequested() const
{
  if (getCurrentStatus() == ModuleStatus::RUNNING) {
    return true;
  }

  if (!parameters_->enable_cusp_detection && !parameters_->enable_reverse_following) {
    return false;
  }

  return shouldActivateModule();
}

bool DirectionChangeModule::isExecutionReady() const
{
  return true;
}

bool DirectionChangeModule::isReadyForNextRequest(
  const double & min_request_time_sec, bool override_requests) const noexcept
{
  (void)min_request_time_sec;
  (void)override_requests;
  return true;
}

void DirectionChangeModule::updateData()
{
  const auto previous_output = getPreviousModuleOutput();
  if (previous_output.path.points.empty()) {
    std::cout << "[MY_DEBUG] [DirectionChange] Previous module output path is empty. Cannot update data." << std::endl;
    return;
  }

  if (planner_data_ && planner_data_->route_handler) {
    const auto centerline_path = getReferencePathFromDirectionChangeLanelets(
      previous_output.path, planner_data_->route_handler);
    if (!centerline_path.points.empty()) {
      reference_path_ = centerline_path;
      std::cout << "[MY_DEBUG] [DirectionChange] Using centerline from direction_change lanelets ("
                << reference_path_.points.size() << " points)" << std::endl;
      return;
    }
  }

  // Fallback: use previous module output when not in direction_change area, or when
  // centerline from lanelets is unavailable (no route_handler, no tagged lane_ids, or empty path).
  reference_path_ = previous_output.path;
}


bool DirectionChangeModule::shouldActivateModule() const
{
  if (reference_path_.points.empty()) {
    std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: Path empty, module inactive" << std::endl;
    return false;
  }

  if (planner_data_->route_handler) {
    std::set<int64_t> checked_lane_ids;
    for (const auto & path_point : reference_path_.points) {
      for (const auto & lane_id : path_point.lane_ids) {
        if (checked_lane_ids.find(lane_id) != checked_lane_ids.end()) {
          continue;  // Already checked this lanelet
        }
        checked_lane_ids.insert(lane_id);

        try {
          const auto lanelet = planner_data_->route_handler->getLaneletsFromId(lane_id);
          if (hasDirectionChangeAreaTag(lanelet)) {
            // Activate only when in tag area and away from goal (avoid re-entry after completion).
            // Goal planner and start planner also follow similar checks for goal arrival
            if (planner_data_->self_odometry) {
              try {
                const auto goal_pose = planner_data_->route_handler->getGoalPose();
                const double dist_to_goal = autoware_utils::calc_distance2d(
                  planner_data_->self_odometry->pose.pose.position, goal_pose.position);
                if (dist_to_goal < parameters_->th_arrived_distance) {
                  std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: at goal (dist="
                            << dist_to_goal << " m), module INACTIVE" << std::endl;
                  return false;
                }
              } catch (...) {
                // No goal or getGoalPose failed; allow activation
              }
            }
            std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: direction_change_lane tag found in lane_id=" << lane_id << ", module ACTIVE" << std::endl;
            return true;  // Tag found and away from goal, activate module
          }
        } catch (...) {
          // Lanelet not found, skip
          continue;
        }
      }
    }
  }

  // No direction_change_lane tag found, module inactive
  std::cout << "[MY_DEBUG] [DirectionChange] shouldActivateModule: No direction_change_lane tag found, module INACTIVE" << std::endl;
  return false;
}

bool DirectionChangeModule::isSustainedStoppedForDirectionSwitch()
{
  if (!planner_data_ || !planner_data_->self_odometry || !clock_) {
    return false;
  }
  const rclcpp::Time now = clock_->now();
  const double v =
    std::abs(planner_data_->self_odometry->twist.twist.linear.x);
  odometry_buffer_direction_switch_.emplace_back(now, v);

  const rclcpp::Duration window = rclcpp::Duration::from_seconds(parameters_->th_stopped_time);
  const rclcpp::Time cutoff = now - window;
  while (!odometry_buffer_direction_switch_.empty() &&
         rclcpp::Time(odometry_buffer_direction_switch_.front().first) < cutoff) {
    odometry_buffer_direction_switch_.pop_front();
  }

  if (odometry_buffer_direction_switch_.size() < 2u) {
    return false;
  }
  const double span_sec =
    (rclcpp::Time(odometry_buffer_direction_switch_.back().first) -
     rclcpp::Time(odometry_buffer_direction_switch_.front().first))
      .seconds();
  if (span_sec < parameters_->th_stopped_time) {
    return false;
  }
  for (const auto & entry : odometry_buffer_direction_switch_) {
    if (entry.second >= parameters_->stop_velocity_threshold) {
      return false;
    }
  }
  return true;
}

BehaviorModuleOutput DirectionChangeModule::plan()
{
  // Note: updateData() is already called by SceneModuleInterface::run() before plan()
  BehaviorModuleOutput output;
  
  // Copy reference_path_ to local variable for stability
  const auto current_reference_path = reference_path_;

  // Detect cusp points using current_reference_path
  cusp_point_indices_ = detectCuspPoints(current_reference_path, parameters_->cusp_detection_angle_threshold_deg);

  
  std::cout << "[MY_DEBUG] [DirectionChange] plan() called: Path points=" << reference_path_.points.size() << ", Cusp points detected=" << cusp_point_indices_.size() << std::endl;
  if (!cusp_point_indices_.empty()) {
    std::stringstream ss;
    ss << "[MY_DEBUG] [DirectionChange] Cusp indices: ";
    for (size_t i = 0; i < cusp_point_indices_.size() && i < 10; ++i) {
      ss << cusp_point_indices_[i];
      if (i < cusp_point_indices_.size() - 1 && i < 9) ss << ", ";
    }
    if (cusp_point_indices_.size() > 10) ss << "... (total: " << cusp_point_indices_.size() << ")";
    std::cout << ss.str() << std::endl;
  }

  // Strategy: Separate forward/backward path publishing
  // - If no cusps: return full path as forward segment
  // - If cusps exist: split at first cusp and publish forward/backward segments separately
  //   This prevents downstream modules from seeing mixed orientations
  
  if (cusp_point_indices_.empty()) {
    // No cusps detected in current reference_path
    // IMPORTANT: If we're already in reverse states, maintain that state
    // This prevents oscillation when cusp detection becomes unstable after passing cusp
    if (current_segment_state_ == PathSegmentState::AT_CUSP ||
        current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) {
      std::cout << "[MY_DEBUG] [DirectionChange] No cusp detected but already in reverse state, maintaining backward path" << std::endl;
      // Continue publishing backward path: reverse orientations and velocities
      output.path = reference_path_;
      for (auto & p : output.path.points) {
        double yaw = tf2::getYaw(p.point.pose.orientation);
        yaw = autoware_utils::normalize_radian(yaw + M_PI);
        p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
        p.point.longitudinal_velocity_mps = -std::abs(p.point.longitudinal_velocity_mps);
      }
      modified_path_ = output.path;
    } else {
      // Not in backward state yet - return full path as forward segment
      std::cout << "[MY_DEBUG] [DirectionChange] No cusp points detected, returning full path as forward segment" << std::endl;
      output.path = reference_path_;
      modified_path_ = output.path;
      current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
    }
  } else {
    // Cusps detected: multi-cusp segment-based strategy
    const size_t num_segments = cusp_point_indices_.size() + 1u;
    if (current_segment_index_ >= num_segments) {
      current_segment_index_ = num_segments - 1u;
    }
    auto segmentBounds = [&](size_t seg_idx, size_t & start, size_t & end) {
      start = (seg_idx == 0u) ? 0u : cusp_point_indices_[seg_idx - 1u];
      end = (seg_idx < cusp_point_indices_.size())
              ? cusp_point_indices_[seg_idx]
              : (current_reference_path.points.size() - 1u);
    };
    size_t c_start = 0, c_end = 0;
    segmentBounds(current_segment_index_, c_start, c_end);
    const bool is_last_segment = (current_segment_index_ >= cusp_point_indices_.size());

    if (c_end < current_reference_path.points.size()) {
      first_cusp_position_ = current_reference_path.points[c_end].point.pose.position;
      has_valid_cusp_ = true;
    } else {
      has_valid_cusp_ = false;
    }
    std::cout << "[MY_DEBUG] [DirectionChange] segment_index=" << current_segment_index_
              << ", c_start=" << c_start << ", c_end=" << c_end
              << ", is_last_segment=" << is_last_segment << std::endl;

    // Critical Safety Check: Lane Continuity with Reverse Exit
    const bool safety_check_passed = checkLaneContinuitySafety(
      reference_path_, cusp_point_indices_, planner_data_->route_handler);
    std::cout << "[MY_DEBUG] [DirectionChange] Safety check result: " << (safety_check_passed ? "PASSED" : "FAILED") << std::endl;
    if (!safety_check_passed) {
      std::cout << "[MY_DEBUG] [DirectionChange] FATAL: Lane continuity safety check failed. Returning path without modification." << std::endl;
      output.path = reference_path_;
      output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
      output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
      return output;
    }

    if (!planner_data_ || !planner_data_->self_odometry) {
      std::cout << "[MY_DEBUG] [DirectionChange] WARNING: No ego odometry available, defaulting to forward segment" << std::endl;
      current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
    } else {
      const auto & ego_pose = planner_data_->self_odometry->pose.pose;
      std::vector<PathPointWithLaneId> current_segment(
        current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_start),
        current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_end) + 1);
      const auto ego_nearest_idx_opt = findNearestIndex(current_segment, ego_pose);
      size_t ego_nearest_idx = 0;
      double distance_to_cusp = 0.0;
      bool found_nearest = false;
      if (ego_nearest_idx_opt) {
        ego_nearest_idx = *ego_nearest_idx_opt;
        if (!is_last_segment && current_segment.size() > 0) {
          const size_t segment_c_end_local = current_segment.size() - 1u;
          distance_to_cusp = calcSignedArcLength(current_segment, ego_nearest_idx, segment_c_end_local);
        }
        found_nearest = true;
      }

      if (!found_nearest) {
        std::cout << "[MY_DEBUG] [DirectionChange] WARNING: Could not find nearest index for ego pose, defaulting to forward segment" << std::endl;
        current_segment_state_ = PathSegmentState::FORWARD_FOLLOWING;
      } else {
        auto stateToString = [](const PathSegmentState & s) {
          switch (s) {
            case PathSegmentState::IDLE:               return "IDLE";
            case PathSegmentState::FORWARD_FOLLOWING:  return "FORWARD_FOLLOWING";
            case PathSegmentState::APPROACHING_CUSP:   return "APPROACHING_CUSP";
            case PathSegmentState::AT_CUSP:            return "AT_CUSP";
            case PathSegmentState::REVERSE_FOLLOWING:  return "REVERSE_FOLLOWING";
            case PathSegmentState::COMPLETED:          return "COMPLETED";
            default:                                   return "UNKNOWN";
          }
        };
        const double vehicle_velocity = std::abs(planner_data_->self_odometry->twist.twist.linear.x);
        std::cout << "[MY_DEBUG] [DirectionChange] "
                  << "state=" << stateToString(current_segment_state_)
                  << ", ego_nearest_idx=" << ego_nearest_idx
                  << ", c_start=" << c_start << ", c_end=" << c_end
                  << ", distance_to_cusp=" << distance_to_cusp
                  << ", vehicle_velocity=" << vehicle_velocity << " [m/s]" << std::endl;

        PathSegmentState new_state = current_segment_state_;

        if (is_last_segment) {
          // Last segment: no APPROACHING/AT_CUSP; keep FORWARD or REVERSE by parity
          if (current_segment_index_ % 2 == 0) {
            new_state = PathSegmentState::FORWARD_FOLLOWING;
          } else {
            new_state = PathSegmentState::REVERSE_FOLLOWING;
          }
        } else {
          switch (current_segment_state_) {
            case PathSegmentState::IDLE:
              break;
            case PathSegmentState::FORWARD_FOLLOWING:
              if (distance_to_cusp <= parameters_->cusp_detection_distance_start_approaching) {
                new_state = PathSegmentState::APPROACHING_CUSP;
              }
              break;
            case PathSegmentState::APPROACHING_CUSP:
              if (distance_to_cusp <= parameters_->cusp_detection_distance_threshold) {
                new_state = PathSegmentState::AT_CUSP;
              }
              break;
            case PathSegmentState::AT_CUSP: {
              // if (isSustainedStoppedForDirectionSwitch()) {
              //  odometry_buffer_direction_switch_.clear();
              if (vehicle_velocity < parameters_->stop_velocity_threshold) {
                // Check if this is the last cusp and close to goal -> transition to COMPLETED
                const bool is_next_cusp_available = (current_segment_index_ < cusp_point_indices_.size());
                double distance_to_goal = std::numeric_limits<double>::max();
                bool goal_available = false;
                
                if (planner_data_->route_handler) {
                  try {
                    const auto goal_pose = planner_data_->route_handler->getGoalPose();
                    distance_to_goal = autoware_utils::calc_distance2d(
                      ego_pose.position, goal_pose.position);
                    goal_available = true;
                  } catch (...) {
                    // Goal not available, continue with normal transition
                  }
                }
                
                // Transition to COMPLETED if all conditions are met:
                // 1. velocity < stop_velocity_threshold (already checked)
                // 2. no next cusp point
                // 3. distance to goal < th_arrived_distance
                if (!is_next_cusp_available && goal_available && 
                    distance_to_goal < parameters_->th_arrived_distance) {
                  new_state = PathSegmentState::COMPLETED;
                  std::cout << "[MY_DEBUG] [DirectionChange] Transition to COMPLETED" << std::endl;
                } else {
                  // Normal transition to next segment
                  current_segment_index_++;
                  new_state = (current_segment_index_ % 2 == 0)
                                ? PathSegmentState::FORWARD_FOLLOWING
                                : PathSegmentState::REVERSE_FOLLOWING;
                }
              }
              break;
            }
            case PathSegmentState::REVERSE_FOLLOWING:
              if (distance_to_cusp <= parameters_->cusp_detection_distance_start_approaching) {
                new_state = PathSegmentState::APPROACHING_CUSP;
              }
              break;
            case PathSegmentState::COMPLETED:
              break;
            default:
              new_state = PathSegmentState::FORWARD_FOLLOWING;
              break;
          }
        }

        if (new_state != current_segment_state_) {
          std::cout << "[MY_DEBUG] [DirectionChange] State transition: "
                    << stateToString(current_segment_state_) << " -> "
                    << stateToString(new_state)
                    << ", segment_index=" << current_segment_index_ << std::endl;
          current_segment_state_ = new_state;
          segmentBounds(current_segment_index_, c_start, c_end);
        }
      }
    }

    // Output current segment [c_start, c_end]
    if (c_end >= current_reference_path.points.size()) {
      output.path = current_reference_path;
    } else {
      output.path.points.assign(
        current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_start),
        current_reference_path.points.begin() + static_cast<std::ptrdiff_t>(c_end) + 1);
    }

    const bool apply_reversal = (current_segment_index_ % 2 == 1);
    if (apply_reversal && !output.path.points.empty()) {
      const double max_yaw_step_rad = autoware_utils::deg2rad(parameters_->reverse_path_densify_max_yaw_step_deg);
      const double max_dist_step = parameters_->reverse_path_densify_max_distance_step;
      densifyPathByYawAndDistance(output.path.points, max_yaw_step_rad, max_dist_step);
      const size_t cusp_local = 0u;
      const double reference_speed_limit =
        (cusp_local < output.path.points.size())
          ? std::abs(static_cast<double>(output.path.points[cusp_local].point.longitudinal_velocity_mps))
          : 2.0;
      const double effective_cusp_speed =
        std::min(parameters_->reverse_initial_speed, parameters_->reverse_speed_limit);
      const double effective_reverse_speed = parameters_->reverse_speed_limit;
      const double backward_cruise_speed = std::min(reference_speed_limit, effective_reverse_speed);
      const double backward_slow_speed = std::min(reference_speed_limit, effective_cusp_speed);
      for (size_t i = 0; i < output.path.points.size(); ++i) {
        auto & p = output.path.points[i];
        double yaw = tf2::getYaw(p.point.pose.orientation);
        yaw = autoware_utils::normalize_radian(yaw + M_PI);
        p.point.pose.orientation = autoware_utils::create_quaternion_from_yaw(yaw);
        p.point.longitudinal_velocity_mps =
          (i == 0) ? -backward_slow_speed : -backward_cruise_speed;
        if (!p.lane_ids.empty() && p.lane_ids.size() > 1) {
          int64_t max_lane_id = *std::max_element(p.lane_ids.begin(), p.lane_ids.end());
          p.lane_ids = {max_lane_id};
        }
      }
      std::cout << "[MY_DEBUG] [DirectionChange] Publishing REVERSE segment: "
                << output.path.points.size() << " points (indices " << c_start << "-" << c_end << ")" << std::endl;
    } else {
      // When in AT_CUSP we must command stop at segment end so the vehicle stops before direction
      // switch. Otherwise the reference path velocities (e.g. from centerline) keep the vehicle
      // creeping and sustained stop is never satisfied, leading to lane departure.
      if (current_segment_state_ == PathSegmentState::AT_CUSP && !output.path.points.empty()) {
        output.path.points.back().point.longitudinal_velocity_mps = 0.0;
      }
      std::cout << "[MY_DEBUG] [DirectionChange] Publishing FORWARD segment: "
                << output.path.points.size() << " points (indices " << c_start << "-" << c_end << ")" << std::endl;
    }

    modified_path_ = output.path;
  }

  // Publish processed path to topic for debugging
  if (path_publisher_ && !output.path.points.empty()) {
    autoware_internal_planning_msgs::msg::PathWithLaneId path_msg;
    path_msg.header.stamp = clock_->now();
    
    // Get frame_id from reference path or planner_data
    if (!reference_path_.points.empty() && planner_data_ && planner_data_->route_handler) {
      path_msg.header.frame_id = planner_data_->route_handler->getRouteHeader().frame_id;
    } else {
      path_msg.header.frame_id = "map";  // Default fallback
    }
    
    path_msg.points.reserve(output.path.points.size());
    for (const auto & point : output.path.points) {
      autoware_internal_planning_msgs::msg::PathPointWithLaneId path_point;
      path_point.point = point.point;
      path_point.lane_ids = point.lane_ids;
      path_msg.points.push_back(path_point);
    }
    
    path_publisher_->publish(path_msg);
    std::string segment_type;
    if (current_segment_state_ == PathSegmentState::FORWARD_FOLLOWING ||
        current_segment_state_ == PathSegmentState::APPROACHING_CUSP ||
        current_segment_state_ == PathSegmentState::AT_CUSP) {
      segment_type = (current_segment_state_ == PathSegmentState::AT_CUSP) ? "FORWARD (stop at cusp)" : "FORWARD";
    } else if (current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) {
      segment_type = "BACKWARD";
    } else {
      segment_type = "UNKNOWN";
    }

    std::cout << "[MY_DEBUG] [DirectionChange] Published "
              << segment_type
              << " segment to topic: " << path_publisher_->get_topic_name()
              << " with " << path_msg.points.size() << " points" << std::endl;

    // Debug logging for stop point analysis
    bool has_stop_point = false;
    double first_point_vel = 0.0;
    double last_point_vel = 0.0;

    size_t stop_point_index = 0;
    if (!path_msg.points.empty()) {
      first_point_vel = path_msg.points.front().point.longitudinal_velocity_mps;
      last_point_vel = path_msg.points.back().point.longitudinal_velocity_mps;

      for (size_t i = 0; i < path_msg.points.size(); ++i) {
        if (std::abs(path_msg.points[i].point.longitudinal_velocity_mps) < 1e-3) {
          has_stop_point = true;
          stop_point_index = i;
          break;
        }
      }
    }

    // Convert state to string
    std::string state_str;
    switch (current_segment_state_) {
      case PathSegmentState::IDLE:               state_str = "IDLE"; break;
      case PathSegmentState::FORWARD_FOLLOWING:  state_str = "FORWARD_FOLLOWING"; break;
      case PathSegmentState::APPROACHING_CUSP:   state_str = "APPROACHING_CUSP"; break;
      case PathSegmentState::AT_CUSP:            state_str = "AT_CUSP"; break;
      case PathSegmentState::REVERSE_FOLLOWING:  state_str = "REVERSE_FOLLOWING"; break;
      case PathSegmentState::COMPLETED:          state_str = "COMPLETED"; break;
      default:                                   state_str = "UNKNOWN"; break;
    }

    std::cout << "[DIRECTION_CHANGE_DEBUG] Generated path analysis:" << std::endl;
    std::cout << "  state: " << state_str << std::endl;
    std::cout << "  path_points: " << path_msg.points.size() << std::endl;
    std::cout << "  has_stop_point: " << (has_stop_point ? "YES" : "NO") << std::endl;
    if (has_stop_point) {
      const auto& stop_point = path_msg.points[stop_point_index].point.pose.position;
      std::cout << "  stop_point_index: " << stop_point_index << std::endl;
      std::cout << "  stop_point_position: x=" << stop_point.x << " y=" << stop_point.y << std::endl;
    }
    std::cout << "  first_point_vel: " << first_point_vel << " [m/s]" << std::endl;
    std::cout << "  last_point_vel: " << last_point_vel << " [m/s]" << std::endl;
  }

  output.turn_signal_info = getPreviousModuleOutput().turn_signal_info;
  
  // Handle drivable area information based on segment state
  // For backward segment, filter drivable_lanes to match the actual path lane_ids
  // to avoid mismatch between path (Lanelet1498) and drivable_area (Lanelet1483)
  if ((current_segment_state_ == PathSegmentState::AT_CUSP ||
       current_segment_state_ == PathSegmentState::REVERSE_FOLLOWING) &&
      !output.path.points.empty()) {
    // Collect lane_ids from backward segment path
    std::set<int64_t> backward_lane_ids;
    for (const auto & point : output.path.points) {
      backward_lane_ids.insert(point.lane_ids.begin(), point.lane_ids.end());
    }
    
    std::cout << "[MY_DEBUG] [DirectionChange] Backward segment lane_ids: ";
    for (const auto & id : backward_lane_ids) {
      std::cout << id << " ";
    }
    std::cout << std::endl;
    
    // Filter drivable_lanes to only include lanes present in backward segment
    auto prev_drivable_info = getPreviousModuleOutput().drivable_area_info;
    output.drivable_area_info = prev_drivable_info;  // Copy structure
    output.drivable_area_info.drivable_lanes.clear();  // Clear lanes for filtering
    
    for (const auto & drivable_lane : prev_drivable_info.drivable_lanes) {
      // Check if any lane in this DrivableLanes is in backward_lane_ids
      bool contains_backward_lane = false;
      std::vector<int64_t> drivable_lane_ids;
      
      // Collect all lane IDs from this DrivableLanes
      drivable_lane_ids.push_back(drivable_lane.right_lane.id());
      drivable_lane_ids.push_back(drivable_lane.left_lane.id());
      for (const auto & middle_lane : drivable_lane.middle_lanes) {
        drivable_lane_ids.push_back(middle_lane.id());
      }
      
      // Check if any of these IDs match backward segment
      for (const auto & id : drivable_lane_ids) {
        if (backward_lane_ids.count(id) > 0) {
          contains_backward_lane = true;
          break;
        }
      }
      
      if (contains_backward_lane) {
        output.drivable_area_info.drivable_lanes.push_back(drivable_lane);
        std::cout << "[MY_DEBUG] [DirectionChange] Keeping drivable_lane with ids: ";
        for (const auto & id : drivable_lane_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      } else {
        std::cout << "[MY_DEBUG] [DirectionChange] Filtering out drivable_lane with ids: ";
        for (const auto & id : drivable_lane_ids) {
          std::cout << id << " ";
        }
        std::cout << std::endl;
      }
    }
    
    std::cout << "[MY_DEBUG] [DirectionChange] Filtered drivable_lanes count: " 
              << output.drivable_area_info.drivable_lanes.size() 
              << " (original: " << prev_drivable_info.drivable_lanes.size() << ")" << std::endl;
  } else {
    // Forward segment or no cusps: preserve drivable area information from previous module
    output.drivable_area_info = getPreviousModuleOutput().drivable_area_info;
  }

  // debug infoを出力
  if (planner_data_ && planner_data_->self_odometry) {
    const auto & ego_pose = planner_data_->self_odometry->pose.pose;

  logDirectionChangeDebugInfo(
    current_reference_path,   // ① input
    output.path,              // ② output
    ego_pose);                // ③ ego pose
  }

  return output;
}

BehaviorModuleOutput DirectionChangeModule::planWaitingApproval()
{
  return plan();
}

CandidateOutput DirectionChangeModule::planCandidate() const
{
  CandidateOutput output;
  output.path_candidate = reference_path_;
  return output;
}

bool DirectionChangeModule::canTransitSuccessState()
{
  // Only complete when we're on the LAST segment and ego has reached its end.
  // With multiple cusps we have multiple segments; we must not exit after the first reverse segment.
  const bool is_last_segment = (current_segment_index_ >= cusp_point_indices_.size());
  if (!is_last_segment) {
    return false;
  }
  if (current_segment_state_ != PathSegmentState::REVERSE_FOLLOWING &&
      current_segment_state_ != PathSegmentState::FORWARD_FOLLOWING) {
    return false;
  }
  if (!planner_data_ || !planner_data_->self_odometry || modified_path_.points.empty()) {
    return false;
  }
  const auto & ego_pose = planner_data_->self_odometry->pose.pose;
  const auto ego_nearest_idx_opt = findNearestIndex(modified_path_.points, ego_pose);
  if (!ego_nearest_idx_opt || *ego_nearest_idx_opt >= modified_path_.points.size()) {
    return false;
  }
  const size_t ego_nearest_idx = *ego_nearest_idx_opt;
  const double remaining_distance = calcSignedArcLength(
    modified_path_.points, ego_nearest_idx, modified_path_.points.size() - 1);
  // const double completion_threshold = 1.0;
  if (remaining_distance <  parameters_->th_arrived_distance) {
    current_segment_state_ = PathSegmentState::COMPLETED;
    std::cout << "[MY_DEBUG] [DirectionChange] Ego completed last segment, state=COMPLETED, module can transit to SUCCESS" << std::endl;
    return true;
  }
  return false;
}

void DirectionChangeModule::setDebugMarkersVisualization() const
{
  debug_data_.cusp_points.clear();
  debug_data_.forward_path = reference_path_;
  debug_data_.reverse_path = modified_path_;

  for (const auto & cusp_idx : cusp_point_indices_) {
    if (cusp_idx < reference_path_.points.size()) {
      debug_data_.cusp_points.push_back(reference_path_.points[cusp_idx].point.pose.position);
    }
  }
}

}  // namespace autoware::behavior_path_planner

