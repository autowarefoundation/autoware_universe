// Copyright 2024 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vru/history.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils/system/time_keeper.hpp>

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <string>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

namespace
{
bool hasPotentialToReach(
  const TrackedObject & object, const Eigen::Vector2d & center_point,
  const Eigen::Vector2d & right_point, const Eigen::Vector2d & left_point,
  const double time_horizon, const double min_object_vel,
  const double max_crosswalk_user_delta_yaw_threshold_for_lanelet)
{
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;
  const auto yaw = autoware_utils::get_rpy(object.kinematics.pose_with_covariance.pose).z;

  constexpr double stop_velocity_th = 0.14;  // [m/s]
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto is_stop_object = estimated_velocity < stop_velocity_th;
  const auto velocity = std::max(min_object_vel, estimated_velocity);

  const double pedestrian_to_crosswalk_center_direction =
    std::atan2(center_point.y() - obj_pos.y, center_point.x() - obj_pos.x);

  const auto
    [pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction] =
      [&]() {
        const double pedestrian_to_crosswalk_right_direction =
          std::atan2(right_point.y() - obj_pos.y, right_point.x() - obj_pos.x);
        const double pedestrian_to_crosswalk_left_direction =
          std::atan2(left_point.y() - obj_pos.y, left_point.x() - obj_pos.x);
        return std::make_pair(
          autoware_utils::normalize_radian(
            pedestrian_to_crosswalk_right_direction - pedestrian_to_crosswalk_center_direction),
          autoware_utils::normalize_radian(
            pedestrian_to_crosswalk_left_direction - pedestrian_to_crosswalk_center_direction));
      }();

  const double pedestrian_heading_rel_direction = [&]() {
    const double pedestrian_heading_direction =
      std::atan2(obj_vel.x * std::sin(yaw), obj_vel.x * std::cos(yaw));
    return autoware_utils::normalize_radian(
      pedestrian_heading_direction - pedestrian_to_crosswalk_center_direction);
  }();

  const double pedestrian_to_crosswalk_min_rel_direction = std::min(
    pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction);
  const double pedestrian_to_crosswalk_max_rel_direction = std::max(
    pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction);
  const double pedestrian_vel_angle_against_crosswalk = [&]() {
    if (pedestrian_heading_rel_direction < pedestrian_to_crosswalk_min_rel_direction) {
      return pedestrian_to_crosswalk_min_rel_direction - pedestrian_heading_rel_direction;
    }
    if (pedestrian_to_crosswalk_max_rel_direction < pedestrian_heading_rel_direction) {
      return pedestrian_to_crosswalk_max_rel_direction - pedestrian_heading_rel_direction;
    }
    return 0.0;
  }();
  const auto heading_for_crosswalk = std::abs(pedestrian_vel_angle_against_crosswalk) <
                                     max_crosswalk_user_delta_yaw_threshold_for_lanelet;
  const auto reachable = std::hypot(center_point.x() - obj_pos.x, center_point.y() - obj_pos.y) <
                         velocity * time_horizon;

  return reachable && (heading_for_crosswalk || is_stop_object);
}
}  // namespace

void CrosswalkUserHistoryManager::initialize()
{
  current_crosswalk_users_.clear();
  predicted_crosswalk_users_ids_.clear();
}

void CrosswalkUserHistoryManager::loadCurrentUsers(
  const TrackedObjects & objects, std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  for (const auto & object : objects.objects) {
    const auto label_for_prediction = utils::changeVRULabelForPrediction(
      object.classification.front().label, object, lanelet_map_ptr);
    if (
      label_for_prediction == ObjectClassification::PEDESTRIAN ||
      label_for_prediction == ObjectClassification::BICYCLE) {
      const std::string object_id = autoware_utils::to_hex_string(object.object_id);
      current_crosswalk_users_.emplace(object_id, object);
    }
  }
}

void CrosswalkUserHistoryManager::removeOldHistory(
  const double current_time, const double buffer_time)
{
  auto invalidated_crosswalk_users =
    utils::removeOldObjectsHistory(current_time, buffer_time, crosswalk_users_history_);
  for (auto it = known_matches_.begin(); it != known_matches_.end();) {
    if (invalidated_crosswalk_users.count(it->second)) {
      it = known_matches_.erase(it);
    } else {
      ++it;
    }
  }
}

void CrosswalkUserHistoryManager::updateHistory(
  const std_msgs::msg::Header & header, const TrackedObject & object, const std::string & object_id)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  CrosswalkUser crosswalk_user;
  crosswalk_user.header = header;
  crosswalk_user.tracked_object = object;

  if (crosswalk_users_history_.count(object_id) == 0) {
    crosswalk_users_history_.emplace(object_id, std::deque<CrosswalkUser>{crosswalk_user});
    return;
  }

  const auto last_object_data = crosswalk_users_history_.at(object_id).back();
  crosswalk_user.intention_history = last_object_data.intention_history;
  crosswalk_user.is_crossing = last_object_data.is_crossing;
  crosswalk_users_history_.at(object_id).push_back(crosswalk_user);
}

std::string CrosswalkUserHistoryManager::tryMatchToDisappeared(const std::string & object_id)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (!match_lost_and_appeared_) {
    return object_id;
  }

  const auto known_match_opt = [&]() -> std::optional<std::string> {
    if (!known_matches_.count(object_id)) {
      return std::nullopt;
    }

    std::string match_id = known_matches_[object_id];
    if (crosswalk_users_history_.count(match_id)) {
      // avoid matching two appeared users to one user in history
      current_crosswalk_users_[match_id] = crosswalk_users_history_[match_id].back().tracked_object;
      return match_id;
    } else {
      RCLCPP_WARN_STREAM(
        node_.get_logger(), "Crosswalk user "
                              << object_id << " was matched to " << match_id
                              << " but history for the crosswalk user was deleted. Rematching");
    }
    return std::nullopt;
  }();
  if (known_match_opt.has_value()) {
    return known_match_opt.value();
  }

  std::string match_id = object_id;
  double best_score = std::numeric_limits<double>::max();
  const auto object_pos =
    current_crosswalk_users_[object_id].kinematics.pose_with_covariance.pose.position;
  for (const auto & [user_id, user_history] : crosswalk_users_history_) {
    if (current_crosswalk_users_.count(user_id)) {
      continue;
    }
    const auto match_candidate_pos =
      user_history.back().tracked_object.kinematics.pose_with_covariance.pose.position;
    const double score =
      std::hypot(match_candidate_pos.x - object_pos.x, match_candidate_pos.y - object_pos.y);
    if (score < best_score) {
      best_score = score;
      match_id = user_id;
    }
  }

  if (object_id != match_id) {
    RCLCPP_INFO_STREAM(
      node_.get_logger(), "[Map Based Prediction]: Matched " << object_id << " to " << match_id);
    current_crosswalk_users_[match_id] = crosswalk_users_history_[match_id].back().tracked_object;
  }

  known_matches_[object_id] = match_id;
  return match_id;
}

bool CrosswalkUserHistoryManager::hasPotentialToReachWithHistory(
  const TrackedObject & object, const Eigen::Vector2d & center_point,
  const Eigen::Vector2d & right_point, const Eigen::Vector2d & left_point,
  const double time_horizon, const double min_object_vel,
  const double max_crosswalk_user_delta_yaw_threshold_for_lanelet, const bool is_crossing)
{
  const auto has_crossing_intention = hasPotentialToReach(
    object, center_point, right_point, left_point, time_horizon, min_object_vel,
    max_crosswalk_user_delta_yaw_threshold_for_lanelet);

  const auto object_id = autoware_utils::to_hex_string(object.object_id);

  if (crosswalk_users_history_.count(object_id) == 0) {
    return has_crossing_intention;
  }

  auto & last_object_data = crosswalk_users_history_.at(object_id).back();
  const auto now = node_.get_clock()->now();

  if (last_object_data.is_crossing != is_crossing) {
    last_object_data.intention_history.clear();
  }
  last_object_data.is_crossing = is_crossing;

  const auto itr = std::find_if(
    last_object_data.intention_history.begin(), last_object_data.intention_history.end(),
    [&center_point](const auto & intention) {
      return std::hypot(
               intention.point.x() - center_point.x(), intention.point.y() - center_point.y()) <
             1e-3;
    });

  if (itr == last_object_data.intention_history.end()) {
    last_object_data.intention_history.push_back(
      CrosswalkUser::Intention{rclcpp::Time(0, 0, RCL_ROS_TIME), now, center_point});
    return has_crossing_intention;
  }

  if (has_crossing_intention) {
    if ((now - itr->last_no_crossing_intention_time).seconds() > crossing_intention_duration_) {
      itr->last_crossing_intention_time = now;
      return true;
    } else {
      return false;
    }
  } else {
    if ((now - itr->last_crossing_intention_time).seconds() > no_crossing_intention_duration_) {
      itr->last_no_crossing_intention_time = now;
      return false;
    } else {
      return true;
    }
  }

  return false;
}

}  // namespace autoware::map_based_prediction
