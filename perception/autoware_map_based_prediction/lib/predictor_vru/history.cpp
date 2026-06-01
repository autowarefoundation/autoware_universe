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

#include <autoware_utils/ros/uuid_helper.hpp>

#include <string>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

void PredictorVru::loadCurrentCrosswalkUsers(const TrackedObjects & objects)
{
  if (!lanelet_map_ptr_) {
    return;
  }

  // removeStaleTrafficLightInfo
  for (auto it = stopped_times_against_green_.begin(); it != stopped_times_against_green_.end();) {
    const bool isDisappeared = std::none_of(
      objects.objects.begin(), objects.objects.end(),
      [&it](autoware_perception_msgs::msg::TrackedObject obj) {
        return autoware_utils::to_hex_string(obj.object_id) == it->first.first;
      });
    if (isDisappeared) {
      it = stopped_times_against_green_.erase(it);
    } else {
      ++it;
    }
  }

  //
  initialize();

  // load current crosswalk users
  for (const auto & object : objects.objects) {
    const auto label_for_prediction = utils::changeVRULabelForPrediction(
      object.classification.front().label, object, lanelet_map_ptr_);
    if (
      label_for_prediction == ObjectClassification::PEDESTRIAN ||
      label_for_prediction == ObjectClassification::BICYCLE) {
      const std::string object_id = autoware_utils::to_hex_string(object.object_id);
      current_crosswalk_users_.emplace(object_id, object);
    }
  }
}

void PredictorVru::removeOldKnownMatches(const double current_time, const double buffer_time)
{
  auto invalidated_crosswalk_users =
    utils::removeOldObjectsHistory(current_time, buffer_time, crosswalk_users_history_);
  // delete matches that point to invalid object
  for (auto it = known_matches_.begin(); it != known_matches_.end();) {
    if (invalidated_crosswalk_users.count(it->second)) {
      it = known_matches_.erase(it);
    } else {
      ++it;
    }
  }
}

void PredictorVru::updateCrosswalkUserHistory(
  const std_msgs::msg::Header & header, const TrackedObject & object, const std::string & object_id)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto now = node_.get_clock()->now();
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

std::string PredictorVru::tryMatchNewObjectToDisappeared(
  const std::string & object_id, std::unordered_map<std::string, TrackedObject> & current_users)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  const auto known_match_opt = [&]() -> std::optional<std::string> {
    if (!known_matches_.count(object_id)) {
      return std::nullopt;
    }

    std::string match_id = known_matches_[object_id];
    // object in the history is already matched to something (possibly itself)
    if (crosswalk_users_history_.count(match_id)) {
      // avoid matching two appeared users to one user in history
      current_users[match_id] = crosswalk_users_history_[match_id].back().tracked_object;
      return match_id;
    } else {
      RCLCPP_WARN_STREAM(
        node_.get_logger(), "Crosswalk user was "
                              << object_id << "was matched to " << match_id
                              << " but history for the crosswalk user was deleted. Rematching");
    }
    return std::nullopt;
  }();
  //  early return if the match is already known
  if (known_match_opt.has_value()) {
    return known_match_opt.value();
  }

  std::string match_id = object_id;
  double best_score = std::numeric_limits<double>::max();
  const auto object_pos = current_users[object_id].kinematics.pose_with_covariance.pose.position;
  for (const auto & [user_id, user_history] : crosswalk_users_history_) {
    // user present in current_users and will be matched to itself
    if (current_users.count(user_id)) {
      continue;
    }
    // TODO(dkoldaev): implement more sophisticated scoring, for now simply dst to last position in
    // history
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
    // avoid matching two appeared users to one user in history
    current_users[match_id] = crosswalk_users_history_[match_id].back().tracked_object;
  }

  known_matches_[object_id] = match_id;
  return match_id;
}

}  // namespace autoware::map_based_prediction
