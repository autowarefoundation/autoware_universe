// Copyright 2026 The Autoware Foundation
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

#include "autoware/map_based_prediction/relevance_classifier.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <tf2/utils.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::map_based_prediction
{

namespace
{

double squaredDistance2d(const geometry_msgs::msg::Point & a, const geometry_msgs::msg::Point & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return dx * dx + dy * dy;
}

}  // namespace

std::optional<CorridorDistance> computeCorridorDistance(
  const std::vector<geometry_msgs::msg::Point> & polyline, const geometry_msgs::msg::Point & point)
{
  if (polyline.size() < 2) {
    return std::nullopt;
  }

  double best_squared_distance = std::numeric_limits<double>::max();
  double best_arc_length = 0.0;
  geometry_msgs::msg::Point best_foot = polyline.front();
  double accumulated_arc_length = 0.0;

  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    const auto & p0 = polyline[i];
    const auto & p1 = polyline[i + 1];
    const double seg_dx = p1.x - p0.x;
    const double seg_dy = p1.y - p0.y;
    const double seg_squared_length = seg_dx * seg_dx + seg_dy * seg_dy;

    double t = 0.0;
    if (seg_squared_length > std::numeric_limits<double>::epsilon()) {
      t = ((point.x - p0.x) * seg_dx + (point.y - p0.y) * seg_dy) / seg_squared_length;
      t = std::clamp(t, 0.0, 1.0);
    }

    geometry_msgs::msg::Point foot;
    foot.x = p0.x + t * seg_dx;
    foot.y = p0.y + t * seg_dy;

    const double squared_distance = squaredDistance2d(point, foot);
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      best_arc_length = accumulated_arc_length + t * std::sqrt(seg_squared_length);
      best_foot = foot;
    }
    accumulated_arc_length += std::sqrt(seg_squared_length);
  }

  CorridorDistance result;
  result.lateral_distance = std::sqrt(best_squared_distance);
  result.arc_length = best_arc_length;
  result.nearest_point = best_foot;
  return result;
}

RelevanceClassifier::RelevanceClassifier(const RelevanceParams & params) : params_(params)
{
}

void RelevanceClassifier::setEgoData(
  const std::vector<geometry_msgs::msg::Point> & trajectory_points,
  const geometry_msgs::msg::Point & ego_position, const double trajectory_time)
{
  trajectory_points_ = trajectory_points;
  ego_position_ = ego_position;
  trajectory_time_ = trajectory_time;
  has_ego_data_ = true;
}

Relevance RelevanceClassifier::classify(const TrackedObject & object, const double current_time)
{
  // Fail-safe: without a usable planning prior, treat every object as HIGH so that the
  // behavior is identical to the feature being disabled.
  const bool corridor_is_stale =
    !has_ego_data_ || (current_time - trajectory_time_) > params_.ego_trajectory_timeout;
  if (!params_.enable || corridor_is_stale || trajectory_points_.size() < 2) {
    return Relevance::HIGH;
  }

  const auto object_id = autoware_utils::to_hex_string(object.object_id);
  auto & state = object_states_[object_id];
  state.last_seen_time = current_time;

  const Relevance one_shot = classifyOnce(object);

  if (one_shot == Relevance::HIGH) {
    // Promote immediately (safe side).
    state.relevance = Relevance::HIGH;
    state.consecutive_low_count = 0;
    return state.relevance;
  }

  // Demote only after demote_frame_count consecutive LOW classifications.
  state.consecutive_low_count += 1;
  if (state.consecutive_low_count >= params_.demote_frame_count) {
    state.relevance = Relevance::LOW;
  }
  return state.relevance;
}

Relevance RelevanceClassifier::classifyOnce(const TrackedObject & object) const
{
  const auto & object_position = object.kinematics.pose_with_covariance.pose.position;

  // Condition 1: close to the ego vehicle.
  const double squared_distance_to_ego = squaredDistance2d(object_position, ego_position_);
  if (squared_distance_to_ego < params_.always_relevant_radius * params_.always_relevant_radius) {
    return Relevance::HIGH;
  }

  const auto corridor_distance = computeCorridorDistance(trajectory_points_, object_position);
  if (!corridor_distance) {
    return Relevance::HIGH;
  }

  // Condition 2: inside the corridor. The margin widens with the arc length to reflect
  // the growing uncertainty of the plan further ahead.
  const double lateral_margin =
    params_.lateral_margin_base + params_.lateral_margin_rate * corridor_distance->arc_length;
  if (corridor_distance->lateral_distance < lateral_margin) {
    return Relevance::HIGH;
  }

  // Condition 3: approaching the corridor (cut-in / crossing precaution). The object
  // twist is given in the object body frame, so rotate it to the map frame first.
  const double yaw = tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
  const auto & twist = object.kinematics.twist_with_covariance.twist;
  const double velocity_x = twist.linear.x * std::cos(yaw) - twist.linear.y * std::sin(yaw);
  const double velocity_y = twist.linear.x * std::sin(yaw) + twist.linear.y * std::cos(yaw);

  const double toward_corridor_x = corridor_distance->nearest_point.x - object_position.x;
  const double toward_corridor_y = corridor_distance->nearest_point.y - object_position.y;

  const double distance_to_corridor = std::hypot(toward_corridor_x, toward_corridor_y);
  if (distance_to_corridor > std::numeric_limits<double>::epsilon()) {
    const double closing_speed =
      (velocity_x * toward_corridor_x + velocity_y * toward_corridor_y) / distance_to_corridor;
    if (closing_speed > std::numeric_limits<double>::epsilon()) {
      const double time_to_corridor = distance_to_corridor / closing_speed;
      if (time_to_corridor < params_.time_to_corridor_threshold) {
        return Relevance::HIGH;
      }
    }
  }

  return Relevance::LOW;
}

void RelevanceClassifier::removeOldHistory(const double current_time, const double buffer_time)
{
  for (auto it = object_states_.begin(); it != object_states_.end();) {
    if (current_time - it->second.last_seen_time > buffer_time) {
      it = object_states_.erase(it);
    } else {
      ++it;
    }
  }
}

}  // namespace autoware::map_based_prediction
