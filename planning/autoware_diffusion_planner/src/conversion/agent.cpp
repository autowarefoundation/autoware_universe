// Copyright 2025 TIER IV, Inc.
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

#include "autoware/diffusion_planner/conversion/agent.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <rclcpp/duration.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{

namespace
{

AgentLabel get_model_label(const TrackedObject & object)
{
  const uint8_t autoware_label =
    autoware::object_recognition_utils::getHighestProbLabel(object.classification);

  switch (autoware_label) {
    case autoware_perception_msgs::msg::ObjectClassification::CAR:
    case autoware_perception_msgs::msg::ObjectClassification::TRUCK:
    case autoware_perception_msgs::msg::ObjectClassification::BUS:
    case autoware_perception_msgs::msg::ObjectClassification::MOTORCYCLE:
    case autoware_perception_msgs::msg::ObjectClassification::TRAILER:
      return AgentLabel::VEHICLE;
    case autoware_perception_msgs::msg::ObjectClassification::BICYCLE:
      return AgentLabel::BICYCLE;
    case autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN:
      return AgentLabel::PEDESTRIAN;
    default:
      return AgentLabel::IGNORE;
  }
}

// Heading of a pose matrix in its own frame.
double get_yaw(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

// Longitudinal speed of an observation (magnitude of the linear twist).
double get_speed(const AgentState & state)
{
  const auto & linear = state.original_info.kinematics.twist_with_covariance.twist.linear;
  return std::hypot(linear.x, linear.y);
}

// Yaw rate of an observation.
double get_yaw_rate(const AgentState & state)
{
  return state.original_info.kinematics.twist_with_covariance.twist.angular.z;
}

// Build a synthesized state on the grid from a template observation, overwriting its planar pose.
// When speed_override is set, the linear twist is rewritten so its magnitude matches (direction is
// irrelevant to the model, which derives the velocity heading from the pose). When it is unset the
// template's real twist is preserved verbatim, which matters for the newest slot consumed by
// postprocessing.
AgentState make_grid_state(
  const TrackedObject & tmpl, const double x, const double y, const double yaw,
  const rclcpp::Time & timestamp, const std::optional<double> & speed_override)
{
  TrackedObject object = tmpl;
  auto & pose = object.kinematics.pose_with_covariance.pose;
  const double z = pose.position.z;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
  if (speed_override.has_value()) {
    auto & linear = object.kinematics.twist_with_covariance.twist.linear;
    linear.x = *speed_override;
    linear.y = 0.0;
    linear.z = 0.0;
  }
  return AgentState(object, timestamp);
}

// Transform every history to the target frame, sort by distance to the frame origin (nearest
// first), and trim to at most max_num_agent entries.
std::vector<AgentHistory> transform_sort_trim(
  std::vector<AgentHistory> histories, const Eigen::Matrix4d & transform,
  const size_t max_num_agent)
{
  for (auto & history : histories) {
    history.apply_transform(transform);
  }

  std::sort(histories.begin(), histories.end(), [](const AgentHistory & a, const AgentHistory & b) {
    const double a_dist =
      std::hypot(a.get_latest_state().pose(0, 3), a.get_latest_state().pose(1, 3));
    const double b_dist =
      std::hypot(b.get_latest_state().pose(0, 3), b.get_latest_state().pose(1, 3));
    return a_dist < b_dist;
  });
  if (histories.size() > max_num_agent) {
    histories.erase(
      histories.begin() + static_cast<std::ptrdiff_t>(max_num_agent), histories.end());
  }
  return histories;
}

}  // namespace

AgentState::AgentState(const TrackedObject & object, const rclcpp::Time & timestamp)
: pose(utils::pose_to_matrix4d(object.kinematics.pose_with_covariance.pose)),
  timestamp(timestamp),
  label(get_model_label(object)),
  object_id(autoware_utils_uuid::to_hex_string(object.object_id)),
  original_info(object)
{
}

// Return the state attribute as an array.
[[nodiscard]] std::array<float, AGENT_STATE_DIM> AgentState::as_array() const noexcept
{
  const auto [cos_yaw, sin_yaw] = utils::rotation_matrix_to_cos_sin(pose.block<3, 3>(0, 0));
  const auto & linear_vel = original_info.kinematics.twist_with_covariance.twist.linear;
  const double velocity_norm = std::hypot(linear_vel.x, linear_vel.y);
  const double velocity_x = velocity_norm * cos_yaw;
  const double velocity_y = velocity_norm * sin_yaw;

  return {
    static_cast<float>(pose(0, 3)),
    static_cast<float>(pose(1, 3)),
    static_cast<float>(cos_yaw),
    static_cast<float>(sin_yaw),
    static_cast<float>(velocity_x),
    static_cast<float>(velocity_y),
    static_cast<float>(original_info.shape.dimensions.y),  // width
    static_cast<float>(original_info.shape.dimensions.x),  // length
    static_cast<float>(label == AgentLabel::VEHICLE),
    static_cast<float>(label == AgentLabel::PEDESTRIAN),
    static_cast<float>(label == AgentLabel::BICYCLE),
  };
}

void AgentData::update_histories(const TrackedObjects & objects)
{
  const rclcpp::Time objects_timestamp(objects.header.stamp);
  std::vector<std::string> found_ids;
  for (const TrackedObject & object : objects.objects) {
    if (get_model_label(object) == AgentLabel::IGNORE) {
      continue;
    }
    if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
      continue;
    }
    const std::string object_id = autoware_utils_uuid::to_hex_string(object.object_id);
    auto it = histories_map_.find(object_id);
    if (it != histories_map_.end()) {
      it->second.update(object, objects_timestamp);
    } else {
      histories_map_.emplace(object_id, AgentHistory(INPUT_T_WITH_CURRENT));
      histories_map_.at(object_id).fill(AgentState(object, objects_timestamp));
    }
    found_ids.push_back(object_id);
  }
  // Remove histories that are not found in the current objects
  for (auto it = histories_map_.begin(); it != histories_map_.end();) {
    if (std::find(found_ids.begin(), found_ids.end(), it->first) == found_ids.end()) {
      it = histories_map_.erase(it);
    } else {
      ++it;
    }
  }
}

std::vector<AgentHistory> AgentData::transformed_and_trimmed_histories(
  const Eigen::Matrix4d & transform, size_t max_num_agent) const
{
  std::vector<AgentHistory> histories;
  histories.reserve(histories_map_.size());
  for (const auto & [_, history] : histories_map_) {
    histories.push_back(history);
  }
  return transform_sort_trim(std::move(histories), transform, max_num_agent);
}

void AgentData::update_histories(
  const TrackedObjects & objects, const HistoryAlignmentParams & params)
{
  const rclcpp::Time objects_timestamp(objects.header.stamp);

  // Dedup: the neighbor subscriber polls the latest message, so the same (stale) message can be
  // served on consecutive ticks. Only ingest messages whose stamp strictly advances.
  if (last_processed_stamp_.has_value() && objects_timestamp <= last_processed_stamp_.value()) {
    return;
  }
  last_processed_stamp_ = objects_timestamp;

  std::vector<std::string> found_ids;
  for (const TrackedObject & object : objects.objects) {
    if (get_model_label(object) == AgentLabel::IGNORE) {
      continue;
    }
    if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
      continue;
    }
    const std::string object_id = autoware_utils_uuid::to_hex_string(object.object_id);
    auto it = histories_map_.find(object_id);
    if (it != histories_map_.end()) {
      it->second.update(object, objects_timestamp);
      it->second.set_live(true);
    } else {
      // New UUID: start a growing buffer with a single real observation. No repeat-fill; the
      // resampling step front-clamps the grid to this observation until more arrive.
      auto [inserted, _] =
        histories_map_.emplace(object_id, AgentHistory(NEIGHBOR_HISTORY_BUFFER_SIZE));
      inserted->second.update(object, objects_timestamp);
      inserted->second.set_live(true);
    }
    found_ids.push_back(object_id);
  }

  // Retain agents absent from this message (mark not live) instead of erasing them, so an agent
  // that briefly drops out or is re-identified does not teleport. Prune only once fully aged out of
  // the history window.
  const double max_age =
    static_cast<double>(INPUT_T) * constants::PREDICTION_TIME_STEP_S + params.prune_grace;
  for (auto it = histories_map_.begin(); it != histories_map_.end();) {
    const bool live = std::find(found_ids.begin(), found_ids.end(), it->first) != found_ids.end();
    if (!live) {
      it->second.set_live(false);
    }
    const double age = (objects_timestamp - it->second.newest_stamp()).seconds();
    if (age > max_age) {
      it = histories_map_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<AgentHistory> AgentData::resample_history(
  const AgentHistory & history, const rclcpp::Time & frame_time,
  const HistoryAlignmentParams & params) const
{
  if (history.empty()) {
    return std::nullopt;
  }

  const auto & raw = history.states();
  const size_t n = raw.size();

  // Snapshot per-observation kinematics (planar pose, speed, yaw rate). Yaw is corrected for
  // tracker heading flips (R3) before any interpolation/extrapolation.
  std::vector<double> obs_x(n);
  std::vector<double> obs_y(n);
  std::vector<double> obs_yaw(n);
  std::vector<double> obs_speed(n);
  std::vector<double> obs_yaw_rate(n);
  std::vector<double> obs_time(n);
  for (size_t i = 0; i < n; ++i) {
    obs_x[i] = raw[i].pose(0, 3);
    obs_y[i] = raw[i].pose(1, 3);
    obs_yaw[i] = get_yaw(raw[i].pose);
    obs_speed[i] = get_speed(raw[i]);
    obs_yaw_rate[i] = get_yaw_rate(raw[i]);
    obs_time[i] = raw[i].timestamp.seconds();
  }

  // R3 orientation-flip pre-pass: the newest observation is assumed best. Walking backward, if an
  // older heading differs from its newer neighbor by more than the flip threshold, rotate it by pi
  // (position and speed magnitude unchanged; the derived velocity direction follows the heading).
  for (size_t idx = n - 1; idx-- > 0;) {
    const double delta = autoware_utils_math::normalize_radian(obs_yaw[idx] - obs_yaw[idx + 1]);
    if (std::abs(delta) > params.flip_yaw_threshold) {
      obs_yaw[idx] = autoware_utils_math::normalize_radian(obs_yaw[idx] + M_PI);
    }
  }

  const double newest_time = obs_time[n - 1];
  const TrackedObject & newest_tmpl = raw[n - 1].original_info;

  constexpr double dt = constants::PREDICTION_TIME_STEP_S;
  const size_t num_timesteps = static_cast<size_t>(INPUT_T_WITH_CURRENT);
  const double ref_sec = frame_time.seconds();

  std::vector<AgentState> grid_states;
  grid_states.reserve(num_timesteps);

  size_t search_start = 0;  // obs times are ascending; carry the bracket search forward
  for (size_t k = 0; k < num_timesteps; ++k) {
    // k = 0 is the oldest slot, k = num_timesteps - 1 is anchored at frame_time.
    const double target_sec = ref_sec - static_cast<double>(num_timesteps - 1 - k) * dt;
    const rclcpp::Time target_time(
      frame_time - rclcpp::Duration::from_seconds(static_cast<double>(num_timesteps - 1 - k) * dt));

    if (target_sec <= obs_time[0]) {
      // Before the first observation: front-clamp (repeat the oldest), preserving its real twist.
      grid_states.push_back(make_grid_state(
        raw[0].original_info, obs_x[0], obs_y[0], obs_yaw[0], target_time, std::nullopt));
    } else if (target_sec <= newest_time) {
      // Within the observed history: interpolate position linearly, yaw along the shortest arc.
      while (search_start + 1 < n && obs_time[search_start + 1] < target_sec) {
        ++search_start;
      }
      const size_t i0 = search_start;
      const size_t i1 = search_start + 1;
      const double span = obs_time[i1] - obs_time[i0];
      const double ratio = (span > 0.0) ? (target_sec - obs_time[i0]) / span : 0.0;
      const double x = obs_x[i0] + ratio * (obs_x[i1] - obs_x[i0]);
      const double y = obs_y[i0] + ratio * (obs_y[i1] - obs_y[i0]);
      const double yaw = interpolate_yaw(obs_yaw[i0], obs_yaw[i1], ratio);
      const double speed = obs_speed[i0] + ratio * (obs_speed[i1] - obs_speed[i0]);
      grid_states.push_back(make_grid_state(raw[i1].original_info, x, y, yaw, target_time, speed));
    } else {
      // After the newest observation.
      if (history.is_live()) {
        // Live agent: extrapolate the leading edge to the grid time via the motion model.
        const MotionState start{obs_x[n - 1], obs_y[n - 1], obs_yaw[n - 1]};
        const MotionState propagated = propagate_motion(
          start, obs_speed[n - 1], obs_yaw_rate[n - 1], target_sec - newest_time, params);
        grid_states.push_back(make_grid_state(
          newest_tmpl, propagated.x, propagated.y, propagated.yaw, target_time, std::nullopt));
      } else {
        // Disappeared agent: freeze at the newest observation (no forward extrapolation). The
        // frozen slots let the agent age out without a phantom or teleport.
        grid_states.push_back(make_grid_state(
          newest_tmpl, obs_x[n - 1], obs_y[n - 1], obs_yaw[n - 1], target_time, std::nullopt));
      }
    }
  }

  return AgentHistory::from_states(grid_states, num_timesteps);
}

std::vector<AgentHistory> AgentData::resampled_transformed_and_trimmed_histories(
  const rclcpp::Time & frame_time, const Eigen::Matrix4d & transform, size_t max_num_agent,
  const HistoryAlignmentParams & params) const
{
  std::vector<AgentHistory> histories;
  histories.reserve(histories_map_.size());
  for (const auto & [_, history] : histories_map_) {
    auto resampled = resample_history(history, frame_time, params);
    if (resampled.has_value()) {
      histories.push_back(std::move(resampled.value()));
    }
  }
  return transform_sort_trim(std::move(histories), transform, max_num_agent);
}

}  // namespace autoware::diffusion_planner
