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

#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include "autoware/diffusion_planner/constants.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"
#include "autoware/diffusion_planner/utils/utils.hpp"

#include <autoware/universe_utils/geometry/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{
void normalize_input_data(InputDataMap & input_data_map, const NormalizationMap & normalization_map)
{
  auto normalize_vector = [](
                            std::vector<float> & data, const std::vector<float> & mean,
                            const std::vector<float> & std_dev) -> void {
    assert(!data.empty() && "Data vector must not be empty");
    assert((mean.size() == std_dev.size()) && "Mean and std must be same size");
    assert((data.size() % std_dev.size() == 0) && "Data size must be divisible by std_dev size");
    auto cols = std_dev.size();
    auto rows = data.size() / cols;

    for (size_t row = 0; row < rows; ++row) {
      const auto offset = row * cols;
      const auto row_begin =
        data.begin() + static_cast<std::vector<float>::difference_type>(offset);

      bool is_zero_row = std::all_of(
        row_begin, row_begin + static_cast<std::vector<float>::difference_type>(cols),
        [](float x) { return std::abs(x) < std::numeric_limits<float>::epsilon(); });

      if (is_zero_row) continue;

      for (size_t col = 0; col < cols; ++col) {
        float m = (mean.size() == 1) ? mean[0] : mean[col];
        float s = (std_dev.size() == 1) ? std_dev[0] : std_dev[col];
        // Prevent division by zero
        if (std::abs(s) < std::numeric_limits<float>::epsilon()) {
          throw std::runtime_error("Standard deviation is zero, cannot normalize data");
        }
        data[offset + col] = (data[offset + col] - m) / s;
      }
    }
  };

  for (auto & [key, value] : input_data_map) {
    // Skip normalization for ego_shape and sampled_trajectories
    if (key == "ego_shape" || key == "sampled_trajectories" || key == "turn_indicators") {
      continue;
    }

    if (normalization_map.find(key) == normalization_map.end()) {
      std::string err{"Missing key " + key + " from normalization map"};
      throw std::runtime_error(err.c_str());
    }

    const auto & [mean, std_dev] = normalization_map.at(key);
    normalize_vector(value, mean, std_dev);
  }
}

std::vector<float> create_ego_current_state(
  const nav_msgs::msg::Odometry & kinematic_state_msg,
  const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration_msg, const float wheel_base)
{
  constexpr float MAX_YAW_RATE = 0.95f;
  constexpr float MAX_STEER_ANGLE = static_cast<float>((2.0 / 3.0) * M_PI);

  const auto & lin = kinematic_state_msg.twist.twist.linear;
  const auto & ang = kinematic_state_msg.twist.twist.angular;

  float yaw_rate;
  float steering_angle;
  const float linear_vel = std::hypot(lin.x, lin.y);
  if (linear_vel < constants::MOVING_VELOCITY_THRESHOLD_MPS) {
    yaw_rate = 0.0f;
    steering_angle = 0.0f;
  } else {
    yaw_rate = std::clamp(static_cast<float>(ang.z), -MAX_YAW_RATE, MAX_YAW_RATE);
    const float raw_steer = std::atan(yaw_rate * wheel_base / linear_vel);
    steering_angle = std::clamp(raw_steer, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
  }

  const float vx = static_cast<float>(lin.x);
  const float vy = static_cast<float>(lin.y);
  const float ax = static_cast<float>(acceleration_msg.accel.accel.linear.x);
  const float ay = static_cast<float>(acceleration_msg.accel.accel.linear.y);

  // x, y, cos_yaw, sin_yaw are always 0, 0, 1, 0 in ego frame
  return {0.0f, 0.0f, 1.0f, 0.0f, vx, vy, ax, ay, steering_angle, yaw_rate};
}

std::vector<float> create_ego_agent_past(
  const std::deque<nav_msgs::msg::Odometry> & odom_msgs, size_t num_timesteps,
  const Eigen::Matrix4d & map_to_ego_transform, const std::optional<rclcpp::Time> & reference_time)
{
  const size_t features_per_timestep = 4;  // x, y, cos, sin
  const size_t total_size = num_timesteps * features_per_timestep;

  std::vector<float> ego_agent_past(total_size, 0.0f);

  // Initialize cos values to 1.0 (identity heading)
  for (size_t t = 0; t < num_timesteps; ++t) {
    ego_agent_past[t * features_per_timestep + EGO_AGENT_PAST_IDX_COS] = 1.0f;
  }

  // If no odometry messages are available, return the default-initialized ego_agent_past
  if (odom_msgs.empty()) {
    return ego_agent_past;
  }

  // Lambda to transform a pose to ego frame and store into the flat array
  auto store_pose = [&](size_t timestep_idx, const geometry_msgs::msg::Pose & pose) {
    const Eigen::Matrix4d pose_map_4x4 = utils::pose_to_matrix4d(pose);
    const Eigen::Matrix4d pose_ego_4x4 = map_to_ego_transform * pose_map_4x4;
    const auto [cos_yaw, sin_yaw] =
      utils::rotation_matrix_to_cos_sin(pose_ego_4x4.block<3, 3>(0, 0));
    const size_t base_idx = timestep_idx * features_per_timestep;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_X] = pose_ego_4x4(0, 3);
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_Y] = pose_ego_4x4(1, 3);
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_COS] = cos_yaw;
    ego_agent_past[base_idx + EGO_AGENT_PAST_IDX_SIN] = sin_yaw;
  };

  if (!reference_time.has_value()) {
    // Legacy behavior: use the last num_timesteps odom messages directly
    const size_t start_idx =
      (odom_msgs.size() >= num_timesteps) ? odom_msgs.size() - num_timesteps : 0;
    for (size_t i = start_idx; i < odom_msgs.size(); ++i) {
      store_pose(i - start_idx, odom_msgs[i].pose.pose);
    }
    return ego_agent_past;
  }

  // Time-based interpolation behavior
  auto stamp_to_sec = [](const builtin_interfaces::msg::Time & stamp) -> double {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
  };

  const double ref_sec = reference_time->seconds();
  constexpr double dt = constants::PREDICTION_TIME_STEP_S;  // 0.1s

  const double first_sec = stamp_to_sec(odom_msgs.front().header.stamp);
  const double last_sec = stamp_to_sec(odom_msgs.back().header.stamp);

  // Both target_sec and odom timestamps are monotonically increasing,
  // so we can carry the search index forward across iterations.
  size_t search_start = 0;

  for (size_t t = 0; t < num_timesteps; ++t) {
    // t=0 is the oldest, t=num_timesteps-1 is the reference time
    const double target_sec = ref_sec - static_cast<double>(num_timesteps - 1 - t) * dt;

    geometry_msgs::msg::Pose interpolated_pose;
    if (target_sec <= first_sec) {
      interpolated_pose = odom_msgs.front().pose.pose;
    } else if (target_sec >= last_sec) {
      interpolated_pose = odom_msgs.back().pose.pose;
    } else {
      // Find the two bracketing odom messages, continuing from previous position
      for (; search_start + 1 < odom_msgs.size(); ++search_start) {
        const double t_next = stamp_to_sec(odom_msgs[search_start + 1].header.stamp);
        if (target_sec <= t_next) {
          break;
        }
      }

      const double t0 = stamp_to_sec(odom_msgs[search_start].header.stamp);
      const double t1 = stamp_to_sec(odom_msgs[search_start + 1].header.stamp);
      const double ratio = (t1 > t0) ? (target_sec - t0) / (t1 - t0) : 0.0;

      interpolated_pose = autoware::universe_utils::calcInterpolatedPose(
        odom_msgs[search_start].pose.pose, odom_msgs[search_start + 1].pose.pose, ratio, false);
    }

    store_pose(t, interpolated_pose);
  }

  return ego_agent_past;
}

std::vector<float> create_sampled_trajectories(const double temperature)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> dist(0.0f, 1.0f);
  std::vector<float> sampled_trajectories((MAX_NUM_NEIGHBORS + 1) * (OUTPUT_T + 1) * POSE_DIM);
  for (float & val : sampled_trajectories) {
    val = dist(gen) * static_cast<float>(temperature);
  }
  return sampled_trajectories;
}

}  // namespace autoware::diffusion_planner::preprocess
