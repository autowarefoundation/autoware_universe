// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__EGO_HISTORY_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__EGO_HISTORY_HPP_

#include "autoware/diffusion_planner/constants.hpp"

#include <Eigen/Core>
#include <rclcpp/time.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <deque>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner::preprocess
{

/**
 * @brief Owns the ego odometry buffer and guarantees it stays monotonic oldest-to-newest.
 *
 * The buffer stores raw (unshifted) odometry. update() enforces the ordering invariant that every
 * consumer relies on: the window prune treats back() as newest, and both select_state() and
 * to_agent_past() assume monotonic timestamps. A backwards stamp (bag loop, sim reset, or an
 * out-of-order message) would otherwise stall pruning and corrupt interpolation, so update() drops
 * any buffered samples not older than an incoming stamp before appending it.
 */
class EgoHistory
{
public:
  explicit EgoHistory(double buffer_window_s = constants::ODOMETRY_BUFFER_WINDOW_S);

  /**
   * @brief Append newly received raw odometry, keep the buffer monotonic, prune to the time window.
   *
   * Null entries are skipped. A single out-of-order stamp trims only the offending tail samples
   * (older history is kept); a large backwards jump empties the stale timeline since every buffered
   * sample is newer than the reset.
   */
  void update(const std::vector<std::shared_ptr<const nav_msgs::msg::Odometry>> & ego_states);

  bool empty() const { return buffer_.empty(); }

  /// @brief Stamp of the newest buffered sample. Precondition: !empty().
  rclcpp::Time newest_stamp() const;

  /**
   * @brief Select the ego state at frame_time. Precondition: !empty().
   *
   * With use_time_interpolation the pose and twist are linearly interpolated between the two
   * buffered samples bracketing frame_time; otherwise, or when frame_time is outside the buffered
   * range, the nearest sample is returned.
   *
   * @return The selected ego state and its absolute time offset from frame_time [s] (0 when
   *         interpolated within the buffer range).
   */
  std::pair<nav_msgs::msg::Odometry, double> select_state(
    const rclcpp::Time & frame_time, bool use_time_interpolation) const;

  /**
   * @brief Build the past ego trajectory expressed in the current ego frame.
   *
   * Resamples the buffer at fixed PREDICTION_TIME_STEP_S intervals backwards from reference_time
   * (t=0 is the oldest step, t=num_timesteps-1 is reference_time). When shift_x is set, the same
   * base_link-to-center shift as the reference transform is applied so the trajectory is consistent
   * with map_to_ego_transform.
   *
   * @return Flat [x, y, cos_yaw, sin_yaw] per timestep, length EGO_HISTORY_SHAPE[1] * 4.
   */
  std::vector<float> to_agent_past(
    const Eigen::Matrix4d & map_to_ego_transform, const rclcpp::Time & reference_time,
    bool use_time_interpolation, bool shift_x, double base_link_to_center) const;

  // --- Static primitives (relocated from utils::select_ego_state /
  // preprocess::create_ego_agent_past).
  //     They operate on an explicit oldest-to-newest buffer and are exposed for focused unit tests;
  //     the instance methods above apply them to the owned buffer.

  static std::pair<nav_msgs::msg::Odometry, double> select_ego_state(
    const std::deque<nav_msgs::msg::Odometry> & buffer, const rclcpp::Time & frame_time,
    bool use_time_interpolation);

  static std::vector<float> create_ego_agent_past(
    const std::deque<nav_msgs::msg::Odometry> & buffer, size_t num_timesteps,
    const Eigen::Matrix4d & map_to_ego_transform,
    const std::optional<rclcpp::Time> & reference_time = std::nullopt,
    bool use_time_interpolation = true);

private:
  std::deque<nav_msgs::msg::Odometry> buffer_;
  double buffer_window_s_;
};

}  // namespace autoware::diffusion_planner::preprocess

#endif  // AUTOWARE__DIFFUSION_PLANNER__PREPROCESSING__EGO_HISTORY_HPP_
