// Copyright 2021 Tier IV, Inc.
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

#include "autoware/pid_longitudinal_controller/smooth_stop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{
void SmoothStop::init(
  const double pred_vel_in_target, const double pred_stop_dist, const rclcpp::Time & current_time)
{
  m_weak_acc_time = current_time;

  // when distance to stopline is near the car
  if (pred_stop_dist < std::numeric_limits<double>::epsilon()) {
    m_strong_acc = m_params.min_strong_acc;
    return;
  }

  m_strong_acc = -std::pow(pred_vel_in_target, 2) / (2 * pred_stop_dist);
  m_strong_acc = std::max(std::min(m_strong_acc, m_params.max_strong_acc), m_params.min_strong_acc);
}

void SmoothStop::setParams(const Params & params)
{
  m_params = params;
}

void SmoothStop::recordMotion(
  const rclcpp::Time & time, const double vel, const double acc, const double time_window)
{
  m_vel_hist.emplace_back(time, vel);
  m_current_acc = acc;
  while (!m_vel_hist.empty() && (time - m_vel_hist.front().first).seconds() > time_window) {
    m_vel_hist.erase(m_vel_hist.begin());
  }
}

namespace
{
std::optional<double> calcTimeToStop(const std::vector<std::pair<rclcpp::Time, double>> & vel_hist)
{
  // return when vel_hist is empty
  if (vel_hist.empty()) {
    return {};
  }
  const rclcpp::Time & current_time = vel_hist.back().first;
  const double vel_hist_size = static_cast<double>(vel_hist.size());

  // calculate some variables for fitting
  double mean_t = 0.0;
  double mean_v = 0.0;
  double sum_tv = 0.0;
  double sum_tt = 0.0;
  for (const auto & vel : vel_hist) {
    const double t = (vel.first - current_time).seconds();
    const double v = vel.second;

    mean_t += t / vel_hist_size;
    mean_v += v / vel_hist_size;
    sum_tv += t * v;
    sum_tt += t * t;
  }

  // return when gradient a (of v = at + b) cannot be calculated.
  // See the following calculation of a
  if (std::abs(vel_hist_size * mean_t * mean_t - sum_tt) < std::numeric_limits<double>::epsilon()) {
    return {};
  }

  // calculate coefficients of linear function (v = at + b)
  const double a =
    (vel_hist_size * mean_t * mean_v - sum_tv) / (vel_hist_size * mean_t * mean_t - sum_tt);
  const double b = mean_v - a * mean_t;

  // return when v is independent of time (v = b)
  if (std::abs(a) < std::numeric_limits<double>::epsilon()) {
    return {};
  }

  // calculate time to stop by substituting v = 0 for v = at + b
  const double time_to_stop = -b / a;
  if (time_to_stop > 0) {
    return time_to_stop;
  }

  return {};
}
}  // namespace

SmoothStop::Result SmoothStop::calculate(const double stop_dist, const double delay_time)
{
  // recordMotion() is called every cycle before calculate(), so the latest sample
  // always reflects the current velocity, acceleration and time.
  const auto & [current_time, current_vel] = m_vel_hist.back();

  // calculate some flags
  const bool is_running = std::abs(current_vel) > m_params.min_running_vel ||
                          std::abs(m_current_acc) > m_params.min_running_acc;

  // when exceeding the stopline (stop_dist is negative in these cases.)
  if (stop_dist < m_params.strong_stop_dist) {  // when exceeding the stopline much
    return Result{m_params.strong_stop_acc, Mode::STRONG_STOP};
  } else if (stop_dist < m_params.weak_stop_dist) {  // when exceeding the stopline a bit
    return Result{m_params.weak_stop_acc, Mode::WEAK_STOP};
  }

  // when the car is running
  if (is_running) {
    // predict time to stop
    const auto time_to_stop = calcTimeToStop(m_vel_hist);
    const bool is_fast_vel = std::abs(current_vel) > m_params.min_fast_vel;

    // when the car will not stop in a certain time
    if (
      (time_to_stop && *time_to_stop > m_params.weak_stop_time + delay_time) ||
      (!time_to_stop && is_fast_vel)) {
      return Result{m_strong_acc, Mode::STRONG};
    }

    m_weak_acc_time = current_time;
    return Result{m_params.weak_acc, Mode::WEAK};
  }

  // for 0.5 seconds after the car stopped
  if ((current_time - m_weak_acc_time).seconds() < 0.5) {
    return Result{m_params.weak_acc, Mode::WEAK};
  }

  // when the car is not running
  return Result{m_params.strong_stop_acc, Mode::STRONG_STOP};
}
}  // namespace autoware::motion::control::pid_longitudinal_controller
