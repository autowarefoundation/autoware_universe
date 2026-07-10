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

#ifndef AUTOWARE__PID_LONGITUDINAL_CONTROLLER__SMOOTH_STOP_HPP_
#define AUTOWARE__PID_LONGITUDINAL_CONTROLLER__SMOOTH_STOP_HPP_

#include "autoware/pid_longitudinal_controller/debug_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::motion::control::pid_longitudinal_controller
{

/**
 * @brief Smooth stop class to implement vehicle specific deceleration profiles
 */
class SmoothStop
{
public:
  struct Params
  {
    double max_strong_acc;
    double min_strong_acc;
    double weak_acc;
    double weak_stop_acc;
    double strong_stop_acc;

    double min_fast_vel;
    double min_running_vel;
    double min_running_acc;
    double weak_stop_time;

    double weak_stop_dist;
    double strong_stop_dist;
  };

  explicit SmoothStop(const Params & params) : m_params(params) {}

  /**
   * @brief initialize the state of the smooth stop
   * @param [in] pred_vel_in_target predicted ego velocity when the stop command will be executed
   * @param [in] pred_stop_dist predicted stop distance when the stop command will be executed
   * @param [in] current_time time at which this initialization occurs
   */
  void init(
    const double pred_vel_in_target, const double pred_stop_dist,
    const rclcpp::Time & current_time);

  /**
   * @brief update the parameters of this smooth stop, e.g. on a dynamic reconfiguration
   * @param [in] params new parameters to apply
   */
  void setParams(const Params & params);

  /**
   * @brief calculate accel command while stopping
   *        Decrease velocity with m_strong_acc,
   *        then loose brake pedal with m_params.weak_acc to stop smoothly
   *        If the car is still running, input m_params.weak_stop_acc
   *        and then m_params.strong_stop_acc in steps not to exceed stopline too much
   * @param [in] stop_dist distance left to travel before stopping [m]
   * @param [in] current_vel current velocity of ego [m/s]
   * @param [in] current_acc current acceleration of ego [m/s²]
   * @param [in] vel_hist history of previous ego velocities as (rclcpp::Time, double[m/s]) pairs
   * @param [in] delay_time assumed time delay when the stop command will actually be executed
   * @param [in] current_time time at which this calculation occurs
   */
  double calculate(
    const double stop_dist, const double current_vel, const double current_acc,
    const std::vector<std::pair<rclcpp::Time, double>> & vel_hist, const double delay_time,
    const rclcpp::Time & current_time, DebugValues & debug_values);

private:
  Params m_params;

  enum class Mode { STRONG = 0, WEAK, WEAK_STOP, STRONG_STOP };

  double m_strong_acc;
  rclcpp::Time m_weak_acc_time;
};
}  // namespace autoware::motion::control::pid_longitudinal_controller

#endif  // AUTOWARE__PID_LONGITUDINAL_CONTROLLER__SMOOTH_STOP_HPP_
