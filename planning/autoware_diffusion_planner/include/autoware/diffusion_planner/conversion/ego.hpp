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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONVERSION__EGO_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONVERSION__EGO_HPP_

#include "Eigen/Dense"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <autoware_perception_msgs/msg/detail/tracked_objects__struct.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace autoware::diffusion_planner
{
constexpr size_t EGO_STATE_DIM = 10;

using nav_msgs::msg::Odometry;

struct EgoState
{
  float x_{0.0f};
  float y_{0.0f};
  float cos_yaw_{1.0f};
  float sin_yaw_{0.0f};
  float vx_{0.0f};
  float vy_{0.0f};
  float ax_{0.0f};
  float ay_{0.0f};
  float steering_angle_{0.0f};
  float yaw_rate_{0.0f};

  std::vector<float> data_;
  static constexpr float MAX_YAW_RATE = 0.95f;
  static constexpr float MAX_STEER_ANGLE = static_cast<float>((2.0 / 3.0) * M_PI);

  EgoState(
    const nav_msgs::msg::Odometry & kinematic_state_msg,
    const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration_msg, float wheel_base);
  [[nodiscard]] std::string to_string() const;

  [[nodiscard]] std::vector<float> as_array() const noexcept { return data_; }
  [[nodiscard]] const float * data_ptr() const noexcept { return data_.data(); }
  [[nodiscard]] float x() const noexcept { return x_; }
  [[nodiscard]] float y() const noexcept { return y_; }
  [[nodiscard]] float cos_yaw() const noexcept { return cos_yaw_; }
  [[nodiscard]] float sin_yaw() const noexcept { return sin_yaw_; }
  [[nodiscard]] float vx() const noexcept { return vx_; }
  [[nodiscard]] float vy() const noexcept { return vy_; }
  [[nodiscard]] float ax() const noexcept { return ax_; }
  [[nodiscard]] float ay() const noexcept { return ay_; }
  [[nodiscard]] float steering_angle() const noexcept { return steering_angle_; }
  [[nodiscard]] float yaw_rate() const noexcept { return yaw_rate_; }
};

}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__CONVERSION__EGO_HPP_
