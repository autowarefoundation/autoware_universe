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

#include "autoware/diffusion_planner/conversion/ego.hpp"

#include "autoware/diffusion_planner/constants.hpp"

#include <string>

namespace autoware::diffusion_planner
{
EgoState::EgoState(
  const nav_msgs::msg::Odometry & kinematic_state_msg,
  const geometry_msgs::msg::AccelWithCovarianceStamped & acceleration_msg, float wheel_base)
{
  const auto & lin = kinematic_state_msg.twist.twist.linear;
  const auto & ang = kinematic_state_msg.twist.twist.angular;

  const float linear_vel = std::hypot(lin.x, lin.y);
  if (linear_vel < constants::MOVING_VELOCITY_THRESHOLD_MPS) {
    yaw_rate_ = 0.0f;
    steering_angle_ = 0.0f;
  } else {
    yaw_rate_ = std::clamp(static_cast<float>(ang.z), -MAX_YAW_RATE, MAX_YAW_RATE);
    float raw_steer = std::atan(yaw_rate_ * wheel_base / std::abs(linear_vel));
    steering_angle_ = std::clamp(raw_steer, -MAX_STEER_ANGLE, MAX_STEER_ANGLE);
  }

  vx_ = lin.x;
  vy_ = lin.y;
  ax_ = acceleration_msg.accel.accel.linear.x;
  ay_ = acceleration_msg.accel.accel.linear.y;
  data_ = {x_, y_, cos_yaw_, sin_yaw_, vx_, vy_, ax_, ay_, steering_angle_, yaw_rate_};
}

[[nodiscard]] std::string EgoState::to_string() const
{
  std::ostringstream oss;
  oss << "EgoState: [";
  oss << "x: " << x_ << ", ";
  oss << "y: " << y_ << ", ";
  oss << "cos_yaw: " << cos_yaw_ << ", ";
  oss << "sin_yaw: " << sin_yaw_ << ", ";
  oss << "vx: " << vx_ << ", ";
  oss << "vy: " << vy_ << ", ";
  oss << "ax: " << ax_ << ", ";
  oss << "ay: " << ay_ << ", ";
  oss << "steering_angle: " << steering_angle_ << ", ";
  oss << "yaw_rate: " << yaw_rate_;
  oss << "]";
  return oss.str();
}

}  // namespace autoware::diffusion_planner
