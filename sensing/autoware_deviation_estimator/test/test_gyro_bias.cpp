// Copyright 2022 Autoware Foundation
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

#include "autoware/deviation_estimator/gyro_bias_module.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <random>
#include <vector>

TEST(DeviationEstimatorGyroBias, SmokeTestDefault)
{
  const geometry_msgs::msg::Vector3 gyro_bias =
    autoware::universe_utils::createVector3(0.01, -0.005, -0.015);

  const double ERROR_RATE = 0.1;
  const int num_data = 1000;
  const double dt = 1.0;

  const int gyro_rate = 100;
  const int ndt_rate = 10;
  const double stddev_gyro = 0.02;
  const rclcpp::Time t_start = rclcpp::Time(0, 0);

  autoware::deviation_estimator::GyroBiasModule gyro_bias_module;

  std::mt19937 engine;
  engine.seed();
  std::normal_distribution<> dist(0.0, stddev_gyro);

  for (int i = 0; i < num_data; ++i) {
    std::vector<geometry_msgs::msg::Vector3Stamped> gyro_data_while_stopped;
    for (int i = 0; i <= gyro_rate * dt; ++i) {
      geometry_msgs::msg::Vector3Stamped gyro;
      gyro.header.stamp = t_start + rclcpp::Duration::from_seconds(1.0 * i / gyro_rate);
      gyro.vector.x = dist(engine) + gyro_bias.x;
      gyro.vector.y = dist(engine) + gyro_bias.y;
      gyro.vector.z = dist(engine) + gyro_bias.z;
      gyro_data_while_stopped.push_back(gyro);
    }

    std::vector<geometry_msgs::msg::PoseStamped> pose_list;
    for (int i = 0; i <= ndt_rate * dt; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = t_start + rclcpp::Duration::from_seconds(1.0 * i / ndt_rate);
      pose.pose.orientation = autoware::universe_utils::createQuaternionFromRPY(0.0, 0.0, 0.0);
      pose_list.push_back(pose);
    }

    autoware::deviation_estimator::TrajectoryData traj_data;
    traj_data.pose_list = pose_list;
    traj_data.gyro_list = gyro_data_while_stopped;

    gyro_bias_module.update_bias(traj_data);
  }

  const geometry_msgs::msg::Vector3 estimated_gyro_bias = gyro_bias_module.get_bias_base_link();
  EXPECT_NEAR(estimated_gyro_bias.x, gyro_bias.x, std::abs(gyro_bias.x) * ERROR_RATE);
  EXPECT_NEAR(estimated_gyro_bias.y, gyro_bias.y, std::abs(gyro_bias.y) * ERROR_RATE);
  EXPECT_NEAR(estimated_gyro_bias.z, gyro_bias.z, std::abs(gyro_bias.z) * ERROR_RATE);
}
