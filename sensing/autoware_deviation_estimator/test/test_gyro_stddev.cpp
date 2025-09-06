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

#include "autoware/deviation_estimator/deviation_estimator.hpp"
#include "autoware/universe_utils/geometry/geometry.hpp"

#include <gtest/gtest.h>

#include <random>
#include <vector>

TEST(DeviationEstimatorGyroStddev, SmokeTestDefault)
{
  const double ERROR_RATE = 0.1;

  const int n_all = 10000;
  const rclcpp::Time t_start = rclcpp::Time(0, 0);
  const double stddev_gyro = 0.03141592;
  const int gyro_rate = 100;
  const int ndt_rate = 10;
  const double t_window = 5;
  const geometry_msgs::msg::Vector3 gyro_bias =
    autoware::universe_utils::createVector3(0.0, 0.0, 0.0);

  std::mt19937 engine;
  engine.seed();
  std::normal_distribution<> dist(0.0, stddev_gyro);

  std::vector<autoware::deviation_estimator::TrajectoryData> traj_data_list;

  for (int tmp = 0; tmp < n_all; ++tmp) {
    std::vector<geometry_msgs::msg::Vector3Stamped> gyro_data_while_stopped;
    for (int i = 0; i <= gyro_rate * t_window; ++i) {
      geometry_msgs::msg::Vector3Stamped gyro;
      gyro.header.stamp = t_start + rclcpp::Duration::from_seconds(1.0 * i / gyro_rate);
      gyro.vector.x = dist(engine) + gyro_bias.x;
      gyro.vector.y = dist(engine) + gyro_bias.y;
      gyro.vector.z = dist(engine) + gyro_bias.z;
      gyro_data_while_stopped.push_back(gyro);
    }

    std::vector<geometry_msgs::msg::PoseStamped> pose_list;
    for (int i = 0; i <= ndt_rate * t_window; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = t_start + rclcpp::Duration::from_seconds(1.0 * i / ndt_rate);
      pose.pose.orientation = autoware::universe_utils::createQuaternionFromRPY(0.0, 0.0, 0.0);
      pose_list.push_back(pose);
    }

    autoware::deviation_estimator::TrajectoryData traj_data;
    traj_data.pose_list = pose_list;
    traj_data.gyro_list = gyro_data_while_stopped;
    traj_data_list.push_back(traj_data);
  }

  geometry_msgs::msg::Vector3 estimated_gyro_stddev =
    autoware::deviation_estimator::estimate_stddev_angular_velocity(traj_data_list, gyro_bias);

  EXPECT_NEAR(estimated_gyro_stddev.x, stddev_gyro, stddev_gyro * ERROR_RATE);
  EXPECT_NEAR(estimated_gyro_stddev.y, stddev_gyro, stddev_gyro * ERROR_RATE);
  EXPECT_NEAR(estimated_gyro_stddev.z, stddev_gyro, stddev_gyro * ERROR_RATE);
}

TEST(DeviationEstimatorGyroStddev, SmokeTestWithBias)
{
  const double ERROR_RATE = 0.1;

  const int n_all = 10000;
  const rclcpp::Time t_start = rclcpp::Time(0, 0);
  const double stddev_gyro = 0.03141592;
  const int gyro_rate = 30;
  const int ndt_rate = 10;
  const double t_window = 5;
  const geometry_msgs::msg::Vector3 gyro_bias =
    autoware::universe_utils::createVector3(0.005, 0.001, -0.01);

  std::mt19937 engine;
  engine.seed();
  std::normal_distribution<> dist(0.0, stddev_gyro);

  std::vector<autoware::deviation_estimator::TrajectoryData> traj_data_list;

  for (int tmp = 0; tmp < n_all; ++tmp) {
    std::vector<geometry_msgs::msg::Vector3Stamped> gyro_data_while_stopped;
    for (int i = 0; i <= gyro_rate * t_window; ++i) {
      geometry_msgs::msg::Vector3Stamped gyro;
      gyro.header.stamp = t_start + rclcpp::Duration::from_seconds(1.0 * i / gyro_rate);
      gyro.vector.x = dist(engine) + gyro_bias.x;
      gyro.vector.y = dist(engine) + gyro_bias.y;
      gyro.vector.z = dist(engine) + gyro_bias.z;
      gyro_data_while_stopped.push_back(gyro);
    }

    std::vector<geometry_msgs::msg::PoseStamped> pose_list;
    for (int i = 0; i <= ndt_rate * t_window; ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = t_start + rclcpp::Duration::from_seconds(1.0 * i / ndt_rate);
      pose.pose.orientation = autoware::universe_utils::createQuaternionFromRPY(0.0, 0.0, 0.0);
      pose_list.push_back(pose);
    }

    autoware::deviation_estimator::TrajectoryData traj_data;
    traj_data.pose_list = pose_list;
    traj_data.gyro_list = gyro_data_while_stopped;
    traj_data_list.push_back(traj_data);
  }

  geometry_msgs::msg::Vector3 estimated_gyro_stddev =
    autoware::deviation_estimator::estimate_stddev_angular_velocity(traj_data_list, gyro_bias);

  EXPECT_NEAR(estimated_gyro_stddev.x, stddev_gyro, stddev_gyro * ERROR_RATE);
  EXPECT_NEAR(estimated_gyro_stddev.y, stddev_gyro, stddev_gyro * ERROR_RATE);
  EXPECT_NEAR(estimated_gyro_stddev.z, stddev_gyro, stddev_gyro * ERROR_RATE);
}
