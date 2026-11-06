// Copyright 2026 Autoware Foundation
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

#include "autoware/imu_corrector/imu_corrector_core.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

namespace autoware::imu_corrector
{
class ImuCorrectorTransformTest : public testing::TestWithParam<int>
{
};

TEST_P(ImuCorrectorTransformTest, TransformsFromImuToOutputFrame)
{
  using namespace std::chrono_literals;
  const auto corrector = std::make_shared<ImuCorrector>(rclcpp::NodeOptions{});
  const auto test_node = std::make_shared<rclcpp::Node>("imu_transform_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(corrector->get_node_base_interface());
  executor.add_node(test_node);

  sensor_msgs::msg::Imu::ConstSharedPtr received;
  const auto subscriber = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "output", rclcpp::QoS{10},
    [&received](sensor_msgs::msg::Imu::ConstSharedPtr msg) { received = msg; });
  const auto publisher = test_node->create_publisher<sensor_msgs::msg::Imu>("input", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster(test_node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = test_node->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "imu_" + std::to_string(GetParam());
  const double yaw = GetParam() * std::acos(-1.0) / 2.0;
  transform.transform.rotation.z = std::sin(yaw / 2.0);
  transform.transform.rotation.w = std::cos(yaw / 2.0);
  // Translation must not affect either vector.
  transform.transform.translation.x = 2.0;
  transform.transform.translation.y = -3.0;
  broadcaster.sendTransform(transform);

  sensor_msgs::msg::Imu input;
  input.header.stamp = test_node->now();
  input.header.frame_id = transform.child_frame_id;
  input.linear_acceleration.y = 1.0;
  input.linear_acceleration.z = 2.0;
  input.angular_velocity.x = 1.0;
  input.angular_velocity.z = 3.0;

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!received && std::chrono::steady_clock::now() < deadline) {
    publisher->publish(input);
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_NE(received, nullptr);
  EXPECT_EQ(received->header.frame_id, "base_link");
  EXPECT_EQ(received->header.stamp, input.header.stamp);
  EXPECT_NEAR(received->linear_acceleration.x, -std::sin(yaw), 1e-9);
  EXPECT_NEAR(received->linear_acceleration.y, std::cos(yaw), 1e-9);
  EXPECT_NEAR(received->linear_acceleration.z, 2.0, 1e-9);
  EXPECT_NEAR(received->angular_velocity.x, std::cos(yaw), 1e-9);
  EXPECT_NEAR(received->angular_velocity.y, std::sin(yaw), 1e-9);
  EXPECT_NEAR(received->angular_velocity.z, 3.0, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(
  IdentityAndQuarterTurns, ImuCorrectorTransformTest, testing::Values(0, -1, 1));
}  // namespace autoware::imu_corrector
