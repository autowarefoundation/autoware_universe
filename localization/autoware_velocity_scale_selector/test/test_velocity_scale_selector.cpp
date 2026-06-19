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

#include "velocity_scale_selector.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

static autoware_map_msgs::msg::LaneletMapBin make_map_bin(const std::string & subtype)
{
  auto map = std::make_shared<lanelet::LaneletMap>();
  lanelet::Polygon3d poly(
    lanelet::utils::getId(),
    {
      lanelet::Point3d(lanelet::utils::getId(), -1.0, -1.0, 0.0),
      lanelet::Point3d(lanelet::utils::getId(), 1.0, -1.0, 0.0),
      lanelet::Point3d(lanelet::utils::getId(), 1.0, 1.0, 0.0),
      lanelet::Point3d(lanelet::utils::getId(), -1.0, 1.0, 0.0),
    });
  poly.setAttribute(lanelet::AttributeName::Type, "feature_environment_specify");
  poly.setAttribute(lanelet::AttributeName::Subtype, subtype);
  map->add(poly);
  return autoware::experimental::lanelet2_utils::to_autoware_map_msgs(map);
}

static rclcpp::NodeOptions base_options()
{
  return rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .append_parameter_override("default_environment_id", 0)
    .append_parameter_override("default_longitudinal_scale_factor", 1.0)
    .append_parameter_override("area_subtype_uniform_road.environment_id", 1)
    .append_parameter_override("environment_1_longitudinal_scale_factor", 1.0075);
}

class TestHelper : public rclcpp::Node
{
public:
  TestHelper()
  : Node("test_helper")
  {
    map_pub_ = create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/velocity_scale_selector/input/lanelet2_map",
      rclcpp::QoS(1).transient_local());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/velocity_scale_selector/input/pose_with_covariance", 10);

    twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/velocity_scale_selector/input/twist_with_covariance", 10);

    result_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "/velocity_scale_selector/output/twist_with_covariance", 10,
      [this](geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg) {
        last_linear_x = msg->twist.twist.linear.x;
      });
  }

  void publish_map(const autoware_map_msgs::msg::LaneletMapBin & bin) { map_pub_->publish(bin); }

  void publish_pose(double x, double y)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    pose_pub_->publish(msg);
  }

  void publish_twist(double linear_x)
  {
    last_linear_x.reset();
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.frame_id = "base_link";
    msg.header.stamp = now();
    msg.twist.twist.linear.x = linear_x;
    twist_pub_->publish(msg);
  }

  std::optional<double> last_linear_x;

private:
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr result_sub_;
};

class VelocityScaleSelectorTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    helper_ = std::make_shared<TestHelper>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(helper_);
  }

  void TearDown() override
  {
    if (node_) executor_->remove_node(node_);
    executor_.reset();
    node_.reset();
    helper_.reset();
  }

  void create_node(const rclcpp::NodeOptions & opts)
  {
    node_ = std::make_shared<autoware::velocity_scale_selector::VelocityScaleSelector>(opts);
    executor_->add_node(node_);
  }

  bool spin_until_result(std::chrono::milliseconds timeout = 3000ms)
  {
    const auto start = std::chrono::steady_clock::now();
    while (!helper_->last_linear_x) {
      executor_->spin_some(10ms);
      std::this_thread::sleep_for(5ms);
      if (std::chrono::steady_clock::now() - start > timeout) return false;
    }
    return true;
  }

  std::shared_ptr<autoware::velocity_scale_selector::VelocityScaleSelector> node_;
  std::shared_ptr<TestHelper> helper_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(VelocityScaleSelectorTest, test_map_not_ready)
{
  create_node(base_options());

  helper_->publish_twist(10.0);

  ASSERT_TRUE(spin_until_result());
  EXPECT_DOUBLE_EQ(helper_->last_linear_x.value(), 10.0);
}

TEST_F(VelocityScaleSelectorTest, test_point_outside_polygon)
{
  create_node(base_options());

  helper_->publish_map(make_map_bin("uniform_road"));
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(10.0, 10.0);
  helper_->publish_twist(10.0);

  ASSERT_TRUE(spin_until_result());
  EXPECT_DOUBLE_EQ(helper_->last_linear_x.value(), 10.0);
}

TEST_F(VelocityScaleSelectorTest, test_point_inside_polygon_known_subtype)
{
  create_node(base_options());

  helper_->publish_map(make_map_bin("uniform_road"));
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(0.0, 0.0);
  helper_->publish_twist(10.0);

  ASSERT_TRUE(spin_until_result());
  EXPECT_DOUBLE_EQ(helper_->last_linear_x.value(), 10.0 * 1.0075);
}

TEST_F(VelocityScaleSelectorTest, test_point_inside_polygon_unknown_subtype)
{
  create_node(base_options());

  helper_->publish_map(make_map_bin("unknown_subtype"));
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(0.0, 0.0);
  helper_->publish_twist(10.0);

  ASSERT_TRUE(spin_until_result());
  EXPECT_DOUBLE_EQ(helper_->last_linear_x.value(), 10.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
