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

#include "covariance_selector.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Polygon.h>

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

// Build a LaneletMapBin with one square polygon from (-1,-1) to (1,1).
// type = "feature_environment_specify", subtype = given string.
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

// Build a flat covariance array with the given diagonal value.
static std::vector<double> diag_cov(double v)
{
  std::vector<double> c(36, 0.0);
  for (int i = 0; i < 6; ++i) c[i * 6 + i] = v;
  return c;
}

// Base NodeOptions shared by most tests:
//   default_output_pose_covariance: diagonal 1.0
//   environment_1_output_pose_covariance: diagonal 2.0
//   area_subtype_uniform_road -> env_id 1
static rclcpp::NodeOptions base_options()
{
  return rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .append_parameter_override("default_environment_id", 0)
    .append_parameter_override("area_subtype_uniform_road.environment_id", 1)
    .append_parameter_override("default_output_pose_covariance", diag_cov(1.0))
    .append_parameter_override("environment_1_output_pose_covariance", diag_cov(2.0));
}

// ---------------------------------------------------------------------------
// Helper node: publishes map/pose and captures output covariance
// ---------------------------------------------------------------------------
class TestHelper : public rclcpp::Node
{
public:
  TestHelper()
  : Node("test_helper")
  {
    map_pub_ = create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/covariance_selector/input/lanelet2_map",
      rclcpp::QoS(1).transient_local());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/covariance_selector/input/pose_with_covariance", 10);

    result_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/covariance_selector/output/pose_with_covariance", 10,
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg) {
        std::array<double, 36> cov{};
        for (size_t i = 0; i < 36; ++i) cov[i] = msg->pose.covariance[i];
        last_covariance = cov;
      });
  }

  void publish_map(const autoware_map_msgs::msg::LaneletMapBin & bin) { map_pub_->publish(bin); }

  void publish_pose(double x, double y)
  {
    last_covariance.reset();
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    pose_pub_->publish(msg);
  }

  std::optional<std::array<double, 36>> last_covariance;

private:
  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr result_sub_;
};

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class CovarianceSelectorTest : public ::testing::Test
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
    node_ = std::make_shared<autoware::covariance_selector::CovarianceSelector>(opts);
    executor_->add_node(node_);
  }

  bool spin_until_result(std::chrono::milliseconds timeout = 3000ms)
  {
    const auto start = std::chrono::steady_clock::now();
    while (!helper_->last_covariance) {
      executor_->spin_some(10ms);
      std::this_thread::sleep_for(5ms);
      if (std::chrono::steady_clock::now() - start > timeout) return false;
    }
    return true;
  }

  std::shared_ptr<autoware::covariance_selector::CovarianceSelector> node_;
  std::shared_ptr<TestHelper> helper_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Branch 1: is_map_ready_ == false → publishes with default_covariance_
TEST_F(CovarianceSelectorTest, test_map_not_ready)
{
  create_node(base_options());

  helper_->publish_pose(0.0, 0.0);  // no map published

  ASSERT_TRUE(spin_until_result());
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[0], 1.0);   // diagonal of default_cov
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[7], 1.0);
}

// Branch 6: within == false (point outside polygon) → default_env_id
// Branch 2: env_id=0 not in covariance_map → default_covariance_
TEST_F(CovarianceSelectorTest, test_point_outside_polygon_env_not_in_map)
{
  create_node(base_options());  // no environment_0 covariance configured

  helper_->publish_map(make_map_bin("uniform_road"));
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(10.0, 10.0);  // outside (-1,-1)to(1,1)

  ASSERT_TRUE(spin_until_result());
  // env_id=0, missing from covariance_map → default_covariance_ (diag 1.0)
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[0], 1.0);
}

// Branch 6: within == false → default_env_id=0
// Branch 3: env_id=0 found in covariance_map → environment-specific covariance
TEST_F(CovarianceSelectorTest, test_point_outside_polygon_env_in_map)
{
  auto opts = base_options();
  opts.append_parameter_override("environment_0_output_pose_covariance", diag_cov(3.0));
  create_node(opts);

  helper_->publish_map(make_map_bin("uniform_road"));
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(10.0, 10.0);  // outside polygon → env_id=0

  ASSERT_TRUE(spin_until_result());
  // env_id=0 found in covariance_map → diag 3.0
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[0], 3.0);
}

// Branch 4: within == true, subtype found in mapping → specific env_id returned
// Branch 3: env_id=1 found in covariance_map → environment_1 covariance
TEST_F(CovarianceSelectorTest, test_point_inside_polygon_known_subtype)
{
  create_node(base_options());

  helper_->publish_map(make_map_bin("uniform_road"));  // uniform_road → env_id=1
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(0.0, 0.0);  // inside polygon

  ASSERT_TRUE(spin_until_result());
  // env_id=1 → environment_1_output_pose_covariance (diag 2.0)
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[0], 2.0);
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[7], 2.0);
}

// Branch 5: within == true, subtype NOT found in mapping → continues, returns default_env_id
// Branch 2: env_id=0 not in covariance_map → default_covariance_
TEST_F(CovarianceSelectorTest, test_point_inside_polygon_unknown_subtype)
{
  create_node(base_options());

  helper_->publish_map(make_map_bin("unknown_subtype"));  // no mapping for this subtype
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(0.0, 0.0);  // inside polygon, but subtype has no env_id mapping

  ASSERT_TRUE(spin_until_result());
  // subtype unknown → default_env_id=0, not in covariance_map → default_covariance_ (diag 1.0)
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[0], 1.0);
}

// Branch 7: default_output_pose_covariance size != 36 → WARN, zero matrix used
TEST_F(CovarianceSelectorTest, test_invalid_default_covariance_size)
{
  auto opts = rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .append_parameter_override("default_environment_id", 0)
    .append_parameter_override(
      "default_output_pose_covariance", std::vector<double>{1.0, 2.0});  // wrong size

  create_node(opts);

  helper_->publish_pose(0.0, 0.0);

  ASSERT_TRUE(spin_until_result());
  // bad size → zero-initialized default_covariance_
  for (size_t i = 0; i < 36; ++i) {
    EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[i], 0.0) << "index " << i;
  }
}

// Branch 8: environment_<id>_output_pose_covariance size != 36 → WARN, entry skipped
TEST_F(CovarianceSelectorTest, test_invalid_env_covariance_size)
{
  auto opts = rclcpp::NodeOptions()
    .automatically_declare_parameters_from_overrides(true)
    .append_parameter_override("default_environment_id", 0)
    .append_parameter_override("area_subtype_uniform_road.environment_id", 1)
    .append_parameter_override("default_output_pose_covariance", diag_cov(1.0))
    .append_parameter_override(
      "environment_1_output_pose_covariance",
      std::vector<double>{9.9, 9.9});  // wrong size → skipped

  create_node(opts);

  helper_->publish_map(make_map_bin("uniform_road"));
  std::this_thread::sleep_for(100ms);

  helper_->publish_pose(0.0, 0.0);  // inside polygon → env_id=1

  ASSERT_TRUE(spin_until_result());
  // env_id=1 was skipped → covariance_map has no entry → default_covariance_ (diag 1.0)
  EXPECT_DOUBLE_EQ(helper_->last_covariance.value()[0], 1.0);
}

// ---------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
