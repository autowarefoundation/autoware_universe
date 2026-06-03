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

#include "autoware/trajectory_selector/trajectory_selector_node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_selector
{

class TrajectorySelectorNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override(
      "filter_names",
      std::vector<std::string>{"autoware::trajectory_validator::plugin::DummyFilter"});
    node_options_.append_parameter_override("dummy.dummy_param", 0.0);

    const auto vehicle_info_param_path = autoware::test_utils::get_absolute_path_to_config(
      "autoware_test_utils", "test_vehicle_info.param.yaml");

    autoware::test_utils::updateNodeOptions(node_options_, {vehicle_info_param_path});

    node_under_test_ =
      std::make_shared<autoware::trajectory_selector::TrajectorySelectorNode>(node_options_);
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node");

    map_pub_ = test_node_->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/trajectory_selector_node/input/lanelet2_map", rclcpp::QoS{1}.transient_local());
    odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/trajectory_selector_node/input/odometry", 1);
    accel_pub_ = test_node_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/trajectory_selector_node/input/acceleration", 1);
    obj_pub_ = test_node_->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      "/trajectory_selector_node/input/objects", 1);
    tl_pub_ = test_node_->create_publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>(
      "/trajectory_selector_node/input/traffic_signals", 1);

    traj_pub_ =
      test_node_->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_selector_node/input/trajectories_generative", 1);

    output_sub_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_selector_node/output/trajectories", 1,
        [this](
          const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg) {
          last_output_ = msg;
        });
  }

  void TearDown() override { rclcpp::shutdown(); }

  bool spin_until(
    std::function<bool()> && condition,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(1000))
  {
    const auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok() && (std::chrono::steady_clock::now() - start) < timeout) {
      if (condition()) return true;
      rclcpp::spin_some(node_under_test_);
      rclcpp::spin_some(test_node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  void publish_context()
  {
    const auto now = node_under_test_->now();

    auto map_msg = autoware::test_utils::makeMapBinMsg("autoware_test_utils", "lanelet2_map.osm");
    map_pub_->publish(map_msg);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "map";
    odom_pub_->publish(odom);

    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = now;
    accel.header.frame_id = "map";
    accel_pub_->publish(accel);

    autoware_perception_msgs::msg::PredictedObjects objects;
    objects.header.stamp = now;
    objects.header.frame_id = "map";
    obj_pub_->publish(objects);

    autoware_perception_msgs::msg::TrafficLightGroupArray tl_signals;
    tl_pub_->publish(tl_signals);
  }

  static void add_trajectory(
    CandidateTrajectories & msg, std::string name, float start_vel, const rclcpp::Time & stamp)
  {
    CandidateTrajectory traj;
    traj.header.stamp = stamp;

    autoware_internal_planning_msgs::msg::GeneratorInfo info;
    info.generator_name.data = std::move(name);
    info.generator_id = autoware_utils_uuid::generate_uuid();

    traj.generator_id = info.generator_id;

    autoware_planning_msgs::msg::TrajectoryPoint p1;
    p1.longitudinal_velocity_mps = start_vel;
    p1.time_from_start = rclcpp::Duration::from_seconds(0.0);
    traj.points.push_back(p1);

    msg.candidate_trajectories.push_back(traj);
    msg.generator_info.push_back(info);
  }

  rclcpp::NodeOptions node_options_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<trajectory_selector::TrajectorySelectorNode> node_under_test_;

  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr obj_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr tl_pub_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    traj_pub_;

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    output_sub_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr last_output_;
};

TEST_F(TrajectorySelectorNodeTest, FiltersTrajectoriesViaPlugin)
{
  publish_context();
  spin_until([] { return false; }, std::chrono::milliseconds(100));

  const auto now = node_under_test_->now();
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;

  add_trajectory(msg, "SafePlanner", 10.0, now);
  add_trajectory(msg, "RejectedPlanner", -999.0, now);

  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until(
    [this] { return last_output_ != nullptr && !last_output_->candidate_trajectories.empty(); },
    std::chrono::milliseconds(1000)));

  EXPECT_EQ(last_output_->candidate_trajectories.size(), 1u);
  ASSERT_EQ(last_output_->generator_info.size(), 1u);
  EXPECT_EQ(last_output_->generator_info.front().generator_name.data, "SafePlanner");
}

TEST_F(TrajectorySelectorNodeTest, HandlesPluginRejection)
{
  publish_context();
  spin_until([] { return false; }, std::chrono::milliseconds(100));

  const auto now = node_under_test_->now();
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "FailingPlanner", -999.0, now);

  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until([this] { return last_output_ != nullptr; }));
  EXPECT_EQ(last_output_->candidate_trajectories.size(), 0u);
}

TEST_F(TrajectorySelectorNodeTest, NoPublishWhenOdometryMissing)
{
  const auto now = node_under_test_->now();

  auto map_msg = autoware::test_utils::makeMapBinMsg("autoware_test_utils", "lanelet2_map.osm");
  map_pub_->publish(map_msg);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header.stamp = now;
  accel_pub_->publish(accel);

  autoware_perception_msgs::msg::PredictedObjects objects;
  objects.header.stamp = now;
  obj_pub_->publish(objects);

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "AnyPlanner", 10.0, now);
  traj_pub_->publish(msg);

  const bool received =
    spin_until([this] { return last_output_ != nullptr; }, std::chrono::milliseconds(500));
  EXPECT_FALSE(received) << "Node must not publish when odometry is unavailable";
}

TEST_F(TrajectorySelectorNodeTest, NoPublishWhenAccelerationMissing)
{
  const auto now = node_under_test_->now();

  auto map_msg = autoware::test_utils::makeMapBinMsg("autoware_test_utils", "lanelet2_map.osm");
  map_pub_->publish(map_msg);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom_pub_->publish(odom);

  autoware_perception_msgs::msg::PredictedObjects objects;
  objects.header.stamp = now;
  obj_pub_->publish(objects);

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "AnyPlanner", 10.0, now);
  traj_pub_->publish(msg);

  const bool received =
    spin_until([this] { return last_output_ != nullptr; }, std::chrono::milliseconds(500));
  EXPECT_FALSE(received) << "Node must not publish when acceleration is unavailable";
}

TEST_F(TrajectorySelectorNodeTest, NoPublishWhenObjectsMissing)
{
  const auto now = node_under_test_->now();

  auto map_msg = autoware::test_utils::makeMapBinMsg("autoware_test_utils", "lanelet2_map.osm");
  map_pub_->publish(map_msg);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now;
  odom_pub_->publish(odom);

  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  accel.header.stamp = now;
  accel_pub_->publish(accel);

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "AnyPlanner", 10.0, now);
  traj_pub_->publish(msg);

  const bool received =
    spin_until([this] { return last_output_ != nullptr; }, std::chrono::milliseconds(500));
  EXPECT_FALSE(received) << "Node must not publish when predicted objects are unavailable";
}

}  // namespace autoware::trajectory_selector
