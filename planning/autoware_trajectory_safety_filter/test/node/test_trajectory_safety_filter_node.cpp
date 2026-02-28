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

#include "autoware/trajectory_safety_filter/trajectory_safety_filter_node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_safety_filter
{

class TrajectorySafetyFilterNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_options_.append_parameter_override(
      "filter_names",
      std::vector<std::string>{"autoware::trajectory_safety_filter::plugin::DummyFilter"});
    node_options_.append_parameter_override("dummy.dummy_param", 0.0);

    const auto test_pkg_share = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    const auto vehicle_info_param_path = autoware::test_utils::get_absolute_path_to_config(
      "autoware_test_utils", "test_vehicle_info.param.yaml");

    autoware::test_utils::updateNodeOptions(node_options_, {vehicle_info_param_path});

    node_under_test_ = std::make_shared<TrajectorySafetyFilter>(node_options_);
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node");

    map_pub_ = test_node_->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
      "/trajectory_safety_filter_node/input/lanelet2_map", rclcpp::QoS{1}.transient_local());
    odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(
      "/trajectory_safety_filter_node/input/odometry", 1);
    accel_pub_ = test_node_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
      "/trajectory_safety_filter_node/input/acceleration", 1);
    obj_pub_ = test_node_->create_publisher<autoware_perception_msgs::msg::PredictedObjects>(
      "/trajectory_safety_filter_node/input/objects", 1);

    traj_pub_ =
      test_node_->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_safety_filter_node/input/trajectories", 1);

    output_sub_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_safety_filter_node/output/trajectories", 1,
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
    auto map_msg = autoware::test_utils::makeMapBinMsg("autoware_test_utils", "lanelet2_map.osm");
    map_pub_->publish(map_msg);

    const auto now = node_under_test_->now();

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
  }

  static void add_trajectory(CandidateTrajectories & msg, std::string name, double start_vel)
  {
    CandidateTrajectory traj;
    autoware_internal_planning_msgs::msg::GeneratorInfo info;
    info.generator_name.data = name;
    info.generator_id = autoware_utils_uuid::generate_uuid();

    traj.generator_id = info.generator_id;

    TrajectoryPoint p1;
    p1.longitudinal_velocity_mps = start_vel;
    traj.points.push_back(p1);

    msg.candidate_trajectories.push_back(traj);
    msg.generator_info.push_back(info);
  }

  rclcpp::NodeOptions node_options_;
  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<TrajectorySafetyFilter> node_under_test_;

  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr obj_pub_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    traj_pub_;

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    output_sub_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr last_output_;
};

TEST_F(TrajectorySafetyFilterNodeTest, FiltersTrajectoriesViaPlugin)
{
  publish_context();
  spin_until([] { return false; }, std::chrono::milliseconds(100));

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;

  add_trajectory(msg, "SafePlanner", 10.0);
  add_trajectory(msg, "RejectedPlanner", -999.0);

  traj_pub_->publish(msg);

  ASSERT_TRUE(
    spin_until([this] { return last_output_ != nullptr; }, std::chrono::milliseconds(1000)));

  EXPECT_EQ(last_output_->candidate_trajectories.size(), 1u);
  EXPECT_EQ(last_output_->generator_info.size(), 1u);
  EXPECT_EQ(last_output_->generator_info.front().generator_name.data, "SafePlanner");
}

TEST_F(TrajectorySafetyFilterNodeTest, UpdateParametersDynamically)
{
  publish_context();
  spin_until([] { return false; }, std::chrono::milliseconds(100));

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "InitialTest", 5.0);

  traj_pub_->publish(msg);
  ASSERT_TRUE(spin_until([this] { return last_output_ != nullptr; }));
  EXPECT_EQ(last_output_->candidate_trajectories.size(), 1u);

  last_output_ = nullptr;

  auto result = node_under_test_->set_parameters({rclcpp::Parameter("dummy.dummy_param", 1.0)});
  ASSERT_TRUE(result[0].successful);

  traj_pub_->publish(msg);
  ASSERT_TRUE(spin_until([this] { return last_output_ != nullptr; }));

  EXPECT_GT(last_output_->candidate_trajectories.size(), 0u);
}

TEST_F(TrajectorySafetyFilterNodeTest, HandlesPluginRejection)
{
  publish_context();
  spin_until([] { return false; }, std::chrono::milliseconds(100));

  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "FailingPlanner", -999.0);

  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until([this] { return last_output_ != nullptr; }));
  EXPECT_EQ(last_output_->candidate_trajectories.size(), 0u);
}
}  // namespace autoware::trajectory_safety_filter
