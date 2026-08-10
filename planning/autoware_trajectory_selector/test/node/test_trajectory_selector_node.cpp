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
#include <rclcpp/parameter_value.hpp>

#include <rosgraph_msgs/msg/clock.hpp>

#include <gtest/gtest.h>

#include <chrono>
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
    node_options_.append_parameter_override("duration_time", 1.0);
    node_options_.append_parameter_override("fallback_period_ms", 150);
    node_options_.append_parameter_override("use_sim_time", true);

    const auto vehicle_info_param_path = autoware::test_utils::get_absolute_path_to_config(
      "autoware_test_utils", "test_vehicle_info.param.yaml");

    autoware::test_utils::updateNodeOptions(node_options_, {vehicle_info_param_path});

    node_under_test_ =
      std::make_shared<autoware::trajectory_selector::TrajectorySelectorNode>(node_options_);

    rclcpp::NodeOptions test_node_options;
    test_node_options.append_parameter_override("use_sim_time", true);
    test_node_ = std::make_shared<rclcpp::Node>("test_helper_node", test_node_options);

    clock_pub_ =
      test_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());

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
    backup_traj_pub_ =
      test_node_->create_publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_selector_node/input/trajectories_backup", 1);

    output_sub_ =
      test_node_->create_subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>(
        "/trajectory_selector_node/output/trajectories", 1,
        [this](
          const autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr msg) {
          last_output_ = msg;
        });

    // Seed the simulated clock at t = 1 s to avoid zero-time edge cases.
    // Publishing immediately triggers the initial (warm-up) timer fire because the node's
    // timer was created when the clock was still at 0, so its first deadline (0 + 150 ms)
    // is already past. After the initial spins the next deadline is t = 1.0 s + 150 ms.
    sim_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME) + rclcpp::Duration(std::chrono::seconds(1));
    publish_clock();
    for (int i = 0; i < 10; ++i) {
      rclcpp::spin_some(node_under_test_->get_node_base_interface());
      rclcpp::spin_some(test_node_);
    }
  }

  void TearDown() override { rclcpp::shutdown(); }

  /** Publish the current sim_time_ on /clock. */
  void publish_clock()
  {
    rosgraph_msgs::msg::Clock msg;
    msg.clock = sim_time_;
    clock_pub_->publish(msg);
  }

  /**
   * Spin both nodes a fixed number of times without advancing simulated time.
   * Use this to let published messages propagate before starting a timed window.
   */
  void spin_for_messages(int iterations = 10)
  {
    for (int i = 0; i < iterations; ++i) {
      rclcpp::spin_some(node_under_test_->get_node_base_interface());
      rclcpp::spin_some(test_node_);
    }
  }

  /**
   * Advance the simulated clock in `step` increments, spinning after each
   * advance, until `condition` returns true or `timeout` of simulated time
   * has elapsed.
   *
   * No wall-clock sleeping is performed; all timing is driven by /clock
   * messages published to the nodes.
   *
   * Two spin_some() calls are made for node_under_test_ after each clock
   * advance: the first delivers the /clock update (which may mark a timer
   * as ready), the second executes any newly-ready timer callback.
   */
  bool spin_until(
    const std::function<bool()> & condition,
    const rclcpp::Duration & timeout = rclcpp::Duration(std::chrono::seconds(1)),
    const rclcpp::Duration & step = rclcpp::Duration(std::chrono::milliseconds(10)))
  {
    const rclcpp::Time deadline = sim_time_ + timeout;
    while (rclcpp::ok()) {
      rclcpp::spin_some(node_under_test_->get_node_base_interface());
      rclcpp::spin_some(test_node_);
      if (condition()) return true;
      if (sim_time_ >= deadline) break;
      sim_time_ = sim_time_ + step;
      publish_clock();
      // First spin: delivers the clock update and may mark the timer as ready.
      rclcpp::spin_some(node_under_test_->get_node_base_interface());
      // Second spin: executes any timer callback that became ready above.
      rclcpp::spin_some(node_under_test_->get_node_base_interface());
      rclcpp::spin_some(test_node_);
    }
    return false;
  }

  void publish_context()
  {
    auto map_msg = autoware::test_utils::makeMapBinMsg("autoware_test_utils", "lanelet2_map.osm");
    map_pub_->publish(map_msg);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = sim_time_;
    odom.header.frame_id = "map";
    odom_pub_->publish(odom);

    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    accel.header.stamp = sim_time_;
    accel.header.frame_id = "map";
    accel_pub_->publish(accel);

    autoware_perception_msgs::msg::PredictedObjects objects;
    objects.header.stamp = sim_time_;
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

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Time sim_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr obj_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrafficLightGroupArray>::SharedPtr tl_pub_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    traj_pub_;
  rclcpp::Publisher<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    backup_traj_pub_;

  rclcpp::Subscription<autoware_internal_planning_msgs::msg::CandidateTrajectories>::SharedPtr
    output_sub_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories::ConstSharedPtr last_output_;
};

TEST_F(TrajectorySelectorNodeTest, FiltersTrajectoriesViaPlugin)
{
  publish_context();
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;

  add_trajectory(msg, "SafePlanner", 10.0, now);
  add_trajectory(msg, "RejectedPlanner", -999.0, now);

  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until(
    [this] { return last_output_ != nullptr && !last_output_->candidate_trajectories.empty(); },
    rclcpp::Duration(std::chrono::seconds(1))));

  EXPECT_EQ(last_output_->candidate_trajectories.size(), 1u);
  ASSERT_EQ(last_output_->generator_info.size(), 1u);
  EXPECT_EQ(last_output_->generator_info.front().generator_name.data, "SafePlanner");
}

TEST_F(TrajectorySelectorNodeTest, ImmediateOutputOnGenerativeInput)
{
  publish_context();
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "GenerativePlanner", 10.0, now);

  last_output_ = nullptr;
  traj_pub_->publish(msg);

  // Should publish immediately (timer is 150 ms sim, so 50 ms sim is "immediate" enough)
  const bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(50)));
  EXPECT_TRUE(received) << "Node should publish immediately on generative input";
}

TEST_F(TrajectorySelectorNodeTest, NoImmediateOutputOnBackupInput)
{
  publish_context();
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "BackupPlanner", 10.0, now);
  bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(50)));
  EXPECT_FALSE(received) << "Node should NOT publish without any input";

  last_output_ = nullptr;
  backup_traj_pub_->publish(msg);

  // Should NOT publish immediately
  received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(50)));
  EXPECT_FALSE(received) << "Node should NOT publish immediately on backup input";

  // Should publish eventually via timer (timer is 150 ms sim)
  received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(200)));
  EXPECT_TRUE(received) << "Node should eventually publish on backup input via timer";
}

TEST_F(TrajectorySelectorNodeTest, TimerOutputWithoutGenerativeInput)
{
  publish_context();
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "SomePlanner", 10.0, now);

  // Publish only to backup, generative is never published
  last_output_ = nullptr;
  backup_traj_pub_->publish(msg);

  const bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(200)));
  EXPECT_TRUE(received) << "Node should publish via timer even if generative input is missing";
}

TEST_F(TrajectorySelectorNodeTest, HandlesPluginRejection)
{
  publish_context();
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "FailingPlanner", -999.0, now);

  traj_pub_->publish(msg);

  ASSERT_TRUE(spin_until([this] { return last_output_ != nullptr; }));
  EXPECT_EQ(last_output_->candidate_trajectories.size(), 0u);
}

TEST_F(TrajectorySelectorNodeTest, NoPublishWhenOdometryMissing)
{
  const auto now = sim_time_;

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

  const bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(500)));
  EXPECT_FALSE(received) << "Node must not publish when odometry is unavailable";
}

TEST_F(TrajectorySelectorNodeTest, NoPublishWhenAccelerationMissing)
{
  const auto now = sim_time_;

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

  const bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(500)));
  EXPECT_FALSE(received) << "Node must not publish when acceleration is unavailable";
}

TEST_F(TrajectorySelectorNodeTest, NoPublishWhenObjectsMissing)
{
  const auto now = sim_time_;

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

  const bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(500)));
  EXPECT_FALSE(received) << "Node must not publish when predicted objects are unavailable";
}

TEST_F(TrajectorySelectorNodeTest, CustomFallbackPeriod)
{
  publish_context();
  node_under_test_->set_parameters(
    {rclcpp::Parameter("fallback_period_ms", 500), rclcpp::Parameter("duration_time", 1.0)});
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "SomePlanner", 10.0, now);

  // Publish only to backup, generative is never published
  last_output_ = nullptr;
  backup_traj_pub_->publish(msg);

  // Should NOT publish within 400ms because fallback_period_ms is 500ms
  bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(400)));
  EXPECT_FALSE(received) << "Node should NOT publish within 400ms when fallback_period_ms is 500ms";

  // Should publish eventually via timer (timer is 500ms, so 600ms total should be enough)
  received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(200)));
  EXPECT_TRUE(received) << "Node should eventually publish on backup input via custom timer";
}

TEST_F(TrajectorySelectorNodeTest, ResetFallbackPeriod)
{
  publish_context();
  node_under_test_->set_parameters(
    {rclcpp::Parameter("fallback_period_ms", 500), rclcpp::Parameter("duration_time", 1.0)});
  spin_for_messages();

  const auto now = sim_time_;
  autoware_internal_planning_msgs::msg::CandidateTrajectories msg;
  add_trajectory(msg, "SomePlanner", 10.0, now);

  // Publish only to backup, generative is never published
  last_output_ = nullptr;
  backup_traj_pub_->publish(msg);

  bool received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(400)));
  EXPECT_FALSE(received) << "Node should NOT publish within 400ms when fallback_period_ms is 500ms";

  // Set new fallback period which resets the fallback timer.
  // update_fallback_timer() is called synchronously inside set_parameters, so the new
  // timer deadline is anchored to the current sim_time_ at the moment of the call.
  node_under_test_->set_parameters({
    rclcpp::Parameter("fallback_period_ms", 400),
  });

  received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(200)));
  EXPECT_FALSE(received) << "Node should NOT publish because timer was reset";

  node_under_test_->set_parameters({rclcpp::Parameter("fallback_period_ms", 200)});

  received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(100)));
  EXPECT_FALSE(received) << "Node should NOT publish because timer was reset";
  received = spin_until(
    [this] { return last_output_ != nullptr; }, rclcpp::Duration(std::chrono::milliseconds(200)));
  EXPECT_TRUE(received) << "Node should publish within 300ms when fallback_period_ms is 200ms";
}

}  // namespace autoware::trajectory_selector
