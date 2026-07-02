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

#include "autoware/trajectory_modifier/trajectory_modifier.hpp"
#include "autoware/trajectory_optimizer/trajectory_optimizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectories.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::PointCloud2;

namespace
{
TrajectoryPoint create_trajectory_point(double x, double y, double velocity)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.position.z = 0.0;
  point.pose.orientation.w = 1.0;
  point.longitudinal_velocity_mps = static_cast<float>(velocity);
  return point;
}

CandidateTrajectories create_straight_trajectories(
  double length, double velocity, const rclcpp::Time & stamp)
{
  CandidateTrajectories msg;
  CandidateTrajectory candidate;
  candidate.header.frame_id = "map";
  candidate.header.stamp = stamp;
  for (double x = 0.0; x <= length; x += 1.0) {
    candidate.points.push_back(create_trajectory_point(x, 0.0, velocity));
  }
  msg.candidate_trajectories.push_back(candidate);
  return msg;
}

CandidateTrajectories create_sharp_turn_trajectories(
  double segment_length, double velocity, const rclcpp::Time & stamp)
{
  CandidateTrajectories msg;
  CandidateTrajectory candidate;
  candidate.header.frame_id = "map";
  candidate.header.stamp = stamp;

  // Straight segment along X axis
  for (double x = 0.0; x <= segment_length; x += 1.0) {
    auto p = create_trajectory_point(x, 0.0, velocity);
    // Heading is 0 (straight)
    p.pose.orientation.w = 1.0;
    p.pose.orientation.z = 0.0;
    candidate.points.push_back(p);
  }

  // Sharp 90-degree turn along Y axis
  for (double y = 1.0; y <= segment_length; y += 1.0) {
    auto p = create_trajectory_point(segment_length, y, velocity);
    // Heading is 90 degrees (yaw = pi/2)
    p.pose.orientation.w = 0.707;
    p.pose.orientation.z = 0.707;
    candidate.points.push_back(p);
  }

  msg.candidate_trajectories.push_back(candidate);
  return msg;
}

Odometry create_odometry(const double velocity)
{
  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = velocity;
  return odom;
}
}  // namespace

class TrajectoryProcessorIntegrationTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    // Setup node options with parameters
    auto modifier_options = rclcpp::NodeOptions{};
    auto optimizer_options = rclcpp::NodeOptions{};
    auto test_options = rclcpp::NodeOptions{};

    // Force all nodes to use simulated time
    modifier_options.append_parameter_override("use_sim_time", true);
    optimizer_options.append_parameter_override("use_sim_time", true);
    test_options.append_parameter_override("use_sim_time", true);

    const auto modifier_dir =
      ament_index_cpp::get_package_share_directory("autoware_trajectory_modifier");
    const auto optimizer_dir =
      ament_index_cpp::get_package_share_directory("autoware_trajectory_optimizer");
    const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.smooth_velocities", true);

    // Remap modifier output to optimizer input and load param files
    modifier_options.arguments(
      {"--ros-args", "--params-file", modifier_dir + "/config/trajectory_modifier.param.yaml",
       "--params-file", test_utils_dir + "/config/test_vehicle_info.param.yaml", "-r",
       "~/output/candidate_trajectories:=/combined/trajectories", "-r",
       "~/input/odometry:=/localization/kinematic_state", "-r",
       "~/input/acceleration:=/localization/acceleration", "-r",
       "~/input/objects:=/perception/object_recognition/objects", "-r",
       "~/input/pointcloud:=/perception/obstacle_segmentation/pointcloud"});
    optimizer_options.arguments(
      {"--ros-args",
       "--params-file",
       optimizer_dir + "/config/trajectory_optimizer.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_qp_smoother.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_point_fixer.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_velocity_optimizer.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_extender.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_spline_smoother.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_kinematic_feasibility_enforcer.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_mpt_optimizer.param.yaml",
       "--params-file",
       optimizer_dir + "/config/plugins/trajectory_temporal_mpt_optimizer.param.yaml",
       "--params-file",
       test_utils_dir + "/config/test_vehicle_info.param.yaml",
       "-r",
       "~/input/trajectories:=/combined/trajectories",
       "-r",
       "~/input/odometry:=/localization/kinematic_state",
       "-r",
       "~/input/acceleration:=/localization/acceleration"});

    modifier_node_ =
      std::make_shared<autoware::trajectory_modifier::TrajectoryModifier>(modifier_options);
    optimizer_node_ =
      std::make_shared<autoware::trajectory_optimizer::TrajectoryOptimizer>(optimizer_options);

    test_node_ = std::make_shared<rclcpp::Node>("test_node", test_options);

    // Simulated Clock Setup
    clock_pub_ = test_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    sim_time_ = rclcpp::Time(1, 0, RCL_ROS_TIME);

    // Publishers for inputs
    pub_tra_ = test_node_->create_publisher<CandidateTrajectories>(
      "/trajectory_modifier/input/candidate_trajectories", 1);
    pub_odo_ = test_node_->create_publisher<Odometry>("/localization/kinematic_state", 1);
    pub_acc_ =
      test_node_->create_publisher<AccelWithCovarianceStamped>("/localization/acceleration", 1);
    pub_obj_ =
      test_node_->create_publisher<PredictedObjects>("/perception/object_recognition/objects", 1);

    // Subscriber for final output
    sub_output_ = test_node_->create_subscription<Trajectory>(
      "/trajectory_optimizer/output/trajectory", 1, [this](const Trajectory::ConstSharedPtr msg) {
        output_received_ = true;
        latest_output_ = *msg;
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(modifier_node_);
    executor_->add_node(optimizer_node_);
    executor_->add_node(test_node_);
  }

  /**
   * Advances the simulated clock and spins the executor to process callbacks deterministically.
   */
  void advance_sim_time_and_spin(std::chrono::milliseconds step)
  {
    sim_time_ = sim_time_ + rclcpp::Duration(step);
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = sim_time_;
    clock_pub_->publish(clock_msg);

    // Spin twice: once to deliver the clock msg, once to execute timers triggered by the clock
    executor_->spin_some();
    executor_->spin_some();
  }

  void publish_mandatory_inputs(const CandidateTrajectories & traj, Odometry odom)
  {
    odom.header.stamp = sim_time_;
    AccelWithCovarianceStamped acc;
    acc.header.stamp = sim_time_;
    acc.header.frame_id = "map";
    pub_tra_->publish(traj);
    pub_odo_->publish(odom);
    pub_acc_->publish(acc);
  }

  /**
   * Spins the executor deterministically in simulated time until an output is received.
   */
  void wait_for_output_sim(int timeout_s = 5)
  {
    const rclcpp::Time deadline = sim_time_ + rclcpp::Duration(std::chrono::seconds(timeout_s));
    const auto wait_condition = [&]() {
      return !output_received_ && sim_time_ < deadline && rclcpp::ok();
    };
    while (wait_condition()) {
      advance_sim_time_and_spin(std::chrono::milliseconds(100));
    }
  }

  std::shared_ptr<autoware::trajectory_modifier::TrajectoryModifier> modifier_node_;
  std::shared_ptr<autoware::trajectory_optimizer::TrajectoryOptimizer> optimizer_node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Time sim_time_;

  rclcpp::Publisher<CandidateTrajectories>::SharedPtr pub_tra_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odo_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_obj_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_output_;

  bool output_received_{false};
  Trajectory latest_output_;
};

TEST_F(TrajectoryProcessorIntegrationTest, BasicPipelineTest)
{
  auto traj = create_straight_trajectories(30.0, 5.0, sim_time_);

  const auto odom = create_odometry(5.0);

  // Publish in a loop to ensure the nodes receive it after discovery
  for (int i = 0; i < 20 && !output_received_; ++i) {
    publish_mandatory_inputs(traj, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);
  EXPECT_GT(latest_output_.points.size(), 0U);
}

TEST_F(TrajectoryProcessorIntegrationTest, ObstacleStopIntegrationTest)
{
  const auto traj = create_straight_trajectories(30.0, 8.0, sim_time_);
  const auto odom = create_odometry(8.0);

  // Create a blocking car at x=20.0
  PredictedObjects objects;
  objects.header.frame_id = "map";
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 20.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  objects.objects.push_back(object);

  // Obstacle stop needs continuous detection (on_time_buffer=0.5s).
  for (int i = 0; i < 20; ++i) {
    objects.header.stamp = sim_time_;
    pub_obj_->publish(objects);
    publish_mandatory_inputs(traj, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);
  // The trajectory should have a stop point (velocity close to 0) before the object at x=20.0
  bool found_stop = false;
  for (const auto & p : latest_output_.points) {
    if (p.longitudinal_velocity_mps < 0.1) {
      EXPECT_LT(p.pose.position.x, 20.0);
      found_stop = true;
      break;
    }
  }
  EXPECT_TRUE(found_stop);
}

TEST_F(TrajectoryProcessorIntegrationTest, StopPointFixerIntegrationTest)
{
  // Short trajectory with a stop point at 0.5m
  CandidateTrajectories msg;
  CandidateTrajectory candidate;
  candidate.header.frame_id = "map";
  candidate.header.stamp = sim_time_;
  candidate.points.push_back(create_trajectory_point(0.0, 0.0, 0.1));
  candidate.points.push_back(create_trajectory_point(0.5, 0.0, 0.0));
  msg.candidate_trajectories.push_back(candidate);

  const auto odom = create_odometry(0.05);

  for (int i = 0; i < 20 && !output_received_; ++i) {
    publish_mandatory_inputs(msg, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);
  // Stop point fixer should have replaced the trajectory with stop points at ego
  EXPECT_NEAR(latest_output_.points.front().pose.position.x, 0.0, 0.1);
  EXPECT_NEAR(latest_output_.points.front().longitudinal_velocity_mps, 0.0, 0.01);
}

TEST_F(TrajectoryProcessorIntegrationTest, KinematicFeasibilityTest)
{
  // Create a sharp L-shape turn with 10m segments
  const auto traj = create_sharp_turn_trajectories(10.0, 5.0, sim_time_);
  const auto odom = create_odometry(5.0);

  for (int i = 0; i < 20 && !output_received_; ++i) {
    publish_mandatory_inputs(traj, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);

  double max_yaw_change = 0.0;

  // Verify that the impossible instantaneous 90-degree jump has been smoothed
  for (size_t i = 1; i < latest_output_.points.size(); ++i) {
    const auto & p1 = latest_output_.points[i - 1];
    const auto & p2 = latest_output_.points[i];

    // Extract yaw from quaternions
    double yaw1 = 2.0 * std::atan2(p1.pose.orientation.z, p1.pose.orientation.w);
    double yaw2 = 2.0 * std::atan2(p2.pose.orientation.z, p2.pose.orientation.w);

    // Normalize angle difference to [-pi, pi]
    double yaw_diff = yaw2 - yaw1;
    while (yaw_diff > M_PI) yaw_diff -= 2.0 * M_PI;
    while (yaw_diff < -M_PI) yaw_diff += 2.0 * M_PI;

    max_yaw_change = std::max(max_yaw_change, std::abs(yaw_diff));
  }

  // The input trajectory had a sudden jump of ~1.57 radians (90 degrees).
  // The optimizer should smooth this. We expect no single step to have a massive yaw jump.
  // 0.3 radians per step is a safe upper bound for a smoothed kinematic trajectory at 0.1s dt.
  EXPECT_LT(max_yaw_change, 0.3) << "Trajectory still contains an impossible kinematic yaw jump!";
}

TEST_F(TrajectoryProcessorIntegrationTest, VelocityOptimizationTest)
{
  // Input: 15.0 m/s is too fast for a sharp turn (violates max_lateral_accel_mps2 = 1.5)
  auto traj = create_sharp_turn_trajectories(20.0, 15.0, sim_time_);

  const auto odom = create_odometry(15.0);
  for (int i = 0; i < 20 && !output_received_; ++i) {
    publish_mandatory_inputs(traj, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);

  // Verify that the velocity drops significantly near the curve (around index where x approaches
  // 20)
  bool velocity_reduced = false;
  for (const auto & p : latest_output_.points) {
    // If we are in the middle of the turn
    if (p.pose.position.x > 15.0 && p.pose.position.y < 5.0) {
      if (p.longitudinal_velocity_mps < 10.0) {  // Velocity must drop well below the 15.0 m/s input
        velocity_reduced = true;
        break;
      }
    }
  }
  EXPECT_TRUE(velocity_reduced) << "Velocity was not adequately reduced for the curve!";
}

TEST_F(TrajectoryProcessorIntegrationTest, SmoothObstacleStopInteractionTest)
{
  // Straight trajectory, moving at 10 m/s
  const auto traj = create_straight_trajectories(40.0, 10.0, sim_time_);
  const auto odom = create_odometry(10.0);

  // Obstacle right in front of the vehicle at x = 30.0
  PredictedObjects objects;
  objects.header.frame_id = "map";
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 30.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 2.0;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  objects.objects.push_back(object);

  for (int i = 0; i < 20 && !output_received_; ++i) {
    objects.header.stamp = sim_time_;
    pub_obj_->publish(objects);
    publish_mandatory_inputs(traj, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);

  // Check deceleration profile (jerk limitation) leading up to the stop point
  float previous_velocity = latest_output_.points.front().longitudinal_velocity_mps;
  float max_deceleration_step = 0.0;

  for (size_t i = 1; i < latest_output_.points.size(); ++i) {
    float current_velocity = latest_output_.points[i].longitudinal_velocity_mps;
    float velocity_drop = previous_velocity - current_velocity;

    if (velocity_drop > max_deceleration_step) {
      max_deceleration_step = velocity_drop;
    }
    previous_velocity = current_velocity;
  }

  // The modifier blindly inserts a 0 m/s point.
  // The optimizer must smooth this. If it didn't smooth it, the step would be 10.0 m/s.
  // We expect a smooth deceleration curve where no single step drop between points is drastic.
  EXPECT_LT(max_deceleration_step, 2.0)
    << "Deceleration profile is too abrupt, optimizer failed to smooth the modifier's stop!";
}

TEST_F(TrajectoryProcessorIntegrationTest, ObstacleStopAndSmoothDecelerationTest)
{
  // 1. Create a straight trajectory at 10.0 m/s
  const auto traj = create_straight_trajectories(50.0, 10.0, sim_time_);
  const auto odom = create_odometry(10.0);

  // 2. Place an obstacle at x=30.0.
  // The Modifier will insert a sudden stop point.
  PredictedObjects objects;
  objects.header.frame_id = "map";
  autoware_perception_msgs::msg::PredictedObject object;
  object.kinematics.initial_pose_with_covariance.pose.position.x = 30.0;
  object.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  object.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 2.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 2.0;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.label = autoware_perception_msgs::msg::ObjectClassification::CAR;
  classification.probability = 1.0;
  object.classification.push_back(classification);
  objects.objects.push_back(object);

  for (int i = 0; i < 20; ++i) {
    objects.header.stamp = sim_time_;
    pub_obj_->publish(objects);
    publish_mandatory_inputs(traj, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);
  ASSERT_GT(latest_output_.points.size(), 1U);

  // 3. Verify interaction:
  // a) The obstacle stop was preserved (velocity reaches ~0 before x=30.0)
  // b) The deceleration was smoothed (no massive single-step drops in velocity)

  bool vehicle_stops = false;
  float max_velocity_drop = 0.0;
  float prev_vel = latest_output_.points.front().longitudinal_velocity_mps;

  for (size_t i = 1; i < latest_output_.points.size(); ++i) {
    const auto & p = latest_output_.points[i];
    float current_vel = p.longitudinal_velocity_mps;

    // Calculate the drop between consecutive points
    float drop = prev_vel - current_vel;
    if (drop > max_velocity_drop) {
      max_velocity_drop = drop;
    }

    // Check if the vehicle stopped before the obstacle
    if (current_vel < 0.1 && p.pose.position.x < 30.0) {
      vehicle_stops = true;
    }

    prev_vel = current_vel;
  }

  EXPECT_TRUE(vehicle_stops) << "The Modifier's Obstacle Stop was lost or ignored!";
  EXPECT_LT(max_velocity_drop, 1.5)
    << "The Optimizer failed to smooth the Modifier's sudden stop. Deceleration is too abrupt!";
}

TEST_F(TrajectoryProcessorIntegrationTest, StopPointFixerAndOptimizerResamplingTest)
{
  // 1. Create a very short trajectory with near-zero velocities.
  // This triggers the Modifier's Stop Point Fixer.
  CandidateTrajectories msg;
  CandidateTrajectory candidate;
  candidate.header.frame_id = "map";
  candidate.header.stamp = sim_time_;
  candidate.points.push_back(create_trajectory_point(0.0, 0.0, 0.05));
  candidate.points.push_back(create_trajectory_point(0.2, 0.0, 0.0));
  candidate.points.push_back(create_trajectory_point(0.4, 0.0, 0.0));
  msg.candidate_trajectories.push_back(candidate);

  const auto odom = create_odometry(0.0);

  for (int i = 0; i < 20 && !output_received_; ++i) {
    publish_mandatory_inputs(msg, odom);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  // 2. Verify interaction:
  // The Optimizer must successfully process the Modifier's collapsed, zero-velocity trajectory
  // without crashing, generating NaNs, or completely stripping all points via the point fixer.

  ASSERT_TRUE(output_received_) << "Optimizer failed to output a trajectory!";
  ASSERT_GT(latest_output_.points.size(), 0U)
    << "Optimizer stripped all points from the stopped trajectory!";

  for (const auto & p : latest_output_.points) {
    // Ensure no NaNs were generated by smoothers attempting to process zero-distance points
    EXPECT_FALSE(std::isnan(p.pose.position.x));
    EXPECT_FALSE(std::isnan(p.pose.position.y));
    EXPECT_FALSE(std::isnan(p.longitudinal_velocity_mps));

    // Ensure the velocity remains bounded at 0
    EXPECT_NEAR(p.longitudinal_velocity_mps, 0.0, 0.01);
  }
}
