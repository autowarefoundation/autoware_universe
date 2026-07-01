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
#include <autoware_test_utils/autoware_test_utils.hpp>
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
  double length, double velocity, rclcpp::Time stamp)
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
  double segment_length, double velocity, rclcpp::Time stamp)
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

    modifier_options.append_parameter_override("use_stop_point_fixer", true);
    modifier_options.append_parameter_override("use_obstacle_stop", true);

    optimizer_options.append_parameter_override("use_akima_spline_interpolation", true);
    optimizer_options.append_parameter_override("use_eb_smoother", false);
    optimizer_options.append_parameter_override("use_qp_smoother", true);
    optimizer_options.append_parameter_override("use_trajectory_point_fixer", true);
    optimizer_options.append_parameter_override("use_velocity_optimizer", true);
    optimizer_options.append_parameter_override("use_trajectory_extender", false);
    optimizer_options.append_parameter_override("use_kinematic_feasibility_enforcer", true);
    optimizer_options.append_parameter_override("use_mpt_optimizer", false);
    optimizer_options.append_parameter_override("use_temporal_mpt_optimizer", false);

    const std::vector<std::string> modifier_plugins = {
      "autoware::trajectory_modifier::plugin::ObstacleStop",
      "autoware::trajectory_modifier::plugin::StopPointFixer"};
    modifier_options.append_parameter_override("plugin_names", modifier_plugins);

    const std::vector<std::string> optimizer_plugins = {
      "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer",
      "autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer",
      "autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother",
      "autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother",
      "autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer"};
    optimizer_options.append_parameter_override("plugin_names", optimizer_plugins);

    // Set mandatory vehicle info parameters that VehicleInfoUtils expects at the root
    for (auto & opt : {&modifier_options, &optimizer_options}) {
      opt->append_parameter_override("wheel_radius", 0.383);
      opt->append_parameter_override("wheel_width", 0.235);
      opt->append_parameter_override("wheel_base", 2.79);
      opt->append_parameter_override("wheel_tread", 1.64);
      opt->append_parameter_override("front_overhang", 1.0);
      opt->append_parameter_override("rear_overhang", 1.1);
      opt->append_parameter_override("left_overhang", 0.128);
      opt->append_parameter_override("right_overhang", 0.128);
      opt->append_parameter_override("vehicle_height", 2.5);
      opt->append_parameter_override("max_steer_angle", 0.70);
    }
    optimizer_options.append_parameter_override("max_vel", 20.0);
    optimizer_options.append_parameter_override("limit.max_acc", 2.0);
    optimizer_options.append_parameter_override("limit.min_acc", -3.0);
    optimizer_options.append_parameter_override("limit.max_jerk", 1.5);
    optimizer_options.append_parameter_override("limit.min_jerk", -1.5);

    // Optimizer plugin parameters
    optimizer_options.append_parameter_override("trajectory_point_fixer.remove_close_points", true);
    optimizer_options.append_parameter_override(
      "trajectory_point_fixer.resample_close_points", true);
    optimizer_options.append_parameter_override(
      "trajectory_point_fixer.min_dist_to_remove_m", 0.01);
    optimizer_options.append_parameter_override(
      "trajectory_point_fixer.min_dist_to_resample_m", 0.05);
    optimizer_options.append_parameter_override(
      "trajectory_point_fixer.stop_detection_velocity_threshold_mps", 0.3);

    optimizer_options.append_parameter_override(
      "trajectory_kinematic_feasibility.max_yaw_rate_rad_s", 0.7);
    optimizer_options.append_parameter_override(
      "trajectory_kinematic_feasibility.time_step_s", 0.1);

    optimizer_options.append_parameter_override("trajectory_qp_smoother.weight_smoothness", 10.0);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.weight_fidelity", 1.0);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.time_step_s", 0.1);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.osqp_eps_abs", 1e-4);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.osqp_eps_rel", 1e-4);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.osqp_max_iter", 4000);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.osqp_verbose", false);
    optimizer_options.append_parameter_override(
      "trajectory_qp_smoother.preserve_input_trajectory_orientation", true);
    optimizer_options.append_parameter_override(
      "trajectory_qp_smoother.max_distance_for_orientation_m", 5.0);
    optimizer_options.append_parameter_override(
      "trajectory_qp_smoother.use_velocity_based_fidelity", false);
    optimizer_options.append_parameter_override(
      "trajectory_qp_smoother.velocity_threshold_mps", 0.2);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.sigmoid_sharpness", 40.0);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.min_fidelity_weight", 0.1);
    optimizer_options.append_parameter_override("trajectory_qp_smoother.max_fidelity_weight", 1.0);
    optimizer_options.append_parameter_override(
      "trajectory_qp_smoother.num_constrained_points_start", 3);
    optimizer_options.append_parameter_override(
      "trajectory_qp_smoother.num_constrained_points_end", 3);

    optimizer_options.append_parameter_override(
      "trajectory_spline_smoother.interpolation_resolution_m", 0.5);
    optimizer_options.append_parameter_override(
      "trajectory_spline_smoother.max_distance_discrepancy_m", 5.0);
    optimizer_options.append_parameter_override(
      "trajectory_spline_smoother.preserve_input_trajectory_orientation", true);

    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.nearest_dist_threshold_m", 1.5);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.nearest_yaw_threshold_deg", 60.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.target_pull_out_speed_mps", 1.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.target_pull_out_acc_mps2", 1.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.max_lateral_accel_mps2", 1.5);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.min_limited_speed_mps", 3.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.set_engage_speed", false);
    optimizer_options.append_parameter_override("trajectory_velocity_optimizer.limit_speed", true);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.limit_lateral_acceleration", false);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.smooth_velocities", false);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.continuous_jerk_smoother.jerk_weight", 30.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.continuous_jerk_smoother.over_v_weight", 3000.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.continuous_jerk_smoother.over_a_weight", 30.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.continuous_jerk_smoother.over_j_weight", 10.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.continuous_jerk_smoother.velocity_tracking_weight", 1.0);
    optimizer_options.append_parameter_override(
      "trajectory_velocity_optimizer.continuous_jerk_smoother.accel_tracking_weight", 300.0);

    // Load default parameters from yaml files
    autoware::test_utils::updateNodeOptions(
      modifier_options, {modifier_dir + "/config/trajectory_modifier.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {optimizer_dir + "/config/trajectory_optimizer.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {optimizer_dir + "/config/plugins/trajectory_qp_smoother.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {optimizer_dir + "/config/plugins/trajectory_point_fixer.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options,
      {optimizer_dir + "/config/plugins/trajectory_velocity_optimizer.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {optimizer_dir + "/config/plugins/trajectory_extender.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {optimizer_dir + "/config/plugins/trajectory_spline_smoother.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options,
      {optimizer_dir + "/config/plugins/trajectory_kinematic_feasibility_enforcer.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {optimizer_dir + "/config/plugins/trajectory_mpt_optimizer.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options,
      {optimizer_dir + "/config/plugins/trajectory_temporal_mpt_optimizer.param.yaml"});

    // Add vehicle info
    autoware::test_utils::updateNodeOptions(
      modifier_options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});
    autoware::test_utils::updateNodeOptions(
      optimizer_options, {test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    // Remap modifier output to optimizer input
    modifier_options.arguments(
      {"--ros-args", "-r", "~/output/candidate_trajectories:=/combined/trajectories", "-r",
       "~/input/odometry:=/localization/kinematic_state", "-r",
       "~/input/acceleration:=/localization/acceleration", "-r",
       "~/input/objects:=/perception/object_recognition/objects", "-r",
       "~/input/pointcloud:=/perception/obstacle_segmentation/pointcloud"});
    optimizer_options.arguments(
      {"--ros-args", "-r", "~/input/trajectories:=/combined/trajectories", "-r",
       "~/input/odometry:=/localization/kinematic_state", "-r",
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

  void publish_mandatory_inputs(
    const CandidateTrajectories & traj, const Odometry & odom,
    const AccelWithCovarianceStamped & acc)
  {
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
    while (!output_received_ && sim_time_ < deadline && rclcpp::ok()) {
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

  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 5.0;

  AccelWithCovarianceStamped acc;
  acc.header.frame_id = "map";

  // Publish in a loop to ensure the nodes receive it after discovery
  for (int i = 0; i < 20 && !output_received_; ++i) {
    odom.header.stamp = sim_time_;
    acc.header.stamp = sim_time_;
    publish_mandatory_inputs(traj, odom, acc);
    advance_sim_time_and_spin(std::chrono::milliseconds(100));
  }
  wait_for_output_sim();

  ASSERT_TRUE(output_received_);
  EXPECT_GT(latest_output_.points.size(), 0U);
}

TEST_F(TrajectoryProcessorIntegrationTest, ObstacleStopIntegrationTest)
{
  auto traj = create_straight_trajectories(30.0, 8.0, sim_time_);

  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 8.0;

  AccelWithCovarianceStamped acc;
  acc.header.frame_id = "map";

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
    odom.header.stamp = sim_time_;
    acc.header.stamp = sim_time_;
    pub_obj_->publish(objects);
    publish_mandatory_inputs(traj, odom, acc);
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

  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 0.05;  // Stationary

  AccelWithCovarianceStamped acc;
  acc.header.frame_id = "map";

  for (int i = 0; i < 20 && !output_received_; ++i) {
    odom.header.stamp = sim_time_;
    acc.header.stamp = sim_time_;
    publish_mandatory_inputs(msg, odom, acc);
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
  auto traj = create_sharp_turn_trajectories(10.0, 5.0, sim_time_);

  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 5.0;

  AccelWithCovarianceStamped acc;
  acc.header.frame_id = "map";

  for (int i = 0; i < 20 && !output_received_; ++i) {
    odom.header.stamp = sim_time_;
    acc.header.stamp = sim_time_;
    publish_mandatory_inputs(traj, odom, acc);
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

  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.orientation.w = 1.0;
  odom.twist.twist.linear.x = 15.0;

  AccelWithCovarianceStamped acc;
  acc.header.frame_id = "map";

  for (int i = 0; i < 20 && !output_received_; ++i) {
    odom.header.stamp = sim_time_;
    acc.header.stamp = sim_time_;
    publish_mandatory_inputs(traj, odom, acc);
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
  auto traj = create_straight_trajectories(40.0, 10.0, sim_time_);

  Odometry odom;
  odom.header.frame_id = "map";
  odom.pose.pose.position.x = 0.0;
  odom.twist.twist.linear.x = 10.0;

  AccelWithCovarianceStamped acc;
  acc.header.frame_id = "map";

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
    odom.header.stamp = sim_time_;
    acc.header.stamp = sim_time_;
    pub_obj_->publish(objects);
    publish_mandatory_inputs(traj, odom, acc);
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
