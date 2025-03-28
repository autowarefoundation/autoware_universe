// Copyright 2025 Tier IV, Inc.
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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <autoware/planning_evaluator/planning_evaluator_node.hpp>
#include <autoware/planning_factor_interface/planning_factor_interface.hpp>

#include "autoware_perception_msgs/msg/predicted_objects.hpp"
#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <tier4_metric_msgs/msg/metric_array.hpp>

#include "boost/lexical_cast.hpp"

#include <iostream>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

using EvalNode = planning_diagnostics::PlanningEvaluatorNode;
using Trajectory = autoware_planning_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_planning_msgs::msg::TrajectoryPoint;
using Objects = autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::PoseWithUuidStamped;
using MetricArrayMsg = tier4_metric_msgs::msg::MetricArray;
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::PlanningFactorArray;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_vehicle_msgs::msg::SteeringReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;

constexpr double epsilon = 1e-6;

class EvalTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("autoware_planning_evaluator");
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    options.arguments(
      {"--ros-args", "-p", "output_metrics:=true", "--params-file",
       share_dir + "/config/planning_evaluator.param.yaml", "--params-file",
       autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    dummy_node = std::make_shared<rclcpp::Node>("planning_evaluator_test_node");
    eval_node = std::make_shared<EvalNode>(options);
    // Enable all logging in the node
    auto ret = rcutils_logging_set_logger_level(
      dummy_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      std::cerr << "Failed to set logging severity to DEBUG\n";
    }
    ret = rcutils_logging_set_logger_level(
      eval_node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK) {
      std::cerr << "Failed to set logging severity to DEBUG\n";
    }

    traj_pub_ =
      rclcpp::create_publisher<Trajectory>(dummy_node, "/planning_evaluator/input/trajectory", 1);
    ref_traj_pub_ = rclcpp::create_publisher<Trajectory>(
      dummy_node, "/planning_evaluator/input/reference_trajectory", 1);
    objects_pub_ =
      rclcpp::create_publisher<Objects>(dummy_node, "/planning_evaluator/input/objects", 1);
    odom_pub_ =
      rclcpp::create_publisher<Odometry>(dummy_node, "/planning_evaluator/input/odometry", 1);
    modified_goal_pub_ = rclcpp::create_publisher<PoseWithUuidStamped>(
      dummy_node, "/planning_evaluator/input/modified_goal", 1);
    blinker_pub_ = rclcpp::create_publisher<TurnIndicatorsReport>(
      dummy_node, "/planning_evaluator/input/turn_indicators_status", 1);
    steering_pub_ = rclcpp::create_publisher<SteeringReport>(
      dummy_node, "/planning_evaluator/input/steering_status", 1);
    planning_factor_interface_ =
      std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
        dummy_node.get(), stop_decision_module_name);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(dummy_node);
    vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*eval_node).getVehicleInfo();
    publishEgoPose(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  ~EvalTest() override { rclcpp::shutdown(); }

  void setTargetMetric(planning_diagnostics::Metric metric, const std::string & postfix = "/mean")
  {
    const auto metric_str = planning_diagnostics::metric_to_str.at(metric);
    const auto is_target_metric = [metric_str, postfix](const auto & metric) {
      return metric.name == metric_str + postfix;
    };
    metric_sub_ = rclcpp::create_subscription<MetricArrayMsg>(
      dummy_node, "/planning_evaluator/metrics", 1, [=](const MetricArrayMsg::ConstSharedPtr msg) {
        const auto it =
          std::find_if(msg->metric_array.begin(), msg->metric_array.end(), is_target_metric);
        if (it != msg->metric_array.end()) {
          metric_value_ = boost::lexical_cast<double>(it->value);
          metric_updated_ = true;
        }
      });
  }
  Trajectory makeTrajectory(const std::vector<std::pair<double, double>> & traj)
  {
    Trajectory t;
    t.header.frame_id = "map";
    TrajectoryPoint p;
    for (const std::pair<double, double> & point : traj) {
      p.pose.position.x = point.first;
      p.pose.position.y = point.second;
      t.points.push_back(p);
    }
    return t;
  }

  Trajectory makeTrajectory(const std::vector<std::tuple<double, double, double>> & traj)
  {
    Trajectory t;
    t.header.frame_id = "map";
    TrajectoryPoint p;
    for (const std::tuple<double, double, double> & point : traj) {
      p.pose.position.x = std::get<0>(point);
      p.pose.position.y = std::get<1>(point);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, std::get<2>(point));
      p.pose.orientation.x = q.x();
      p.pose.orientation.y = q.y();
      p.pose.orientation.z = q.z();
      p.pose.orientation.w = q.w();
      t.points.push_back(p);
    }
    return t;
  }

  void publishTrajectory(const Trajectory & traj)
  {
    traj_pub_->publish(traj);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  void publishReferenceTrajectory(const Trajectory & traj)
  {
    ref_traj_pub_->publish(traj);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  void publishObjects(const Objects & obj)
  {
    objects_pub_->publish(obj);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  double publishTrajectoryAndGetMetric(const Trajectory & traj)
  {
    metric_updated_ = false;
    traj_pub_->publish(traj);
    while (!metric_updated_) {
      rclcpp::spin_some(eval_node);
      rclcpp::spin_some(dummy_node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return metric_value_;
  }

  double publishModifiedGoalAndGetMetric(const double x, const double y, const double yaw)
  {
    metric_updated_ = false;

    PoseWithUuidStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = dummy_node->now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    goal.pose.orientation.x = q.x();
    goal.pose.orientation.y = q.y();
    goal.pose.orientation.z = q.z();
    goal.pose.orientation.w = q.w();
    modified_goal_pub_->publish(goal);

    while (!metric_updated_) {
      rclcpp::spin_some(eval_node);
      rclcpp::spin_some(dummy_node);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    return metric_value_;
  }

  void publishEgoPose(
    const double x, const double y, const double yaw, const double x_vel, const double y_vel)
  {
    Odometry odom;
    odom.header.frame_id = "map";
    odom.header.stamp = dummy_node->now();
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = x_vel;
    odom.twist.twist.linear.y = y_vel;

    odom_pub_->publish(odom);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  void publishStopPlanningFactor(
    const double distance, const double stop_point_x, const double stop_point_y,
    const size_t sleep_time = 100)
  {
    Pose stop_point = Pose();
    stop_point.position.x = stop_point_x;
    stop_point.position.y = stop_point_y;

    planning_factor_interface_->add(
      distance, stop_point, PlanningFactor::STOP, SafetyFactorArray({}));
    planning_factor_interface_->publish();

    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  }

  void publishTurnIndicatorsReport(bool enable, bool is_left, const size_t sleep_time = 100)
  {
    TurnIndicatorsReport msg;
    msg.stamp = dummy_node->now();
    msg.report =
      enable ? (is_left ? TurnIndicatorsReport::ENABLE_LEFT : TurnIndicatorsReport::ENABLE_RIGHT)
             : TurnIndicatorsReport::DISABLE;
    blinker_pub_->publish(msg);

    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  }

  void publishSteeringAngle(const float angle, const size_t sleep_time = 100)
  {
    SteeringReport msg;
    msg.stamp = dummy_node->now();
    msg.steering_tire_angle = angle;
    steering_pub_->publish(msg);
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(sleep_time));
  }

  // Latest metric value
  bool metric_updated_ = false;
  double metric_value_;
  // Node
  rclcpp::Node::SharedPtr dummy_node;
  EvalNode::SharedPtr eval_node;
  // Trajectory publishers
  rclcpp::Publisher<Trajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr ref_traj_pub_;
  rclcpp::Publisher<Objects>::SharedPtr objects_pub_;
  rclcpp::Publisher<PoseWithUuidStamped>::SharedPtr modified_goal_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<MetricArrayMsg>::SharedPtr metric_sub_;
  rclcpp::Publisher<SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<TurnIndicatorsReport>::SharedPtr blinker_pub_;
  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  std::string stop_decision_module_name = "out_of_lane";

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

public:
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
};

TEST_F(EvalTest, TestCurvature)
{
  setTargetMetric(planning_diagnostics::Metric::curvature);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -1.0);
  t = makeTrajectory({{0.0, 0.0}, {2.0, -2.0}, {4.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.5);
}

TEST_F(EvalTest, TestPointInterval)
{
  setTargetMetric(planning_diagnostics::Metric::point_interval);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
  // double the average interval
  TrajectoryPoint p;
  p.pose.position.x = 0.0;
  p.pose.position.y = 6.0;
  t.points.push_back(p);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 2.0);
}

TEST_F(EvalTest, TestRelativeAngle)
{
  setTargetMetric(planning_diagnostics::Metric::relative_angle);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -M_PI_4);
  // add an angle of PI/4 to bring the average to 0
  TrajectoryPoint p;
  p.pose.position.x = 1.0;
  p.pose.position.y = 2.0;
  t.points.push_back(p);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
}

TEST_F(EvalTest, TestResampledRelativeAngle)
{
  setTargetMetric(planning_diagnostics::Metric::resampled_relative_angle);
  Trajectory t = makeTrajectory({{0.0, 0.0, 0.0}, {vehicle_info_.vehicle_length_m, 0.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  t = makeTrajectory(
    {{0.0, 0.0, 0.0}, {vehicle_info_.vehicle_length_m, vehicle_info_.vehicle_length_m, M_PI_4}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), M_PI_4);
}

TEST_F(EvalTest, TestLength)
{
  setTargetMetric(planning_diagnostics::Metric::length);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 3.0);
  TrajectoryPoint p;
  p.pose.position.x = 3.0;
  p.pose.position.y = 3.0;
  t.points.push_back(p);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 6.0);
}

TEST_F(EvalTest, TestVelocity)
{
  setTargetMetric(planning_diagnostics::Metric::velocity);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}});
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
}

TEST_F(EvalTest, TestDuration)
{
  setTargetMetric(planning_diagnostics::Metric::duration);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}, {0.0, 2.0}, {0.0, 3.0}});
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 3.0);
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 3.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
}

TEST_F(EvalTest, TestAcceleration)
{
  setTargetMetric(planning_diagnostics::Metric::acceleration);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}});
  t.points[0].acceleration_mps2 = 1.0;
  t.points[1].acceleration_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
  t.points[0].acceleration_mps2 = -1.0;
  t.points[1].acceleration_mps2 = -1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -1.0);
  t.points[0].acceleration_mps2 = 0.0;
  t.points[1].acceleration_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.5);
}

TEST_F(EvalTest, TestJerk)
{
  setTargetMetric(planning_diagnostics::Metric::jerk);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {0.0, 1.0}});
  t.points[0].longitudinal_velocity_mps = 1.0;
  t.points[0].acceleration_mps2 = 1.0;
  t.points[1].longitudinal_velocity_mps = 2.0;
  t.points[1].acceleration_mps2 = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  t.points[0].longitudinal_velocity_mps = 1.0;
  t.points[0].acceleration_mps2 = 1.0;
  t.points[1].longitudinal_velocity_mps = 1.0;
  t.points[1].acceleration_mps2 = 0.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), -1.0);
}

TEST_F(EvalTest, TestLateralDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::lateral_deviation);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}});
  publishReferenceTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  Trajectory t2 = makeTrajectory({{0.0, 1.0}, {1.0, 1.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0);
}

TEST_F(EvalTest, TestYawDeviation)
{
  auto setYaw = [](geometry_msgs::msg::Quaternion & msg, const double yaw_rad) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
  };
  setTargetMetric(planning_diagnostics::Metric::yaw_deviation);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}});
  for (auto & p : t.points) {
    setYaw(p.pose.orientation, M_PI);
  }
  publishReferenceTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  Trajectory t2 = t;
  for (auto & p : t2.points) {
    setYaw(p.pose.orientation, 0.0);
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), -M_PI);
  for (auto & p : t2.points) {
    setYaw(p.pose.orientation, -M_PI);
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 0.0);
}

TEST_F(EvalTest, TestVelocityDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::velocity_deviation);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}});
  for (auto & p : t.points) {
    p.longitudinal_velocity_mps = 0.0;
  }
  publishReferenceTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  for (auto & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 1.0);
}

TEST_F(EvalTest, TestStability)
{
  setTargetMetric(planning_diagnostics::Metric::stability);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);
  t.points.back().pose.position.x = 0.0;
  t.points.back().pose.position.y = 0.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);

  Trajectory t2 = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t2);
  t2.points.back().pose.position.x = 4.0;
  t2.points.back().pose.position.y = 3.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0 / 4);
}

TEST_F(EvalTest, TestFrechet)
{
  setTargetMetric(planning_diagnostics::Metric::stability_frechet);
  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t);
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.0);

  // variation in the last point: simple distance from previous last point
  t.points.back().pose.position.x = 0.0;
  t.points.back().pose.position.y = 0.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), std::sqrt(18.0));
  Trajectory t2 = makeTrajectory({{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}, {3.0, 3.0}});
  publishTrajectory(t2);
  t2.points.back().pose.position.x = 4.0;
  t2.points.back().pose.position.y = 3.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0);

  // variations in the middle points: cannot go back to previous points that minimize the distance
  t2.points[2].pose.position.x = 0.5;
  t2.points[2].pose.position.y = 0.5;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), std::sqrt(2 * (1.5 * 1.5)));
}

TEST_F(EvalTest, TestObstacleDistance)
{
  setTargetMetric(planning_diagnostics::Metric::obstacle_distance);
  Objects objs;
  autoware_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  objs.objects.push_back(obj);
  publishObjects(objs);

  Trajectory t = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 0.5);
  Trajectory t2 = makeTrajectory({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t2), 1.0);  // (0.0 + 1.0 + 2.0) / 3
}

TEST_F(EvalTest, TestObstacleTTC)
{
  setTargetMetric(planning_diagnostics::Metric::obstacle_ttc);
  Objects objs;
  autoware_perception_msgs::msg::PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  objs.objects.push_back(obj);
  publishObjects(objs);

  Trajectory t = makeTrajectory({{3.0, 0.0}, {0.0, 0.0}, {-1.0, 0.0}});
  for (TrajectoryPoint & p : t.points) {
    p.longitudinal_velocity_mps = 1.0;
  }
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 3.0);
  // if no exact collision point, last point before collision is used
  t.points[1].pose.position.x = 1.0;
  EXPECT_DOUBLE_EQ(publishTrajectoryAndGetMetric(t), 2.0);
}

TEST_F(EvalTest, TestModifiedGoalLongitudinalDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::modified_goal_longitudinal_deviation);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(1.0, 0.0, 0.0), 1.0, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(1.0, 0.0, M_PI_2), 0.0, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(0.0, 1.0, 0.0), 0.0, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(0.0, 1.0, M_PI_2), 1.0, epsilon);
}

TEST_F(EvalTest, TestModifiedGoalLateralDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::modified_goal_lateral_deviation);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(1.0, 0.0, 0.0), 0.0, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(1.0, 0.0, M_PI_2), 1.0, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(0.0, 1.0, 0.0), 1.0, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(0.0, 1.0, M_PI_2), 0.0, epsilon);
}

TEST_F(EvalTest, TestModifiedGoalYawDeviation)
{
  setTargetMetric(planning_diagnostics::Metric::modified_goal_yaw_deviation);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(0.0, 0.0, M_PI_2), M_PI_2, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(1.0, 1.0, -M_PI_2), M_PI_2, epsilon);
  EXPECT_NEAR(publishModifiedGoalAndGetMetric(1.0, 1.0, -M_PI_4), M_PI_4, epsilon);
}

TEST_F(EvalTest, TestStopDecisionDistance)
{
  setTargetMetric(
    planning_diagnostics::Metric::stop_decision,
    "/" + stop_decision_module_name + "/distance_to_stop");
  metric_updated_ = false;
  publishEgoPose(0.0, 0.0, 0.0, 2.0, 0.0);
  publishStopPlanningFactor(10.0, 10.0, 0.0);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 10.0);
}

TEST_F(EvalTest, TestStopDecisionDuration)
{
  setTargetMetric(
    planning_diagnostics::Metric::stop_decision,
    "/" + stop_decision_module_name + "/keep_duration");
  metric_updated_ = false;
  publishEgoPose(0.0, 0.0, 0.0, 2.0, 0.0);
  publishStopPlanningFactor(10.0, 10.0, 0.0, 500);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NEAR(metric_value_, 0.0, 0.1);

  metric_updated_ = false;
  publishStopPlanningFactor(100.0, 100.0, 0.0, 500);
  publishStopPlanningFactor(100.0, 100.0, 0.0);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NEAR(metric_value_, 0.5, 0.1);
}

TEST_F(EvalTest, TestStopDecisionChange)
{
  setTargetMetric(
    planning_diagnostics::Metric::stop_decision,
    "/" + stop_decision_module_name + "/keep_duration");
  metric_updated_ = false;
  publishEgoPose(0.0, 0.0, 0.0, 2.0, 0.0);
  publishStopPlanningFactor(5.0, 5.0, 0.0, 200);
  publishStopPlanningFactor(100.0, 100.0, 0.0, 200);
  publishStopPlanningFactor(5.0, 5.0, 0.0, 1000);
  publishStopPlanningFactor(5.0, 5.0, 0.0);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NEAR(metric_value_, 1.0, 0.1);
}

TEST_F(EvalTest, TestAbnormalStopDecisionDistance)
{
  setTargetMetric(
    planning_diagnostics::Metric::stop_decision,
    "/" + stop_decision_module_name + "/distance_to_stop");
  metric_updated_ = false;
  publishEgoPose(0.0, 0.0, 0.0, 10.0, 0.0);
  publishStopPlanningFactor(5.0, 5.0, 0.0);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 5.0);
}

TEST_F(EvalTest, TestAbnormalStopDecisionDuration)
{
  setTargetMetric(
    planning_diagnostics::Metric::stop_decision,
    "/" + stop_decision_module_name + "/keep_duration");
  metric_updated_ = false;
  publishEgoPose(0.0, 0.0, 0.0, 10.0, 0.0);
  publishStopPlanningFactor(15.0, 15.0, 0.0, 500);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NEAR(metric_value_, 0.0, 0.1);

  metric_updated_ = false;
  publishStopPlanningFactor(5.0, 5.0, 0.0, 1000);
  publishStopPlanningFactor(5.0, 5.0, 0.0);
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_NEAR(metric_value_, 1.0, 0.1);
}

TEST_F(EvalTest, TestBlinkerChangeCount)
{
  setTargetMetric(planning_diagnostics::Metric::blinker_change_count, "/count_in_duration");
  metric_updated_ = false;
  publishTurnIndicatorsReport(false, false);  // no blinker
  publishTurnIndicatorsReport(true, true);    // left blinker
  publishTurnIndicatorsReport(true, true);    // keep left blinker
  publishTurnIndicatorsReport(true, true);    // keep left blinker
  publishTurnIndicatorsReport(false, false);  // no blinker
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 1.0);

  metric_updated_ = false;
  publishTurnIndicatorsReport(false, false);  // no blinker
  publishTurnIndicatorsReport(true, true);    // left blinker
  publishTurnIndicatorsReport(true, true);    // left blinker
  publishTurnIndicatorsReport(true, false);   // right blinker
  publishTurnIndicatorsReport(true, true);    // left blinker
  publishTurnIndicatorsReport(true, false);   // right blinker
  publishTurnIndicatorsReport(true, false);   // right blinker
  publishTurnIndicatorsReport(false, false);  // no blinker
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 5.0);
}

TEST_F(EvalTest, TestBlinkerChangeCountTimeOut)
{
  setTargetMetric(planning_diagnostics::Metric::blinker_change_count, "/count_in_duration");
  metric_updated_ = false;
  publishTurnIndicatorsReport(false, false);  // no blinker
  publishTurnIndicatorsReport(true, true);    // left blinker
  publishTurnIndicatorsReport(false, false);  // no blinker
  publishTurnIndicatorsReport(true, false);   // right blinker

  rclcpp::sleep_for(std::chrono::milliseconds(10000));  // wait for 10 seconds to time out
  publishTurnIndicatorsReport(false, false);            // no blinker
  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 0.0);
}

TEST_F(EvalTest, TestSteeringChangeCount)
{
  setTargetMetric(planning_diagnostics::Metric::steer_change_count, "/count_in_duration");
  metric_updated_ = false;
  publishSteeringAngle(0.0, 100);   // init
  publishSteeringAngle(0.0, 100);   // steer_rate around 0
  publishSteeringAngle(0.05, 100);  // steer_rate positive
  publishSteeringAngle(0.10, 100);  // steer_rate positive
  publishSteeringAngle(0.10, 100);  // steer_rate around 0
  publishSteeringAngle(0.05, 100);  // steer_rate negative
  publishSteeringAngle(0.0, 100);   // steer_rate negative
  publishSteeringAngle(0.0, 100);   // steer_rate around 0

  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 2.0);
}

TEST_F(EvalTest, TestSteeringChangeCountTimeOut)
{
  setTargetMetric(planning_diagnostics::Metric::steer_change_count, "/count_in_duration");
  metric_updated_ = false;
  publishSteeringAngle(0.0, 100);   // init
  publishSteeringAngle(0.0, 100);   // steer_rate around 0
  publishSteeringAngle(0.05, 100);  // steer_rate positive
  publishSteeringAngle(0.0, 100);   // steer_rate negative

  rclcpp::sleep_for(std::chrono::milliseconds(10000));  // wait for 10 seconds to time out
  publishSteeringAngle(0.0, 100);                       // steer_rate around 0
  publishSteeringAngle(0.05, 100);                      // steer_rate positive
  publishSteeringAngle(0.05, 100);                      // steer_rate around 0

  while (!metric_updated_) {
    rclcpp::spin_some(eval_node);
    rclcpp::spin_some(dummy_node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_DOUBLE_EQ(metric_value_, 1.0);
}
