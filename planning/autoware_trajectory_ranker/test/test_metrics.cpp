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

#include "autoware/trajectory_ranker/data_structs.hpp"
#include "autoware/trajectory_ranker/interface/data_interface.hpp"
#include "autoware/trajectory_ranker/metrics/distance_metric.hpp"
#include "autoware/trajectory_ranker/metrics/lateral_acceleration_metric.hpp"
#include "autoware/trajectory_ranker/metrics/lateral_deviation_metric.hpp"
#include "autoware/trajectory_ranker/metrics/longitudinal_jerk_metric.hpp"
#include "autoware/trajectory_ranker/metrics/steering_consistency_metric.hpp"
#include "autoware/trajectory_ranker/metrics/time_to_collision_metric.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <vector>

namespace autoware::trajectory_ranker::metrics
{

using autoware::trajectory_ranker::CoreData;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;

class TestMetrics : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node for testing
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_metrics");

    // Set required vehicle parameters
    node_->declare_parameter<double>("wheel_radius", 0.383);
    node_->declare_parameter<double>("wheel_width", 0.235);
    node_->declare_parameter<double>("wheel_base", 2.79);
    node_->declare_parameter<double>("wheel_tread", 1.64);
    node_->declare_parameter<double>("front_overhang", 1.0);
    node_->declare_parameter<double>("rear_overhang", 1.1);
    node_->declare_parameter<double>("left_overhang", 0.5);
    node_->declare_parameter<double>("right_overhang", 0.5);
    node_->declare_parameter<double>("vehicle_height", 2.5);
    node_->declare_parameter<double>("max_steer_angle", 0.7);

    // Setup vehicle info
    vehicle_info_ = std::make_shared<vehicle_info_utils::VehicleInfo>(
      vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo());

    // Create sample result
    createSampleResult();
  }

  void TearDown() override { rclcpp::shutdown(); }

  void createSampleResult()
  {
    auto points = std::make_shared<TrajectoryPoints>();

    // Create a simple trajectory
    for (size_t i = 0; i < 20; ++i) {
      TrajectoryPoint pt;
      pt.pose.position.x = static_cast<double>(i) * 1.0;
      pt.pose.position.y = 0.0;
      pt.pose.position.z = 0.0;
      pt.pose.orientation.w = 1.0;
      pt.longitudinal_velocity_mps = 5.0;
      pt.lateral_velocity_mps = 0.0;
      pt.acceleration_mps2 = 0.5;  // Constant acceleration
      pt.heading_rate_rps = 0.0;
      pt.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.2);
      points->push_back(pt);
    }

    auto ideal = std::make_shared<TrajectoryPoints>();
    auto objects = std::make_shared<PredictedObjects>();
    auto lanes = std::make_shared<lanelet::ConstLanelets>();
    auto core_data = std::make_shared<CoreData>(points, ideal, objects, lanes, "test");

    result_ =
      std::make_shared<autoware::trajectory_ranker::DataInterface>(core_data, 6);  // 6 metrics
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<vehicle_info_utils::VehicleInfo> vehicle_info_;
  std::shared_ptr<autoware::trajectory_ranker::DataInterface> result_;
};

TEST_F(TestMetrics, TravelDistanceMetric)
{
  TravelDistance metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  EXPECT_EQ(metric.name(), "TravelDistance");
  EXPECT_FALSE(metric.is_deviation());

  metric.evaluate(result_, 100.0);

  // Just check evaluation doesn't throw
  EXPECT_NO_THROW();
}

TEST_F(TestMetrics, LateralAccelerationMetric)
{
  LateralAcceleration metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  EXPECT_EQ(metric.name(), "LateralAcceleration");
  EXPECT_TRUE(metric.is_deviation());

  EXPECT_NO_THROW(metric.evaluate(result_, 3.0));
}

TEST_F(TestMetrics, LateralDeviationMetric)
{
  LateralDeviation metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  EXPECT_EQ(metric.name(), "LateralDeviation");
  EXPECT_TRUE(metric.is_deviation());

  // For now, just test that evaluation doesn't throw
  EXPECT_NO_THROW(metric.evaluate(result_, 1.0));
}

TEST_F(TestMetrics, LongitudinalJerkMetric)
{
  LongitudinalJerk metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  EXPECT_EQ(metric.name(), "LongitudinalJerk");
  EXPECT_TRUE(metric.is_deviation());

  EXPECT_NO_THROW(metric.evaluate(result_, 2.0));
}

TEST_F(TestMetrics, SteeringConsistencyMetric)
{
  // Create a result with previous points
  auto prev_points = std::make_shared<TrajectoryPoints>();
  for (size_t i = 0; i < 20; ++i) {
    TrajectoryPoint pt;
    pt.pose.position.x = static_cast<double>(i) * 1.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 0.0;
    pt.pose.orientation.w = 1.0;
    pt.longitudinal_velocity_mps = 5.0;
    prev_points->push_back(pt);
  }
  result_->setup(prev_points);

  SteeringConsistency metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  EXPECT_EQ(metric.name(), "SteeringConsistency");
  EXPECT_TRUE(metric.is_deviation());

  // For now, just test that evaluation doesn't throw
  EXPECT_NO_THROW(metric.evaluate(result_, 0.5));
}

TEST_F(TestMetrics, TimeToCollisionMetric)
{
  TimeToCollision metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  EXPECT_EQ(metric.name(), "TimeToCollision");
  EXPECT_FALSE(metric.is_deviation());

  // Create predicted objects
  auto objects = std::make_shared<PredictedObjects>();
  PredictedObject obj;
  obj.kinematics.initial_pose_with_covariance.pose.position.x = 100.0;  // Far away
  obj.kinematics.initial_pose_with_covariance.pose.position.y = 0.0;
  obj.kinematics.initial_pose_with_covariance.pose.orientation.w = 1.0;
  obj.kinematics.initial_twist_with_covariance.twist.linear.x = 0.0;  // Stationary

  autoware_perception_msgs::msg::PredictedPath path;
  path.confidence = 1.0;
  path.time_step = rclcpp::Duration::from_seconds(0.1);
  geometry_msgs::msg::Pose path_pose;
  path_pose.position = obj.kinematics.initial_pose_with_covariance.pose.position;
  path_pose.orientation = obj.kinematics.initial_pose_with_covariance.pose.orientation;
  path.path.push_back(path_pose);
  path.path.push_back(path_pose);  // Stationary object

  obj.kinematics.predicted_paths.push_back(path);
  objects->objects.push_back(obj);

  // For now, just test that evaluation doesn't throw
  EXPECT_NO_THROW(metric.evaluate(result_, 10.0));
}

TEST_F(TestMetrics, MetricWithEmptyTrajectory)
{
  // Create empty result
  auto empty_points = std::make_shared<TrajectoryPoints>();
  auto empty_ideal = std::make_shared<TrajectoryPoints>();
  auto empty_objects = std::make_shared<PredictedObjects>();
  auto empty_lanes = std::make_shared<lanelet::ConstLanelets>();
  auto empty_core_data =
    std::make_shared<CoreData>(empty_points, empty_ideal, empty_objects, empty_lanes, "empty");
  auto empty_result =
    std::make_shared<autoware::trajectory_ranker::DataInterface>(empty_core_data, 6);

  TravelDistance metric;
  metric.init(vehicle_info_, 0.1);
  metric.set_index(0);

  // Should handle empty trajectory gracefully
  EXPECT_NO_THROW(metric.evaluate(empty_result, 100.0));
}

}  // namespace autoware::trajectory_ranker::metrics

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
