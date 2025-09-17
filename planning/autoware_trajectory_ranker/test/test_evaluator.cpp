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
#include "autoware/trajectory_ranker/evaluation.hpp"

#include <autoware/route_handler/route_handler.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{

using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;

class TestEvaluator : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS node for testing
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_evaluator");

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

    // Setup route handler (mock)
    route_handler_ = std::make_shared<route_handler::RouteHandler>();

    // Setup vehicle info
    vehicle_info_ = std::make_shared<vehicle_info_utils::VehicleInfo>(
      vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo());

    // Create evaluator
    evaluator_ = std::make_unique<Evaluator>(route_handler_, vehicle_info_, node_->get_logger());

    // Create sample trajectory
    createSampleTrajectory();
  }

  void TearDown() override { rclcpp::shutdown(); }

  void createSampleTrajectory()
  {
    sample_points_ = std::make_shared<TrajectoryPoints>();
    for (size_t i = 0; i < 10; ++i) {
      TrajectoryPoint pt;
      pt.pose.position.x = static_cast<double>(i) * 1.0;
      pt.pose.position.y = 0.0;
      pt.pose.position.z = 0.0;
      pt.pose.orientation.w = 1.0;
      pt.longitudinal_velocity_mps = 5.0;
      pt.lateral_velocity_mps = 0.0;
      pt.acceleration_mps2 = 0.0;
      pt.heading_rate_rps = 0.0;
      pt.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.5);
      sample_points_->push_back(pt);
    }
  }

  std::shared_ptr<CoreData> createCoreData(const std::string & tag, double offset_y = 0.0)
  {
    auto points = std::make_shared<TrajectoryPoints>();

    // Create trajectory with y offset
    for (size_t i = 0; i < 10; ++i) {
      TrajectoryPoint pt;
      pt.pose.position.x = static_cast<double>(i) * 1.0;
      pt.pose.position.y = offset_y;
      pt.pose.position.z = 0.0;
      pt.pose.orientation.w = 1.0;
      pt.longitudinal_velocity_mps = 5.0;
      pt.lateral_velocity_mps = 0.0;
      pt.acceleration_mps2 = 0.0;
      pt.heading_rate_rps = 0.0;
      pt.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.5);
      points->push_back(pt);
    }

    // Create CoreData with proper constructor
    auto ideal = std::make_shared<TrajectoryPoints>();
    auto objects = std::make_shared<PredictedObjects>();
    auto lanes = std::make_shared<lanelet::ConstLanelets>();

    return std::make_shared<CoreData>(points, ideal, objects, lanes, tag);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<route_handler::RouteHandler> route_handler_;
  std::shared_ptr<vehicle_info_utils::VehicleInfo> vehicle_info_;
  std::unique_ptr<Evaluator> evaluator_;
  std::shared_ptr<TrajectoryPoints> sample_points_;
};

TEST_F(TestEvaluator, LoadMetric)
{
  // Test loading a metric
  EXPECT_NO_THROW(
    evaluator_->load_metric("autoware::trajectory_ranker::metrics::TravelDistance", 0, 0.1));
}

TEST_F(TestEvaluator, UnloadMetric)
{
  // Load and then unload a metric
  evaluator_->load_metric("autoware::trajectory_ranker::metrics::TravelDistance", 0, 0.1);
  EXPECT_NO_THROW(
    evaluator_->unload_metric("autoware::trajectory_ranker::metrics::TravelDistance"));
}

TEST_F(TestEvaluator, AddTrajectory)
{
  auto core_data = createCoreData("test_trajectory");
  EXPECT_NO_THROW(evaluator_->add(core_data));
}

TEST_F(TestEvaluator, Setup)
{
  EXPECT_NO_THROW(evaluator_->setup(sample_points_));
}

TEST_F(TestEvaluator, Clear)
{
  auto core_data = createCoreData("test_trajectory");
  evaluator_->add(core_data);
  evaluator_->clear();
  EXPECT_EQ(evaluator_->results().size(), 0);
}

TEST_F(TestEvaluator, GetByTag)
{
  auto core_data = createCoreData("test_trajectory");
  evaluator_->add(core_data);

  auto result = evaluator_->get("test_trajectory");
  EXPECT_NE(result, nullptr);
  EXPECT_EQ(result->tag(), "test_trajectory");

  auto not_found = evaluator_->get("non_existent");
  EXPECT_EQ(not_found, nullptr);
}

TEST_F(TestEvaluator, BestWithoutMetrics)
{
  // Add multiple trajectories
  evaluator_->add(createCoreData("trajectory_1", 0.0));
  evaluator_->add(createCoreData("trajectory_2", 1.0));
  evaluator_->add(createCoreData("trajectory_3", 2.0));

  // Setup with previous trajectory
  evaluator_->setup(sample_points_);

  // Create evaluation parameters for no metrics case
  auto params = std::make_shared<EvaluatorParameters>(0, 10);
  params->metrics_max_value = {};
  params->score_weight = {};

  // Without metrics loaded, should still return a result
  auto best = evaluator_->best(params);
  EXPECT_NE(best, nullptr);
}

TEST_F(TestEvaluator, BestWithExclude)
{
  // Add multiple trajectories
  evaluator_->add(createCoreData("trajectory_1", 0.0));
  evaluator_->add(createCoreData("trajectory_2", 1.0));
  evaluator_->add(createCoreData("trajectory_3", 2.0));

  evaluator_->setup(sample_points_);

  auto params = std::make_shared<EvaluatorParameters>(1, 10);
  params->metrics_max_value = {1.0};
  params->score_weight = {1.0};

  // Exclude trajectory_1
  auto best = evaluator_->best(params, "trajectory_1");
  EXPECT_NE(best, nullptr);
  EXPECT_NE(best->tag(), "trajectory_1");
}

TEST_F(TestEvaluator, MultipleMetricsEvaluation)
{
  // Load multiple metrics
  evaluator_->load_metric("autoware::trajectory_ranker::metrics::TravelDistance", 0, 0.1);
  evaluator_->load_metric("autoware::trajectory_ranker::metrics::LateralAcceleration", 1, 0.1);

  // Add trajectories
  evaluator_->add(createCoreData("straight", 0.0));
  evaluator_->add(createCoreData("offset", 2.0));

  evaluator_->setup(sample_points_);

  // Create parameters with weights for both metrics
  auto params = std::make_shared<EvaluatorParameters>(2, 10);
  params->metrics_max_value = {10.0, 3.0};  // Max distance, max lateral accel
  params->score_weight = {0.5, 0.5};        // Equal weights

  auto best = evaluator_->best(params);
  EXPECT_NE(best, nullptr);

  // Check that all results were evaluated
  auto results = evaluator_->results();
  EXPECT_GT(results.size(), 0);
}

}  // namespace autoware::trajectory_ranker

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
