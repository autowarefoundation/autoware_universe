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

#include "autoware/trajectory_ranker/trajectory_ranker.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_trajectory_ranker/autoware_trajectory_ranker_param.hpp>
#include <autoware_trajectory_validator/msg/risk_level.hpp>
#include <autoware_utils_uuid/uuid_helper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/candidate_trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::trajectory_ranker
{
namespace
{
using autoware_internal_planning_msgs::msg::CandidateTrajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_trajectory_validator::msg::RiskLevel;

CandidateTrajectory make_candidate_trajectory()
{
  CandidateTrajectory trajectory;
  trajectory.generator_id = autoware_utils_uuid::generate_uuid();
  trajectory.header.frame_id = "map";

  for (size_t i = 0; i < 5; ++i) {
    TrajectoryPoint point;
    point.pose.position.x = static_cast<double>(i);
    point.pose.orientation.w = 1.0;
    point.longitudinal_velocity_mps = 3.0;
    point.time_from_start = rclcpp::Duration::from_seconds(static_cast<double>(i) * 0.5);
    trajectory.points.push_back(point);
  }
  return trajectory;
}

RankerInputTrajectory make_input(
  const TrajectorySource source, const RiskLevel::_level_type risk_level = RiskLevel::SAFE)
{
  RankerInputTrajectory input;
  input.candidate_trajectory = make_candidate_trajectory();
  input.risk_level = risk_level;
  input.source = source;
  return input;
}

trajectory_ranker_params::Params make_params(
  const bool enable_safety, const bool enable_source, const bool enable_evaluation)
{
  trajectory_ranker_params::Params params;

  params.safety.enable = enable_safety;
  params.safety.levels = {"safe", "low_caution", "high_caution", "danger", "fatal"};
  params.safety.penalty = {0.0, 0.1, 0.4, 1.0, 1.0};
  params.safety.scale = 20.0;

  params.source.enable = enable_source;
  params.source.levels = {"diffusion_planner", "backup_planner_go", "backup_planner_stop"};
  params.source.penalty = {0.0, 0.45, 0.5};
  params.source.scale = 5.0;

  params.evaluation.enable = enable_evaluation;
  params.evaluation.plugin_names = {};
  params.evaluation.scale = 1.0;
  params.evaluation.sampling_number = 16;
  params.evaluation.sampling_resolution = 0.5;
  params.evaluation.trajectory_history_size = 10;

  return params;
}

}  // namespace

class TrajectoryRankerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    auto node_options = rclcpp::NodeOptions{};
    const auto autoware_test_utils_dir =
      ament_index_cpp::get_package_share_directory("autoware_test_utils");
    autoware::test_utils::updateNodeOptions(
      node_options, {autoware_test_utils_dir + "/config/test_vehicle_info.param.yaml"});

    node_ = std::make_shared<rclcpp::Node>("test_trajectory_ranker_integration", node_options);

    vehicle_info_ = std::make_shared<vehicle_info_utils::VehicleInfo>(
      vehicle_info_utils::VehicleInfoUtils(*node_).getVehicleInfo());
  }

  void TearDown() override { rclcpp::shutdown(); }

  void configure_ranker(const trajectory_ranker_params::Params & params)
  {
    params_ = params;
    evaluator_ =
      std::make_shared<Evaluator>(vehicle_info_, node_->get_logger(), params_.evaluation);
    ranker_ = std::make_unique<TrajectoryRanker>(evaluator_, params_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<vehicle_info_utils::VehicleInfo> vehicle_info_;
  std::shared_ptr<Evaluator> evaluator_;
  std::unique_ptr<TrajectoryRanker> ranker_;
  trajectory_ranker_params::Params params_;
  RankerContext context_{};
};

// Case 1: DiffusionPlanner_ vs MinimumRuleBasedPlanner_Go
TEST_F(TrajectoryRankerIntegrationTest, TestSourceOnly_PrefersDiffusionOverBackupGo)
{
  configure_ranker(make_params(false, true, false));

  RankerInputTrajectories inputs = {
    make_input(TrajectorySource::BACKUP_PLANNER_GO),
    make_input(TrajectorySource::DIFFUSION_PLANNER),
  };

  const auto result = ranker_->process(inputs, context_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(
    result->best_trajectory_info.input_trajectory.source, TrajectorySource::DIFFUSION_PLANNER);

  ASSERT_EQ(result->scored_trajectories.scored_candidate_trajectories.size(), 2u);
  const auto & scored = result->scored_trajectories.scored_candidate_trajectories;
  // Input order preserved: [GO, DIFFUSION]
  EXPECT_GT(scored[1].score, scored[0].score)
    << "Diffusion planner should be preferred over backup go";
}

// Case 2: DiffusionPlanner_ vs MinimumRuleBasedPlanner_Stop
TEST_F(TrajectoryRankerIntegrationTest, TestSourceOnly_PrefersDiffusionOverBackupStop)
{
  configure_ranker(make_params(false, true, false));

  RankerInputTrajectories inputs = {
    make_input(TrajectorySource::BACKUP_PLANNER_STOP),
    make_input(TrajectorySource::DIFFUSION_PLANNER),
  };

  const auto result = ranker_->process(inputs, context_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(
    result->best_trajectory_info.input_trajectory.source, TrajectorySource::DIFFUSION_PLANNER);

  ASSERT_EQ(result->scored_trajectories.scored_candidate_trajectories.size(), 2u);
  const auto & scored = result->scored_trajectories.scored_candidate_trajectories;
  // Input order preserved: [STOP, DIFFUSION]
  EXPECT_GT(scored[1].score, scored[0].score)
    << "Diffusion planner should be preferred over backup stop";
}

// Case 3: MinimumRuleBasedPlanner_Go vs MinimumRuleBasedPlanner_Stop
TEST_F(TrajectoryRankerIntegrationTest, TestSourceOnly_PrefersBackupGoOverBackupStop)
{
  configure_ranker(make_params(false, true, false));

  RankerInputTrajectories inputs = {
    make_input(TrajectorySource::BACKUP_PLANNER_STOP),
    make_input(TrajectorySource::BACKUP_PLANNER_GO),
  };

  const auto result = ranker_->process(inputs, context_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(
    result->best_trajectory_info.input_trajectory.source, TrajectorySource::BACKUP_PLANNER_GO);

  ASSERT_EQ(result->scored_trajectories.scored_candidate_trajectories.size(), 2u);
  const auto & scored = result->scored_trajectories.scored_candidate_trajectories;
  // Input order preserved: [STOP, GO]
  EXPECT_GT(scored[1].score, scored[0].score) << "Backup go should be preferred over backup stop";
}

// Case 4: one trajectory per risk level; SAFE should score highest
TEST_F(TrajectoryRankerIntegrationTest, TestSafetyOnly_PrefersSafestRiskLevel)
{
  configure_ranker(make_params(true, false, false));

  // Worst-first so selection cannot be explained by input order alone.
  // Source is fixed; only risk level differs.
  RankerInputTrajectories inputs = {
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::FATAL),
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::DANGER),
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::HIGH_CAUTION),
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::LOW_CAUTION),
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::SAFE),
  };

  const auto result = ranker_->process(inputs, context_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->best_trajectory_info.input_trajectory.risk_level, RiskLevel::SAFE);

  ASSERT_EQ(result->scored_trajectories.scored_candidate_trajectories.size(), 5u);
  const auto & scored = result->scored_trajectories.scored_candidate_trajectories;
  // Input order: [FATAL, DANGER, HIGH_CAUTION, LOW_CAUTION, SAFE]
  // Penalties:    [1.0,   1.0,    0.4,          0.1,         0.0 ]
  EXPECT_FLOAT_EQ(scored[0].score, scored[1].score)
    << "DANGER and FATAL share the same default penalty";
  EXPECT_GT(scored[2].score, scored[1].score)
    << "HIGH_CAUTION should score higher than DANGER/FATAL";
  EXPECT_GT(scored[3].score, scored[2].score)
    << "LOW_CAUTION should score higher than HIGH_CAUTION";
  EXPECT_GT(scored[4].score, scored[3].score) << "SAFE should score higher than LOW_CAUTION";
}

// Case 5: safety + source interaction.
// low_caution diffusion (penalty 20*0.1 + 5*0.0) beats safe backup_go (20*0.0 + 5*0.45).
TEST_F(
  TrajectoryRankerIntegrationTest, TestSafetyAndSource_PrefersLowCautionDiffusionOverSafeBackupGo)
{
  configure_ranker(make_params(true, true, false));

  RankerInputTrajectories inputs = {
    make_input(TrajectorySource::BACKUP_PLANNER_GO, RiskLevel::SAFE),
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::LOW_CAUTION),
  };

  const auto result = ranker_->process(inputs, context_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(
    result->best_trajectory_info.input_trajectory.source, TrajectorySource::DIFFUSION_PLANNER);
  EXPECT_EQ(result->best_trajectory_info.input_trajectory.risk_level, RiskLevel::LOW_CAUTION);

  ASSERT_EQ(result->scored_trajectories.scored_candidate_trajectories.size(), 2u);
  const auto & scored = result->scored_trajectories.scored_candidate_trajectories;
  // Input order: [SAFE backup_go, LOW_CAUTION diffusion]
  EXPECT_GT(scored[1].score, scored[0].score)
    << "LOW_CAUTION diffusion should score higher than SAFE backup_go";
}

// Case 6: safety + source interaction.
// high_caution diffusion (penalty 20*0.4 + 5*0.0) loses to safe backup_go (20*0.0 + 5*0.45).
TEST_F(
  TrajectoryRankerIntegrationTest, TestSafetyAndSource_PrefersSafeBackupGoOverHighCautionDiffusion)
{
  configure_ranker(make_params(true, true, false));

  RankerInputTrajectories inputs = {
    make_input(TrajectorySource::DIFFUSION_PLANNER, RiskLevel::HIGH_CAUTION),
    make_input(TrajectorySource::BACKUP_PLANNER_GO, RiskLevel::SAFE),
  };

  const auto result = ranker_->process(inputs, context_);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(
    result->best_trajectory_info.input_trajectory.source, TrajectorySource::BACKUP_PLANNER_GO);
  EXPECT_EQ(result->best_trajectory_info.input_trajectory.risk_level, RiskLevel::SAFE);

  ASSERT_EQ(result->scored_trajectories.scored_candidate_trajectories.size(), 2u);
  const auto & scored = result->scored_trajectories.scored_candidate_trajectories;
  // Input order: [HIGH_CAUTION diffusion, SAFE backup_go]
  EXPECT_GT(scored[1].score, scored[0].score)
    << "SAFE backup_go should score higher than HIGH_CAUTION diffusion";
}

}  // namespace autoware::trajectory_ranker
