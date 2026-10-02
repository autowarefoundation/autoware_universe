// Copyright 2026 Xinchen Lin
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

#include "autoware/diffusion_planner/diffusion_planner_core.hpp"
#include "autoware/diffusion_planner/dimensions.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

namespace autoware::diffusion_planner
{

// Inject only the scorer so these core postprocessing tests do not load a neural model.
class DiffusionPlannerCoreTestPeer
{
public:
  static void load_camp_model(DiffusionPlannerCore & core)
  {
    core.camp_model_ = trajectory_ranker::load_camp_fixed_weight_model(CAMP_TEST_MODEL_PATH);
  }

  static const std::optional<CampPreviousPlan> & previous_plan(const DiffusionPlannerCore & core)
  {
    return core.camp_previous_plan_;
  }
};

namespace
{

constexpr int kCandidateCount = 8;

DiffusionPlannerParams make_params()
{
  DiffusionPlannerParams params{};
  params.camp_enabled = true;
  params.batch_size = kCandidateCount;
  params.temperature_list.assign(kCandidateCount, 1.0);
  params.temperature_list.front() = 0.0;
  params.velocity_smoothing_window = 1;
  params.stopping_threshold = 0.01;
  params.ignore_neighbors = true;
  params.traffic_light_group_msg_timeout_seconds = 0.2;
  params.line_string_max_step_m = 1.0;
  params.ego_snap_to_prev_trajectory = EgoSnapParams{true, 2.0, 10.0, 1};
  return params;
}

VehicleInfo make_vehicle_info()
{
  VehicleInfo info{};
  info.wheel_base_m = 2.7;
  info.front_overhang_m = 1.0;
  info.rear_overhang_m = 1.1;
  info.wheel_tread_m = 1.6;
  info.left_overhang_m = 0.2;
  info.right_overhang_m = 0.2;
  return info;
}

InferenceOutput make_inference_output()
{
  InferenceOutput output;
  output.is_denormalized = true;
  output.outputs.first.assign(kCandidateCount * MAX_NUM_AGENTS * OUTPUT_T * POSE_DIM, 0.0F);
  output.outputs.second.assign(kCandidateCount * TURN_INDICATOR_OUTPUT_DIM, 0.0F);
  for (int candidate = 0; candidate < kCandidateCount; ++candidate) {
    output.outputs.second.at(
      candidate * TURN_INDICATOR_OUTPUT_DIM + TURN_INDICATOR_OUTPUT_DISABLE) = 1.0F;
    for (int time = 0; time < OUTPUT_T; ++time) {
      const std::size_t base =
        (static_cast<std::size_t>(candidate) * MAX_NUM_AGENTS * OUTPUT_T + time) * POSE_DIM;
      output.outputs.first.at(base) = 0.125F * (candidate + 1) * (time + 1);
      output.outputs.first.at(base + 2) = 1.0F;
    }
  }
  return output;
}

CampTensorContext make_tensor_context()
{
  CampTensorContext context;
  context.lanes.assign(NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM, 0.0F);
  context.route_lanes.assign(NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM, 0.0F);
  context.route_speed_limits.assign(NUM_SEGMENTS_IN_ROUTE, 0.0F);
  for (auto * tensor : {&context.lanes, &context.route_lanes}) {
    for (int point = 0; point < POINTS_PER_SEGMENT; ++point) {
      const auto base = static_cast<std::size_t>(point) * SEGMENT_POINT_DIM;
      tensor->at(base + X) = -20.0F + 6.0F * point;
      tensor->at(base + dX) = 6.0F;
      tensor->at(base + LB_Y) = 5.0F;
      tensor->at(base + RB_Y) = -5.0F;
      tensor->at(base + TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT) = 1.0F;
    }
  }
  context.route_speed_limits.front() = 20.0F;
  return context;
}

class CampOutputHistoryTest : public ::testing::Test
{
protected:
  DiffusionPlannerParams params_{make_params()};
  DiffusionPlannerCore core_{params_, make_vehicle_info()};
  LaneletRoute::SharedPtr route_{std::make_shared<LaneletRoute>()};
  InferenceOutput inference_{make_inference_output()};
  CampTensorContext tensors_{make_tensor_context()};
  const rclcpp::Time first_time_{10, 0, RCL_ROS_TIME};
  const rclcpp::Time next_time_{10, 100000000, RCL_ROS_TIME};

  void SetUp() override
  {
    core_.set_map(std::make_shared<lanelet::LaneletMap>());
    DiffusionPlannerCoreTestPeer::load_camp_model(core_);
  }

  std::optional<FrameContext> frame(
    const rclcpp::Time & stamp, const double x = 0.0, const double y = 0.0,
    const LaneletRoute::ConstSharedPtr & route = nullptr)
  {
    auto odometry = std::make_shared<Odometry>();
    odometry->header.stamp = stamp;
    odometry->pose.pose.position.x = x;
    odometry->pose.pose.position.y = y;
    odometry->pose.pose.orientation.w = 1.0;
    return core_.create_frame_context(
      odometry, std::make_shared<AccelWithCovarianceStamped>(), nullptr, {},
      std::make_shared<TurnIndicatorsReport>(), route ? route : route_, stamp);
  }

  PlannerOutput output(const FrameContext & context)
  {
    return core_.create_planner_output(inference_, context, context.frame_time, UUID{}, tensors_);
  }
};

TEST(CampOutputHistoryDefaults, CampIsDisabledWithoutExplicitInitialization)
{
  DiffusionPlannerParams params;
  EXPECT_FALSE(params.camp_enabled);
}

TEST_F(CampOutputHistoryTest, SelectedNonzeroRowAnchorsTheNextFrame)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  const auto selected = output(*first);
  ASSERT_EQ(selected.selected_candidate_index, 7U);
  ASSERT_EQ(selected.candidate_trajectories.candidate_trajectories.size(), kCandidateCount);
  EXPECT_EQ(
    selected.trajectory.points,
    selected.candidate_trajectories.candidate_trajectories.at(7).points);

  const auto second = frame(next_time_, 0.75, 0.2);
  ASSERT_TRUE(second);
  ASSERT_TRUE(second->snapped_pose);
  ASSERT_TRUE(second->snapped_interpolation_time_s);
  EXPECT_NEAR(second->snapped_pose->operator()(0, 3), 0.75, 1.0e-9);
  EXPECT_NEAR(second->snapped_pose->operator()(1, 3), 0.0, 1.0e-9);
  EXPECT_NEAR(*second->snapped_interpolation_time_s, 0.075, 1.0e-9);
}

TEST_F(CampOutputHistoryTest, DisabledCampKeepsRowZeroEvenWithALoadedScorer)
{
  params_.camp_enabled = false;
  core_.update_params(params_);
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  const auto selected = core_.create_planner_output(inference_, *first, first_time_, UUID{});
  EXPECT_EQ(selected.selected_candidate_index, 0U);
  EXPECT_TRUE(selected.camp_candidate_costs.empty());
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));

  const auto second = frame(next_time_, 0.75, 0.2);
  ASSERT_TRUE(second);
  ASSERT_TRUE(second->snapped_pose);
  ASSERT_TRUE(second->snapped_interpolation_time_s);
  EXPECT_NEAR(second->snapped_pose->operator()(0, 3), 0.125, 1.0e-9);
  EXPECT_NEAR(*second->snapped_interpolation_time_s, 0.1, 1.0e-9);
}

TEST_F(CampOutputHistoryTest, FailedPostprocessingDoesNotReplacePreviousOutput)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  ASSERT_EQ(output(*first).selected_candidate_index, 7U);
  auto failed_frame = *first;
  failed_frame.ego_to_map_transform(0, 3) = 100.0;
  EXPECT_THROW(
    core_.create_planner_output(inference_, failed_frame, next_time_, UUID{}),
    std::invalid_argument);
  const auto & previous = DiffusionPlannerCoreTestPeer::previous_plan(core_);
  ASSERT_TRUE(previous);
  EXPECT_DOUBLE_EQ(previous->origin_seconds, first_time_.seconds());
  EXPECT_NEAR(previous->states.front().x_m, 1.0, 1.0e-9);

  const auto second = frame(next_time_, 0.75, 0.2);
  ASSERT_TRUE(second);
  ASSERT_TRUE(second->snapped_pose);
  EXPECT_NEAR(second->snapped_pose->operator()(0, 3), 0.75, 1.0e-9);
}

TEST_F(CampOutputHistoryTest, ExplicitEpisodeResetClearsBothPlanConsumers)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  output(*first);
  core_.reset();
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));
  const auto second = frame(next_time_, 0.75, 0.2);
  ASSERT_TRUE(second);
  EXPECT_FALSE(second->snapped_pose);
  EXPECT_DOUBLE_EQ(second->ego_kinematic_state.pose.pose.position.y, 0.2);
  EXPECT_TRUE(core_.is_map_loaded());
  EXPECT_TRUE(core_.get_route());
}

TEST_F(CampOutputHistoryTest, RouteContentChangesResetButEquivalentMessagesDoNot)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  output(*first);
  auto equivalent_route = std::make_shared<LaneletRoute>(*route_);
  const auto second = frame(next_time_, 0.75, 0.2, equivalent_route);
  ASSERT_TRUE(second);
  EXPECT_TRUE(second->snapped_pose);

  auto changed_route = std::make_shared<LaneletRoute>(*route_);
  changed_route->goal_pose.position.x = 50.0;
  const auto next = frame(rclcpp::Time(10, 200000000, RCL_ROS_TIME), 0.75, 0.2, changed_route);
  ASSERT_TRUE(next);
  EXPECT_FALSE(next->snapped_pose);
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));
}

TEST_F(CampOutputHistoryTest, MapReplacementClearsPreviousOutput)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  output(*first);
  core_.set_map(std::make_shared<lanelet::LaneletMap>());
  const auto second = frame(next_time_, 0.75, 0.2);
  ASSERT_TRUE(second);
  EXPECT_FALSE(second->snapped_pose);
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));
}

TEST_F(CampOutputHistoryTest, BackwardsOdometryClockClearsPreviousOutput)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  output(*first);
  const auto rewound = frame(rclcpp::Time(9, 900000000, RCL_ROS_TIME), 0.75, 0.2);
  ASSERT_TRUE(rewound);
  EXPECT_FALSE(rewound->snapped_pose);
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));
}

TEST_F(CampOutputHistoryTest, RejectsIncompleteTurnIndicatorOutput)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  inference_.outputs.second.pop_back();
  EXPECT_THROW(output(*first), std::invalid_argument);
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));
}

TEST_F(CampOutputHistoryTest, RejectsIncompletePredictionOutput)
{
  const auto first = frame(first_time_);
  ASSERT_TRUE(first);
  inference_.outputs.first.pop_back();
  EXPECT_THROW(output(*first), std::invalid_argument);
  EXPECT_FALSE(DiffusionPlannerCoreTestPeer::previous_plan(core_));
}

}  // namespace
}  // namespace autoware::diffusion_planner
