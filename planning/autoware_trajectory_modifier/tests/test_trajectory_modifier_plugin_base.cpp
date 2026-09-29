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

#include "autoware/trajectory_modifier/trajectory_modifier_data.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_parameters.hpp"
#include "autoware/trajectory_modifier/trajectory_modifier_plugin_base.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::trajectory_modifier::TrajectoryModifierData;
using autoware::trajectory_modifier::TrajectoryModifierParams;
using autoware::trajectory_modifier::plugin::ProcessingResult;
using autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase;
using autoware::trajectory_modifier::plugin::TrajectoryPoints;

class TestTrajectoryModifierPlugin : public TrajectoryModifierPluginBase
{
public:
  ProcessingResult process(
    TrajectoryPoints & trajectory_points, TrajectoryModifierData & data) override
  {
    if (!enabled_) {
      return ProcessingResult::Unchanged;
    }
    data.semantic_speed_tracker.add_stop_candidate(trajectory_points.size());
    return ProcessingResult::Modified;
  }

  void update_params(const TrajectoryModifierParams & params) override
  {
    enabled_ = params.use_stop_point_fixer;
  }

  [[nodiscard]] std::string logger_name() const { return get_logger().get_name(); }
  [[nodiscard]] bool initialized() const { return initialized_; }

protected:
  void on_initialize(const TrajectoryModifierParams & params) override
  {
    initialized_ = true;
    update_params(params);
  }

private:
  bool initialized_{false};
};

class TrajectoryModifierPluginBaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("trajectory_modifier_plugin_base_test");
    time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>();
  }

  void TearDown() override
  {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
};

TEST_F(TrajectoryModifierPluginBaseTest, InitializesCommonState)
{
  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryModifierPlugin plugin;

  plugin.initialize(
    "autoware::trajectory_modifier::plugin::TestTrajectoryModifierPlugin", "processor_0",
    node_.get(), time_keeper_, nullptr, params);

  EXPECT_TRUE(plugin.initialized());
  EXPECT_EQ(plugin.logger_name(), std::string(node_->get_logger().get_name()));
  EXPECT_EQ(plugin.get_name(), "processor_0");
  EXPECT_EQ(plugin.get_short_name(), "TestTrajectoryModifierPlugin");
  EXPECT_TRUE(plugin.get_planning_factors().empty());
  plugin.publish_debug_data("candidate_0");
  plugin.publish_planning_factor();
}

TEST_F(TrajectoryModifierPluginBaseTest, SupportsRepeatedClassesWithUniqueInstances)
{
  const std::string class_name =
    "autoware::trajectory_modifier::plugin::TestTrajectoryModifierPlugin";
  TrajectoryModifierParams params;
  TestTrajectoryModifierPlugin first;
  TestTrajectoryModifierPlugin second;

  first.initialize(class_name, "processor_0", node_.get(), time_keeper_, nullptr, params);
  second.initialize(class_name, "processor_1", node_.get(), time_keeper_, nullptr, params);

  EXPECT_EQ(first.get_short_name(), second.get_short_name());
  EXPECT_NE(first.get_name(), second.get_name());
}

TEST_F(TrajectoryModifierPluginBaseTest, ProcessesCommonRuntimeData)
{
  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryModifierPlugin plugin;
  plugin.initialize("TestPlugin", node_.get(), time_keeper_, nullptr, params);

  TrajectoryPoints trajectory_points(3);
  TrajectoryModifierData data;
  EXPECT_EQ(plugin.process(trajectory_points, data), ProcessingResult::Modified);

  const auto candidates = data.semantic_speed_tracker.take_stop_point_candidates();
  ASSERT_EQ(candidates.size(), 1U);
  EXPECT_EQ(candidates.front(), trajectory_points.size());
  EXPECT_EQ(data.current_odometry, nullptr);
  EXPECT_EQ(data.predicted_objects, nullptr);

  params.use_stop_point_fixer = false;
  plugin.update_params(params);
  EXPECT_EQ(plugin.process(trajectory_points, data), ProcessingResult::Unchanged);
}

TEST(TrajectoryModifierParamsTest, ContainsModifierAndOptimizerParameters)
{
  TrajectoryModifierParams params;
  params.use_obstacle_stop = false;
  params.use_qp_smoother = false;

  EXPECT_FALSE(params.use_obstacle_stop);
  EXPECT_FALSE(params.use_qp_smoother);
  EXPECT_FALSE(params.plugin_names.empty());
}

TEST(TrajectoryModifierParamsTest, PreservesDefaultCombinedPipelineOrder)
{
  const TrajectoryModifierParams params;
  const std::vector<std::string> expected = {
    "autoware::trajectory_modifier::plugin::StopPointFixer",
    "autoware::trajectory_modifier::plugin::SurroundObstacleStop",
    "autoware::trajectory_modifier::plugin::ObstacleStop",
    "autoware::trajectory_modifier::plugin::TrafficLightStop",
    "autoware::trajectory_modifier::plugin::VelocityModifier",
    "autoware::trajectory_modifier::plugin::TrajectoryPointFixer",
    "autoware::trajectory_modifier::plugin::TrajectoryKinematicFeasibilityEnforcer",
    "autoware::trajectory_modifier::plugin::TrajectoryQPSmoother",
    "autoware::trajectory_modifier::plugin::TrajectoryKinematicFeasibilityEnforcer",
    "autoware::trajectory_modifier::plugin::TrajectoryVelocityOptimizer",
    "autoware::trajectory_modifier::plugin::TrajectoryEBSmootherOptimizer",
    "autoware::trajectory_modifier::plugin::TrajectorySplineSmoother",
    "autoware::trajectory_modifier::plugin::TrajectoryMPTOptimizer",
    "autoware::trajectory_modifier::plugin::TrajectoryExtender"};

  EXPECT_EQ(params.plugin_names, expected);
  EXPECT_EQ(
    std::count(
      params.plugin_names.begin(), params.plugin_names.end(),
      "autoware::trajectory_modifier::plugin::TrajectoryKinematicFeasibilityEnforcer"),
    2);
}

TEST_F(TrajectoryModifierPluginBaseTest, ResetsMutableDataBetweenCandidates)
{
  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryModifierPlugin plugin;
  plugin.initialize("TestPlugin", node_.get(), time_keeper_, nullptr, params);

  TrajectoryPoints trajectory_points(2);
  TrajectoryModifierData first_candidate;
  TrajectoryModifierData second_candidate;
  plugin.process(trajectory_points, first_candidate);

  EXPECT_TRUE(second_candidate.semantic_speed_tracker.take_stop_point_candidates().empty());
  EXPECT_EQ(first_candidate.semantic_speed_tracker.take_stop_point_candidates().size(), 1U);
}

TEST_F(TrajectoryModifierPluginBaseTest, AppliesRuntimeParametersToEveryPluginInstance)
{
  TrajectoryModifierParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryModifierPlugin first;
  TestTrajectoryModifierPlugin second;
  first.initialize("TestPlugin", "first", node_.get(), time_keeper_, nullptr, params);
  second.initialize("TestPlugin", "second", node_.get(), time_keeper_, nullptr, params);

  params.use_stop_point_fixer = false;
  first.update_params(params);
  second.update_params(params);
  TrajectoryPoints trajectory_points(2);
  TrajectoryModifierData data;

  EXPECT_EQ(first.process(trajectory_points, data), ProcessingResult::Unchanged);
  EXPECT_EQ(second.process(trajectory_points, data), ProcessingResult::Unchanged);
}

TEST_F(TrajectoryModifierPluginBaseTest, LoadsEveryPluginThroughCommonInterface)
{
  pluginlib::ClassLoader<TrajectoryModifierPluginBase> loader(
    "autoware_trajectory_modifier",
    "autoware::trajectory_modifier::plugin::TrajectoryModifierPluginBase");
  const std::vector<std::string> plugin_classes = {
    "autoware::trajectory_modifier::plugin::TrajectoryPointFixer",
    "autoware::trajectory_modifier::plugin::TrajectoryKinematicFeasibilityEnforcer",
    "autoware::trajectory_modifier::plugin::TrajectoryQPSmoother",
    "autoware::trajectory_modifier::plugin::TrajectoryEBSmootherOptimizer",
    "autoware::trajectory_modifier::plugin::TrajectorySplineSmoother",
    "autoware::trajectory_modifier::plugin::TrajectoryVelocityOptimizer",
    "autoware::trajectory_modifier::plugin::TrajectoryExtender",
    "autoware::trajectory_modifier::plugin::TrajectoryMPTOptimizer",
    "autoware::trajectory_modifier::plugin::TrajectoryTemporalMPTOptimizer",
    "autoware::trajectory_modifier::plugin::StopPointFixer",
    "autoware::trajectory_modifier::plugin::ObstacleStop",
    "autoware::trajectory_modifier::plugin::VelocityModifier",
    "autoware::trajectory_modifier::plugin::SurroundObstacleStop",
    "autoware::trajectory_modifier::plugin::TrafficLightStop"};

  std::vector<std::shared_ptr<TrajectoryModifierPluginBase>> plugins;
  for (const auto & class_name : plugin_classes) {
    EXPECT_TRUE(loader.isClassAvailable(class_name));
    plugins.push_back(loader.createSharedInstance(class_name));
  }

  const auto repeated = loader.createSharedInstance(plugin_classes.front());
  EXPECT_NE(plugins.front().get(), repeated.get());
}
