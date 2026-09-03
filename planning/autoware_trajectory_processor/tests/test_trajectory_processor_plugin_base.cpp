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

#include "autoware/trajectory_processor/trajectory_processor_data.hpp"
#include "autoware/trajectory_processor/trajectory_processor_parameters.hpp"
#include "autoware/trajectory_processor/trajectory_processor_plugin_base.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::trajectory_processor::TrajectoryProcessorData;
using autoware::trajectory_processor::TrajectoryProcessorParams;
using autoware::trajectory_processor::plugin::ProcessingResult;
using autoware::trajectory_processor::plugin::TrajectoryPoints;
using autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase;

class TestTrajectoryProcessorPlugin : public TrajectoryProcessorPluginBase
{
public:
  ProcessingResult process(
    TrajectoryPoints & trajectory_points, TrajectoryProcessorData & data) override
  {
    if (!enabled_) {
      return ProcessingResult::Unchanged;
    }
    data.semantic_speed_tracker.add_stop_candidate(trajectory_points.size());
    return ProcessingResult::Modified;
  }

  void update_params(const TrajectoryProcessorParams & params) override
  {
    enabled_ = params.use_stop_point_fixer;
  }

  [[nodiscard]] std::string logger_name() const { return get_logger().get_name(); }
  [[nodiscard]] bool initialized() const { return initialized_; }

protected:
  void on_initialize(const TrajectoryProcessorParams & params) override
  {
    initialized_ = true;
    update_params(params);
  }

private:
  bool initialized_{false};
};

class TrajectoryProcessorPluginBaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("trajectory_processor_plugin_base_test");
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

TEST_F(TrajectoryProcessorPluginBaseTest, InitializesCommonState)
{
  TrajectoryProcessorParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryProcessorPlugin plugin;

  plugin.initialize(
    "autoware::trajectory_processor::plugin::TestTrajectoryProcessorPlugin", "processor_0",
    node_.get(), time_keeper_, nullptr, params);

  EXPECT_TRUE(plugin.initialized());
  EXPECT_EQ(plugin.logger_name(), std::string(node_->get_logger().get_name()));
  EXPECT_EQ(plugin.get_name(), "processor_0");
  EXPECT_EQ(plugin.get_short_name(), "TestTrajectoryProcessorPlugin");
  EXPECT_TRUE(plugin.get_planning_factors().empty());
  plugin.publish_debug_data("candidate_0");
  plugin.publish_planning_factor();
}

TEST_F(TrajectoryProcessorPluginBaseTest, SupportsRepeatedClassesWithUniqueInstances)
{
  const std::string class_name =
    "autoware::trajectory_processor::plugin::TestTrajectoryProcessorPlugin";
  TrajectoryProcessorParams params;
  TestTrajectoryProcessorPlugin first;
  TestTrajectoryProcessorPlugin second;

  first.initialize(class_name, "processor_0", node_.get(), time_keeper_, nullptr, params);
  second.initialize(class_name, "processor_1", node_.get(), time_keeper_, nullptr, params);

  EXPECT_EQ(first.get_short_name(), second.get_short_name());
  EXPECT_NE(first.get_name(), second.get_name());
}

TEST_F(TrajectoryProcessorPluginBaseTest, ProcessesCommonRuntimeData)
{
  TrajectoryProcessorParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryProcessorPlugin plugin;
  plugin.initialize("TestPlugin", node_.get(), time_keeper_, nullptr, params);

  TrajectoryPoints trajectory_points(3);
  TrajectoryProcessorData data;
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

TEST(TrajectoryProcessorParamsTest, ContainsModifierAndOptimizerParameters)
{
  TrajectoryProcessorParams params;
  params.use_obstacle_stop = false;
  params.use_qp_smoother = false;

  EXPECT_FALSE(params.use_obstacle_stop);
  EXPECT_FALSE(params.use_qp_smoother);
  EXPECT_FALSE(params.plugin_names.empty());
}

TEST(TrajectoryProcessorParamsTest, PreservesDefaultCombinedPipelineOrder)
{
  const TrajectoryProcessorParams params;
  const std::vector<std::string> expected = {
    "autoware::trajectory_processor::plugin::StopPointFixer",
    "autoware::trajectory_processor::plugin::SurroundObstacleStop",
    "autoware::trajectory_processor::plugin::ObstacleStop",
    "autoware::trajectory_processor::plugin::TrafficLightStop",
    "autoware::trajectory_processor::plugin::VelocityModifier",
    "autoware::trajectory_processor::plugin::TrajectoryPointFixer",
    "autoware::trajectory_processor::plugin::TrajectoryKinematicFeasibilityEnforcer",
    "autoware::trajectory_processor::plugin::TrajectoryQPSmoother",
    "autoware::trajectory_processor::plugin::TrajectoryKinematicFeasibilityEnforcer",
    "autoware::trajectory_processor::plugin::TrajectoryVelocityOptimizer",
    "autoware::trajectory_processor::plugin::TrajectoryEBSmootherOptimizer",
    "autoware::trajectory_processor::plugin::TrajectorySplineSmoother",
    "autoware::trajectory_processor::plugin::TrajectoryMPTOptimizer",
    "autoware::trajectory_processor::plugin::TrajectoryExtender"};

  EXPECT_EQ(params.plugin_names, expected);
  EXPECT_EQ(
    std::count(
      params.plugin_names.begin(), params.plugin_names.end(),
      "autoware::trajectory_processor::plugin::TrajectoryKinematicFeasibilityEnforcer"),
    2);
}

TEST_F(TrajectoryProcessorPluginBaseTest, ResetsMutableDataBetweenCandidates)
{
  TrajectoryProcessorParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryProcessorPlugin plugin;
  plugin.initialize("TestPlugin", node_.get(), time_keeper_, nullptr, params);

  TrajectoryPoints trajectory_points(2);
  TrajectoryProcessorData first_candidate;
  TrajectoryProcessorData second_candidate;
  plugin.process(trajectory_points, first_candidate);

  EXPECT_TRUE(second_candidate.semantic_speed_tracker.take_stop_point_candidates().empty());
  EXPECT_EQ(first_candidate.semantic_speed_tracker.take_stop_point_candidates().size(), 1U);
}

TEST_F(TrajectoryProcessorPluginBaseTest, AppliesRuntimeParametersToEveryPluginInstance)
{
  TrajectoryProcessorParams params;
  params.use_stop_point_fixer = true;
  TestTrajectoryProcessorPlugin first;
  TestTrajectoryProcessorPlugin second;
  first.initialize("TestPlugin", "first", node_.get(), time_keeper_, nullptr, params);
  second.initialize("TestPlugin", "second", node_.get(), time_keeper_, nullptr, params);

  params.use_stop_point_fixer = false;
  first.update_params(params);
  second.update_params(params);
  TrajectoryPoints trajectory_points(2);
  TrajectoryProcessorData data;

  EXPECT_EQ(first.process(trajectory_points, data), ProcessingResult::Unchanged);
  EXPECT_EQ(second.process(trajectory_points, data), ProcessingResult::Unchanged);
}

TEST_F(TrajectoryProcessorPluginBaseTest, LoadsEveryPluginThroughCommonInterface)
{
  pluginlib::ClassLoader<TrajectoryProcessorPluginBase> loader(
    "autoware_trajectory_processor",
    "autoware::trajectory_processor::plugin::TrajectoryProcessorPluginBase");
  const std::vector<std::string> plugin_classes = {
    "autoware::trajectory_processor::plugin::TrajectoryPointFixer",
    "autoware::trajectory_processor::plugin::TrajectoryKinematicFeasibilityEnforcer",
    "autoware::trajectory_processor::plugin::TrajectoryQPSmoother",
    "autoware::trajectory_processor::plugin::TrajectoryEBSmootherOptimizer",
    "autoware::trajectory_processor::plugin::TrajectorySplineSmoother",
    "autoware::trajectory_processor::plugin::TrajectoryVelocityOptimizer",
    "autoware::trajectory_processor::plugin::TrajectoryExtender",
    "autoware::trajectory_processor::plugin::TrajectoryMPTOptimizer",
    "autoware::trajectory_processor::plugin::TrajectoryTemporalMPTOptimizer",
    "autoware::trajectory_processor::plugin::StopPointFixer",
    "autoware::trajectory_processor::plugin::ObstacleStop",
    "autoware::trajectory_processor::plugin::VelocityModifier",
    "autoware::trajectory_processor::plugin::SurroundObstacleStop",
    "autoware::trajectory_processor::plugin::TrafficLightStop"};

  std::vector<std::shared_ptr<TrajectoryProcessorPluginBase>> plugins;
  for (const auto & class_name : plugin_classes) {
    EXPECT_TRUE(loader.isClassAvailable(class_name));
    plugins.push_back(loader.createSharedInstance(class_name));
  }

  const auto repeated = loader.createSharedInstance(plugin_classes.front());
  EXPECT_NE(plugins.front().get(), repeated.get());
}
