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

// These tests update optimizer parameters through the ROS parameter services, publish a candidate
// trajectory to activate the node's lazy parameter-refresh path, and inspect the node and loaded
// plugin caches through a test-only accessor. Unsupported runtime updates are retained as complete
// DISABLED_ regression tests.

#include "parameter_update_test_accessor.hpp"
#include "test_utils.hpp"

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

using autoware::trajectory_optimizer::ParameterUpdateTestAccessor;
using autoware::trajectory_optimizer::TrajectoryOptimizer;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using trajectory_optimizer_test_utils::is_plugin_list_parameter;
using trajectory_optimizer_test_utils::is_read_only;
using trajectory_optimizer_test_utils::list_declared_parameters;
using trajectory_optimizer_test_utils::make_optimizer_node_options;
using trajectory_optimizer_test_utils::make_updated_parameter;
using trajectory_optimizer_test_utils::ParameterCase;
using trajectory_optimizer_test_utils::set_parameter_atomically;
using trajectory_optimizer_test_utils::set_parameters_atomically;

class ParameterServiceFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    caller_ = std::make_shared<rclcpp::Node>("trajectory_optimizer_parameter_update_test_client");
    node_ = std::make_shared<TrajectoryOptimizer>(make_optimizer_node_options());
    executor_.add_node(caller_);
    executor_.add_node(node_->get_node_base_interface());
    client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      caller_, node_->get_node_base_interface()->get_fully_qualified_name());
    trajectory_pub_ = caller_->create_publisher<CandidateTrajectories>(
      "/trajectory_optimizer/input/trajectories", 1);
    ASSERT_TRUE(wait_for_parameter_service());
  }

  void TearDown() override
  {
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(caller_);
    client_.reset();
    trajectory_pub_.reset();
    node_.reset();
    caller_.reset();
  }

  bool wait_for_parameter_service()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
    while (std::chrono::steady_clock::now() < deadline) {
      if (client_->service_is_ready()) {
        return true;
      }
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    return false;
  }

  template <class FutureT>
  auto spin_until_complete(FutureT & future)
  {
    return executor_.spin_until_future_complete(future, std::chrono::seconds{5});
  }

  std::vector<rcl_interfaces::msg::ParameterDescriptor> describe_parameters(
    const std::vector<std::string> & names)
  {
    auto future = client_->describe_parameters(names);
    EXPECT_EQ(spin_until_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names)
  {
    auto future = client_->get_parameters(names);
    EXPECT_EQ(spin_until_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    return future.get();
  }

  void trigger_parameter_update()
  {
    const auto old_stamp = ParameterUpdateTestAccessor::params(*node_).__stamp;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds{5};
    while (ParameterUpdateTestAccessor::params(*node_).__stamp == old_stamp &&
           std::chrono::steady_clock::now() < deadline) {
      trajectory_pub_->publish(CandidateTrajectories{});
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds{10});
    }
    ASSERT_NE(ParameterUpdateTestAccessor::params(*node_).__stamp, old_stamp);
  }

  template <class PluginT>
  std::shared_ptr<PluginT> find_plugin() const
  {
    for (const auto & plugin : ParameterUpdateTestAccessor::plugins(*node_)) {
      if (const auto typed = std::dynamic_pointer_cast<PluginT>(plugin)) {
        return typed;
      }
    }
    return nullptr;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr caller_;
  std::shared_ptr<TrajectoryOptimizer> node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr trajectory_pub_;
};

}  // namespace

TEST_F(ParameterServiceFixture, UpdatesWritableScalarParametersThroughParameterService)
{
  const auto names = list_declared_parameters(*client_, executor_);
  ASSERT_FALSE(names.empty());

  const auto descriptors = describe_parameters(names);
  const auto current_values = get_parameters(names);
  ASSERT_EQ(descriptors.size(), current_values.size());

  std::vector<ParameterCase> cases;
  for (size_t i = 0; i < current_values.size(); ++i) {
    const auto & name = current_values.at(i).get_name();
    if (is_read_only(descriptors.at(i)) || is_plugin_list_parameter(name)) {
      continue;
    }
    const auto updated = make_updated_parameter(current_values.at(i), descriptors.at(i));
    if (updated) {
      cases.push_back({name, *updated});
    }
  }

  ASSERT_FALSE(cases.empty());
  for (const auto & test_case : cases) {
    set_parameter_atomically(*client_, executor_, test_case.value);
    const auto stored = get_parameters({test_case.name});
    ASSERT_EQ(stored.size(), 1U);
    EXPECT_EQ(stored.front(), test_case.value) << test_case.name;
  }
}

TEST_F(ParameterServiceFixture, UpdatesNodeAndPluginParameterCachesAfterTrajectoryCallback)
{
  using autoware::trajectory_optimizer::plugin::TrajectoryExtender;
  using autoware::trajectory_optimizer::plugin::TrajectoryKinematicFeasibilityEnforcer;
  using autoware::trajectory_optimizer::plugin::TrajectoryMPTOptimizer;
  using autoware::trajectory_optimizer::plugin::TrajectoryPointFixer;
  using autoware::trajectory_optimizer::plugin::TrajectoryQPSmoother;
  using autoware::trajectory_optimizer::plugin::TrajectorySplineSmoother;
  using autoware::trajectory_optimizer::plugin::TrajectoryTemporalMPTOptimizer;
  using autoware::trajectory_optimizer::plugin::TrajectoryVelocityOptimizer;

  set_parameters_atomically(
    *client_, executor_,
    {rclcpp::Parameter{"use_trajectory_point_fixer", false},
     rclcpp::Parameter{"trajectory_point_fixer.remove_close_points", false},
     rclcpp::Parameter{"trajectory_point_fixer.min_dist_to_remove_m", 0.02},
     rclcpp::Parameter{"use_kinematic_feasibility_enforcer", false},
     rclcpp::Parameter{"trajectory_kinematic_feasibility.max_yaw_rate_rad_s", 0.6},
     rclcpp::Parameter{"trajectory_kinematic_feasibility.time_step_s", 0.2},
     rclcpp::Parameter{"use_qp_smoother", false},
     rclcpp::Parameter{"trajectory_qp_smoother.weight_smoothness", 12.0},
     rclcpp::Parameter{"trajectory_qp_smoother.osqp_max_iter", int64_t{120}},
     rclcpp::Parameter{"trajectory_qp_smoother.osqp_verbose", true},
     rclcpp::Parameter{"use_akima_spline_interpolation", false},
     rclcpp::Parameter{"trajectory_spline_smoother.interpolation_resolution_m", 0.4},
     rclcpp::Parameter{"trajectory_spline_smoother.preserve_input_trajectory_orientation", true},
     rclcpp::Parameter{"use_trajectory_extender", true},
     rclcpp::Parameter{"trajectory_extender.nearest_dist_threshold_m", 1.25},
     rclcpp::Parameter{"trajectory_extender.backward_trajectory_extension_m", 4.0},
     rclcpp::Parameter{"use_velocity_optimizer", false},
     rclcpp::Parameter{"max_vel", 10.0},
     rclcpp::Parameter{"limit.max_acc", 0.8},
     rclcpp::Parameter{"limit.min_acc", -2.0},
     rclcpp::Parameter{"limit.max_jerk", 1.2},
     rclcpp::Parameter{"limit.min_jerk", -1.2},
     rclcpp::Parameter{"trajectory_velocity_optimizer.nearest_dist_threshold_m", 1.25},
     rclcpp::Parameter{"trajectory_velocity_optimizer.nearest_yaw_threshold_deg", 50.0},
     rclcpp::Parameter{"trajectory_velocity_optimizer.target_pull_out_speed_mps", 0.8},
     rclcpp::Parameter{"trajectory_velocity_optimizer.target_pull_out_acc_mps2", 0.7},
     rclcpp::Parameter{"trajectory_velocity_optimizer.max_lateral_accel_mps2", 1.2},
     rclcpp::Parameter{"trajectory_velocity_optimizer.min_limited_speed_mps", 2.5},
     rclcpp::Parameter{"trajectory_velocity_optimizer.set_engage_speed", true},
     rclcpp::Parameter{"trajectory_velocity_optimizer.limit_speed", false},
     rclcpp::Parameter{"trajectory_velocity_optimizer.limit_lateral_acceleration", true},
     rclcpp::Parameter{"trajectory_velocity_optimizer.smooth_velocities", false},
     rclcpp::Parameter{"trajectory_velocity_optimizer.continuous_jerk_smoother.jerk_weight", 25.0},
     rclcpp::Parameter{
       "trajectory_velocity_optimizer.continuous_jerk_smoother.over_v_weight", 2500.0},
     rclcpp::Parameter{
       "trajectory_velocity_optimizer.continuous_jerk_smoother.over_a_weight", 25.0},
     rclcpp::Parameter{"trajectory_velocity_optimizer.continuous_jerk_smoother.over_j_weight", 8.0},
     rclcpp::Parameter{
       "trajectory_velocity_optimizer.continuous_jerk_smoother.velocity_tracking_weight", 1.5},
     rclcpp::Parameter{
       "trajectory_velocity_optimizer.continuous_jerk_smoother.accel_tracking_weight", 250.0},
     rclcpp::Parameter{"use_mpt_optimizer", true},
     rclcpp::Parameter{"trajectory_mpt_optimizer.corridor_width_m", 3.0},
     rclcpp::Parameter{"trajectory_mpt_optimizer.enable_adaptive_width", false},
     rclcpp::Parameter{"trajectory_mpt_optimizer.output_delta_arc_length_m", 0.8},
     rclcpp::Parameter{"trajectory_mpt_optimizer.ego_nearest_yaw_threshold_deg", 30.0},
     rclcpp::Parameter{"trajectory_mpt_optimizer.acceleration_moving_average_window", int64_t{7}},
     rclcpp::Parameter{"use_temporal_mpt_optimizer", true},
     rclcpp::Parameter{"trajectory_temporal_mpt_optimizer.min_points_for_optimization", int64_t{3}},
     rclcpp::Parameter{"trajectory_temporal_mpt_optimizer.enable_debug_info", true},
     rclcpp::Parameter{"trajectory_temporal_mpt_optimizer.publish_debug_topics", true},
     rclcpp::Parameter{"trajectory_temporal_mpt_optimizer.write_replay_fixture", true},
     rclcpp::Parameter{
       "trajectory_temporal_mpt_optimizer.replay_fixture_directory", "/tmp/temporal_mpt_test"},
     rclcpp::Parameter{"trajectory_temporal_mpt_optimizer.log_replay_fixture_to_console", true}});

  trigger_parameter_update();

  const auto & node_params = ParameterUpdateTestAccessor::params(*node_);
  EXPECT_FALSE(node_params.use_trajectory_point_fixer);
  EXPECT_DOUBLE_EQ(node_params.trajectory_point_fixer.min_dist_to_remove_m, 0.02);
  EXPECT_DOUBLE_EQ(node_params.limit.max_acc, 0.8);
  EXPECT_DOUBLE_EQ(node_params.max_vel, 10.0);

  const auto point_fixer = find_plugin<TrajectoryPointFixer>();
  ASSERT_NE(point_fixer, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*point_fixer));
  EXPECT_FALSE(ParameterUpdateTestAccessor::params(*point_fixer).remove_close_points);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*point_fixer).min_dist_to_remove_m, 0.02);

  const auto feasibility = find_plugin<TrajectoryKinematicFeasibilityEnforcer>();
  ASSERT_NE(feasibility, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*feasibility));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*feasibility).max_yaw_rate_rad_s, 0.6);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*feasibility).time_step_s, 0.2);

  const auto qp = find_plugin<TrajectoryQPSmoother>();
  ASSERT_NE(qp, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*qp));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*qp).weight_smoothness, 12.0);
  EXPECT_EQ(ParameterUpdateTestAccessor::params(*qp).osqp_max_iter, 120);
  EXPECT_TRUE(ParameterUpdateTestAccessor::params(*qp).osqp_verbose);

  const auto spline = find_plugin<TrajectorySplineSmoother>();
  ASSERT_NE(spline, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*spline));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*spline).interpolation_resolution_m, 0.4);
  EXPECT_TRUE(ParameterUpdateTestAccessor::params(*spline).preserve_input_trajectory_orientation);

  const auto extender = find_plugin<TrajectoryExtender>();
  ASSERT_NE(extender, nullptr);
  EXPECT_TRUE(ParameterUpdateTestAccessor::enabled(*extender));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*extender).nearest_dist_threshold_m, 1.25);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*extender).backward_trajectory_extension_m, 4.0);

  const auto velocity = find_plugin<TrajectoryVelocityOptimizer>();
  ASSERT_NE(velocity, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*velocity));
  const auto & velocity_params = ParameterUpdateTestAccessor::params(*velocity);
  EXPECT_DOUBLE_EQ(velocity_params.default_max_velocity_mps, 10.0);
  EXPECT_DOUBLE_EQ(velocity_params.nearest_dist_threshold_m, 1.25);
  EXPECT_DOUBLE_EQ(velocity_params.nearest_yaw_threshold_deg, 50.0);
  EXPECT_DOUBLE_EQ(velocity_params.target_pull_out_speed_mps, 0.8);
  EXPECT_DOUBLE_EQ(velocity_params.target_pull_out_acc_mps2, 0.7);
  EXPECT_DOUBLE_EQ(velocity_params.max_lateral_accel_mps2, 1.2);
  EXPECT_DOUBLE_EQ(velocity_params.min_limited_speed_mps, 2.5);
  EXPECT_TRUE(velocity_params.set_engage_speed);
  EXPECT_FALSE(velocity_params.limit_speed);
  EXPECT_TRUE(velocity_params.limit_lateral_acceleration);
  EXPECT_FALSE(velocity_params.smooth_velocities);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.jerk_weight, 25.0);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.over_v_weight, 2500.0);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.over_a_weight, 25.0);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.over_j_weight, 8.0);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.velocity_tracking_weight, 1.5);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.accel_tracking_weight, 250.0);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.max_accel, 0.8);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.min_decel, -2.0);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.max_jerk, 1.2);
  EXPECT_DOUBLE_EQ(velocity_params.continuous_jerk_smoother_params.min_jerk, -1.2);

  const auto mpt = find_plugin<TrajectoryMPTOptimizer>();
  ASSERT_NE(mpt, nullptr);
  EXPECT_TRUE(ParameterUpdateTestAccessor::enabled(*mpt));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*mpt).corridor_width_m, 3.0);
  EXPECT_FALSE(ParameterUpdateTestAccessor::params(*mpt).enable_adaptive_width);
  EXPECT_EQ(ParameterUpdateTestAccessor::params(*mpt).acceleration_moving_average_window, 7);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::trajectory_params(*mpt).output_delta_arc_length, 0.8);
  EXPECT_NEAR(
    ParameterUpdateTestAccessor::nearest_params(*mpt).yaw_threshold, 0.5235987755982988, 1.0e-9);

  const auto temporal_mpt = find_plugin<TrajectoryTemporalMPTOptimizer>();
  ASSERT_NE(temporal_mpt, nullptr);
  EXPECT_TRUE(ParameterUpdateTestAccessor::enabled(*temporal_mpt));
  const auto & temporal_params = ParameterUpdateTestAccessor::params(*temporal_mpt);
  EXPECT_EQ(temporal_params.min_points_for_optimization, 3U);
  EXPECT_TRUE(temporal_params.enable_debug_info);
  EXPECT_TRUE(temporal_params.publish_debug_topics);
  EXPECT_TRUE(temporal_params.write_replay_fixture);
  EXPECT_EQ(temporal_params.replay_fixture_directory, "/tmp/temporal_mpt_test");
  EXPECT_TRUE(temporal_params.log_replay_fixture_to_console);
}

TEST_F(ParameterServiceFixture, RejectsReadOnlyParametersThroughParameterService)
{
  const auto names = list_declared_parameters(*client_, executor_);
  ASSERT_FALSE(names.empty());

  const auto descriptors = describe_parameters(names);
  const auto current_values = get_parameters(names);
  ASSERT_EQ(descriptors.size(), current_values.size());

  for (size_t i = 0; i < current_values.size(); ++i) {
    if (!is_read_only(descriptors.at(i))) {
      continue;
    }
    const auto updated = make_updated_parameter(current_values.at(i), descriptors.at(i));
    if (!updated) {
      continue;
    }

    auto future = client_->set_parameters_atomically({*updated});
    ASSERT_EQ(spin_until_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_FALSE(future.get().successful) << current_values.at(i).get_name();
    return;
  }

  GTEST_SKIP() << "No writable alternative for a declared read-only optimizer parameter.";
}

TEST_F(ParameterServiceFixture, DISABLED_UpdatesPluginListAtRuntime)
{
  const std::vector<std::string> expected_names = {
    "autoware::trajectory_optimizer::plugin::TrajectoryExtender",
    "autoware::trajectory_optimizer::plugin::TrajectoryPointFixer"};
  set_parameter_atomically(*client_, executor_, rclcpp::Parameter{"plugin_names", expected_names});
  trigger_parameter_update();

  EXPECT_EQ(ParameterUpdateTestAccessor::params(*node_).plugin_names, expected_names);
  const auto & plugins = ParameterUpdateTestAccessor::plugins(*node_);
  ASSERT_EQ(plugins.size(), expected_names.size());
  for (size_t i = 0; i < plugins.size(); ++i) {
    EXPECT_EQ(plugins.at(i)->get_name(), expected_names.at(i));
  }
}

TEST_F(ParameterServiceFixture, DISABLED_UpdatesElasticBandInternalParametersAtRuntime)
{
  using autoware::trajectory_optimizer::plugin::TrajectoryEBSmootherOptimizer;

  set_parameters_atomically(
    *client_, executor_,
    {rclcpp::Parameter{"elastic_band_params.common.output_delta_arc_length", 0.4},
     rclcpp::Parameter{"elastic_band_params.common.output_backward_traj_length", 4.0},
     rclcpp::Parameter{"elastic_band_params.ego_nearest_dist_threshold", 2.5},
     rclcpp::Parameter{"elastic_band_params.ego_nearest_yaw_threshold", 0.8}});
  trigger_parameter_update();

  const auto eb = find_plugin<TrajectoryEBSmootherOptimizer>();
  ASSERT_NE(eb, nullptr);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*node_).elastic_band_params.common.output_delta_arc_length,
    0.4);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::common_params(*eb).output_delta_arc_length, 0.4);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::common_params(*eb).output_backward_traj_length, 4.0);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::nearest_params(*eb).dist_threshold, 2.5);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::nearest_params(*eb).yaw_threshold, 0.8);
}
