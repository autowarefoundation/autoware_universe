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

// These tests update modifier parameters through the ROS parameter services, publish the mandatory
// inputs and a candidate trajectory to activate the node's lazy parameter-refresh path, and inspect
// the node and loaded plugin caches through a test-only accessor. Unsupported runtime updates are
// retained as complete DISABLED_ regression tests.

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

using autoware::trajectory_modifier::ParameterUpdateTestAccessor;
using autoware::trajectory_modifier::TrajectoryModifier;
using autoware_internal_planning_msgs::msg::CandidateTrajectories;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using nav_msgs::msg::Odometry;
using trajectory_optimizer_test_utils::is_plugin_list_parameter;
using trajectory_optimizer_test_utils::is_read_only;
using trajectory_optimizer_test_utils::list_declared_parameters;
using trajectory_optimizer_test_utils::make_modifier_node_options;
using trajectory_optimizer_test_utils::make_updated_parameter;
using trajectory_optimizer_test_utils::ParameterCase;
using trajectory_optimizer_test_utils::set_parameter_atomically;
using trajectory_optimizer_test_utils::set_parameters_atomically;

class ModifierParameterServiceFixture : public ::testing::Test
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
    caller_ = std::make_shared<rclcpp::Node>("trajectory_modifier_parameter_update_test_client");
    node_ = std::make_shared<TrajectoryModifier>(make_modifier_node_options());
    executor_.add_node(caller_);
    executor_.add_node(node_->get_node_base_interface());
    client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      caller_, node_->get_node_base_interface()->get_fully_qualified_name());
    trajectory_pub_ = caller_->create_publisher<CandidateTrajectories>(
      "/trajectory_modifier/input/candidate_trajectories", 1);
    odometry_pub_ = caller_->create_publisher<Odometry>("/trajectory_modifier/input/odometry", 1);
    acceleration_pub_ = caller_->create_publisher<AccelWithCovarianceStamped>(
      "/trajectory_modifier/input/acceleration", 1);
    ASSERT_TRUE(wait_for_parameter_service());
  }

  void TearDown() override
  {
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(caller_);
    client_.reset();
    acceleration_pub_.reset();
    odometry_pub_.reset();
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
      odometry_pub_->publish(Odometry{});
      acceleration_pub_->publish(AccelWithCovarianceStamped{});
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
  std::shared_ptr<TrajectoryModifier> node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
  rclcpp::Publisher<CandidateTrajectories>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr acceleration_pub_;
};

}  // namespace

TEST_F(ModifierParameterServiceFixture, UpdatesWritableScalarParametersThroughParameterService)
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

TEST_F(ModifierParameterServiceFixture, UpdatesNodeAndPluginParameterCachesAfterTrajectoryCallback)
{
  using autoware::trajectory_modifier::plugin::ObstacleStop;
  using autoware::trajectory_modifier::plugin::StopPointFixer;
  using autoware::trajectory_modifier::plugin::SurroundObstacleStop;
  using autoware::trajectory_modifier::plugin::TrafficLightStop;
  using autoware::trajectory_modifier::plugin::VelocityModifier;

  set_parameters_atomically(
    *client_, executor_,
    {rclcpp::Parameter{"use_stop_point_fixer", false},
     rclcpp::Parameter{"stop_point_fixer.force_stop_long_stopped_trajectories", false},
     rclcpp::Parameter{"stop_point_fixer.velocity_threshold", 0.2},
     rclcpp::Parameter{"stop_point_fixer.min_distance_threshold", 1.5},
     rclcpp::Parameter{"use_velocity_modifier", false},
     rclcpp::Parameter{"use_obstacle_stop", false},
     rclcpp::Parameter{"use_surround_obstacle_stop", true},
     rclcpp::Parameter{"use_traffic_light_stop", false},
     rclcpp::Parameter{"trajectory_time_step", 0.2},
     rclcpp::Parameter{"stopping_constraints.nominal_deceleration", 1.2},
     rclcpp::Parameter{"stopping_constraints.maximum_deceleration", 3.5},
     rclcpp::Parameter{"stopping_constraints.jerk_limit", 2.5},
     rclcpp::Parameter{"stopping_constraints.arrived_distance_threshold", 0.4},
     rclcpp::Parameter{"obstacle_stop.use_objects", false},
     rclcpp::Parameter{"obstacle_stop.stop_margin", 5.0},
     rclcpp::Parameter{"obstacle_stop.obstacle_tracking.on_time_buffer", 0.4},
     rclcpp::Parameter{
       "obstacle_stop.objects.object_types", std::vector<std::string>{"car", "pedestrian"}},
     rclcpp::Parameter{"obstacle_stop.objects.max_velocity_th", 4.0},
     rclcpp::Parameter{"obstacle_stop.pointcloud.voxel_grid_filter.min_size", int64_t{4}},
     rclcpp::Parameter{"obstacle_stop.rss_params.ego_decel", 3.0},
     rclcpp::Parameter{"surround_obstacle_stop.use_objects", false},
     rclcpp::Parameter{
       "surround_obstacle_stop.object_types", std::vector<std::string>{"car", "bicycle"}},
     rclcpp::Parameter{"surround_obstacle_stop.base_distance_th", 0.6},
     rclcpp::Parameter{"surround_obstacle_stop.base.car", 0.7},
     rclcpp::Parameter{"surround_obstacle_stop.hysteresis_time", 0.3},
     rclcpp::Parameter{"traffic_light_stop.stop_margin", 1.0},
     rclcpp::Parameter{"traffic_light_stop.stop_for_red_light", false},
     rclcpp::Parameter{"traffic_light_stop.treat_unknown_light_as_red", true},
     rclcpp::Parameter{"traffic_light_stop.crossing_time_limit", 3.0}});

  trigger_parameter_update();

  const auto & node_params = ParameterUpdateTestAccessor::params(*node_);
  EXPECT_FALSE(node_params.use_stop_point_fixer);
  EXPECT_DOUBLE_EQ(node_params.trajectory_time_step, 0.2);
  EXPECT_DOUBLE_EQ(node_params.stopping_constraints.nominal_deceleration, 1.2);
  EXPECT_DOUBLE_EQ(node_params.obstacle_stop.stop_margin, 5.0);
  EXPECT_DOUBLE_EQ(node_params.surround_obstacle_stop.base.car, 0.7);
  EXPECT_DOUBLE_EQ(node_params.traffic_light_stop.stop_margin, 1.0);

  const auto stop_point_fixer = find_plugin<StopPointFixer>();
  ASSERT_NE(stop_point_fixer, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*stop_point_fixer));
  EXPECT_FALSE(
    ParameterUpdateTestAccessor::params(*stop_point_fixer).force_stop_long_stopped_trajectories);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*stop_point_fixer).velocity_threshold, 0.2);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*stop_point_fixer).min_distance_threshold, 1.5);

  const auto velocity_modifier = find_plugin<VelocityModifier>();
  ASSERT_NE(velocity_modifier, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*velocity_modifier));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::trajectory_time_step(*velocity_modifier), 0.2);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*velocity_modifier).nominal_deceleration, 1.2);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*velocity_modifier).maximum_deceleration, 3.5);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*velocity_modifier).jerk_limit, 2.5);

  const auto obstacle_stop = find_plugin<ObstacleStop>();
  ASSERT_NE(obstacle_stop, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*obstacle_stop));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::trajectory_time_step(*obstacle_stop), 0.2);
  EXPECT_FALSE(ParameterUpdateTestAccessor::params(*obstacle_stop).use_objects);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*obstacle_stop).stop_margin, 5.0);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*obstacle_stop).obstacle_tracking.on_time_buffer, 0.4);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*obstacle_stop).objects.max_velocity_th, 4.0);
  EXPECT_EQ(
    ParameterUpdateTestAccessor::params(*obstacle_stop).objects.object_types,
    (std::vector<std::string>{"car", "pedestrian"}));
  EXPECT_EQ(
    ParameterUpdateTestAccessor::params(*obstacle_stop).pointcloud.voxel_grid_filter.min_size, 4);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*obstacle_stop).rss_params.ego_decel, 3.0);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::stopping_params(*obstacle_stop).nominal_deceleration, 1.2);

  const auto surround_stop = find_plugin<SurroundObstacleStop>();
  ASSERT_NE(surround_stop, nullptr);
  EXPECT_TRUE(ParameterUpdateTestAccessor::enabled(*surround_stop));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::trajectory_time_step(*surround_stop), 0.2);
  EXPECT_FALSE(ParameterUpdateTestAccessor::params(*surround_stop).use_objects);
  EXPECT_EQ(
    ParameterUpdateTestAccessor::params(*surround_stop).object_types,
    (std::vector<std::string>{"car", "bicycle"}));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*surround_stop).base_distance_th, 0.6);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*surround_stop).base.car, 0.7);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*surround_stop).hysteresis_time, 0.3);

  const auto traffic_light_stop = find_plugin<TrafficLightStop>();
  ASSERT_NE(traffic_light_stop, nullptr);
  EXPECT_FALSE(ParameterUpdateTestAccessor::enabled(*traffic_light_stop));
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::params(*traffic_light_stop).stop_margin, 1.0);
  EXPECT_FALSE(ParameterUpdateTestAccessor::params(*traffic_light_stop).stop_for_red_light);
  EXPECT_TRUE(ParameterUpdateTestAccessor::params(*traffic_light_stop).treat_unknown_light_as_red);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::params(*traffic_light_stop).crossing_time_limit, 3.0);
  EXPECT_DOUBLE_EQ(
    ParameterUpdateTestAccessor::stopping_params(*traffic_light_stop).nominal_deceleration, 1.2);
}

TEST_F(ModifierParameterServiceFixture, DISABLED_UpdatesPluginListAtRuntime)
{
  const std::vector<std::string> expected_names = {
    "autoware::trajectory_modifier::plugin::VelocityModifier",
    "autoware::trajectory_modifier::plugin::StopPointFixer"};
  set_parameter_atomically(*client_, executor_, rclcpp::Parameter{"plugin_names", expected_names});
  trigger_parameter_update();

  EXPECT_EQ(ParameterUpdateTestAccessor::params(*node_).plugin_names, expected_names);
  const auto & plugins = ParameterUpdateTestAccessor::plugins(*node_);
  ASSERT_EQ(plugins.size(), expected_names.size());
  for (size_t i = 0; i < plugins.size(); ++i) {
    EXPECT_EQ(plugins.at(i)->get_name(), expected_names.at(i));
  }
}

TEST_F(ModifierParameterServiceFixture, DISABLED_UpdatesTrajectoryTimeStepInAllPlugins)
{
  using autoware::trajectory_modifier::plugin::StopPointFixer;
  using autoware::trajectory_modifier::plugin::TrafficLightStop;

  set_parameter_atomically(*client_, executor_, rclcpp::Parameter{"trajectory_time_step", 0.2});
  trigger_parameter_update();

  const auto stop_point_fixer = find_plugin<StopPointFixer>();
  const auto traffic_light_stop = find_plugin<TrafficLightStop>();
  ASSERT_NE(stop_point_fixer, nullptr);
  ASSERT_NE(traffic_light_stop, nullptr);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::trajectory_time_step(*stop_point_fixer), 0.2);
  EXPECT_DOUBLE_EQ(ParameterUpdateTestAccessor::trajectory_time_step(*traffic_light_stop), 0.2);
}
