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

#include "autoware/trajectory_processor/trajectory_optimizer.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{

using autoware::trajectory_optimizer::TrajectoryOptimizer;

struct ParameterCase
{
  std::string name;
  rclcpp::Parameter value;
};

rclcpp::NodeOptions make_optimizer_node_options()
{
  auto options = rclcpp::NodeOptions{};
  options.append_parameter_override("use_sim_time", true);
  options.append_parameter_override("trajectory_velocity_optimizer.smooth_velocities", true);

  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_trajectory_processor");
  const auto test_utils_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");

  options.arguments(
    {"--ros-args",
     "--params-file",
     package_dir + "/config/trajectory_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_qp_smoother.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_point_fixer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_velocity_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_extender.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_spline_smoother.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_kinematic_feasibility_enforcer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_mpt_optimizer.param.yaml",
     "--params-file",
     package_dir + "/config/plugins/trajectory_temporal_mpt_optimizer.param.yaml",
     "--params-file",
     test_utils_dir + "/config/test_vehicle_info.param.yaml"});

  return options;
}

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
    ASSERT_TRUE(wait_for_parameter_service());
  }

  void TearDown() override
  {
    executor_.remove_node(node_->get_node_base_interface());
    executor_.remove_node(caller_);
    client_.reset();
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

  void set_parameter_atomically(const rclcpp::Parameter & parameter)
  {
    auto future = client_->set_parameters_atomically({parameter});
    ASSERT_EQ(spin_until_complete(future), rclcpp::FutureReturnCode::SUCCESS);
    const auto result = future.get();
    EXPECT_TRUE(result.successful) << parameter.get_name() << ": " << result.reason;
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr caller_;
  std::shared_ptr<TrajectoryOptimizer> node_;
  std::shared_ptr<rclcpp::AsyncParametersClient> client_;
};

bool is_plugin_list_parameter(const std::string & name)
{
  return name == "plugin_names";
}

bool is_read_only(const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  return descriptor.read_only;
}

std::optional<rclcpp::Parameter> make_updated_parameter(
  const rclcpp::Parameter & current, const rcl_interfaces::msg::ParameterDescriptor & descriptor)
{
  const auto & name = current.get_name();
  switch (current.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return rclcpp::Parameter{name, !current.as_bool()};
    case rclcpp::ParameterType::PARAMETER_INTEGER: {
      const auto current_value = static_cast<int64_t>(current.as_int());
      if (!descriptor.integer_range.empty()) {
        const auto & range = descriptor.integer_range.front();
        const auto step = static_cast<int64_t>(range.step == 0 ? 1 : range.step);
        const auto from_value = static_cast<int64_t>(range.from_value);
        const auto to_value = static_cast<int64_t>(range.to_value);
        const auto next = current_value + step <= to_value ? current_value + step : from_value;
        if (next != current_value) {
          return rclcpp::Parameter{name, next};
        }
        return std::nullopt;
      }
      return rclcpp::Parameter{name, current_value == 0 ? 1 : current_value + 1};
    }
    case rclcpp::ParameterType::PARAMETER_DOUBLE: {
      const auto current_value = current.as_double();
      if (!descriptor.floating_point_range.empty()) {
        const auto & range = descriptor.floating_point_range.front();
        const auto mid = (range.from_value + range.to_value) * 0.5;
        if (mid != current_value) {
          return rclcpp::Parameter{name, mid};
        }
        const auto next = current_value + (range.step == 0.0 ? 1.0e-3 : range.step);
        if (next <= range.to_value && next != current_value) {
          return rclcpp::Parameter{name, next};
        }
        return std::nullopt;
      }
      const auto delta = current_value == 0.0 ? 1.0e-3 : current_value * 0.01;
      return rclcpp::Parameter{name, current_value + delta};
    }
    case rclcpp::ParameterType::PARAMETER_STRING:
      return rclcpp::Parameter{name, current.as_string() + "_updated_by_test"};
    default:
      return std::nullopt;
  }
}

std::vector<std::string> list_declared_parameters(
  rclcpp::AsyncParametersClient & client, rclcpp::executors::SingleThreadedExecutor & executor)
{
  auto future = client.list_parameters({}, 10);
  EXPECT_EQ(
    executor.spin_until_future_complete(future, std::chrono::seconds{5}),
    rclcpp::FutureReturnCode::SUCCESS);
  return future.get().names;
}

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
    set_parameter_atomically(test_case.value);
    const auto stored = get_parameters({test_case.name});
    ASSERT_EQ(stored.size(), 1U);
    EXPECT_EQ(stored.front(), test_case.value) << test_case.name;
  }
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
  GTEST_SKIP()
    << "plugin_names is declared writable, but optimizer plugins are only constructed during "
    << "startup. "
    << "Enable this once runtime load/unload/reorder support is implemented.";
}

TEST_F(ParameterServiceFixture, DISABLED_UpdatesElasticBandInternalParametersAtRuntime)
{
  GTEST_SKIP() << "ElasticBandOptimizer currently refreshes enabled only. Enable this once "
               << "elastic_band_params "
               << "are forwarded to the smoother's internal state on parameter updates.";
}
