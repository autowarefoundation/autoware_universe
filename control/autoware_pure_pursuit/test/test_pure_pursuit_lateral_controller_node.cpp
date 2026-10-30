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

#include "autoware/pure_pursuit/autoware_pure_pursuit_lateral_controller_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <functional>
#include <memory>
#include <vector>

namespace
{
using autoware::pure_pursuit::PurePursuitLateralControllerConfig;
using autoware::pure_pursuit::PurePursuitLateralControllerNode;

struct Mapping
{
  rclcpp::Parameter parameter;
  std::function<double(const PurePursuitLateralControllerConfig &)> field;
  double expected;
};

std::vector<Mapping> single_field_mappings()
{
  using Config = PurePursuitLateralControllerConfig;
  return {
    {rclcpp::Parameter("ld_velocity_ratio", 2.3),
     [](const Config & c) { return c.ld_velocity_ratio; }, 2.3},
    {rclcpp::Parameter("ld_lateral_error_ratio", 3.5),
     [](const Config & c) { return c.ld_lateral_error_ratio; }, 3.5},
    {rclcpp::Parameter("ld_curvature_ratio", 110.0),
     [](const Config & c) { return c.ld_curvature_ratio; }, 110.0},
    {rclcpp::Parameter("long_ld_lateral_error_threshold", 0.45),
     [](const Config & c) { return c.long_ld_lateral_error_threshold; }, 0.45},
    {rclcpp::Parameter("min_lookahead_distance", 4.25),
     [](const Config & c) { return c.min_lookahead_distance; }, 4.25},
    {rclcpp::Parameter("max_lookahead_distance", 14.0),
     [](const Config & c) { return c.max_lookahead_distance; }, 14.0},
    {rclcpp::Parameter("reverse_min_lookahead_distance", 6.5),
     [](const Config & c) { return c.reverse_min_lookahead_distance; }, 6.5},
    {rclcpp::Parameter("converged_steer_rad", 0.11),
     [](const Config & c) { return c.converged_steer_rad_; }, 0.11},
    {rclcpp::Parameter("prediction_ds", 0.35), [](const Config & c) { return c.prediction_ds; },
     0.35},
    {rclcpp::Parameter("prediction_distance_length", 22.0),
     [](const Config & c) { return c.prediction_distance_length; }, 22.0},
    {rclcpp::Parameter("resampling_ds", 0.12), [](const Config & c) { return c.resampling_ds; },
     0.12},
    {rclcpp::Parameter("curvature_calculation_distance", 4.2),
     [](const Config & c) { return c.curvature_calculation_distance; }, 4.2},
    {rclcpp::Parameter("enable_path_smoothing", true),
     [](const Config & c) { return c.enable_path_smoothing; }, 1.0},
    {rclcpp::Parameter("path_filter_moving_ave_num", 26),
     [](const Config & c) { return c.path_filter_moving_ave_num; }, 26.0},
    {rclcpp::Parameter("wheel_base", 2.9), [](const Config & c) { return c.wheel_base; }, 2.9},
    {rclcpp::Parameter("max_steer_angle", 0.65),
     [](const Config & c) { return c.max_steering_angle; }, 0.65},
  };
}

class PurePursuitLateralControllerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  /// Build a node with the shipped parameter file and the vehicle values that the vehicle
  /// description package provides in production. The given parameters replace those values.
  rclcpp::Node & make_node(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory("autoware_pure_pursuit");

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/config/pure_pursuit.param.yaml"});

    options.append_parameter_override("wheel_radius", 0.39);
    options.append_parameter_override("wheel_width", 0.42);
    options.append_parameter_override("wheel_base", 2.74);
    options.append_parameter_override("wheel_tread", 1.63);
    options.append_parameter_override("front_overhang", 1.0);
    options.append_parameter_override("rear_overhang", 1.03);
    options.append_parameter_override("left_overhang", 0.1);
    options.append_parameter_override("right_overhang", 0.1);
    options.append_parameter_override("vehicle_height", 2.5);
    options.append_parameter_override("max_steer_angle", 0.70);

    for (const auto & parameter : parameters) {
      options.parameter_overrides().push_back(parameter);
    }

    node_ = std::make_shared<rclcpp::Node>("test_node", options);
    return *node_;
  }

  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace

/// createConfig copies each parameter into one field of the configuration. Every parameter
/// is given a value that no other parameter has, so a parameter that is not read, or that
/// is copied into the field of another parameter, leaves a field with an unexpected value.
TEST_F(PurePursuitLateralControllerNodeTest, EachParameterIsCopiedIntoItsOwnField)
{
  const auto mappings = single_field_mappings();
  std::vector<rclcpp::Parameter> parameters;
  for (const auto & mapping : mappings) {
    parameters.push_back(mapping.parameter);
  }
  auto & node = make_node(parameters);

  const auto config = PurePursuitLateralControllerNode::createConfig(node);

  for (const auto & mapping : mappings) {
    EXPECT_DOUBLE_EQ(mapping.field(config), mapping.expected) << mapping.parameter.get_name();
  }
}
