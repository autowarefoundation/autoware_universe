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

#include <in_lane_mrm_planner_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>

namespace
{

// Builds a ParamListener for a node whose `param_name` is overridden with `value`.
// Construction declares + validates all parameters, throwing on a validation failure.
void build_param_listener_with_override(const std::string & param_name, const double value)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter(param_name, value)});
  auto node = std::make_shared<rclcpp::Node>("in_lane_mrm_planner_param_test", options);
  ::in_lane_mrm_planner::ParamListener listener(node->get_node_parameters_interface());
  (void)listener.get_params();
}

class ParamValidationTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

}  // namespace

TEST_F(ParamValidationTest, AcceptsDefaultNegativeRelaxationParams)
{
  // Defaults (all negative) must pass validation.
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("in_lane_mrm_planner_param_test", options);
  EXPECT_NO_THROW({
    ::in_lane_mrm_planner::ParamListener listener(node->get_node_parameters_interface());
    (void)listener.get_params();
  });
}

TEST_F(ParamValidationTest, RejectsZeroRelaxationStep)
{
  // step == 0 would stall the relaxation loop; lt<>(0.0) must reject it at load time.
  EXPECT_THROW(
    build_param_listener_with_override("mrm_velocity.step_jerk_relaxation", 0.0),
    rclcpp::exceptions::InvalidParameterValueException);
  EXPECT_THROW(
    build_param_listener_with_override("mrm_velocity.step_deceleration_relaxation", 0.0),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(ParamValidationTest, RejectsPositiveTargetAndMaxParams)
{
  // Wrong-sign (positive) targets/limits must be rejected for every profile.
  for (const std::string profile : {"moderate", "emergency"}) {
    const std::string prefix = "mrm_velocity.profiles." + profile + ".";
    EXPECT_THROW(
      build_param_listener_with_override(prefix + "target_jerk", 5.0),
      rclcpp::exceptions::InvalidParameterValueException)
      << profile;
    EXPECT_THROW(
      build_param_listener_with_override(prefix + "target_deceleration", 3.0),
      rclcpp::exceptions::InvalidParameterValueException)
      << profile;
    EXPECT_THROW(
      build_param_listener_with_override(prefix + "max_jerk_relaxation", 20.0),
      rclcpp::exceptions::InvalidParameterValueException)
      << profile;
    EXPECT_THROW(
      build_param_listener_with_override(prefix + "max_deceleration_relaxation", 6.0),
      rclcpp::exceptions::InvalidParameterValueException)
      << profile;
  }
}

TEST_F(ParamValidationTest, ProfileDefaultsMatchL4Constraints)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("in_lane_mrm_planner_param_test", options);
  ::in_lane_mrm_planner::ParamListener listener(node->get_node_parameters_interface());
  const auto params = listener.get_params();
  const auto & moderate = params.mrm_velocity.profiles.moderate;
  EXPECT_DOUBLE_EQ(moderate.target_deceleration, -3.0);
  EXPECT_DOUBLE_EQ(moderate.target_jerk, -5.0);
  EXPECT_DOUBLE_EQ(moderate.max_deceleration_relaxation, -4.0);
  EXPECT_DOUBLE_EQ(moderate.max_jerk_relaxation, -10.0);
  const auto & emergency = params.mrm_velocity.profiles.emergency;
  EXPECT_DOUBLE_EQ(emergency.target_deceleration, -6.0);
  EXPECT_DOUBLE_EQ(emergency.target_jerk, -20.0);
  EXPECT_DOUBLE_EQ(emergency.max_deceleration_relaxation, -8.0);
  EXPECT_DOUBLE_EQ(emergency.max_jerk_relaxation, -30.0);
}

TEST_F(ParamValidationTest, RejectsNegativeBrakeDelayTime)
{
  // Brake dead time is a physical duration; negative values must be rejected at load time.
  EXPECT_THROW(
    build_param_listener_with_override("mrm_velocity.brake_delay_time", -0.1),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(ParamValidationTest, AcceptsZeroAndPositiveBrakeDelayTime)
{
  EXPECT_NO_THROW(build_param_listener_with_override("mrm_velocity.brake_delay_time", 0.0));
  EXPECT_NO_THROW(build_param_listener_with_override("mrm_velocity.brake_delay_time", 0.5));
}
