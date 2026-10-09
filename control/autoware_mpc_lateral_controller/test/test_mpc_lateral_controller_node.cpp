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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace
{
using autoware::motion::control::mpc_lateral_controller::MpcLateralControllerConfig;
using autoware::motion::control::mpc_lateral_controller::MpcLateralControllerNode;

constexpr double deg2rad = M_PI / 180.0;

/// One parameter, the field of the configuration it has to reach, and the value expected
/// there.
struct Mapping
{
  rclcpp::Parameter parameter;
  std::function<double(const MpcLateralControllerConfig &)> field;
  double expected;
};

/// Every parameter that reaches a single field. Each value differs from the value the
/// shipped parameter files carry and from every other value here, so that a parameter
/// which is not read, or which is read into the field of another, fails the test.
std::vector<Mapping> single_field_mappings()
{
  using Config = MpcLateralControllerConfig;
  return {
    {rclcpp::Parameter("ctrl_period", 0.04), [](const Config & c) { return c.ctrl_period; }, 0.04},
    {rclcpp::Parameter("enable_path_smoothing", true),
     [](const Config & c) { return c.trajectory_filtering.enable_path_smoothing; }, 1.0},
    {rclcpp::Parameter("path_filter_moving_ave_num", 26),
     [](const Config & c) { return c.trajectory_filtering.path_filter_moving_ave_num; }, 26.0},
    {rclcpp::Parameter("curvature_smoothing_num_traj", 16),
     [](const Config & c) { return c.trajectory_filtering.curvature_smoothing_num_traj; }, 16.0},
    {rclcpp::Parameter("curvature_smoothing_num_ref_steer", 17),
     [](const Config & c) { return c.trajectory_filtering.curvature_smoothing_num_ref_steer; },
     17.0},
    {rclcpp::Parameter("traj_resample_dist", 0.15),
     [](const Config & c) { return c.trajectory_filtering.traj_resample_dist; }, 0.15},
    {rclcpp::Parameter("extend_trajectory_for_end_yaw_control", true),
     [](const Config & c) { return c.trajectory_filtering.extend_trajectory_for_end_yaw_control; },
     1.0},
    {rclcpp::Parameter("use_steer_prediction", true),
     [](const Config & c) { return c.use_steer_prediction; }, 1.0},
    {rclcpp::Parameter("vehicle_model_steer_tau", 0.29),
     [](const Config & c) { return c.mpc_param.steer_tau; }, 0.29},
    {rclcpp::Parameter("stop_state_entry_ego_speed", 0.0021),
     [](const Config & c) { return c.stop_state_entry_ego_speed; }, 0.0021},
    {rclcpp::Parameter("stop_state_entry_target_speed", 0.0032),
     [](const Config & c) { return c.stop_state_entry_target_speed; }, 0.0032},
    {rclcpp::Parameter("converged_steer_rad", 0.13),
     [](const Config & c) { return c.converged_steer_rad; }, 0.13},
    {rclcpp::Parameter("keep_steer_control_until_converged", false),
     [](const Config & c) { return c.keep_steer_control_until_converged; }, 0.0},
    {rclcpp::Parameter("new_traj_duration_time", 1.4),
     [](const Config & c) { return c.new_traj_duration_time; }, 1.4},
    {rclcpp::Parameter("new_traj_end_dist", 0.35),
     [](const Config & c) { return c.new_traj_end_dist; }, 0.35},
    {rclcpp::Parameter("mpc_converged_threshold_rps", 0.017),
     [](const Config & c) { return c.mpc_converged_threshold_rps; }, 0.017},
    {rclcpp::Parameter("wheel_base", 2.9), [](const Config & c) { return c.wheelbase; }, 2.9},
    {rclcpp::Parameter("max_steer_angle", 0.65), [](const Config & c) { return c.steer_lim; },
     0.65},
    {rclcpp::Parameter("input_delay", 0.28), [](const Config & c) { return c.input_delay; }, 0.28},
    {rclcpp::Parameter("steering_offset.max_update_th", 0.023),
     [](const Config & c) { return c.steer_offset_max_update_th; }, 0.023},
    {rclcpp::Parameter("steering_offset.steer_offset_filter_cutoff_hz", 0.55),
     [](const Config & c) { return c.steer_offset_filter_cutoff_hz; }, 0.55},
    {rclcpp::Parameter("steering_lpf_cutoff_hz", 3.3),
     [](const Config & c) { return c.steering_lpf_cutoff_hz; }, 3.3},
    {rclcpp::Parameter("error_deriv_lpf_cutoff_hz", 5.4),
     [](const Config & c) { return c.error_deriv_lpf_cutoff_hz; }, 5.4},
    {rclcpp::Parameter("ego_nearest_dist_threshold", 3.6),
     [](const Config & c) { return c.ego_nearest_dist_threshold; }, 3.6},
    {rclcpp::Parameter("ego_nearest_yaw_threshold", 1.07),
     [](const Config & c) { return c.ego_nearest_yaw_threshold; }, 1.07},
    {rclcpp::Parameter("use_delayed_initial_state", false),
     [](const Config & c) { return c.use_delayed_initial_state; }, 0.0},
    {rclcpp::Parameter("trajectory_reference_mode", std::string("temporal")),
     [](const Config & c) { return c.use_temporal_trajectory; }, 1.0},
    {rclcpp::Parameter("publish_debug_trajectories", false),
     [](const Config & c) { return c.publish_debug_trajectories; }, 0.0},
    {rclcpp::Parameter("mpc_prediction_horizon", 41),
     [](const Config & c) { return c.mpc_param.prediction_horizon; }, 41.0},
    {rclcpp::Parameter("mpc_prediction_dt", 0.11),
     [](const Config & c) { return c.mpc_param.prediction_dt; }, 0.11},
    {rclcpp::Parameter("mpc_weight_lat_error", 1.01),
     [](const Config & c) { return c.mpc_param.nominal_weight.lat_error; }, 1.01},
    {rclcpp::Parameter("mpc_weight_heading_error", 0.02),
     [](const Config & c) { return c.mpc_param.nominal_weight.heading_error; }, 0.02},
    {rclcpp::Parameter("mpc_weight_heading_error_squared_vel", 0.31),
     [](const Config & c) { return c.mpc_param.nominal_weight.heading_error_squared_vel; }, 0.31},
    {rclcpp::Parameter("mpc_weight_steering_input", 1.02),
     [](const Config & c) { return c.mpc_param.nominal_weight.steering_input; }, 1.02},
    {rclcpp::Parameter("mpc_weight_steering_input_squared_vel", 0.26),
     [](const Config & c) { return c.mpc_param.nominal_weight.steering_input_squared_vel; }, 0.26},
    {rclcpp::Parameter("mpc_weight_lat_jerk", 0.12),
     [](const Config & c) { return c.mpc_param.nominal_weight.lat_jerk; }, 0.12},
    {rclcpp::Parameter("mpc_weight_steer_rate", 0.03),
     [](const Config & c) { return c.mpc_param.nominal_weight.steer_rate; }, 0.03},
    {rclcpp::Parameter("mpc_weight_steer_acc", 0.0000013),
     [](const Config & c) { return c.mpc_param.nominal_weight.steer_acc; }, 0.0000013},
    {rclcpp::Parameter("mpc_weight_terminal_lat_error", 1.03),
     [](const Config & c) { return c.mpc_param.nominal_weight.terminal_lat_error; }, 1.03},
    {rclcpp::Parameter("mpc_weight_terminal_heading_error", 0.14),
     [](const Config & c) { return c.mpc_param.nominal_weight.terminal_heading_error; }, 0.14},
    {rclcpp::Parameter("mpc_low_curvature_weight_lat_error", 0.16),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.lat_error; }, 0.16},
    {rclcpp::Parameter("mpc_low_curvature_weight_heading_error", 0.07),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.heading_error; }, 0.07},
    {rclcpp::Parameter("mpc_low_curvature_weight_heading_error_squared_vel", 0.32),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.heading_error_squared_vel; },
     0.32},
    {rclcpp::Parameter("mpc_low_curvature_weight_steering_input", 1.04),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.steering_input; }, 1.04},
    {rclcpp::Parameter("mpc_low_curvature_weight_steering_input_squared_vel", 0.27),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.steering_input_squared_vel; },
     0.27},
    {rclcpp::Parameter("mpc_low_curvature_weight_lat_jerk", 0.05),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.lat_jerk; }, 0.05},
    {rclcpp::Parameter("mpc_low_curvature_weight_steer_rate", 0.06),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.steer_rate; }, 0.06},
    {rclcpp::Parameter("mpc_low_curvature_weight_steer_acc", 0.0000017),
     [](const Config & c) { return c.mpc_param.low_curvature_weight.steer_acc; }, 0.0000017},
    {rclcpp::Parameter("mpc_low_curvature_thresh_curvature", 0.021),
     [](const Config & c) { return c.mpc_param.low_curvature_thresh_curvature; }, 0.021},
    {rclcpp::Parameter("mpc_zero_ff_steer_deg", 0.45),
     [](const Config & c) { return c.mpc_param.zero_ff_steer_deg; }, 0.45},
    {rclcpp::Parameter("mpc_acceleration_limit", 2.1),
     [](const Config & c) { return c.mpc_param.acceleration_limit; }, 2.1},
    {rclcpp::Parameter("mpc_velocity_time_constant", 0.33),
     [](const Config & c) { return c.mpc_param.velocity_time_constant; }, 0.33},
    {rclcpp::Parameter("mpc_min_prediction_length", 5.5),
     [](const Config & c) { return c.mpc_param.min_prediction_length; }, 5.5},
  };
}

class MpcLateralControllerNodeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  /// A node that carries the shipped parameter files, the values the trajectory follower
  /// node and the vehicle description package provide in production, and the given
  /// parameters over all of them.
  rclcpp::Node & make_node(const std::vector<rclcpp::Parameter> & parameters = {})
  {
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("autoware_mpc_lateral_controller");

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/param/lateral_controller_defaults.param.yaml",
       "--params-file", share_dir + "/param/steer_offset.param.yaml"});

    options.append_parameter_override("ctrl_period", 0.03);
    options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
    options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);

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
    // The trajectory follower node declares this one before building the controller.
    node_->declare_parameter<double>("ctrl_period");
    return *node_;
  }

  std::shared_ptr<rclcpp::Node> node_;
};
}  // namespace

/// createConfig copies each parameter into one field of the configuration. Every parameter
/// is given a value that no other parameter has, so a parameter that is not read, or that
/// is copied into the field of another parameter, leaves a field with an unexpected value.
TEST_F(MpcLateralControllerNodeTest, EachParameterIsCopiedIntoItsOwnField)
{
  const auto mappings = single_field_mappings();
  std::vector<rclcpp::Parameter> parameters;
  for (const auto & mapping : mappings) {
    parameters.push_back(mapping.parameter);
  }
  auto & node = make_node(parameters);

  const auto config = MpcLateralControllerNode::createConfig(node);

  for (const auto & mapping : mappings) {
    EXPECT_DOUBLE_EQ(mapping.field(config), mapping.expected) << mapping.parameter.get_name();
  }
}

/// Each limit is paired with the curvature at the same position in the other list, and is
/// given in degrees per second but kept in radians per second.
TEST_F(MpcLateralControllerNodeTest, SteerRateLimitsByCurvatureArePairedAndInRadians)
{
  auto & node = make_node({
    rclcpp::Parameter("curvature_list_for_steer_rate_lim", std::vector<double>{0.003, 0.004}),
    rclcpp::Parameter("steer_rate_lim_dps_list_by_curvature", std::vector<double>{30.0, 45.0}),
  });

  const auto config = MpcLateralControllerNode::createConfig(node);

  ASSERT_EQ(config.steer_rate_lim_by_curvature.size(), 2u);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_curvature[0].first, 0.003);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_curvature[0].second, 30.0 * deg2rad);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_curvature[1].first, 0.004);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_curvature[1].second, 45.0 * deg2rad);
}

TEST_F(MpcLateralControllerNodeTest, SteerRateLimitsByVelocityArePairedAndInRadians)
{
  auto & node = make_node({
    rclcpp::Parameter("velocity_list_for_steer_rate_lim", std::vector<double>{5.0, 25.0}),
    rclcpp::Parameter("steer_rate_lim_dps_list_by_velocity", std::vector<double>{90.0, 20.0}),
  });

  const auto config = MpcLateralControllerNode::createConfig(node);

  ASSERT_EQ(config.steer_rate_lim_by_velocity.size(), 2u);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_velocity[0].first, 5.0);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_velocity[0].second, 90.0 * deg2rad);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_velocity[1].first, 25.0);
  EXPECT_DOUBLE_EQ(config.steer_rate_lim_by_velocity[1].second, 20.0 * deg2rad);
}

/// The trajectory follower node declares trajectory_reference_mode before it builds the
/// controller, so the controller reads the value already declared instead of declaring it.
/// A temporal mode lost on this path would leave the controller following the path by
/// distance, which changes the command too little to notice while driving.
TEST_F(MpcLateralControllerNodeTest, ReferenceModeDeclaredByTheTrajectoryFollowerIsApplied)
{
  auto & node = make_node();
  node.declare_parameter<std::string>("trajectory_reference_mode", "temporal");

  const auto config = MpcLateralControllerNode::createConfig(node);

  EXPECT_TRUE(config.use_temporal_trajectory);
}
