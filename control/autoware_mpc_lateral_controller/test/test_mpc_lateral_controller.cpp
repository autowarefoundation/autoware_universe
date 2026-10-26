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

#include "autoware/mpc_lateral_controller/mpc_lateral_controller.hpp"

#include <rclcpp/time.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::motion::control::mpc_lateral_controller::MpcLateralController;
using autoware::motion::control::mpc_lateral_controller::MpcLateralControllerConfig;
using autoware::motion::control::mpc_lateral_controller::no_log;
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

geometry_msgs::msg::Quaternion make_orientation(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

/// \param x position along the map x axis [m]
/// \param y position along the map y axis [m]
/// \param yaw heading of the point [rad], counter-clockwise from the x axis
TrajectoryPoint make_point(const double x, const double y, const double yaw)
{
  TrajectoryPoint point;
  point.pose.position.x = x;
  point.pose.position.y = y;
  point.pose.orientation = make_orientation(yaw);
  return point;
}

Trajectory make_trajectory(std::vector<TrajectoryPoint> points)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points = std::move(points);
  return trajectory;
}

/// The paths below carry no target speed. Input::planned_at writes one onto every point,
/// which keeps that speed beside the measured speed in the test rather than buried in the
/// call that builds the path.
///
/// The controller resamples the input at traj_resample_dist (0.1 m), so three points
/// spanning the prediction horizon give the same coverage as a longer path. Three is also
/// the shortest path the controller accepts, because the curvature calculation needs that
/// many.
Trajectory straight_path()
{
  return make_trajectory(
    {make_point(0.0, 0.0, 0.0), make_point(10.0, 0.0, 0.0), make_point(20.0, 0.0, 0.0)});
}

Trajectory left_curve_path()
{
  return make_trajectory(
    {make_point(0.0, 0.0, 0.0), make_point(9.9, 1.0, 0.2), make_point(19.4, 4.0, 0.4)});
}

Trajectory right_curve_path()
{
  return make_trajectory(
    {make_point(0.0, 0.0, 0.0), make_point(9.9, -1.0, -0.2), make_point(19.4, -4.0, -0.4)});
}

/// Extend a straight path by moving its far end further along, leaving the near end and
/// the heading where they were. The controller decides that a path has changed shape from
/// how far this end has moved. Extending the path moves that end without bending the path,
/// so the command the controller produces does not change.
Trajectory straight_path_extended_by(const double distance)
{
  return make_trajectory(
    {make_point(0.0, 0.0, 0.0), make_point(10.0, 0.0, 0.0), make_point(20.0 + distance, 0.0, 0.0)});
}

/// A path that follows a circle of the given radius, curving to the left. The ego pose is
/// fixed at the map origin facing along the x axis, and this path starts there with that
/// heading, so the ego begins on the path with no lateral and no heading error.
///
/// The points are spaced half a metre apart, close enough that the circle survives the
/// resampling the controller applies. The path reaches three metres behind the ego, so
/// that the search for the nearest point has a segment on either side of the ego.
Trajectory arc_path(const double radius)
{
  std::vector<TrajectoryPoint> points;
  for (double distance = -3.0; distance <= 40.0; distance += 0.5) {
    const double angle = distance / radius;
    points.push_back(make_point(radius * std::sin(angle), radius * (1.0 - std::cos(angle)), angle));
  }
  return make_trajectory(std::move(points));
}

/// A path whose points carry a time_from_start. The temporal reference mode requires those
/// times to increase along the path and rejects the path when they do not.
Trajectory straight_path_with_time_from_start(const bool increasing)
{
  auto trajectory = straight_path();
  for (size_t index = 0; index < trajectory.points.size(); ++index) {
    trajectory.points[index].time_from_start.sec = increasing ? static_cast<int32_t>(index) : 0;
  }
  return trajectory;
}

/// One control cycle of input. Each call names the quantity it sets, so a test states the
/// situation the controller is given. The ego pose is fixed at the map origin facing along
/// the x axis, so the path alone decides the distance between the ego and the path.
class Input
{
public:
  /// Path to follow.
  Input & following(Trajectory path)
  {
    input_.current_trajectory = std::move(path);
    return *this;
  }

  /// Target speed written to every point of the path [m/s]. What the path requests, not
  /// what the vehicle reports.
  Input & planned_at(const double speed)
  {
    for (auto & point : input_.current_trajectory.points) {
      point.longitudinal_velocity_mps = static_cast<float>(speed);
    }
    return *this;
  }

  /// Measured longitudinal speed of the ego [m/s].
  Input & driving_at(const double speed)
  {
    input_.current_odometry.twist.twist.linear.x = speed;
    return *this;
  }

  /// Measured front tire angle [rad], as the vehicle reports it. Not the commanded angle.
  /// Zero unless a test says otherwise.
  Input & steering_at(const double angle)
  {
    input_.current_steering.steering_tire_angle = static_cast<float>(angle);
    return *this;
  }

  /// Time written to the path. The controller keeps the paths it has been given so that it
  /// can compare their shapes. To decide which of those paths are too old to keep, the
  /// controller compares these stamps instead of reading the clock. A path whose stamp is
  /// still zero is therefore never dropped.
  Input & stamped_at(const double seconds)
  {
    input_.current_trajectory.header.stamp.sec = static_cast<int32_t>(seconds);
    input_.current_trajectory.header.stamp.nanosec =
      static_cast<uint32_t>((seconds - std::floor(seconds)) * 1e9);
    return *this;
  }

  Input & without_autoware_control()
  {
    input_.current_operation_mode.is_autoware_control_enabled = false;
    return *this;
  }

  Input & in_operation_mode(const uint8_t mode)
  {
    input_.current_operation_mode.mode = mode;
    return *this;
  }

  operator InputData() const { return input_; }

private:
  static InputData make_default()
  {
    InputData input;
    input.current_odometry.header.frame_id = "map";
    input.current_odometry.pose.pose.orientation = make_orientation(0.0);
    input.current_operation_mode.is_autoware_control_enabled = true;
    input.current_operation_mode.mode = autoware_adapi_v1_msgs::msg::OperationModeState::AUTONOMOUS;
    return input;
  }

  InputData input_ = make_default();
};

/// A time given to the controller as the time of a cycle [s].
rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(std::llround(seconds * 1e9)), RCL_ROS_TIME);
}

/// The times of a run of control cycles. A test names the time of the first cycle and the
/// interval. Each call to next_cycle returns a time one interval later than the call before.
class CycleClock
{
public:
  /// Time of the first cycle [s].
  CycleClock & first_at(const double seconds)
  {
    next_ = seconds;
    return *this;
  }

  /// Interval between two cycles [s].
  CycleClock & every(const double seconds)
  {
    interval_ = seconds;
    return *this;
  }

  /// Time of the next cycle [s].
  double next_cycle()
  {
    const double time = next_;
    next_ += interval_;
    return time;
  }

private:
  double next_{0.0};
  double interval_{0.0};
};

constexpr double deg2rad = M_PI / 180.0;

constexpr double wheel_base = 2.74;
constexpr double ctrl_period = 0.03;

/// Distance between the commanded and the measured angle below which the controller calls
/// the steering converged. Carried by the shipped parameter file as converged_steer_rad.
constexpr double converged_steer_rad = 0.1;

/// The controller only decides that the command has settled once it holds this much of
/// the command record. Written into the controller rather than read from a parameter.
constexpr double convergence_history_sec = 1.0;

/// Most of the steering offset the controller applies during one cycle. Carried by the
/// shipped parameter file as steering_offset.max_update_th.
constexpr double steer_offset_max_update_th = 0.01;

/// How far the end of the path has to move before the controller treats it as a new
/// shape. Carried by the shipped parameter file as new_traj_end_dist.
constexpr double new_traj_end_dist = 0.3;

/// How long the controller keeps each path it has been given, so that it can compare a
/// later shape against that path. Carried by the shipped parameter file as
/// new_traj_duration_time.
constexpr double new_traj_duration_time = 1.0;

/// Measured speed at or below which the vehicle counts as stopped. Carried by the
/// shipped parameter file as stop_state_entry_ego_speed.
constexpr double stop_state_entry_ego_speed = 0.001;

/// Target speed below which the path counts as asking for a stop. Carried by the
/// shipped parameter file as stop_state_entry_target_speed.
constexpr double stop_state_entry_target_speed = 0.001;

/// A speed step small enough to land on the other side of either stop threshold.
constexpr double speed_step = 0.0001;

/// Radius of the circle the steady state steering tests follow [m]. The controller
/// replaces a feed-forward angle below mpc_zero_ff_steer_deg (0.5 deg) with zero, which
/// this wheelbase reaches at a radius of about 314 m, so the radius stays well below
/// that.
constexpr double arc_radius = 30.0;

/// Speed those tests drive the circle at [m/s]. The weight on the heading error grows
/// with the square of the speed, so at a high speed the optimisation arrives near the
/// same angle even without the feed-forward that decides it. A low speed keeps the two
/// apart.
constexpr double arc_speed = 1.0;

/// Cycles those tests run. The command reaches the settled angle in about twenty cycles.
/// The limit on how far the command may move during one cycle sets that number, not the
/// optimisation.
constexpr int arc_settling_cycles = 60;

/// How far the settled command may differ from the angle the geometry gives. A
/// controller that approximates tan(angle) by the angle itself commands L / R instead of
/// atan(L / R). The two differ by 0.3 per cent at the radius above, so this margin
/// accepts such a controller as well.
constexpr double arc_relative_tolerance = 0.01;

/// Cycles needed for the command record to cover the given duration. The record gains
/// one entry per cycle, so n cycles cover (n - 1) control periods.
int cycles_spanning(const double seconds)
{
  return static_cast<int>(std::ceil(seconds / ctrl_period)) + 1;
}

/// Cycles the controller takes to stop reporting a shape change. It stops once it has
/// dropped the last path of the old shape, and that path arrived one control period
/// before the changed one.
int cycles_to_forget_shape_change()
{
  return static_cast<int>(std::ceil((new_traj_duration_time - ctrl_period) / ctrl_period));
}

/// The values the shipped parameter files carry. The vehicle values are the ones the vehicle
/// description package provides in production.
MpcLateralControllerConfig make_default_config()
{
  MpcLateralControllerConfig config;
  config.ctrl_period = ctrl_period;

  auto & filtering = config.trajectory_filtering;
  filtering.enable_path_smoothing = false;
  filtering.path_filter_moving_ave_num = 25;
  filtering.curvature_smoothing_num_traj = 15;
  filtering.curvature_smoothing_num_ref_steer = 15;
  filtering.traj_resample_dist = 0.1;
  filtering.extend_trajectory_for_end_yaw_control = false;

  config.stop_state_entry_ego_speed = stop_state_entry_ego_speed;
  config.stop_state_entry_target_speed = stop_state_entry_target_speed;
  config.converged_steer_rad = converged_steer_rad;
  config.keep_steer_control_until_converged = true;
  config.new_traj_duration_time = new_traj_duration_time;
  config.new_traj_end_dist = new_traj_end_dist;
  config.mpc_converged_threshold_rps = 0.01;

  config.ego_nearest_dist_threshold = 3.0;
  config.ego_nearest_yaw_threshold = 1.046;

  config.enable_auto_steering_offset_removal = true;
  config.steer_offset_max_update_th = steer_offset_max_update_th;
  config.steer_offset_filter_cutoff_hz = 0.5;

  config.steering_lpf_cutoff_hz = 3.0;
  config.error_deriv_lpf_cutoff_hz = 5.0;

  config.wheelbase = wheel_base;
  config.steer_lim = 0.70;
  config.vehicle_model_type = "kinematics";
  config.mass_fl = 600.0;
  config.mass_fr = 600.0;
  config.mass_rl = 600.0;
  config.mass_rr = 600.0;
  config.cf = 155494.663;
  config.cr = 155494.663;

  config.steer_rate_lim_by_curvature = {
    {0.001, 40.0 * deg2rad}, {0.002, 50.0 * deg2rad}, {0.01, 60.0 * deg2rad}};
  config.steer_rate_lim_by_velocity = {
    {10.0, 60.0 * deg2rad}, {15.0, 50.0 * deg2rad}, {20.0, 40.0 * deg2rad}};

  config.qp_solver_type = "osqp";
  auto & mpc = config.mpc_param;
  mpc.prediction_horizon = 50;
  mpc.prediction_dt = 0.1;
  mpc.steer_tau = 0.27;
  mpc.zero_ff_steer_deg = 0.5;
  mpc.acceleration_limit = 2.0;
  mpc.velocity_time_constant = 0.3;
  mpc.min_prediction_length = 5.0;
  mpc.low_curvature_thresh_curvature = 0.0;

  auto & nominal = mpc.nominal_weight;
  nominal.lat_error = 1.0;
  nominal.heading_error = 0.0;
  nominal.heading_error_squared_vel = 0.3;
  nominal.steering_input = 1.0;
  nominal.steering_input_squared_vel = 0.25;
  nominal.lat_jerk = 0.1;
  nominal.steer_rate = 0.0;
  nominal.steer_acc = 0.000001;
  nominal.terminal_lat_error = 1.0;
  nominal.terminal_heading_error = 0.1;

  auto & low_curvature = mpc.low_curvature_weight;
  low_curvature.lat_error = 0.1;
  low_curvature.heading_error = 0.0;
  low_curvature.heading_error_squared_vel = 0.3;
  low_curvature.steering_input = 1.0;
  low_curvature.steering_input_squared_vel = 0.25;
  low_curvature.lat_jerk = 0.0;
  low_curvature.steer_rate = 0.0;
  low_curvature.steer_acc = 0.000001;

  config.input_delay = 0.24;
  config.use_steer_prediction = false;
  config.use_delayed_initial_state = true;
  config.use_temporal_trajectory = false;
  config.publish_debug_trajectories = true;
  return config;
}

/// Some of the behaviours below only appear after the controller has run for a while,
/// because the controller decides them from a record of earlier cycles. The controller
/// keeps two such records. It reads a different time for each record when it decides
/// which entries are too old to keep, so there is one helper per record.
///
/// This helper repeats an input the test has assembled, once in each cycle it runs. Use it
/// for the record of commands, because the controller compares the times
/// of the cycles to decide which commands are too old to keep.
LateralOutput run_cycles(
  MpcLateralController & controller, const InputData & input, CycleClock & clock, const int cycles)
{
  LateralOutput output;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    output = controller.run(input, at(clock.next_cycle())).output;
  }
  return output;
}

/// This helper follows a path once in each cycle it runs, and writes the time of the cycle
/// to the path as its stamp. Use it for the record of paths. To decide which paths are
/// too old to keep, the controller compares the stamps on the paths it keeps instead of
/// the times of the cycles. A path whose stamp is still zero is therefore never dropped.
///
/// The vehicle follows the path at 1 m/s and reports its steering as straight ahead. No
/// test that uses this helper needs it anywhere else.
LateralOutput follow_path_for_cycles(
  MpcLateralController & controller, const Trajectory & path, CycleClock & clock, const int cycles)
{
  LateralOutput output;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const double time = clock.next_cycle();
    output =
      controller
        .run(Input().following(path).planned_at(1.0).driving_at(1.0).stamped_at(time), at(time))
        .output;
  }
  return output;
}

/// Drive a curve at speed until the command stops moving, and report the command reached.
/// Five cycles are enough for the low pass filters inside the controller to settle.
float settle_on_curve(MpcLateralController & controller, CycleClock & clock, const int cycles)
{
  const InputData driving = Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0);
  float command = 0.0f;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    command =
      controller.run(driving, at(clock.next_cycle())).output.control_cmd.steering_tire_angle;
  }
  return command;
}

/// Drive a circle until the command settles, and report the command reached.
///
/// The ego pose stays on the circle, so the lateral and the heading error stay at zero
/// and the command is the angle the controller holds that state with. The command of one
/// cycle is returned as the measured angle of the next, which is the relation that holds
/// once the steering of a vehicle has reached the commanded angle. Nothing else about a
/// vehicle is modelled, so the result does not depend on how quickly one responds.
float settle_on_arc(MpcLateralController & controller, CycleClock & clock, const int cycles)
{
  const Trajectory path = arc_path(arc_radius);
  float command = 0.0f;
  for (int cycle = 0; cycle < cycles; ++cycle) {
    const InputData driving =
      Input().following(path).planned_at(arc_speed).driving_at(arc_speed).steering_at(command);
    command =
      controller.run(driving, at(clock.next_cycle())).output.control_cmd.steering_tire_angle;
  }
  return command;
}

/// The controller offers three vehicle models. Each one has to accept the same input and
/// steer to the same side, so the shared expectations run against all of them.
class MpcLateralControllerModelTest : public ::testing::TestWithParam<std::string>
{
};

INSTANTIATE_TEST_SUITE_P(
  VehicleModels, MpcLateralControllerModelTest,
  ::testing::Values("kinematics", "kinematics_no_delay", "dynamics"),
  [](const ::testing::TestParamInfo<std::string> & info) { return info.param; });

}  // namespace

TEST(MpcLateralControllerTest, IsNotReadyWithoutTrajectory)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto input = Input().following(Trajectory{}).planned_at(0.0).driving_at(0.0);

  const auto ready = controller.isReady(input);

  EXPECT_FALSE(ready);
}

TEST(MpcLateralControllerTest, IsNotReadyWithTwoPointTrajectory)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto two_points = make_trajectory({make_point(0.0, 0.0, 0.0), make_point(10.0, 0.0, 0.0)});
  const auto input = Input().following(two_points).planned_at(1.0).driving_at(1.0);

  const auto ready = controller.isReady(input);

  EXPECT_FALSE(ready);
}

TEST_P(MpcLateralControllerModelTest, IsReadyWithStraightTrajectory)
{
  auto config = make_default_config();
  config.vehicle_model_type = GetParam();
  MpcLateralController controller(config, no_log);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto ready = controller.isReady(input);

  EXPECT_TRUE(ready);
}

TEST_P(MpcLateralControllerModelTest, LeftCurveCommandsPositiveSteering)
{
  auto config = make_default_config();
  config.vehicle_model_type = GetParam();
  MpcLateralController controller(config, no_log);
  const auto input = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_GT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST_P(MpcLateralControllerModelTest, RightCurveCommandsNegativeSteering)
{
  auto config = make_default_config();
  config.vehicle_model_type = GetParam();
  MpcLateralController controller(config, no_log);
  const auto input = Input().following(right_curve_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// These two tests check steering angle more precisely in kinetics models
/// A vehicle of wheelbase L held at a steering angle drives a circle of radius
/// L / tan(angle), so a vehicle already on a circle of radius R stays on it at one angle
/// only, atan(L / R). That is the angle to command once the vehicle sits on the circle and
/// its steering has reached the commanded angle.
///
/// These tests don't check the gains(i.e. mpc_weight_*) because the optimization reaches a cost of
/// zero regardless of them.
///
/// The dynamics model is intentionally excluded: its calculation nature requires a simulated
/// vehicle and thus it's out of scope of unit test.
TEST(MpcLateralControllerTest, KinematicsModelSteersAnArcAtTheAngleTheRadiusRequires)
{
  auto config = make_default_config();
  config.vehicle_model_type = "kinematics";
  MpcLateralController controller(config, no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);

  const auto command = settle_on_arc(controller, clock, arc_settling_cycles);

  const double angle_holding_the_circle = std::atan(wheel_base / arc_radius);
  EXPECT_NEAR(command, angle_holding_the_circle, arc_relative_tolerance * angle_holding_the_circle);
}

TEST(MpcLateralControllerTest, KinematicsNoDelayModelSteersAnArcAtTheAngleTheRadiusRequires)
{
  auto config = make_default_config();
  config.vehicle_model_type = "kinematics_no_delay";
  MpcLateralController controller(config, no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);

  const auto command = settle_on_arc(controller, clock, arc_settling_cycles);

  const double angle_holding_the_circle = std::atan(wheel_base / arc_radius);
  EXPECT_NEAR(command, angle_holding_the_circle, arc_relative_tolerance * angle_holding_the_circle);
}

/// use_steer_prediction selects where the optimisation gets its initial angle. One source
/// is the measurement. The other is an estimate derived from the commands already issued.
/// The measured angle here is away from zero, and the commands issued along a straight path
/// leave the estimate at zero, so the two sources disagree.
TEST(MpcLateralControllerTest, SteerPredictionTakesTheInitialAngleFromTheCommandHistory)
{
  auto config = make_default_config();
  config.use_steer_prediction = true;
  MpcLateralController controller(config, no_log);
  const auto input =
    Input().following(straight_path()).planned_at(1.0).driving_at(1.0).steering_at(0.2);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
}

/// The steering offset is the calibration bias of the steering system, supplied from
/// outside the controller. The controller adds the offset to the measured angle. The
/// vehicle therefore appears turned by that amount, and the command turns the other way.
/// Without an offset, the command on this straight path is zero.
TEST(MpcLateralControllerTest, PositiveSteeringOffsetTurnsTheCommandNegative)
{
  MpcLateralController controller(make_default_config(), no_log);
  controller.set_steering_offset(0.5 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST(MpcLateralControllerTest, NegativeSteeringOffsetTurnsTheCommandPositive)
{
  MpcLateralController controller(make_default_config(), no_log);
  controller.set_steering_offset(-0.5 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_GT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// The controller applies at most steer_offset_max_update_th of offset during one cycle.
/// An offset below that limit reaches the command in full. An offset above it moves the
/// command by the limit and no more. These two tests use an offset below and above the
/// limit.
TEST(MpcLateralControllerTest, SteeringOffsetBelowTheUpdateLimitReachesTheCommandInFull)
{
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  MpcLateralController at_limit(make_default_config(), no_log);
  at_limit.set_steering_offset(steer_offset_max_update_th);
  const auto command_at_limit = at_limit.run(input, at(0.0)).output.control_cmd.steering_tire_angle;

  MpcLateralController below_limit(make_default_config(), no_log);
  below_limit.set_steering_offset(0.5 * steer_offset_max_update_th);

  const auto output = below_limit.run(input, at(0.0)).output;

  EXPECT_GT(output.control_cmd.steering_tire_angle, command_at_limit);
}

TEST(MpcLateralControllerTest, SteeringOffsetPastTheUpdateLimitIsCappedWithinOneCycle)
{
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  MpcLateralController at_limit(make_default_config(), no_log);
  at_limit.set_steering_offset(steer_offset_max_update_th);
  const auto command_at_limit = at_limit.run(input, at(0.0)).output.control_cmd.steering_tire_angle;

  MpcLateralController past_limit(make_default_config(), no_log);
  past_limit.set_steering_offset(2.0 * steer_offset_max_update_th);

  const auto output = past_limit.run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, command_at_limit);
}

/// The limit applies to one cycle, not to the offset as a whole. The controller therefore
/// applies more of a large offset on each cycle that follows.
TEST(MpcLateralControllerTest, SteeringOffsetPastTheUpdateLimitIsAppliedFurtherOnLaterCycles)
{
  MpcLateralController controller(make_default_config(), no_log);
  controller.set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto first_cycle =
    controller.run(input, at(clock.next_cycle())).output.control_cmd.steering_tire_angle;

  const auto output = controller.run(input, at(clock.next_cycle())).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, first_cycle);
}

/// With the automatic removal disabled, the controller ignores the offset it is given.
/// The command is then the same as the command for a controller given no offset.
TEST(MpcLateralControllerTest, SteeringOffsetIsDroppedWhileAutomaticRemovalIsOff)
{
  auto config = make_default_config();
  config.enable_auto_steering_offset_removal = false;
  MpcLateralController controller(config, no_log);
  controller.set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
}

/// is_steer_converged tells the longitudinal controller that the vehicle has settled on the
/// angle it was asked for, which is what lets it turn the steering off while stopped. The
/// decision needs the command record to cover convergence_history_sec. These two tests
/// use a record shorter and longer than that duration, which records where the answer
/// changes.
TEST(MpcLateralControllerTest, SteeringIsNotConvergedBeforeTheHistoryWindowIsFilled)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  const int cycles_short_of_history = cycles_spanning(convergence_history_sec) - 1;  // 34
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);

  const auto output = run_cycles(controller, input, clock, cycles_short_of_history);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

TEST(MpcLateralControllerTest, SteeringIsConvergedOnceTheHistoryWindowIsFilled)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);

  const auto output = run_cycles(controller, input, clock, cycles_filling_history);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

/// The controller compares each path it is given against the ones it still holds, and
/// treats the path as a new shape when the far end has moved further than
/// new_traj_end_dist. A new shape withdraws the convergence report until the path settles.
TEST(MpcLateralControllerTest, PathEndExtendedWithinTheThresholdKeepsTheConvergenceReport)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto original = straight_path_extended_by(0.0);
  const auto barely_extended = straight_path_extended_by(0.9 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  follow_path_for_cycles(controller, original, clock, cycles_filling_history);

  const auto output = follow_path_for_cycles(controller, barely_extended, clock, 1);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

TEST(MpcLateralControllerTest, PathEndExtendedBeyondTheThresholdWithdrawsTheConvergenceReport)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  follow_path_for_cycles(controller, original, clock, cycles_filling_history);

  const auto output = follow_path_for_cycles(controller, extended, clock, 1);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

/// The controller drops each path once the newest path it keeps is new_traj_duration_time
/// later. Only then does the controller stop comparing against the old shape. These two
/// tests use a duration shorter and longer than that point.
TEST(MpcLateralControllerTest, PathShapeChangeIsStillRememberedBeforeTheRetentionTime)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const int cycles_after_change = cycles_to_forget_shape_change() - 1;          // 32
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  follow_path_for_cycles(controller, original, clock, cycles_filling_history);
  follow_path_for_cycles(controller, extended, clock, 1);

  const auto output = follow_path_for_cycles(controller, extended, clock, cycles_after_change);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

TEST(MpcLateralControllerTest, PathShapeChangeIsForgottenAfterTheRetentionTime)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const int cycles_after_change = cycles_to_forget_shape_change();              // 33
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  follow_path_for_cycles(controller, original, clock, cycles_filling_history);
  follow_path_for_cycles(controller, extended, clock, 1);

  const auto output = follow_path_for_cycles(controller, extended, clock, cycles_after_change);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

/// While the vehicle is stopped the controller repeats the command it kept from the last
/// cycle instead of steering. It does so only once it accepts that the vehicle has reached
/// that command. keep_steer_control_until_converged decides whether that acceptance is
/// required, and converged_steer_rad sets the distance below which the angle counts as
/// reached. Two of these tests place the measured angle inside and outside that distance.
/// The third repeats the outside case with the guard disabled.
TEST(MpcLateralControllerTest, StopStateHoldsTheCommandWhenTheMeasuredAngleIsWithinTheThreshold)
{
  MpcLateralController controller(make_default_config(), no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const auto measured = static_cast<float>(settled - 0.9 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller.run(stopped, at(clock.next_cycle())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST(MpcLateralControllerTest, StopStateIsNotEnteredWhenTheMeasuredAngleIsOutsideTheThreshold)
{
  MpcLateralController controller(make_default_config(), no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const auto measured = static_cast<float>(settled - 1.1 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller.run(stopped, at(clock.next_cycle())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

TEST(MpcLateralControllerTest, StopStateIgnoresTheMeasuredAngleWhileTheGuardIsOff)
{
  auto config = make_default_config();
  config.keep_steer_control_until_converged = false;
  MpcLateralController controller(config, no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const auto measured = static_cast<float>(settled - 1.1 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller.run(stopped, at(clock.next_cycle())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

/// The vehicle counts as stopped while its measured speed does not exceed
/// stop_state_entry_ego_speed. The threshold value itself still counts as stopped, so
/// these two tests sit at the threshold and one step above it.
TEST(MpcLateralControllerTest, StopStateIsEnteredAtTheEgoSpeedThreshold)
{
  MpcLateralController controller(make_default_config(), no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(stop_state_entry_ego_speed);

  const auto output = controller.run(stopped, at(clock.next_cycle())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST(MpcLateralControllerTest, StopStateIsNotEnteredJustAboveTheEgoSpeedThreshold)
{
  MpcLateralController controller(make_default_config(), no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const InputData moving = Input()
                             .following(left_curve_path())
                             .planned_at(0.0)
                             .driving_at(stop_state_entry_ego_speed + speed_step);

  const auto output = controller.run(moving, at(clock.next_cycle())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

/// The path counts as asking for a stop while its target speed stays below
/// stop_state_entry_target_speed. Unlike the measured speed, the threshold value itself
/// does not count, so these two tests sit at the threshold and one step below it.
TEST(MpcLateralControllerTest, StopStateIsEnteredJustBelowTheTargetSpeedThreshold)
{
  MpcLateralController controller(make_default_config(), no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const InputData stopping = Input()
                               .following(left_curve_path())
                               .planned_at(stop_state_entry_target_speed - speed_step)
                               .driving_at(0.0);

  const auto output = controller.run(stopping, at(clock.next_cycle())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST(MpcLateralControllerTest, StopStateIsNotEnteredAtTheTargetSpeedThreshold)
{
  MpcLateralController controller(make_default_config(), no_log);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto settled = settle_on_curve(controller, clock, 5);
  const InputData driving =
    Input().following(left_curve_path()).planned_at(stop_state_entry_target_speed).driving_at(0.0);

  const auto output = controller.run(driving, at(clock.next_cycle())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

/// While Autoware is not driving the vehicle the controller drops the command it kept from
/// the previous cycle. The command from the last engaged cycle therefore does not persist.
TEST(MpcLateralControllerTest, CommandIsResetWhileNotUnderAutowareControl)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto engaged = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto steered =
    controller.run(engaged, at(clock.next_cycle())).output.control_cmd.steering_tire_angle;
  const InputData disengaged =
    Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0).without_autoware_control();

  const auto output = controller.run(disengaged, at(clock.next_cycle())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, steered);
}

/// Autoware drives the vehicle only in the autonomous operation mode. Any other mode
/// counts as not being in control, the same as the enable flag being off.
TEST(MpcLateralControllerTest, CommandIsResetWhileTheOperationModeIsNotAutonomous)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto engaged = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);
  auto clock = CycleClock().first_at(0.0).every(ctrl_period);
  const auto steered =
    controller.run(engaged, at(clock.next_cycle())).output.control_cmd.steering_tire_angle;
  const InputData stopped_mode =
    Input()
      .following(left_curve_path())
      .planned_at(1.0)
      .driving_at(1.0)
      .in_operation_mode(autoware_adapi_v1_msgs::msg::OperationModeState::STOP);

  const auto output = controller.run(stopped_mode, at(clock.next_cycle())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, steered);
}

/// Losing control clears the value the optimisation kept from the previous cycle. The
/// first cycle after control returns therefore starts from the measured angle, not from
/// the value the command had reached.
TEST(MpcLateralControllerTest, TheCycleAfterControlReturnsDiffersFromAnUninterruptedOne)
{
  const auto driving = Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0);
  const InputData disengaged =
    Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0).without_autoware_control();
  auto uninterrupted_clock = CycleClock().first_at(0.0).every(ctrl_period);
  MpcLateralController uninterrupted(make_default_config(), no_log);
  settle_on_curve(uninterrupted, uninterrupted_clock, 5);
  uninterrupted.run(driving, at(uninterrupted_clock.next_cycle()));
  const auto without_gap = uninterrupted.run(driving, at(uninterrupted_clock.next_cycle()))
                             .output.control_cmd.steering_tire_angle;

  auto interrupted_clock = CycleClock().first_at(0.0).every(ctrl_period);
  MpcLateralController interrupted(make_default_config(), no_log);
  settle_on_curve(interrupted, interrupted_clock, 5);
  interrupted.run(disengaged, at(interrupted_clock.next_cycle()));

  const auto output = interrupted.run(driving, at(interrupted_clock.next_cycle())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, without_gap);
}

/// The temporal reference mode follows the path by the time stamped on each point, and
/// therefore requires those times to increase along the path. One test meets that
/// requirement and one does not.
TEST(MpcLateralControllerTest, TemporalReferenceModeAcceptsIncreasingTimeFromStart)
{
  auto config = make_default_config();
  config.use_temporal_trajectory = true;
  MpcLateralController controller(config, no_log);
  const auto input =
    Input().following(straight_path_with_time_from_start(true)).planned_at(1.0).driving_at(1.0);

  const auto ready = controller.isReady(input);

  EXPECT_TRUE(ready);
}

TEST(MpcLateralControllerTest, TemporalReferenceModeRejectsRepeatedTimeFromStart)
{
  auto config = make_default_config();
  config.use_temporal_trajectory = true;
  MpcLateralController controller(config, no_log);
  const auto input =
    Input().following(straight_path_with_time_from_start(false)).planned_at(1.0).driving_at(1.0);

  const auto ready = controller.isReady(input);

  EXPECT_FALSE(ready);
}

/// A path the vehicle is meant to reverse along carries negative target speeds. The same
/// left-hand curve then has to be steered the other way.
TEST(MpcLateralControllerTest, BackwardPathReversesTheSteeringDirection)
{
  MpcLateralController controller(make_default_config(), no_log);
  const auto input = Input().following(left_curve_path()).planned_at(-1.0).driving_at(-1.0);

  const auto output = controller.run(input, at(0.0)).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}
