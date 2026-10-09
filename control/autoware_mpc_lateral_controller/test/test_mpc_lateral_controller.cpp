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
#include <memory>
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

/// The times of a run of control cycles. Each call names the value it sets, so a test
/// states the time of the first cycle, the interval and the number of cycles.
class Cycles
{
public:
  /// Time of the first cycle [s].
  Cycles & first_at(const double seconds)
  {
    first_ = seconds;
    return *this;
  }

  /// Interval between two cycles [s].
  Cycles & every(const double seconds)
  {
    interval_ = seconds;
    return *this;
  }

  /// Number of cycles.
  Cycles & count(const int cycles)
  {
    count_ = cycles;
    return *this;
  }

  int size() const { return count_; }

  /// Time of the cycle at the given position, counted from zero [s].
  double time_of(const int cycle) const { return first_ + cycle * interval_; }

  /// Time of the last cycle [s].
  double last() const { return time_of(count_ - 1); }

  /// Time of the cycle that would follow the last one [s].
  double next() const { return time_of(count_); }

private:
  double first_{0.0};
  double interval_{0.0};
  int count_{0};
};

/// The settings a test varies when it builds the controller. make_config gives everything
/// else the value the shipped parameter files carry.
struct ControllerOptions
{
  /// Selects the model the optimisation predicts with: kinematics, kinematics_no_delay
  /// or dynamics.
  std::string vehicle_model_type = "kinematics";

  /// Derives the angle the optimisation starts from an estimate of the commands already
  /// issued, instead of from the measurement.
  bool use_steer_prediction = false;

  /// Keeps the controller steering while stopped until it accepts that the vehicle has
  /// reached the angle it was asked for.
  bool keep_steer_control_until_converged = true;

  /// Selects how the reference path is followed: by the time stamped on each point when
  /// true, or by distance along the path when false.
  bool use_temporal_trajectory = false;

  /// Lets the controller apply the steering offset it is given. With this setting off, the
  /// offset is dropped and never reaches the command.
  bool enable_auto_steering_offset_removal = true;
};

constexpr double deg2rad = M_PI / 180.0;

class MpcLateralControllerTest : public ::testing::Test
{
protected:
  static constexpr double wheel_base = 2.74;
  static constexpr double ctrl_period = 0.03;

  /// Distance between the commanded and the measured angle below which the controller calls
  /// the steering converged. Carried by the shipped parameter file as converged_steer_rad.
  static constexpr double converged_steer_rad = 0.1;

  /// The controller only decides that the command has settled once it holds this much of
  /// the command record. Written into the controller rather than read from a parameter.
  static constexpr double convergence_history_sec = 1.0;

  /// Most of the steering offset the controller applies during one cycle. Carried by the
  /// shipped parameter file as steering_offset.max_update_th.
  static constexpr double steer_offset_max_update_th = 0.01;

  /// How far the end of the path has to move before the controller treats it as a new
  /// shape. Carried by the shipped parameter file as new_traj_end_dist.
  static constexpr double new_traj_end_dist = 0.3;

  /// How long the controller keeps each path it has been given, so that it can compare a
  /// later shape against that path. Carried by the shipped parameter file as
  /// new_traj_duration_time.
  static constexpr double new_traj_duration_time = 1.0;

  /// Measured speed at or below which the vehicle counts as stopped. Carried by the
  /// shipped parameter file as stop_state_entry_ego_speed.
  static constexpr double stop_state_entry_ego_speed = 0.001;

  /// Target speed below which the path counts as asking for a stop. Carried by the
  /// shipped parameter file as stop_state_entry_target_speed.
  static constexpr double stop_state_entry_target_speed = 0.001;

  /// A speed step small enough to land on the other side of either stop threshold.
  static constexpr double speed_step = 0.0001;

  /// Radius of the circle the steady state steering tests follow [m]. The controller
  /// replaces a feed-forward angle below mpc_zero_ff_steer_deg (0.5 deg) with zero, which
  /// this wheelbase reaches at a radius of about 314 m, so the radius stays well below
  /// that.
  static constexpr double arc_radius = 30.0;

  /// Speed those tests drive the circle at [m/s]. The weight on the heading error grows
  /// with the square of the speed, so at a high speed the optimisation arrives near the
  /// same angle even without the feed-forward that decides it. A low speed keeps the two
  /// apart.
  static constexpr double arc_speed = 1.0;

  /// Cycles those tests run. The command reaches the settled angle in about twenty cycles.
  /// The limit on how far the command may move during one cycle sets that number, not the
  /// optimisation.
  static constexpr int arc_settling_cycles = 60;

  /// How far the settled command may differ from the angle the geometry gives. A
  /// controller that approximates tan(angle) by the angle itself commands L / R instead of
  /// atan(L / R). The two differ by 0.3 per cent at the radius above, so this margin
  /// accepts such a controller as well.
  static constexpr double arc_relative_tolerance = 0.01;

  /// Cycles needed for the command record to cover the given duration. The record gains
  /// one entry per cycle, so n cycles cover (n - 1) control periods.
  static int cycles_spanning(const double seconds)
  {
    return static_cast<int>(std::ceil(seconds / ctrl_period)) + 1;
  }

  /// Cycles the controller takes to stop reporting a shape change. It stops once it has
  /// dropped the last path of the old shape, and that path arrived one control period
  /// before the changed one.
  static int cycles_to_forget_shape_change()
  {
    return static_cast<int>(std::ceil((new_traj_duration_time - ctrl_period) / ctrl_period));
  }

  /// The values the shipped parameter files carry, except for the settings the test
  /// changes. The vehicle values are the ones the vehicle description package provides in
  /// production.
  static MpcLateralControllerConfig make_config(const ControllerOptions & controller_options)
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
    config.keep_steer_control_until_converged =
      controller_options.keep_steer_control_until_converged;
    config.new_traj_duration_time = new_traj_duration_time;
    config.new_traj_end_dist = new_traj_end_dist;
    config.mpc_converged_threshold_rps = 0.01;

    config.ego_nearest_dist_threshold = 3.0;
    config.ego_nearest_yaw_threshold = 1.046;

    config.enable_auto_steering_offset_removal =
      controller_options.enable_auto_steering_offset_removal;
    config.steer_offset_max_update_th = steer_offset_max_update_th;
    config.steer_offset_filter_cutoff_hz = 0.5;

    config.steering_lpf_cutoff_hz = 3.0;
    config.error_deriv_lpf_cutoff_hz = 5.0;

    config.wheelbase = wheel_base;
    config.steer_lim = 0.70;
    config.vehicle_model_type = controller_options.vehicle_model_type;
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
    config.use_steer_prediction = controller_options.use_steer_prediction;
    config.use_delayed_initial_state = true;
    config.use_temporal_trajectory = controller_options.use_temporal_trajectory;
    config.publish_debug_trajectories = true;
    return config;
  }

  std::unique_ptr<MpcLateralController> make_controller(const MpcLateralControllerConfig & config)
  {
    return std::make_unique<MpcLateralController>(config, no_log);
  }

  std::unique_ptr<MpcLateralController> make_controller(
    const ControllerOptions & controller_options = ControllerOptions{})
  {
    return make_controller(make_config(controller_options));
  }

  /// Some of the behaviours below only appear after the controller has run for a while,
  /// because the controller decides them from a record of earlier cycles. The controller
  /// keeps two such records. It reads a different time for each record when it decides
  /// which entries are too old to keep, so there is one helper per record.
  ///
  /// This helper repeats an input the test has assembled, once at each time of the given
  /// cycles. Use it for the record of commands, because the controller compares the times
  /// of the cycles to decide which commands are too old to keep.
  LateralOutput run_cycles(
    MpcLateralController & controller, const InputData & input, const Cycles & cycles)
  {
    LateralOutput output;
    for (int cycle = 0; cycle < cycles.size(); ++cycle) {
      output = controller.run(input, at(cycles.time_of(cycle))).output;
    }
    return output;
  }

  /// This helper follows a path once at each time of the given cycles, and writes that time
  /// to the path as its stamp. Use it for the record of paths. To decide which paths are
  /// too old to keep, the controller compares the stamps on the paths it keeps instead of
  /// the times of the cycles. A path whose stamp is still zero is therefore never dropped.
  ///
  /// The vehicle follows the path at 1 m/s and reports its steering as straight ahead. No
  /// test that uses this helper needs it anywhere else.
  LateralOutput follow_path_for_cycles(
    MpcLateralController & controller, const Trajectory & path, const Cycles & cycles)
  {
    LateralOutput output;
    for (int cycle = 0; cycle < cycles.size(); ++cycle) {
      const double time = cycles.time_of(cycle);
      output =
        controller
          .run(Input().following(path).planned_at(1.0).driving_at(1.0).stamped_at(time), at(time))
          .output;
    }
    return output;
  }

  /// Drive a curve at speed until the command stops moving, and report the command reached.
  /// Five cycles are enough for the low pass filters inside the controller to settle.
  float settle_on_curve(MpcLateralController & controller, const Cycles & cycles)
  {
    const InputData driving = Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0);
    float command = 0.0f;
    for (int cycle = 0; cycle < cycles.size(); ++cycle) {
      command =
        controller.run(driving, at(cycles.time_of(cycle))).output.control_cmd.steering_tire_angle;
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
  float settle_on_arc(MpcLateralController & controller, const Cycles & cycles)
  {
    const Trajectory path = arc_path(arc_radius);
    float command = 0.0f;
    for (int cycle = 0; cycle < cycles.size(); ++cycle) {
      const InputData driving =
        Input().following(path).planned_at(arc_speed).driving_at(arc_speed).steering_at(command);
      command =
        controller.run(driving, at(cycles.time_of(cycle))).output.control_cmd.steering_tire_angle;
    }
    return command;
  }
};

/// The controller offers three vehicle models. Each one has to accept the same input and
/// steer to the same side, so the shared expectations run against all of them.
class MpcLateralControllerModelTest : public MpcLateralControllerTest,
                                      public ::testing::WithParamInterface<std::string>
{
};

INSTANTIATE_TEST_SUITE_P(
  VehicleModels, MpcLateralControllerModelTest,
  ::testing::Values("kinematics", "kinematics_no_delay", "dynamics"),
  [](const ::testing::TestParamInfo<std::string> & info) { return info.param; });

}  // namespace

TEST_F(MpcLateralControllerTest, IsNotReadyWithoutTrajectory)
{
  auto controller = make_controller();
  const auto input = Input().following(Trajectory{}).planned_at(0.0).driving_at(0.0);

  const auto ready = controller->isReady(input);

  EXPECT_FALSE(ready);
}

TEST_F(MpcLateralControllerTest, IsNotReadyWithTwoPointTrajectory)
{
  auto controller = make_controller();
  const auto two_points = make_trajectory({make_point(0.0, 0.0, 0.0), make_point(10.0, 0.0, 0.0)});
  const auto input = Input().following(two_points).planned_at(1.0).driving_at(1.0);

  const auto ready = controller->isReady(input);

  EXPECT_FALSE(ready);
}

TEST_P(MpcLateralControllerModelTest, IsReadyWithStraightTrajectory)
{
  ControllerOptions options;
  options.vehicle_model_type = GetParam();
  auto controller = make_controller(options);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto ready = controller->isReady(input);

  EXPECT_TRUE(ready);
}

TEST_P(MpcLateralControllerModelTest, StraightTrajectoryKeepsSteeringNeutral)
{
  ControllerOptions options;
  options.vehicle_model_type = GetParam();
  auto controller = make_controller(options);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
  EXPECT_FALSE(output.control_cmd_horizon.controls.empty());
}

TEST_P(MpcLateralControllerModelTest, LeftCurveCommandsPositiveSteering)
{
  ControllerOptions options;
  options.vehicle_model_type = GetParam();
  auto controller = make_controller(options);
  const auto input = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_GT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST_P(MpcLateralControllerModelTest, RightCurveCommandsNegativeSteering)
{
  ControllerOptions options;
  options.vehicle_model_type = GetParam();
  auto controller = make_controller(options);
  const auto input = Input().following(right_curve_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

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
TEST_F(MpcLateralControllerTest, KinematicsModelSteersAnArcAtTheAngleTheRadiusRequires)
{
  ControllerOptions options;
  options.vehicle_model_type = "kinematics";
  auto controller = make_controller(options);
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(arc_settling_cycles);

  const auto command = settle_on_arc(*controller, cycles);

  const double angle_holding_the_circle = std::atan(wheel_base / arc_radius);
  EXPECT_NEAR(command, angle_holding_the_circle, arc_relative_tolerance * angle_holding_the_circle);
}

TEST_F(MpcLateralControllerTest, KinematicsNoDelayModelSteersAnArcAtTheAngleTheRadiusRequires)
{
  ControllerOptions options;
  options.vehicle_model_type = "kinematics_no_delay";
  auto controller = make_controller(options);
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(arc_settling_cycles);

  const auto command = settle_on_arc(*controller, cycles);

  const double angle_holding_the_circle = std::atan(wheel_base / arc_radius);
  EXPECT_NEAR(command, angle_holding_the_circle, arc_relative_tolerance * angle_holding_the_circle);
}

/// use_steer_prediction selects where the optimisation gets its initial angle. One source
/// is the measurement. The other is an estimate derived from the commands already issued.
/// The measured angle here is away from zero, and the commands issued along a straight path
/// leave the estimate at zero, so the two sources disagree.
TEST_F(MpcLateralControllerTest, SteerPredictionTakesTheInitialAngleFromTheCommandHistory)
{
  ControllerOptions options;
  options.use_steer_prediction = true;
  auto controller = make_controller(options);
  const auto input =
    Input().following(straight_path()).planned_at(1.0).driving_at(1.0).steering_at(0.2);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
}

/// The steering offset is the calibration bias of the steering system, supplied from
/// outside the controller. The controller adds the offset to the measured angle. The
/// vehicle therefore appears turned by that amount, and the command turns the other way.
/// A zero offset leaves the command at the value StraightTrajectoryKeepsSteeringNeutral
/// asserts.
TEST_F(MpcLateralControllerTest, PositiveSteeringOffsetTurnsTheCommandNegative)
{
  auto controller = make_controller();
  controller->set_steering_offset(0.5 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST_F(MpcLateralControllerTest, NegativeSteeringOffsetTurnsTheCommandPositive)
{
  auto controller = make_controller();
  controller->set_steering_offset(-0.5 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_GT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// The controller applies at most steer_offset_max_update_th of offset during one cycle.
/// An offset below that limit reaches the command in full. An offset above it moves the
/// command by the limit and no more. These two tests use an offset below and above the
/// limit.
TEST_F(MpcLateralControllerTest, SteeringOffsetBelowTheUpdateLimitReachesTheCommandInFull)
{
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  auto at_limit = make_controller();
  at_limit->set_steering_offset(steer_offset_max_update_th);
  const auto command_at_limit =
    at_limit->run(input, at(0.0)).output.control_cmd.steering_tire_angle;

  auto below_limit = make_controller();
  below_limit->set_steering_offset(0.5 * steer_offset_max_update_th);

  const auto output = below_limit->run(input, at(0.0)).output;

  EXPECT_GT(output.control_cmd.steering_tire_angle, command_at_limit);
}

TEST_F(MpcLateralControllerTest, SteeringOffsetPastTheUpdateLimitIsCappedWithinOneCycle)
{
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  auto at_limit = make_controller();
  at_limit->set_steering_offset(steer_offset_max_update_th);
  const auto command_at_limit =
    at_limit->run(input, at(0.0)).output.control_cmd.steering_tire_angle;

  auto past_limit = make_controller();
  past_limit->set_steering_offset(2.0 * steer_offset_max_update_th);

  const auto output = past_limit->run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, command_at_limit);
}

/// The limit applies to one cycle, not to the offset as a whole. The controller therefore
/// applies more of a large offset on each cycle that follows.
TEST_F(MpcLateralControllerTest, SteeringOffsetPastTheUpdateLimitIsAppliedFurtherOnLaterCycles)
{
  auto controller = make_controller();
  controller->set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  const auto first_cycle = controller->run(input, at(0.0)).output.control_cmd.steering_tire_angle;

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, first_cycle);
}

/// With the automatic removal disabled, the controller ignores the offset it is given.
/// The command is then the same as the command for a controller given no offset.
TEST_F(MpcLateralControllerTest, SteeringOffsetIsDroppedWhileAutomaticRemovalIsOff)
{
  ControllerOptions options;
  options.enable_auto_steering_offset_removal = false;
  auto controller = make_controller(options);
  controller->set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
}

/// is_steer_converged tells the longitudinal controller that the vehicle has settled on the
/// angle it was asked for, which is what lets it turn the steering off while stopped. The
/// decision needs the command record to cover convergence_history_sec. These two tests
/// use a record shorter and longer than that duration, which records where the answer
/// changes.
TEST_F(MpcLateralControllerTest, SteeringIsNotConvergedBeforeTheHistoryWindowIsFilled)
{
  auto controller = make_controller();
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  const int cycles_short_of_history = cycles_spanning(convergence_history_sec) - 1;  // 34
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(cycles_short_of_history);

  const auto output = run_cycles(*controller, input, cycles);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

TEST_F(MpcLateralControllerTest, SteeringIsConvergedOnceTheHistoryWindowIsFilled)
{
  auto controller = make_controller();
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(cycles_filling_history);

  const auto output = run_cycles(*controller, input, cycles);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

/// The controller compares each path it is given against the ones it still holds, and
/// treats the path as a new shape when the far end has moved further than
/// new_traj_end_dist. A new shape withdraws the convergence report until the path settles.
TEST_F(MpcLateralControllerTest, PathEndExtendedWithinTheThresholdKeepsTheConvergenceReport)
{
  auto controller = make_controller();
  const auto original = straight_path_extended_by(0.0);
  const auto barely_extended = straight_path_extended_by(0.9 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const auto following_original =
    Cycles().first_at(0.0).every(ctrl_period).count(cycles_filling_history);
  const auto following_extended =
    Cycles().first_at(following_original.next()).every(ctrl_period).count(1);
  follow_path_for_cycles(*controller, original, following_original);

  const auto output = follow_path_for_cycles(*controller, barely_extended, following_extended);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

TEST_F(MpcLateralControllerTest, PathEndExtendedBeyondTheThresholdWithdrawsTheConvergenceReport)
{
  auto controller = make_controller();
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const auto following_original =
    Cycles().first_at(0.0).every(ctrl_period).count(cycles_filling_history);
  const auto following_extended =
    Cycles().first_at(following_original.next()).every(ctrl_period).count(1);
  follow_path_for_cycles(*controller, original, following_original);

  const auto output = follow_path_for_cycles(*controller, extended, following_extended);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

/// The controller drops each path once the newest path it keeps is new_traj_duration_time
/// later. Only then does the controller stop comparing against the old shape. These two
/// tests use a duration shorter and longer than that point.
TEST_F(MpcLateralControllerTest, PathShapeChangeIsStillRememberedBeforeTheRetentionTime)
{
  auto controller = make_controller();
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const auto following_original =
    Cycles().first_at(0.0).every(ctrl_period).count(cycles_filling_history);
  const auto following_extended =
    Cycles().first_at(following_original.next()).every(ctrl_period).count(1);
  const int cycles_after_change = cycles_to_forget_shape_change() - 1;  // 32
  const auto following_further =
    Cycles().first_at(following_extended.next()).every(ctrl_period).count(cycles_after_change);
  follow_path_for_cycles(*controller, original, following_original);
  follow_path_for_cycles(*controller, extended, following_extended);

  const auto output = follow_path_for_cycles(*controller, extended, following_further);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

TEST_F(MpcLateralControllerTest, PathShapeChangeIsForgottenAfterTheRetentionTime)
{
  auto controller = make_controller();
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  const int cycles_filling_history = cycles_spanning(convergence_history_sec);  // 35
  const auto following_original =
    Cycles().first_at(0.0).every(ctrl_period).count(cycles_filling_history);
  const auto following_extended =
    Cycles().first_at(following_original.next()).every(ctrl_period).count(1);
  const int cycles_after_change = cycles_to_forget_shape_change();  // 33
  const auto following_further =
    Cycles().first_at(following_extended.next()).every(ctrl_period).count(cycles_after_change);
  follow_path_for_cycles(*controller, original, following_original);
  follow_path_for_cycles(*controller, extended, following_extended);

  const auto output = follow_path_for_cycles(*controller, extended, following_further);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

/// While the vehicle is stopped the controller repeats the command it kept from the last
/// cycle instead of steering. It does so only once it accepts that the vehicle has reached
/// that command. keep_steer_control_until_converged decides whether that acceptance is
/// required, and converged_steer_rad sets the distance below which the angle counts as
/// reached. Two of these tests place the measured angle inside and outside that distance.
/// The third repeats the outside case with the guard disabled.
TEST_F(MpcLateralControllerTest, StopStateHoldsTheCommandWhenTheMeasuredAngleIsWithinTheThreshold)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const auto measured = static_cast<float>(settled - 0.9 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller->run(stopped, at(cycles.last())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIsNotEnteredWhenTheMeasuredAngleIsOutsideTheThreshold)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const auto measured = static_cast<float>(settled - 1.1 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller->run(stopped, at(cycles.last())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIgnoresTheMeasuredAngleWhileTheGuardIsOff)
{
  ControllerOptions options;
  options.keep_steer_control_until_converged = false;
  auto controller = make_controller(options);
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const auto measured = static_cast<float>(settled - 1.1 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller->run(stopped, at(cycles.last())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

/// The vehicle counts as stopped while its measured speed does not exceed
/// stop_state_entry_ego_speed. The threshold value itself still counts as stopped, so
/// these two tests sit at the threshold and one step above it.
TEST_F(MpcLateralControllerTest, StopStateIsEnteredAtTheEgoSpeedThreshold)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(stop_state_entry_ego_speed);

  const auto output = controller->run(stopped, at(cycles.last())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIsNotEnteredJustAboveTheEgoSpeedThreshold)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const InputData moving = Input()
                             .following(left_curve_path())
                             .planned_at(0.0)
                             .driving_at(stop_state_entry_ego_speed + speed_step);

  const auto output = controller->run(moving, at(cycles.last())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

/// The path counts as asking for a stop while its target speed stays below
/// stop_state_entry_target_speed. Unlike the measured speed, the threshold value itself
/// does not count, so these two tests sit at the threshold and one step below it.
TEST_F(MpcLateralControllerTest, StopStateIsEnteredJustBelowTheTargetSpeedThreshold)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const InputData stopping = Input()
                               .following(left_curve_path())
                               .planned_at(stop_state_entry_target_speed - speed_step)
                               .driving_at(0.0);

  const auto output = controller->run(stopping, at(cycles.last())).output;

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIsNotEnteredAtTheTargetSpeedThreshold)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  const auto settled = settle_on_curve(*controller, cycles);
  const InputData driving =
    Input().following(left_curve_path()).planned_at(stop_state_entry_target_speed).driving_at(0.0);

  const auto output = controller->run(driving, at(cycles.last())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

/// While Autoware is not driving the vehicle the controller drops the command it kept from
/// the previous cycle. The command from the last engaged cycle therefore does not persist.
TEST_F(MpcLateralControllerTest, CommandIsResetWhileNotUnderAutowareControl)
{
  auto controller = make_controller();
  const auto engaged = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);
  const auto steered = controller->run(engaged, at(0.0)).output.control_cmd.steering_tire_angle;
  const InputData disengaged =
    Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0).without_autoware_control();

  const auto output = controller->run(disengaged, at(0.0)).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, steered);
}

/// Autoware drives the vehicle only in the autonomous operation mode. Any other mode
/// counts as not being in control, the same as the enable flag being off.
TEST_F(MpcLateralControllerTest, CommandIsResetWhileTheOperationModeIsNotAutonomous)
{
  auto controller = make_controller();
  const auto engaged = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);
  const auto steered = controller->run(engaged, at(0.0)).output.control_cmd.steering_tire_angle;
  const InputData stopped_mode =
    Input()
      .following(left_curve_path())
      .planned_at(1.0)
      .driving_at(1.0)
      .in_operation_mode(autoware_adapi_v1_msgs::msg::OperationModeState::STOP);

  const auto output = controller->run(stopped_mode, at(0.0)).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, steered);
}

/// Losing control clears the value the optimisation kept from the previous cycle. The
/// first cycle after control returns therefore starts from the measured angle, not from
/// the value the command had reached.
TEST_F(MpcLateralControllerTest, TheCycleAfterControlReturnsDiffersFromAnUninterruptedOne)
{
  const auto driving = Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0);
  const InputData disengaged =
    Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0).without_autoware_control();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  auto uninterrupted = make_controller();
  settle_on_curve(*uninterrupted, cycles);
  uninterrupted->run(driving, at(cycles.last()));
  const auto without_gap =
    uninterrupted->run(driving, at(cycles.last())).output.control_cmd.steering_tire_angle;

  auto interrupted = make_controller();
  settle_on_curve(*interrupted, cycles);
  interrupted->run(disengaged, at(cycles.last()));

  const auto output = interrupted->run(driving, at(cycles.last())).output;

  EXPECT_NE(output.control_cmd.steering_tire_angle, without_gap);
}

/// The temporal reference mode follows the path by the time stamped on each point, and
/// therefore requires those times to increase along the path. One test meets that
/// requirement and one does not.
TEST_F(MpcLateralControllerTest, TemporalReferenceModeAcceptsIncreasingTimeFromStart)
{
  ControllerOptions options;
  options.use_temporal_trajectory = true;
  auto controller = make_controller(options);
  const auto input =
    Input().following(straight_path_with_time_from_start(true)).planned_at(1.0).driving_at(1.0);

  const auto ready = controller->isReady(input);

  EXPECT_TRUE(ready);
}

TEST_F(MpcLateralControllerTest, TemporalReferenceModeRejectsRepeatedTimeFromStart)
{
  ControllerOptions options;
  options.use_temporal_trajectory = true;
  auto controller = make_controller(options);
  const auto input =
    Input().following(straight_path_with_time_from_start(false)).planned_at(1.0).driving_at(1.0);

  const auto ready = controller->isReady(input);

  EXPECT_FALSE(ready);
}

/// A path the vehicle is meant to reverse along carries negative target speeds. The same
/// left-hand curve then has to be steered the other way.
TEST_F(MpcLateralControllerTest, BackwardPathReversesTheSteeringDirection)
{
  auto controller = make_controller();
  const auto input = Input().following(left_curve_path()).planned_at(-1.0).driving_at(-1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// A vehicle model name the controller does not know leaves it without a model, so the
/// controller is never ready to run.
TEST_F(MpcLateralControllerTest, IsNotReadyWithAnUnknownVehicleModel)
{
  ControllerOptions options;
  options.vehicle_model_type = "not_a_model";
  auto controller = make_controller(options);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto ready = controller->isReady(input);

  EXPECT_FALSE(ready);
}

// Everything from here to the end of the file is removed together with the tests it
// supports. Each of those tests holds an expectation that a later change to the controller
// is meant to break, so none of them runs unless the test executable is given
// --gtest_also_run_disabled_tests. Nothing above this line uses what is declared below it.

namespace
{
/// A path built to make every tuning parameter of the controller reach the command.
/// It carries four features, each of which reaches a different group of parameters.
///
/// The path runs straight for three metres and then follows a circle of radius 30 m. The
/// controller picks the weight set of each point of the prediction from the curvature
/// there, so a path that is straight on one part and curved on another uses both sets in
/// the same cycle.
///
/// The target speed steps up part way along, which is what makes the controller smooth the
/// speed of the reference. A path whose target speed already matches the measured speed
/// leaves that smoothing with nothing to do.
///
/// The path is displaced from the map origin, where the ego stays, so the controller sees
/// a lateral and a heading error. Without an error the optimisation returns the
/// feed-forward angle and the weights do not reach the command at all.
Trajectory tuning_probe_path()
{
  constexpr double radius = 30.0;
  constexpr double straight_length = 3.0;
  constexpr double lateral_error = 0.5;
  constexpr double heading_error = 0.1;
  constexpr double speed_before_step = 3.0;
  constexpr double speed_after_step = 12.0;
  constexpr double step_distance = 4.0;

  std::vector<TrajectoryPoint> points;
  for (double distance = -3.0; distance <= 60.0; distance += 0.25) {
    const double angle = distance < straight_length ? 0.0 : (distance - straight_length) / radius;
    const double x =
      distance < straight_length ? distance : straight_length + radius * std::sin(angle);
    const double y = distance < straight_length ? 0.0 : radius * (1.0 - std::cos(angle));

    // Move the path away from the ego rather than the ego away from the path, so that the
    // ego pose stays where every other test in this file keeps it.
    const double placed_x = x * std::cos(heading_error) + y * std::sin(heading_error);
    const double placed_y =
      -x * std::sin(heading_error) + y * std::cos(heading_error) - lateral_error;

    auto point = make_point(placed_x, placed_y, angle - heading_error);
    point.longitudinal_velocity_mps =
      static_cast<float>(distance < step_distance ? speed_before_step : speed_after_step);
    points.push_back(point);
  }
  return make_trajectory(std::move(points));
}

}  // namespace

/// TO BE CONFIRMED: the steering offset is subtracted from the command but not from the
/// horizon. The first entry of the horizon therefore differs from the command sent in the
/// same cycle. The difference may be intentional. The offset corrects a bias in the
/// steering of the ego vehicle. The design may intend the horizon to carry no such
/// correction, because a reader may treat the horizon as a plan rather than as commands to
/// send. This test records the command and the horizon as they are now, so that a fix, or a
/// written statement of the intent, changes this test too.
///
/// A change that makes the two agree therefore fails this test, which is why it carries
/// the DISABLED_ prefix and runs only when the test executable is given
/// --gtest_also_run_disabled_tests. It is removed once the question above is settled.
TEST_F(MpcLateralControllerTest, DISABLED_SteeringOffsetReachesTheCommandButNotTheHorizon)
{
  auto controller = make_controller();
  controller->set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input, at(0.0)).output;

  EXPECT_NE(
    output.control_cmd.steering_tire_angle,
    output.control_cmd_horizon.controls.front().steering_tire_angle);
}

/// TO BE CONFIRMED: while the vehicle is stopped, the command is the one kept from the
/// previous cycle. The horizon is the optimisation result of this cycle, so the command and
/// the horizon describe different motions. The difference may be intentional. The horizon
/// may be meant to show what the controller would do after the vehicle starts to move
/// again. The command reports what the controller does while the vehicle is stopped. This
/// test records the command and the horizon as they are now.
///
/// A change that makes the two agree therefore fails this test, which is why it carries
/// the DISABLED_ prefix and runs only when the test executable is given
/// --gtest_also_run_disabled_tests. It is removed once the question above is settled.
TEST_F(MpcLateralControllerTest, DISABLED_StopStateHoldsTheCommandButNotTheHorizon)
{
  auto controller = make_controller();
  const auto cycles = Cycles().first_at(0.0).every(ctrl_period).count(5);
  settle_on_curve(*controller, cycles);
  const auto stopped = Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0);

  const auto output = controller->run(stopped, at(cycles.last())).output;

  EXPECT_NE(
    output.control_cmd.steering_tire_angle,
    output.control_cmd_horizon.controls.front().steering_tire_angle);
}

/// Every value the controller reads to tune the optimisation has to reach the command.
/// This test gives each of them a value of its own and pins the command that follows, so
/// that a value which stops reaching the command, or which is read into the wrong field,
/// changes the result.
///
/// The expected angles below carry no meaning of their own. They are what this controller
/// returns today, recorded so that a change becomes visible. Because they were read off a
/// run rather than derived from what the controller is meant to do, every change to the
/// optimisation moves them, and a correct change fails this test as surely as a mistake
/// does. The test therefore carries the DISABLED_ prefix and runs only when the test
/// executable is given --gtest_also_run_disabled_tests. It is removed once the values it
/// guards can be read directly.
///
/// Two values do not reach the command. low_curvature_weight.steer_rate and
/// low_curvature_weight.steer_acc are declared as parameters and can be set, but the code
/// that builds the weight on the steering rate and on the steering acceleration reads the
/// nominal pair whatever the curvature is, so nothing reads the two below the threshold.
TEST_F(MpcLateralControllerTest, DISABLED_EveryTuningParameterReachesTheCommand)
{
  auto config = make_config(ControllerOptions{});
  auto & mpc = config.mpc_param;
  // A different value for each, so that reading one into the field of another changes the
  // command.
  mpc.nominal_weight.lat_error = 0.11;
  mpc.nominal_weight.heading_error = 0.22;
  mpc.nominal_weight.heading_error_squared_vel = 0.33;
  mpc.nominal_weight.steering_input = 0.44;
  mpc.nominal_weight.steering_input_squared_vel = 0.55;
  mpc.nominal_weight.lat_jerk = 0.66;
  // The weight on the steering rate is divided by the square of the control period and
  // the one on the steering acceleration by its fourth power, so a value of the size the
  // others carry would raise these two terms above every other term by six orders of
  // magnitude and leave the command at zero. Both stay near the size the shipped files
  // give them.
  mpc.nominal_weight.steer_rate = 1.0e-3;
  mpc.nominal_weight.steer_acc = 3.0e-6;
  mpc.nominal_weight.terminal_lat_error = 0.99;
  mpc.nominal_weight.terminal_heading_error = 1.11;
  mpc.low_curvature_weight.lat_error = 1.22;
  mpc.low_curvature_weight.heading_error = 1.33;
  mpc.low_curvature_weight.heading_error_squared_vel = 1.44;
  mpc.low_curvature_weight.steering_input = 1.55;
  mpc.low_curvature_weight.steering_input_squared_vel = 1.66;
  mpc.low_curvature_weight.lat_jerk = 1.77;
  mpc.steer_tau = 0.23;
  mpc.prediction_dt = 0.09;
  config.input_delay = 0.15;
  mpc.zero_ff_steer_deg = 0.4;
  mpc.min_prediction_length = 4.3;
  mpc.velocity_time_constant = 2.5;
  mpc.acceleration_limit = 1.9;
  // The straight part of the path sits below this threshold and the curved part above
  // it, so the controller uses both weight sets within one prediction. The shipped value
  // is zero, which no curvature is below, so the set below the threshold is never used.
  mpc.low_curvature_thresh_curvature = 0.02;
  // A short prediction gives the weights on its last point a share of the command large
  // enough to see. Over the shipped fifty points that share falls to a ten thousandth of
  // the command, which a float no longer separates.
  mpc.prediction_horizon = 10;
  // The shipped solver stops at a tolerance, which leaves the last digits of its answer
  // free to move between releases of that solver and between machines. The other solver
  // the controller offers returns the answer of the same cost function directly, which
  // is what a recorded value needs. It also ignores the cap on how far the command may
  // move during one control period, and with the shipped cap that cap rather than the
  // cost function would decide the command of the first cycle.
  config.qp_solver_type = "unconstraint_fast";
  // The controller predicts over the larger of mpc_prediction_dt and a step it derives
  // from mpc_min_prediction_length, so one cycle only shows whichever of the two is
  // larger. The first command below leaves the derived step larger, which is the case the
  // shipped values give, and the second raises mpc_prediction_dt above it.
  auto with_a_longer_step = config;
  with_a_longer_step.mpc_param.prediction_dt = 0.5;
  auto controller = make_controller(config);
  auto controller_with_a_longer_step = make_controller(with_a_longer_step);
  const auto input = Input().following(tuning_probe_path()).driving_at(3.0);

  const auto command = controller->run(input, at(0.0)).output.control_cmd.steering_tire_angle;
  const auto command_with_a_longer_step =
    controller_with_a_longer_step->run(input, at(0.0)).output.control_cmd.steering_tire_angle;

  EXPECT_FLOAT_EQ(command, -0.002773088f);
  EXPECT_FLOAT_EQ(command_with_a_longer_step, -0.0034486512f);
}
