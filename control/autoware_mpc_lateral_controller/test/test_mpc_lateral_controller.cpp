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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::motion::control::mpc_lateral_controller::MpcLateralController;
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerBase;
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

Trajectory make_trajectory(
  const TrajectoryPoint & first, const TrajectoryPoint & second, const TrajectoryPoint & third)
{
  Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.points = {first, second, third};
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
    make_point(0.0, 0.0, 0.0), make_point(10.0, 0.0, 0.0), make_point(20.0, 0.0, 0.0));
}

Trajectory left_curve_path()
{
  return make_trajectory(
    make_point(0.0, 0.0, 0.0), make_point(9.9, 1.0, 0.2), make_point(19.4, 4.0, 0.4));
}

Trajectory right_curve_path()
{
  return make_trajectory(
    make_point(0.0, 0.0, 0.0), make_point(9.9, -1.0, -0.2), make_point(19.4, -4.0, -0.4));
}

/// Extend a straight path by moving its far end further along, leaving the near end and
/// the heading where they were. The controller decides that a path has changed shape from
/// how far this end has moved. Extending the path moves that end without bending the path,
/// so the command the controller produces does not change.
Trajectory straight_path_extended_by(const double distance)
{
  return make_trajectory(
    make_point(0.0, 0.0, 0.0), make_point(10.0, 0.0, 0.0), make_point(20.0 + distance, 0.0, 0.0));
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

/// The settings a test varies when it builds the controller. Everything else is fixed by
/// the shipped parameter files.
struct ControllerOptions
{
  /// Selects the model the optimisation predicts with: kinematics, kinematics_no_delay
  /// or dynamics.
  std::string vehicle_model_type = "kinematics";

  /// Takes the angle the optimisation starts from out of an estimate built from the
  /// commands already issued, instead of out of the measurement.
  bool use_steer_prediction = false;

  /// Keeps the controller steering while stopped until it accepts that the vehicle has
  /// reached the angle it was asked for.
  bool keep_steer_control_until_converged = true;

  /// Lets the test drive the clock the controller reads, instead of the machine clock.
  bool use_simulated_time = false;

  /// Selects how the reference path is followed: spatial by distance along the path, or
  /// temporal by the time stamped on each point.
  std::string trajectory_reference_mode = "spatial";

  /// Lets the controller apply the steering offset it is given. With this setting off, the
  /// offset is dropped and never reaches the command.
  bool enable_auto_steering_offset_removal = true;
};

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

  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  /// Move the clock the node reads. Works only when the controller was built with
  /// simulated time, which is what lets a test set the time instead of the machine.
  void advance_clock(const double seconds)
  {
    clock_time_ns_ += static_cast<int64_t>(seconds * 1e9);
    const auto result =
      rcl_set_ros_time_override(node_->get_clock()->get_clock_handle(), clock_time_ns_);
    ASSERT_EQ(result, RCL_RET_OK);
  }

  rclcpp::NodeOptions make_node_options(const ControllerOptions & controller_options)
  {
    const auto share_dir =
      ament_index_cpp::get_package_share_directory("autoware_mpc_lateral_controller");

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/param/lateral_controller_defaults.param.yaml",
       "--params-file", share_dir + "/param/steer_offset.param.yaml"});

    // Provided by the trajectory follower node in production.
    options.append_parameter_override("steer_offset_param_name", "steer_offset");
    options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
    options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);
    options.append_parameter_override("vehicle_model_type", controller_options.vehicle_model_type);
    options.append_parameter_override(
      "use_steer_prediction", controller_options.use_steer_prediction);
    options.append_parameter_override(
      "keep_steer_control_until_converged", controller_options.keep_steer_control_until_converged);
    options.append_parameter_override(
      "trajectory_reference_mode", controller_options.trajectory_reference_mode);
    options.append_parameter_override(
      "steering_offset.enable_auto_steering_offset_removal",
      controller_options.enable_auto_steering_offset_removal);

    // Provided by the vehicle description package in production.
    options.append_parameter_override("wheel_radius", 0.39);
    options.append_parameter_override("wheel_width", 0.42);
    options.append_parameter_override("wheel_base", wheel_base);
    options.append_parameter_override("wheel_tread", 1.63);
    options.append_parameter_override("front_overhang", 1.0);
    options.append_parameter_override("rear_overhang", 1.03);
    options.append_parameter_override("left_overhang", 0.1);
    options.append_parameter_override("right_overhang", 0.1);
    options.append_parameter_override("vehicle_height", 2.5);
    options.append_parameter_override("max_steer_angle", 0.70);

    // Read by the dynamics vehicle model only.
    options.append_parameter_override("vehicle.mass_fl", 600.0);
    options.append_parameter_override("vehicle.mass_fr", 600.0);
    options.append_parameter_override("vehicle.mass_rl", 600.0);
    options.append_parameter_override("vehicle.mass_rr", 600.0);
    options.append_parameter_override("vehicle.cf", 155494.663);
    options.append_parameter_override("vehicle.cr", 155494.663);

    if (controller_options.use_simulated_time) {
      options.append_parameter_override("use_sim_time", true);
    }

    return options;
  }

  /// Build the controller behind the base class interface, so that the tests only
  /// exercise the interface the trajectory follower node uses.
  std::unique_ptr<LateralControllerBase> make_controller(
    const ControllerOptions & controller_options = ControllerOptions{})
  {
    // Kept alive for the whole test: a controller holds a reference to the node it was
    // built from, and a test may build several controllers and compare their commands.
    node_ = nodes_.emplace_back(
      std::make_shared<rclcpp::Node>("test_node", make_node_options(controller_options)));
    // The trajectory follower node declares this one before building the controller.
    node_->declare_parameter<double>("ctrl_period", ctrl_period);
    auto diag_updater = std::make_shared<diagnostic_updater::Updater>(node_.get());
    return std::make_unique<MpcLateralController>(*node_, diag_updater);
  }

  /// Some of the behaviours below only appear after the controller has run for a while,
  /// because the controller decides them from a record of earlier cycles. The controller
  /// keeps two such records. It reads a different time for each record when it decides
  /// which entries are too old to keep, so there is one helper per record.
  ///
  /// This helper repeats an input the test has assembled, and moves the clock forward by
  /// one control period before each cycle. Use it for the record of commands, because the
  /// controller reads the clock to decide which commands are too old to keep.
  LateralOutput run_cycles(
    LateralControllerBase & controller, const InputData & input, const int cycles)
  {
    LateralOutput output;
    for (int cycle = 0; cycle < cycles; ++cycle) {
      advance_clock(ctrl_period);
      output = controller.run(input);
    }
    return output;
  }

  /// This helper follows a path, and moves the clock and the stamp on the path forward
  /// together before each cycle. Use it for the record of paths. To decide which paths are
  /// too old to keep, the controller compares the stamps on the paths it keeps instead of
  /// reading the clock. A path whose stamp is still zero is therefore never dropped.
  ///
  /// The stamp continues from the time set by the previous call. A test can therefore
  /// follow one path for a while and then pass a different path to change the shape.
  ///
  /// The vehicle follows the path at 1 m/s and reports its steering as straight ahead. No
  /// test that uses this helper needs it anywhere else.
  LateralOutput follow_path_for_cycles(
    LateralControllerBase & controller, const Trajectory & path, const int cycles)
  {
    LateralOutput output;
    for (int cycle = 0; cycle < cycles; ++cycle) {
      advance_clock(ctrl_period);
      stamp_seconds_ += ctrl_period;
      output = controller.run(
        Input().following(path).planned_at(1.0).driving_at(1.0).stamped_at(stamp_seconds_));
    }
    return output;
  }

  /// Drive a curve at speed until the command stops moving, and report the command reached.
  /// Five cycles are enough for the low pass filters inside the controller to settle.
  static float settle_on_curve(LateralControllerBase & controller)
  {
    const InputData driving = Input().following(left_curve_path()).planned_at(3.0).driving_at(3.0);
    float command = 0.0f;
    for (int cycle = 0; cycle < 5; ++cycle) {
      command = controller.run(driving).control_cmd.steering_tire_angle;
    }
    return command;
  }

  std::vector<std::shared_ptr<rclcpp::Node>> nodes_;
  std::shared_ptr<rclcpp::Node> node_;

  /// Starts away from zero so that the durations the controller takes over its own history
  /// never reach back before the start of the clock.
  int64_t clock_time_ns_ = 1000000000;

  /// Time written to each path, advanced together with the clock.
  double stamp_seconds_ = 0.0;
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
  auto trajectory = straight_path();
  trajectory.points.pop_back();
  const auto input = Input().following(trajectory).planned_at(1.0).driving_at(1.0);

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

  const auto output = controller->run(input);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
  EXPECT_FALSE(output.control_cmd_horizon.controls.empty());
}

TEST_P(MpcLateralControllerModelTest, LeftCurveCommandsPositiveSteering)
{
  ControllerOptions options;
  options.vehicle_model_type = GetParam();
  auto controller = make_controller(options);
  const auto input = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input);

  EXPECT_GT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST_P(MpcLateralControllerModelTest, RightCurveCommandsNegativeSteering)
{
  ControllerOptions options;
  options.vehicle_model_type = GetParam();
  auto controller = make_controller(options);
  const auto input = Input().following(right_curve_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input);

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// use_steer_prediction picks whether the initial angle of the optimisation comes from the
/// measurement or from an estimate built out of the commands already issued. The measured
/// angle here is deliberately away from zero, where the commands issued along a straight
/// path leave the estimate, so that the two sources disagree and the choice between them
/// shows in the command.
TEST_F(MpcLateralControllerTest, SteerPredictionTakesTheInitialAngleFromTheCommandHistory)
{
  ControllerOptions options;
  options.use_steer_prediction = true;
  auto controller = make_controller(options);
  const auto input =
    Input().following(straight_path()).planned_at(1.0).driving_at(1.0).steering_at(0.2);

  const auto output = controller->run(input);

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

  const auto output = controller->run(input);

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST_F(MpcLateralControllerTest, NegativeSteeringOffsetTurnsTheCommandPositive)
{
  auto controller = make_controller();
  controller->set_steering_offset(-0.5 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input);

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
  const auto command_at_limit = at_limit->run(input).control_cmd.steering_tire_angle;

  auto below_limit = make_controller();
  below_limit->set_steering_offset(0.5 * steer_offset_max_update_th);

  const auto output = below_limit->run(input);

  EXPECT_GT(output.control_cmd.steering_tire_angle, command_at_limit);
}

TEST_F(MpcLateralControllerTest, SteeringOffsetPastTheUpdateLimitIsCappedWithinOneCycle)
{
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  auto at_limit = make_controller();
  at_limit->set_steering_offset(steer_offset_max_update_th);
  const auto command_at_limit = at_limit->run(input).control_cmd.steering_tire_angle;

  auto past_limit = make_controller();
  past_limit->set_steering_offset(2.0 * steer_offset_max_update_th);

  const auto output = past_limit->run(input);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, command_at_limit);
}

/// The limit applies to one cycle, not to the offset as a whole. The controller therefore
/// applies more of a large offset on each cycle that follows.
TEST_F(MpcLateralControllerTest, SteeringOffsetPastTheUpdateLimitIsAppliedFurtherOnLaterCycles)
{
  auto controller = make_controller();
  controller->set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);
  const auto first_cycle = controller->run(input).control_cmd.steering_tire_angle;

  const auto output = controller->run(input);

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

  const auto output = controller->run(input);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
}

/// is_steer_converged tells the longitudinal controller that the vehicle has settled on the
/// angle it was asked for, which is what lets it turn the steering off while stopped. The
/// decision needs the command record to cover convergence_history_sec. These two tests
/// use a record shorter and longer than that duration, which records where the answer
/// changes.
TEST_F(MpcLateralControllerTest, SteeringIsNotConvergedBeforeTheHistoryWindowIsFilled)
{
  ControllerOptions options;
  options.use_simulated_time = true;
  auto controller = make_controller(options);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = run_cycles(*controller, input, cycles_spanning(convergence_history_sec) - 1);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

TEST_F(MpcLateralControllerTest, SteeringIsConvergedOnceTheHistoryWindowIsFilled)
{
  ControllerOptions options;
  options.use_simulated_time = true;
  auto controller = make_controller(options);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = run_cycles(*controller, input, cycles_spanning(convergence_history_sec));

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

/// The controller compares each path it is given against the ones it still holds, and
/// treats the path as a new shape when the far end has moved further than
/// new_traj_end_dist. A new shape withdraws the convergence report until the path settles.
TEST_F(MpcLateralControllerTest, PathEndExtendedWithinTheThresholdKeepsTheConvergenceReport)
{
  ControllerOptions options;
  options.use_simulated_time = true;
  auto controller = make_controller(options);
  const auto original = straight_path_extended_by(0.0);
  const auto barely_extended = straight_path_extended_by(0.9 * new_traj_end_dist);
  follow_path_for_cycles(*controller, original, cycles_spanning(convergence_history_sec));

  const auto output = follow_path_for_cycles(*controller, barely_extended, 1);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

TEST_F(MpcLateralControllerTest, PathEndExtendedBeyondTheThresholdWithdrawsTheConvergenceReport)
{
  ControllerOptions options;
  options.use_simulated_time = true;
  auto controller = make_controller(options);
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  follow_path_for_cycles(*controller, original, cycles_spanning(convergence_history_sec));

  const auto output = follow_path_for_cycles(*controller, extended, 1);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

/// The controller drops each path once the newest path it keeps is new_traj_duration_time
/// later. Only then does the controller stop comparing against the old shape. These two
/// tests use a duration shorter and longer than that point.
TEST_F(MpcLateralControllerTest, PathShapeChangeIsStillRememberedBeforeTheRetentionTime)
{
  ControllerOptions options;
  options.use_simulated_time = true;
  auto controller = make_controller(options);
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  follow_path_for_cycles(*controller, original, cycles_spanning(convergence_history_sec));
  follow_path_for_cycles(*controller, extended, 1);

  const auto output =
    follow_path_for_cycles(*controller, extended, cycles_to_forget_shape_change() - 1);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

TEST_F(MpcLateralControllerTest, PathShapeChangeIsForgottenAfterTheRetentionTime)
{
  ControllerOptions options;
  options.use_simulated_time = true;
  auto controller = make_controller(options);
  const auto original = straight_path_extended_by(0.0);
  const auto extended = straight_path_extended_by(1.1 * new_traj_end_dist);
  follow_path_for_cycles(*controller, original, cycles_spanning(convergence_history_sec));
  follow_path_for_cycles(*controller, extended, 1);

  const auto output =
    follow_path_for_cycles(*controller, extended, cycles_to_forget_shape_change());

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
  const auto settled = settle_on_curve(*controller);
  const auto measured = static_cast<float>(settled - 0.9 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller->run(stopped);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIsNotEnteredWhenTheMeasuredAngleIsOutsideTheThreshold)
{
  auto controller = make_controller();
  const auto settled = settle_on_curve(*controller);
  const auto measured = static_cast<float>(settled - 1.1 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller->run(stopped);

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIgnoresTheMeasuredAngleWhileTheGuardIsOff)
{
  ControllerOptions options;
  options.keep_steer_control_until_converged = false;
  auto controller = make_controller(options);
  const auto settled = settle_on_curve(*controller);
  const auto measured = static_cast<float>(settled - 1.1 * converged_steer_rad);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0).steering_at(measured);

  const auto output = controller->run(stopped);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

/// The vehicle counts as stopped while its measured speed does not exceed
/// stop_state_entry_ego_speed. The threshold value itself still counts as stopped, so
/// these two tests sit at the threshold and one step above it.
TEST_F(MpcLateralControllerTest, StopStateIsEnteredAtTheEgoSpeedThreshold)
{
  auto controller = make_controller();
  const auto settled = settle_on_curve(*controller);
  const auto stopped =
    Input().following(left_curve_path()).planned_at(0.0).driving_at(stop_state_entry_ego_speed);

  const auto output = controller->run(stopped);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIsNotEnteredJustAboveTheEgoSpeedThreshold)
{
  auto controller = make_controller();
  const auto settled = settle_on_curve(*controller);
  const InputData moving = Input()
                             .following(left_curve_path())
                             .planned_at(0.0)
                             .driving_at(stop_state_entry_ego_speed + speed_step);

  const auto output = controller->run(moving);

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

/// The path counts as asking for a stop while its target speed stays below
/// stop_state_entry_target_speed. Unlike the measured speed, the threshold value itself
/// does not count, so these two tests sit at the threshold and one step below it.
TEST_F(MpcLateralControllerTest, StopStateIsEnteredJustBelowTheTargetSpeedThreshold)
{
  auto controller = make_controller();
  const auto settled = settle_on_curve(*controller);
  const InputData stopping = Input()
                               .following(left_curve_path())
                               .planned_at(stop_state_entry_target_speed - speed_step)
                               .driving_at(0.0);

  const auto output = controller->run(stopping);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, settled);
}

TEST_F(MpcLateralControllerTest, StopStateIsNotEnteredAtTheTargetSpeedThreshold)
{
  auto controller = make_controller();
  const auto settled = settle_on_curve(*controller);
  const InputData driving =
    Input().following(left_curve_path()).planned_at(stop_state_entry_target_speed).driving_at(0.0);

  const auto output = controller->run(driving);

  EXPECT_NE(output.control_cmd.steering_tire_angle, settled);
}

/// While Autoware is not driving the vehicle the controller drops the command it kept from
/// the previous cycle. The command from the last engaged cycle therefore does not persist.
TEST_F(MpcLateralControllerTest, CommandIsResetWhileNotUnderAutowareControl)
{
  auto controller = make_controller();
  const auto engaged = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);
  const auto steered = controller->run(engaged).control_cmd.steering_tire_angle;
  const InputData disengaged =
    Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0).without_autoware_control();

  const auto output = controller->run(disengaged);

  EXPECT_NE(output.control_cmd.steering_tire_angle, steered);
}

/// Autoware drives the vehicle only in the autonomous operation mode. Any other mode
/// counts as not being in control, the same as the enable flag being off.
TEST_F(MpcLateralControllerTest, CommandIsResetWhileTheOperationModeIsNotAutonomous)
{
  auto controller = make_controller();
  const auto engaged = Input().following(left_curve_path()).planned_at(1.0).driving_at(1.0);
  const auto steered = controller->run(engaged).control_cmd.steering_tire_angle;
  const InputData stopped_mode =
    Input()
      .following(left_curve_path())
      .planned_at(1.0)
      .driving_at(1.0)
      .in_operation_mode(autoware_adapi_v1_msgs::msg::OperationModeState::STOP);

  const auto output = controller->run(stopped_mode);

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
  auto uninterrupted = make_controller();
  settle_on_curve(*uninterrupted);
  uninterrupted->run(driving);
  const auto without_gap = uninterrupted->run(driving).control_cmd.steering_tire_angle;

  auto interrupted = make_controller();
  settle_on_curve(*interrupted);
  interrupted->run(disengaged);

  const auto output = interrupted->run(driving);

  EXPECT_NE(output.control_cmd.steering_tire_angle, without_gap);
}

/// The temporal reference mode follows the path by the time stamped on each point, and
/// therefore requires those times to increase along the path. One test meets that
/// requirement and one does not.
TEST_F(MpcLateralControllerTest, TemporalReferenceModeAcceptsIncreasingTimeFromStart)
{
  ControllerOptions options;
  options.trajectory_reference_mode = "temporal";
  auto controller = make_controller(options);
  const auto input =
    Input().following(straight_path_with_time_from_start(true)).planned_at(1.0).driving_at(1.0);

  const auto ready = controller->isReady(input);

  EXPECT_TRUE(ready);
}

TEST_F(MpcLateralControllerTest, TemporalReferenceModeRejectsRepeatedTimeFromStart)
{
  ControllerOptions options;
  options.trajectory_reference_mode = "temporal";
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

  const auto output = controller->run(input);

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// TO BE CONFIRMED: the steering offset is subtracted from the command but not from the
/// horizon. The first entry of the horizon therefore differs from the command sent in the
/// same cycle. The difference may be intentional. The offset corrects a bias in the
/// steering of the ego vehicle. The design may intend the horizon to carry no such
/// correction, because a reader may treat the horizon as a plan rather than as commands to
/// send. This test records the command and the horizon as they are now, so that a fix, or a
/// written statement of the intent, changes this test too.
TEST_F(MpcLateralControllerTest, SteeringOffsetReachesTheCommandButNotTheHorizon)
{
  auto controller = make_controller();
  controller->set_steering_offset(2.0 * steer_offset_max_update_th);
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = controller->run(input);

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
TEST_F(MpcLateralControllerTest, StopStateHoldsTheCommandButNotTheHorizon)
{
  auto controller = make_controller();
  settle_on_curve(*controller);
  const auto stopped = Input().following(left_curve_path()).planned_at(0.0).driving_at(0.0);

  const auto output = controller->run(stopped);

  EXPECT_NE(
    output.control_cmd.steering_tire_angle,
    output.control_cmd_horizon.controls.front().steering_tire_angle);
}
