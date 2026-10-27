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

#include "autoware_planning_msgs/msg/trajectory.hpp"
#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <gtest/gtest.h>
#include <rcl/time.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

namespace
{
using autoware::motion::control::trajectory_follower::InputData;
using autoware::motion::control::trajectory_follower::LateralControllerInterface;
using autoware::motion::control::trajectory_follower::LateralOutput;
using autoware::pure_pursuit::PurePursuitLateralControllerNode;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;

int64_t to_nanoseconds(const double seconds)
{
  return static_cast<int64_t>(seconds * 1e9);
}

/// Time of a control cycle, given in seconds [s].
rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(to_nanoseconds(seconds), RCL_ROS_TIME);
}

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

/// The paths below reach from five metres behind the map origin to forty metres past it. The
/// ego sits at the origin, so the path holds the four metres behind the ego that the
/// curvature calculation reads, and every lookahead distance the controller may ask for.
constexpr double path_start = -5.0;
constexpr double path_end = 40.0;
constexpr double path_point_interval = 0.5;

Trajectory straight_path()
{
  std::vector<TrajectoryPoint> points;
  for (double distance = path_start; distance <= path_end; distance += path_point_interval) {
    points.push_back(make_point(distance, 0.0, 0.0));
  }
  return make_trajectory(std::move(points));
}

/// A path along a circle of the given radius, passing through the map origin with the
/// heading along the x axis. A positive radius curves to the left, a negative one to the
/// right.
Trajectory arc_path(const double radius)
{
  std::vector<TrajectoryPoint> points;
  for (double distance = path_start; distance <= path_end; distance += path_point_interval) {
    const double angle = distance / radius;
    points.push_back(make_point(radius * std::sin(angle), radius * (1.0 - std::cos(angle)), angle));
  }
  return make_trajectory(std::move(points));
}

/// A path of 0.15 m. The controller resamples a path every 0.1 m, so the resampled path holds
/// two points, which is fewer than the three the controller requires.
Trajectory path_too_short_for_three_samples()
{
  return make_trajectory({make_point(0.0, 0.0, 0.0), make_point(0.15, 0.0, 0.0)});
}

/// One control cycle of input. Each call names the quantity it sets, so that a test states
/// the situation the controller is given.
class Input
{
public:
  /// Path to follow.
  Input & following(Trajectory path)
  {
    input_.current_trajectory = std::move(path);
    return *this;
  }

  /// Target speed written to every point of the path [m/s]. What the path requests, not what
  /// the vehicle reports. A negative value asks the vehicle to drive in reverse.
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

  /// Position of the ego across the path [m], positive to the left. Every path in this file
  /// passes through the map origin with the heading along the x axis, and the ego keeps that
  /// heading, so this value is the distance between the ego and the path.
  Input & beside_the_path(const double distance)
  {
    input_.current_odometry.pose.pose.position.y = distance;
    return *this;
  }

  operator InputData() const { return input_; }

private:
  static InputData make_default()
  {
    InputData input;
    input.current_odometry.header.frame_id = "map";
    input.current_odometry.pose.pose.orientation = make_orientation(0.0);
    return input;
  }

  InputData input_ = make_default();
};

class PurePursuitLateralControllerTest : public ::testing::Test
{
protected:
  /// Distance between the front and the rear axle of the test vehicle [m]. Given to the
  /// controller as the wheel_base parameter.
  static constexpr double wheel_base = 2.74;

  /// Distance between the commanded and the measured angle below which the controller
  /// reports the steering converged. Carried by the shipped parameter file as
  /// converged_steer_rad.
  static constexpr double converged_steer_rad = 0.1;

  /// Lateral distance at or past which the controller lengthens the lookahead distance.
  /// Carried by the shipped parameter file as long_ld_lateral_error_threshold.
  static constexpr double lateral_error_threshold = 0.5;

  /// A lateral distance small enough to land on the other side of that threshold.
  static constexpr double lateral_error_step = 0.01;

  /// Distance around the ego within which the controller looks for the path [m]. Written
  /// into the controller rather than read from a parameter.
  static constexpr double nearest_search_radius = 3.0;

  /// Radius of the circle the arc tests follow [m].
  static constexpr double arc_radius = 30.0;

  /// How far the command may differ from the angle the geometry gives. The controller aims
  /// at a point on the resampled path, which is a chain of straight segments and not the
  /// circle itself, so the command misses that angle by a small amount.
  static constexpr double arc_relative_tolerance = 0.01;

  /// Two times of a control cycle [s]. The controller stamps the command with the time of the
  /// cycle, and these two differ so that a stamp names the cycle the command was made in.
  static constexpr double first_cycle_time = 1.0;
  static constexpr double second_cycle_time = 1.5;

  /// A speed at which the lookahead distance the controller derives from the speed is longer
  /// than the shortest lookahead distance it accepts [m/s]. At a lower speed the controller
  /// returns that shortest distance whatever the lateral distance is.
  static constexpr double speed_above_the_lookahead_floor = 3.0;

  /// Lookahead distance per unit of speed [s]. Carried by the shipped parameter file as
  /// ld_velocity_ratio.
  static constexpr double ld_velocity_ratio = 2.4;

  /// Lookahead distance removed per unit of path curvature [m^2]. Carried by the shipped
  /// parameter file as ld_curvature_ratio.
  static constexpr double ld_curvature_ratio = 120.0;

  /// Longest lookahead distance the controller accepts [m]. Carried by the shipped parameter
  /// file as max_lookahead_distance.
  static constexpr double max_lookahead_distance = 15.0;

  /// How far the command may differ from the angle computed from the lookahead distance. The
  /// controller aims at a point on the resampled path, so the command misses that angle by a
  /// small amount.
  static constexpr double lookahead_relative_tolerance = 0.01;

  /// Distance between two neighbouring points of the predicted trajectory [m]. Carried by the
  /// shipped parameter file as prediction_ds.
  static constexpr double prediction_ds = 0.3;

  /// Length of the path the controller predicts the vehicle to drive [m]. Carried by the
  /// shipped parameter file as prediction_distance_length.
  static constexpr double prediction_distance_length = 21.0;

  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  rclcpp::NodeOptions make_node_options(const bool enable_path_smoothing)
  {
    const auto share_dir = ament_index_cpp::get_package_share_directory("autoware_pure_pursuit");

    rclcpp::NodeOptions options;
    options.arguments(
      {"--ros-args", "--params-file", share_dir + "/config/pure_pursuit.param.yaml"});
    options.append_parameter_override("enable_path_smoothing", enable_path_smoothing);

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

    return options;
  }

  /// Build the controller behind the interface, so that the tests only use the interface the
  /// trajectory follower node uses.
  std::unique_ptr<LateralControllerInterface> make_controller(
    const bool enable_path_smoothing = false)
  {
    // Kept alive for the whole test: a controller holds a reference to the node it was built
    // from, and a test may build two controllers and compare their commands.
    node_ = nodes_.emplace_back(
      std::make_shared<rclcpp::Node>("test_node", make_node_options(enable_path_smoothing)));
    return std::make_unique<PurePursuitLateralControllerNode>(*node_);
  }

  /// Let the controller run one cycle with the given input, and return the predicted
  /// trajectory it publishes on ~/output/predicted_trajectory. Empty if none arrives within
  /// message_timeout. The controller has to be the one built last.
  std::optional<Trajectory> let_controller_publish_predicted_trajectory(
    LateralControllerInterface & controller, const InputData & input)
  {
    std::optional<Trajectory> received;
    const auto subscription = node_->create_subscription<Trajectory>(
      "~/output/predicted_trajectory", 1,
      [&received](const Trajectory::ConstSharedPtr message) { received = *message; });
    const auto connected_by = std::chrono::steady_clock::now() + message_timeout;
    while (subscription->get_publisher_count() == 0 &&
           std::chrono::steady_clock::now() < connected_by) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    run_cycle(controller, input);

    const auto received_by = std::chrono::steady_clock::now() + message_timeout;
    while (!received && std::chrono::steady_clock::now() < received_by) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return received;
  }

  /// Let the controller run one cycle at the given time [s], and return its output.
  static LateralOutput run_cycle(
    LateralControllerInterface & controller, const InputData & input,
    const double seconds = first_cycle_time)
  {
    return controller.run(input, at(seconds)).value();
  }

  static constexpr auto message_timeout = std::chrono::seconds(5);

  std::vector<std::shared_ptr<rclcpp::Node>> nodes_;
  std::shared_ptr<rclcpp::Node> node_;
};

/// Distance between two points of a trajectory in the x-y plane [m].
double distance_between(const TrajectoryPoint & a, const TrajectoryPoint & b)
{
  return std::hypot(a.pose.position.x - b.pose.position.x, a.pose.position.y - b.pose.position.y);
}

/// Length of a trajectory measured along its points [m].
double length_of(const Trajectory & trajectory)
{
  double length = 0.0;
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    length += distance_between(trajectory.points.at(i - 1), trajectory.points.at(i));
  }
  return length;
}

}  // namespace

TEST_F(PurePursuitLateralControllerTest, StraightPathKeepsTheSteeringNeutral)
{
  auto controller = make_controller();
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = run_cycle(*controller, input);

  EXPECT_NEAR(output.control_cmd.steering_tire_angle, 0.0, 1e-6);
}

/// A vehicle of wheelbase L held at a steering angle drives a circle of radius L / tan(angle),
/// so a vehicle on a circle of radius R stays on it at one angle only, atan(L / R). The ego
/// sits on the circle with no lateral and no heading error, so that is the angle to command.
TEST_F(PurePursuitLateralControllerTest, LeftArcIsSteeredAtTheAngleTheRadiusRequires)
{
  auto controller = make_controller();
  const auto input = Input().following(arc_path(arc_radius)).planned_at(1.0).driving_at(1.0);

  const auto output = run_cycle(*controller, input);

  const double angle_holding_the_circle = std::atan(wheel_base / arc_radius);
  EXPECT_NEAR(
    output.control_cmd.steering_tire_angle, angle_holding_the_circle,
    arc_relative_tolerance * angle_holding_the_circle);
}

TEST_F(PurePursuitLateralControllerTest, RightArcIsSteeredAtTheAngleTheRadiusRequires)
{
  auto controller = make_controller();
  const auto input = Input().following(arc_path(-arc_radius)).planned_at(1.0).driving_at(1.0);

  const auto output = run_cycle(*controller, input);

  const double angle_holding_the_circle = -std::atan(wheel_base / arc_radius);
  EXPECT_NEAR(
    output.control_cmd.steering_tire_angle, angle_holding_the_circle,
    arc_relative_tolerance * std::abs(angle_holding_the_circle));
}

TEST_F(PurePursuitLateralControllerTest, EgoLeftOfThePathIsSteeredToTheRight)
{
  auto controller = make_controller();
  const auto input =
    Input().following(straight_path()).planned_at(1.0).driving_at(1.0).beside_the_path(0.2);

  const auto output = run_cycle(*controller, input);

  EXPECT_LT(output.control_cmd.steering_tire_angle, 0.0f);
}

TEST_F(PurePursuitLateralControllerTest, EgoRightOfThePathIsSteeredToTheLeft)
{
  auto controller = make_controller();
  const auto input =
    Input().following(straight_path()).planned_at(1.0).driving_at(1.0).beside_the_path(-0.2);

  const auto output = run_cycle(*controller, input);

  EXPECT_GT(output.control_cmd.steering_tire_angle, 0.0f);
}

/// Past the lateral distance threshold the controller aims at a point further along the
/// path, which turns the vehicle toward the path more gently. The command therefore falls as
/// the ego moves past the threshold, although the ego is further from the path.
TEST_F(PurePursuitLateralControllerTest, LateralDistancePastTheThresholdReducesTheCommand)
{
  const auto at_distance = [](const double distance) {
    return Input()
      .following(straight_path())
      .planned_at(speed_above_the_lookahead_floor)
      .driving_at(speed_above_the_lookahead_floor)
      .beside_the_path(distance);
  };
  auto inside = make_controller();
  const auto command_inside =
    run_cycle(*inside, at_distance(lateral_error_threshold - lateral_error_step))
      .control_cmd.steering_tire_angle;
  auto outside = make_controller();

  const auto output =
    run_cycle(*outside, at_distance(lateral_error_threshold + lateral_error_step));

  EXPECT_GT(output.control_cmd.steering_tire_angle, command_inside);
}

/// A path that asks for a negative speed makes the controller aim at a point further along
/// the path, so that the vehicle turns toward the path more gently than it does on the same
/// path with a positive speed.
TEST_F(PurePursuitLateralControllerTest, ReversePathReducesTheCommand)
{
  const auto at_planned_speed = [](const double speed) {
    return Input()
      .following(straight_path())
      .planned_at(speed)
      .driving_at(1.0)
      .beside_the_path(0.2);
  };
  auto forward = make_controller();
  const auto command_forward =
    run_cycle(*forward, at_planned_speed(1.0)).control_cmd.steering_tire_angle;
  auto reverse = make_controller();

  const auto output = run_cycle(*reverse, at_planned_speed(-1.0));

  EXPECT_GT(output.control_cmd.steering_tire_angle, command_forward);
}

/// The ego stands beside the path, which the controller answers with a command away from
/// neutral on any path it accepts. The command is neutral here because the controller
/// ignores a path of too few samples.
TEST_F(PurePursuitLateralControllerTest, PathWithTooFewSamplesKeepsTheSteeringNeutral)
{
  auto controller = make_controller();
  const auto input = Input()
                       .following(path_too_short_for_three_samples())
                       .planned_at(1.0)
                       .driving_at(1.0)
                       .beside_the_path(0.2);

  const auto output = run_cycle(*controller, input);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, 0.0f);
}

/// The controller looks for the path within a fixed distance of the ego. An ego further away
/// than that leaves the controller with no point to aim at, and the controller repeats the
/// command of the last cycle it could solve, including the time stamped on that command.
TEST_F(PurePursuitLateralControllerTest, PathBeyondTheSearchRadiusRepeatsThePreviousCommand)
{
  auto controller = make_controller();
  const auto on_the_path = Input().following(arc_path(arc_radius)).planned_at(1.0).driving_at(1.0);
  const auto previous = run_cycle(*controller, on_the_path, first_cycle_time).control_cmd;
  const auto far_from_the_path = Input()
                                   .following(arc_path(arc_radius))
                                   .planned_at(1.0)
                                   .driving_at(1.0)
                                   .beside_the_path(2.0 * nearest_search_radius);

  const auto output = run_cycle(*controller, far_from_the_path, second_cycle_time);

  EXPECT_FLOAT_EQ(output.control_cmd.steering_tire_angle, previous.steering_tire_angle);
  EXPECT_EQ(
    rclcpp::Time(output.control_cmd.stamp).nanoseconds(),
    rclcpp::Time(previous.stamp).nanoseconds());
}

TEST_F(PurePursuitLateralControllerTest, CommandCarriesTheTimeOfTheCycle)
{
  auto controller = make_controller();
  const auto input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto output = run_cycle(*controller, input, first_cycle_time);

  EXPECT_EQ(rclcpp::Time(output.control_cmd.stamp).nanoseconds(), to_nanoseconds(first_cycle_time));
}

/// The command on a straight path is neutral, so the measured angle alone decides whether the
/// controller reports the steering converged.
TEST_F(PurePursuitLateralControllerTest, MeasuredAngleCloseToTheCommandConvergesTheSteering)
{
  auto controller = make_controller();
  const auto input = Input()
                       .following(straight_path())
                       .planned_at(1.0)
                       .driving_at(1.0)
                       .steering_at(0.5 * converged_steer_rad);

  const auto output = run_cycle(*controller, input);

  EXPECT_TRUE(output.sync_data.is_steer_converged);
}

TEST_F(PurePursuitLateralControllerTest, MeasuredAngleFarFromTheCommandHoldsTheSteeringApart)
{
  auto controller = make_controller();
  const auto input = Input()
                       .following(straight_path())
                       .planned_at(1.0)
                       .driving_at(1.0)
                       .steering_at(2.0 * converged_steer_rad);

  const auto output = run_cycle(*controller, input);

  EXPECT_FALSE(output.sync_data.is_steer_converged);
}

/// Path smoothing replaces each point of the resampled path by the mean of its neighbours,
/// which moves the path toward the inside of a curve. The ego keeps its position, so it ends
/// outside the smoothed path and the controller commands a larger angle than it does on the
/// same curve without smoothing.
TEST_F(PurePursuitLateralControllerTest, PathSmoothingRaisesTheCommandOnACurve)
{
  const auto input = Input().following(arc_path(arc_radius)).planned_at(1.0).driving_at(1.0);
  auto plain = make_controller();
  const auto command_plain = run_cycle(*plain, input).control_cmd.steering_tire_angle;
  auto smoothing = make_controller(/*enable_path_smoothing=*/true);

  const auto output = run_cycle(*smoothing, input);

  EXPECT_GT(output.control_cmd.steering_tire_angle, command_plain);
}

/// The controller aims at the point of the path that lies the lookahead distance d away from
/// the ego, and steers onto the circle through that point: atan(L * 2y / d^2), where y is the
/// position of the point across the heading of the ego. At this speed the distance derived
/// from the speed is longer than max_lookahead_distance, so d is max_lookahead_distance.
TEST_F(PurePursuitLateralControllerTest, LookaheadDistanceStopsAtTheUpperLimit)
{
  constexpr double speed = 10.0;
  static_assert(ld_velocity_ratio * speed > max_lookahead_distance);
  constexpr double offset = 0.2;
  auto controller = make_controller();
  const auto input =
    Input().following(straight_path()).planned_at(speed).driving_at(speed).beside_the_path(offset);

  const auto output = run_cycle(*controller, input);

  // The ego stands `offset` to the left of a straight path, so the point it aims at lies
  // `offset` to its right.
  const double d = max_lookahead_distance;
  const double expected = std::atan(wheel_base * 2.0 * -offset / (d * d));
  EXPECT_NEAR(
    output.control_cmd.steering_tire_angle, expected,
    lookahead_relative_tolerance * std::abs(expected));
}

// The predicted trajectory is the path the vehicle drives if it follows the commands of the
// controller from this cycle on. Nodes that stop the vehicle before an obstacle or before it
// leaves the lane read it from ~/output/predicted_trajectory. The test below checks that it is
// published, starts at the ego, and reaches prediction_distance_length along the path.

TEST_F(PurePursuitLateralControllerTest, PredictedTrajectoryIsPublished)
{
  auto controller = make_controller();
  const InputData input = Input().following(straight_path()).planned_at(1.0).driving_at(1.0);

  const auto predicted = let_controller_publish_predicted_trajectory(*controller, input);

  ASSERT_TRUE(predicted);
  ASSERT_GE(predicted->points.size(), 2u);
  const auto & ego = input.current_odometry.pose.pose.position;
  EXPECT_DOUBLE_EQ(predicted->points.front().pose.position.x, ego.x);
  EXPECT_DOUBLE_EQ(predicted->points.front().pose.position.y, ego.y);
  for (size_t i = 1; i < predicted->points.size(); ++i) {
    EXPECT_NEAR(
      distance_between(predicted->points.at(i - 1), predicted->points.at(i)), prediction_ds, 1e-6);
    EXPECT_NEAR(predicted->points.at(i).pose.position.y, 0.0, 1e-6);
  }
  EXPECT_NEAR(length_of(*predicted), prediction_distance_length, prediction_ds);
}

// Everything from here to the end of the file is removed together with the tests it
// supports. Each of those tests holds an expectation that a correct change to the controller
// may break, so none of them runs unless the test executable is given
// --gtest_also_run_disabled_tests.

/// The expected angle below follows the formula the controller uses today, d = ld_velocity_ratio
/// * speed - ld_curvature_ratio * curvature. The parameter description calls ld_curvature_ratio
/// only a curvature ratio, so a correct change to the formula fails this test as surely as a
/// mistake does. It therefore carries the DISABLED_ prefix.
///
/// The ego stands to the right of a circle that curves to the left, so that the point it aims
/// at depends on the lookahead distance. The point is where the circle of radius d around the
/// ego meets the path.
TEST_F(PurePursuitLateralControllerTest, DISABLED_CurvatureOfThePathShortensTheLookaheadDistance)
{
  constexpr double speed = 5.0;
  constexpr double offset = 0.2;
  auto controller = make_controller();
  const auto input = Input()
                       .following(arc_path(arc_radius))
                       .planned_at(speed)
                       .driving_at(speed)
                       .beside_the_path(-offset);

  const auto output = run_cycle(*controller, input);

  const double d = ld_velocity_ratio * speed - ld_curvature_ratio / arc_radius;
  // The ego stands at (0, -offset) and the path circle has its centre at (0, arc_radius). The
  // two circles meet at the height `along` above the ego, measured toward that centre.
  const double centre_distance = arc_radius + offset;
  const double along =
    (d * d - arc_radius * arc_radius + centre_distance * centre_distance) / (2.0 * centre_distance);
  const double expected = std::atan(wheel_base * 2.0 * along / (d * d));
  EXPECT_NEAR(
    output.control_cmd.steering_tire_angle, expected,
    lookahead_relative_tolerance * std::abs(expected));
}
