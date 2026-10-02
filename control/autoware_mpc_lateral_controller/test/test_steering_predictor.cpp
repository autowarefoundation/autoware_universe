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

#include "autoware/mpc_lateral_controller/steering_predictor.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{
using autoware::motion::control::mpc_lateral_controller::SteeringPredictor;

constexpr double steer_tau = 0.1;
constexpr double steer_delay = 0.0;
/// One cycle of the runs below. The answer of a cycle does not yet hold the command given
/// in that same cycle, so the cycle is short against the time constant to keep that delay
/// inside the tolerance the tests allow.
constexpr double cycle = 0.001;

/// The angle the steering is asked to hold throughout each test below.
constexpr double commanded_angle = 0.2;

rclcpp::Time at(const double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

/// Issue the same command every cycle until the given time, and report the angle the
/// predictor arrives at.
///
/// The predictor answers with the response of a first order lag to the commands it has
/// been given, so a run that holds one command is a step response. A command is stored
/// before the prediction of the same cycle is read, which is the order the controller
/// uses.
double hold_command_until(SteeringPredictor & predictor, const double seconds)
{
  double predicted = 0.0;
  for (double time = 0.0; time <= seconds + 0.5 * cycle; time += cycle) {
    predictor.storeSteerCmd(commanded_angle, at(time));
    predicted = predictor.calcSteerPrediction(at(time));
  }
  return predicted;
}

/// The response of a first order lag to a step command: y(t) = u * (1 - exp(-t / tau)).
double first_order_step_response(const double command, const double time, const double tau)
{
  return command * (1.0 - std::exp(-time / tau));
}

}  // namespace

/// A first order lag reaches 63.2 per cent of the command after one time constant.
TEST(SteeringPredictorTest, ReachesSixtyThreePerCentAfterOneTimeConstant)
{
  SteeringPredictor predictor(steer_tau, steer_delay);

  const double predicted = hold_command_until(predictor, 1.0 * steer_tau);

  EXPECT_NEAR(
    predicted, first_order_step_response(commanded_angle, 1.0 * steer_tau, steer_tau),
    0.01 * commanded_angle);
}

/// The same lag reaches 95 per cent after three time constants.
TEST(SteeringPredictorTest, ReachesNinetyFivePerCentAfterThreeTimeConstants)
{
  SteeringPredictor predictor(steer_tau, steer_delay);

  const double predicted = hold_command_until(predictor, 3.0 * steer_tau);

  EXPECT_NEAR(
    predicted, first_order_step_response(commanded_angle, 3.0 * steer_tau, steer_tau),
    0.01 * commanded_angle);
}
