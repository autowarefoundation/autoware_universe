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

// Node-level I/O test: drives the node with dummy input topics.

#include "mrm_steering_hold_stop_operator_node.hpp"

#include <gmock/gmock.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace autoware::mrm_steering_hold_stop_operator
{

using std::chrono_literals::operator""ms;

namespace debug_index
{
constexpr size_t kState = 0;
constexpr size_t kHeldSteeringAngle = 5;
constexpr size_t kTargetAcceleration = 8;
constexpr size_t kTargetJerk = 9;
constexpr size_t kProfile = 10;
}  // namespace debug_index

class NodeIoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();

    helper_ = std::make_shared<rclcpp::Node>("test_helper");
    pub_trigger_ = helper_->create_publisher<InLaneStopTrigger>(
      ns_ + "/input/trigger", rclcpp::QoS{1}.reliable().transient_local());
    pub_steering_ =
      helper_->create_publisher<SteeringReport>(ns_ + "/input/steering_status", rclcpp::QoS{1});
    pub_odometry_ =
      helper_->create_publisher<Odometry>(ns_ + "/input/kinematic_state", rclcpp::QoS{1});
    pub_acceleration_ = helper_->create_publisher<AccelWithCovarianceStamped>(
      ns_ + "/input/acceleration", rclcpp::QoS{1});
    sub_control_ = helper_->create_subscription<Control>(
      ns_ + "/output/control", rclcpp::QoS{10},
      [this](const Control::ConstSharedPtr msg) { received_controls_.push_back(*msg); });
    sub_debug_status_ = helper_->create_subscription<Float32MultiArrayStamped>(
      ns_ + "/debug/status", rclcpp::QoS{10},
      [this](const Float32MultiArrayStamped::ConstSharedPtr msg) { last_debug_status_ = msg; });

    executor_->add_node(helper_);
  }

  void TearDown() override
  {
    if (node_) {
      executor_->remove_node(node_);
    }
    executor_->remove_node(helper_);
  }

  void createNode()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"update_rate", 50.0},
      {"input_freshness_timeout", 0.5},
      {"low_speed_threshold", 1.0},
      {"profiles.moderate.target_acceleration", -3.0},
      {"profiles.moderate.target_jerk", -5.0},
      {"profiles.emergency.target_acceleration", -6.0},
      {"profiles.emergency.target_jerk", -20.0},
    });
    node_ = std::make_shared<MrmSteeringHoldStopOperator>(options);
    executor_->add_node(node_);
  }

  void publishInputs(const double steering, const double velocity, const double acceleration)
  {
    SteeringReport steering_msg;
    steering_msg.stamp = helper_->now();
    steering_msg.steering_tire_angle = static_cast<float>(steering);
    pub_steering_->publish(steering_msg);

    Odometry odometry_msg;
    odometry_msg.header.stamp = helper_->now();
    odometry_msg.twist.twist.linear.x = velocity;
    pub_odometry_->publish(odometry_msg);

    AccelWithCovarianceStamped accel_msg;
    accel_msg.header.stamp = helper_->now();
    accel_msg.accel.accel.linear.x = acceleration;
    pub_acceleration_->publish(accel_msg);
  }

  void publishTrigger(const bool active, const ProfileType profile)
  {
    InLaneStopTrigger trigger;
    trigger.stamp = helper_->now();
    trigger.trigger = active;
    trigger.profile = profile;
    pub_trigger_->publish(trigger);
  }

  // Spin while invoking on_tick each iteration, until pred() or timeout.
  bool spinUntil(
    const std::chrono::milliseconds timeout, const std::function<bool()> & pred,
    const std::function<void()> & on_tick = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (on_tick) {
        on_tick();
      }
      executor_->spin_some();
      if (pred()) {
        return true;
      }
      std::this_thread::sleep_for(5ms);
    }
    return pred();
  }

  bool debugStateIs(const State state) const
  {
    return last_debug_status_ &&
           last_debug_status_->data.at(debug_index::kState) == static_cast<float>(state);
  }

  const std::string ns_ = "/mrm_steering_hold_stop_operator";

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::shared_ptr<MrmSteeringHoldStopOperator> node_;
  std::shared_ptr<rclcpp::Node> helper_;

  rclcpp::Publisher<InLaneStopTrigger>::SharedPtr pub_trigger_;
  rclcpp::Publisher<SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odometry_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;
  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Subscription<Float32MultiArrayStamped>::SharedPtr sub_debug_status_;

  std::vector<Control> received_controls_;
  Float32MultiArrayStamped::ConstSharedPtr last_debug_status_;
};

TEST_F(NodeIoTest, MirrorsMeasuredStateWhileTriggerOff)
{
  createNode();
  const bool got = spinUntil(
    2000ms, [this] { return received_controls_.size() >= 5; },
    [this] { publishInputs(0.2, 5.0, -0.3); });
  ASSERT_TRUE(got);

  const auto & last = received_controls_.back();
  EXPECT_NEAR(last.lateral.steering_tire_angle, 0.2, 1e-6);
  EXPECT_NEAR(last.longitudinal.velocity, 5.0, 1e-6);
  EXPECT_NEAR(last.longitudinal.acceleration, -0.3, 1e-6);
  EXPECT_NEAR(last.longitudinal.jerk, 0.0, 1e-6);
}

TEST_F(NodeIoTest, PublishesNothingWithoutInputs)
{
  createNode();
  spinUntil(500ms, [] { return false; });
  EXPECT_TRUE(received_controls_.empty());
}

TEST_F(NodeIoTest, DeceleratesOpenLoopAfterTrigger)
{
  createNode();
  // Reach MIRROR first.
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return !received_controls_.empty(); },
    [this] { publishInputs(0.2, 5.0, -0.3); }));
  received_controls_.clear();

  // Trigger ON, then stop feeding inputs: deceleration must continue open-loop.
  publishTrigger(true, InLaneStopTrigger::PROFILE_MODERATE);
  ASSERT_TRUE(spinUntil(2000ms, [this] { return received_controls_.size() >= 10; }));

  for (size_t i = 0; i + 1 < received_controls_.size(); ++i) {
    EXPECT_NEAR(received_controls_[i].lateral.steering_tire_angle, 0.2, 1e-6);
    EXPECT_GE(
      received_controls_[i].longitudinal.acceleration + 1e-6,
      received_controls_[i + 1].longitudinal.acceleration)
      << "acceleration must be non-increasing";
    EXPECT_GE(received_controls_[i].longitudinal.acceleration, -3.0 - 1e-6);
  }

  // Trigger OFF with fresh inputs: back to mirror.
  received_controls_.clear();
  publishTrigger(false, InLaneStopTrigger::PROFILE_MODERATE);
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return received_controls_.size() >= 3; },
    [this] { publishInputs(0.4, 4.0, -0.1); }));
  EXPECT_NEAR(received_controls_.back().lateral.steering_tire_angle, 0.4, 1e-6);
}

TEST_F(NodeIoTest, EmergencyProfileUsesEmergencyParameters)
{
  createNode();
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return debugStateIs(State::kMirror); },
    [this] { publishInputs(0.1, 0.5, 0.0); }));

  // Below low_speed_threshold the deceleration starts directly at target_acceleration.
  publishTrigger(true, InLaneStopTrigger::PROFILE_EMERGENCY);
  ASSERT_TRUE(spinUntil(2000ms, [this] { return debugStateIs(State::kDecelerating); }));
  received_controls_.clear();
  ASSERT_TRUE(spinUntil(2000ms, [this] { return !received_controls_.empty(); }));

  EXPECT_NEAR(received_controls_.back().longitudinal.acceleration, -6.0, 1e-6);
  EXPECT_FLOAT_EQ(last_debug_status_->data.at(debug_index::kTargetAcceleration), -6.0f);
  EXPECT_FLOAT_EQ(last_debug_status_->data.at(debug_index::kTargetJerk), -20.0f);
  EXPECT_FLOAT_EQ(
    last_debug_status_->data.at(debug_index::kProfile),
    static_cast<float>(InLaneStopTrigger::PROFILE_EMERGENCY));
}

TEST_F(NodeIoTest, UnknownProfileDeceleratesWithModerateParameters)
{
  createNode();
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return debugStateIs(State::kMirror); },
    [this] { publishInputs(0.1, 5.0, 0.0); }));

  publishTrigger(true, InLaneStopTrigger::PROFILE_UNKNOWN);
  ASSERT_TRUE(spinUntil(2000ms, [this] { return debugStateIs(State::kDecelerating); }));

  EXPECT_FLOAT_EQ(last_debug_status_->data.at(debug_index::kTargetAcceleration), -3.0f);
  EXPECT_FLOAT_EQ(last_debug_status_->data.at(debug_index::kTargetJerk), -5.0f);
  EXPECT_FLOAT_EQ(
    last_debug_status_->data.at(debug_index::kProfile),
    static_cast<float>(InLaneStopTrigger::PROFILE_UNKNOWN));
}

TEST_F(NodeIoTest, ProfileChangeWhileTriggerOn)
{
  createNode();
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return debugStateIs(State::kMirror); },
    [this] { publishInputs(0.2, 5.0, -0.3); }));

  publishTrigger(true, InLaneStopTrigger::PROFILE_MODERATE);
  ASSERT_TRUE(spinUntil(2000ms, [this] { return debugStateIs(State::kDecelerating); }));

  // Escalate to emergency while the measured steering moves: only the targets change.
  publishTrigger(true, InLaneStopTrigger::PROFILE_EMERGENCY);
  ASSERT_TRUE(spinUntil(
    2000ms,
    [this] { return last_debug_status_->data.at(debug_index::kTargetAcceleration) == -6.0f; },
    [this] { publishInputs(0.4, 5.0, -0.3); }));
  received_controls_.clear();
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return !received_controls_.empty(); },
    [this] { publishInputs(0.4, 5.0, -0.3); }));

  EXPECT_TRUE(debugStateIs(State::kDecelerating));
  EXPECT_FLOAT_EQ(last_debug_status_->data.at(debug_index::kHeldSteeringAngle), 0.2f);
  EXPECT_NEAR(received_controls_.back().lateral.steering_tire_angle, 0.2, 1e-6);
}

TEST_F(NodeIoTest, LateJoinReceivesLatchedTrigger)
{
  // The trigger is published before the node exists (e.g. the node restarts during MRM).
  publishTrigger(true, InLaneStopTrigger::PROFILE_MODERATE);
  executor_->spin_some();

  createNode();
  ASSERT_TRUE(spinUntil(
    2000ms, [this] { return debugStateIs(State::kDecelerating); },
    [this] { publishInputs(0.3, 5.0, -0.3); }));
  received_controls_.clear();
  ASSERT_TRUE(spinUntil(2000ms, [this] { return !received_controls_.empty(); }));

  // Initialized from the measurements available after the restart.
  EXPECT_FLOAT_EQ(last_debug_status_->data.at(debug_index::kHeldSteeringAngle), 0.3f);
  EXPECT_NEAR(received_controls_.back().lateral.steering_tire_angle, 0.3, 1e-6);
}

}  // namespace autoware::mrm_steering_hold_stop_operator
