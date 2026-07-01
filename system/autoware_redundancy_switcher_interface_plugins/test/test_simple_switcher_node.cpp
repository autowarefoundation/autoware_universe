//  Copyright 2026 The Autoware Contributors
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

// Tests for SimpleSwitcherNode.
// encode_signals: static pure function, no ROS required.
// Switching behavior: exercised via topic pub/sub with an executor.

#include "simple_switcher_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <tier4_system_msgs/msg/active_control_unit.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::redundancy_switcher
{

// ---------------------------------------------------------------------------
// encode_signals (static pure function)
// ---------------------------------------------------------------------------

TEST(EncodeSignals, AllFalse)
{
  EXPECT_EQ(SimpleSwitcherNode::encode_signals(false, false, false), 0x00u);
}

TEST(EncodeSignals, Stable)
{
  EXPECT_EQ(SimpleSwitcherNode::encode_signals(true, false, false), 0x01u);
}

TEST(EncodeSignals, SelfInterrupted)
{
  EXPECT_EQ(SimpleSwitcherNode::encode_signals(false, true, false), 0x02u);
}

TEST(EncodeSignals, Faulted)
{
  EXPECT_EQ(SimpleSwitcherNode::encode_signals(false, false, true), 0x04u);
}

// ---------------------------------------------------------------------------
// Switching behavior (requires ROS)
// ---------------------------------------------------------------------------

template <typename Pred>
static bool spin_until(
  rclcpp::Executor & exec, Pred pred,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(500))
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred() && std::chrono::steady_clock::now() < deadline) {
    exec.spin_some(std::chrono::milliseconds(10));
  }
  return pred();
}

class SimpleSwitcherNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    // Large period to keep timer from interfering with expected message counts.
    opts.parameter_overrides({rclcpp::Parameter("publish_period_ms", int64_t{10000})});
    switcher_ = std::make_shared<SimpleSwitcherNode>(opts);
    helper_ = std::make_shared<rclcpp::Node>("test_simple_switcher_helper");
    exec_.add_node(switcher_);
    exec_.add_node(helper_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    sub_active_ = helper_->create_subscription<tier4_system_msgs::msg::ActiveControlUnit>(
      "/system/simple_switcher/status/active_control_unit", qos,
      [this](const tier4_system_msgs::msg::ActiveControlUnit::ConstSharedPtr msg) {
        last_active_ids_ = msg->ids;
      });

    pub_self_main_ = helper_->create_publisher<std_msgs::msg::Empty>(
      "/system/simple_switcher/request/self_interruption/main_ecu", qos);
    pub_self_sub_ = helper_->create_publisher<std_msgs::msg::Empty>(
      "/system/simple_switcher/request/self_interruption/sub_ecu", qos);
    pub_reset_ = helper_->create_publisher<std_msgs::msg::Empty>(
      "/system/simple_switcher/request/reset", qos);
    pub_priority_main_ = helper_->create_publisher<std_msgs::msg::UInt16>(
      "/system/simple_switcher/request/priority/main_ecu", qos);
    pub_priority_sub_ = helper_->create_publisher<std_msgs::msg::UInt16>(
      "/system/simple_switcher/request/priority/sub_ecu", qos);

    // Drain the startup publish_status() from the constructor.
    spin_until(exec_, [this] { return !last_active_ids_.empty(); });
    last_active_ids_.clear();
  }

  void publish_and_wait(
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub, bool clear_first = true)
  {
    if (clear_first) last_active_ids_.clear();
    pub->publish(std_msgs::msg::Empty{});
    spin_until(exec_, [this] { return !last_active_ids_.empty(); });
  }

  void publish_priority_main(uint16_t p, bool clear_first = true)
  {
    if (clear_first) last_active_ids_.clear();
    std_msgs::msg::UInt16 msg;
    msg.data = p;
    pub_priority_main_->publish(msg);
    spin_until(exec_, [this] { return !last_active_ids_.empty(); });
  }

  void publish_priority_sub(uint16_t p, bool clear_first = true)
  {
    if (clear_first) last_active_ids_.clear();
    std_msgs::msg::UInt16 msg;
    msg.data = p;
    pub_priority_sub_->publish(msg);
    spin_until(exec_, [this] { return !last_active_ids_.empty(); });
  }

  std::shared_ptr<SimpleSwitcherNode> switcher_;
  std::shared_ptr<rclcpp::Node> helper_;
  rclcpp::executors::SingleThreadedExecutor exec_;

  rclcpp::Subscription<tier4_system_msgs::msg::ActiveControlUnit>::SharedPtr sub_active_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_self_main_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_self_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_reset_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_priority_main_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_priority_sub_;

  std::vector<uint8_t> last_active_ids_;
};

// --- Self-interruption ---

TEST_F(SimpleSwitcherNodeTest, SelfMain_SwitchesToSub)
{
  publish_and_wait(pub_self_main_);
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 1u);  // sub_ecu_id=1
}

TEST_F(SimpleSwitcherNodeTest, SelfSub_StaysMain)
{
  publish_and_wait(pub_self_sub_);
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 0u);  // main_ecu_id=0
}

TEST_F(SimpleSwitcherNodeTest, BothSelfInterrupted_ActiveIsEmpty)
{
  publish_and_wait(pub_self_main_);
  publish_and_wait(pub_self_sub_);
  EXPECT_TRUE(last_active_ids_.empty());
}

// --- Reset ---

TEST_F(SimpleSwitcherNodeTest, Reset_AfterMainInterruption_RestoresMain)
{
  publish_and_wait(pub_self_main_);
  publish_and_wait(pub_reset_);
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 0u);  // main_ecu_id=0
}

// --- Priority switching ---

TEST_F(SimpleSwitcherNodeTest, Priority_SubHigher_SwitchesToSub)
{
  // main_priority=0 (default), sub_priority=0 (default) → equal, stays main.
  // Set main_priority=10 so sub (still 0) becomes higher priority.
  publish_priority_main(10);
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 1u);  // sub_ecu_id=1
}

TEST_F(SimpleSwitcherNodeTest, Priority_MainHigher_StaysMain)
{
  // Start with sub at higher priority (sub=10, main=default 0) → active=main.
  publish_priority_sub(10);
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 0u);  // main_ecu_id=0
}

TEST_F(SimpleSwitcherNodeTest, Priority_Equal_KeepsCurrentActive)
{
  // Make sub active first (main=5 > sub=0 → sub wins), then set equal.
  // Equal priorities must keep the currently active ECU (sub).
  publish_priority_main(5);
  ASSERT_EQ(last_active_ids_[0], 1u);  // sub is now active

  publish_priority_sub(5);  // both=5 → equal → no change
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 1u);  // sub stays active
}

TEST_F(SimpleSwitcherNodeTest, Priority_ClearsInterruptionFlags)
{
  // After main self-interrupts (active=sub), a priority update that gives
  // main lower value should restore main and clear both interruption flags.
  publish_and_wait(pub_self_main_);  // active=sub
  ASSERT_EQ(last_active_ids_[0], 1u);

  // sub_priority stays 0; set main_priority=0 too → equal, no change via priority.
  // Instead, set sub to higher value so main wins.
  publish_priority_sub(5);  // sub=5, main=0 → main wins, flags cleared
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 0u);  // main restored

  // Verify flags cleared: self-sub should now switch to main (no dual-interrupted faulted state).
  publish_and_wait(pub_self_sub_);
  ASSERT_EQ(last_active_ids_.size(), 1u);
  EXPECT_EQ(last_active_ids_[0], 0u);  // main is still active (sub interrupted only)
}

}  // namespace autoware::redundancy_switcher

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
