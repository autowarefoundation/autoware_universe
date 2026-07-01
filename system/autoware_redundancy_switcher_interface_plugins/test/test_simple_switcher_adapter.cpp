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

// Tests for SimpleSwitcherAdapter.
// Static helper: no ROS context required.
// execute() tests: require rclcpp (topic-based publish verification).

#include "switcher_adapter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/core_logic/i_processor.hpp>
#include <redundancy_switcher_interface/plugin/command_bus.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>

namespace autoware::redundancy_switcher
{

// ---------------------------------------------------------------------------
// decode_signals (static pure function)
// ---------------------------------------------------------------------------

TEST(DecodeSwitcherSignals, AllFalse)
{
  const auto s = SimpleSwitcherAdapter::decode_signals(0x00);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(DecodeSwitcherSignals, IsStable)
{
  const auto s = SimpleSwitcherAdapter::decode_signals(0x01);
  EXPECT_TRUE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(DecodeSwitcherSignals, IsSelfInterrupted)
{
  const auto s = SimpleSwitcherAdapter::decode_signals(0x02);
  EXPECT_FALSE(s.is_stable);
  EXPECT_TRUE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

TEST(DecodeSwitcherSignals, IsFaulted)
{
  const auto s = SimpleSwitcherAdapter::decode_signals(0x04);
  EXPECT_FALSE(s.is_stable);
  EXPECT_FALSE(s.is_self_interrupted);
  EXPECT_TRUE(s.is_faulted);
}

TEST(DecodeSwitcherSignals, StableAndSelfInterrupted)
{
  const auto s = SimpleSwitcherAdapter::decode_signals(0x03);
  EXPECT_TRUE(s.is_stable);
  EXPECT_TRUE(s.is_self_interrupted);
  EXPECT_FALSE(s.is_faulted);
}

// ---------------------------------------------------------------------------
// execute() — topic publication (requires ROS)
// ---------------------------------------------------------------------------

class NullProcessor : public IProcessor
{
public:
  std::vector<OutputCommand> handle(const InputEvent &) override { return {}; }
  DomainSnapshot snapshot() const override { return {}; }
};

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

class SimpleSwitcherAdapterMainEcuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("is_main_ecu", true)});
    node_ = std::make_shared<rclcpp::Node>("test_switcher_adapter_main", opts);
    auto proc = std::make_shared<NullProcessor>();
    auto bus = std::make_shared<CommandBus>();
    gateway_ = std::make_shared<EventGateway>(proc, bus);
    adapter_ = std::make_shared<SimpleSwitcherAdapter>();
    adapter_->initialize(node_.get(), gateway_);
    exec_.add_node(node_);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<EventGateway> gateway_;
  std::shared_ptr<SimpleSwitcherAdapter> adapter_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

TEST_F(SimpleSwitcherAdapterMainEcuTest, Execute_UpdatePriority_PublishesToMainPriorityTopic)
{
  std::optional<uint16_t> received;
  auto sub = node_->create_subscription<std_msgs::msg::UInt16>(
    "/system/simple_switcher/request/priority/main_ecu", rclcpp::QoS(1).reliable(),
    [&received](const std_msgs::msg::UInt16::ConstSharedPtr msg) { received = msg->data; });

  adapter_->execute(OutputCommand{UpdatePriorityCommand{42}});
  EXPECT_TRUE(spin_until(exec_, [&] { return received.has_value(); }));
  EXPECT_EQ(*received, 42u);
}

TEST_F(SimpleSwitcherAdapterMainEcuTest, Execute_SelfInterruption_PublishesToMainSelfTopic)
{
  bool received = false;
  auto sub = node_->create_subscription<std_msgs::msg::Empty>(
    "/system/simple_switcher/request/self_interruption/main_ecu", rclcpp::QoS(1).reliable(),
    [&received](const std_msgs::msg::Empty::ConstSharedPtr) { received = true; });

  adapter_->execute(OutputCommand{SelfInterruptionCommand{}});
  EXPECT_TRUE(spin_until(exec_, [&] { return received; }));
}

TEST_F(SimpleSwitcherAdapterMainEcuTest, Execute_Reset_PublishesToResetTopic)
{
  bool received = false;
  auto sub = node_->create_subscription<std_msgs::msg::Empty>(
    "/system/simple_switcher/request/reset", rclcpp::QoS(1).reliable(),
    [&received](const std_msgs::msg::Empty::ConstSharedPtr) { received = true; });

  adapter_->execute(OutputCommand{ResetCommand{}});
  EXPECT_TRUE(spin_until(exec_, [&] { return received; }));
}

class SimpleSwitcherAdapterSubEcuTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({rclcpp::Parameter("is_main_ecu", false)});
    node_ = std::make_shared<rclcpp::Node>("test_switcher_adapter_sub", opts);
    auto proc = std::make_shared<NullProcessor>();
    auto bus = std::make_shared<CommandBus>();
    gateway_ = std::make_shared<EventGateway>(proc, bus);
    adapter_ = std::make_shared<SimpleSwitcherAdapter>();
    adapter_->initialize(node_.get(), gateway_);
    exec_.add_node(node_);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<EventGateway> gateway_;
  std::shared_ptr<SimpleSwitcherAdapter> adapter_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

TEST_F(SimpleSwitcherAdapterSubEcuTest, Execute_UpdatePriority_PublishesToSubPriorityTopic)
{
  std::optional<uint16_t> received;
  auto sub = node_->create_subscription<std_msgs::msg::UInt16>(
    "/system/simple_switcher/request/priority/sub_ecu", rclcpp::QoS(1).reliable(),
    [&received](const std_msgs::msg::UInt16::ConstSharedPtr msg) { received = msg->data; });

  adapter_->execute(OutputCommand{UpdatePriorityCommand{10}});
  EXPECT_TRUE(spin_until(exec_, [&] { return received.has_value(); }));
  EXPECT_EQ(*received, 10u);
}

TEST_F(SimpleSwitcherAdapterSubEcuTest, Execute_SelfInterruption_PublishesToSubSelfTopic)
{
  bool received = false;
  auto sub = node_->create_subscription<std_msgs::msg::Empty>(
    "/system/simple_switcher/request/self_interruption/sub_ecu", rclcpp::QoS(1).reliable(),
    [&received](const std_msgs::msg::Empty::ConstSharedPtr) { received = true; });

  adapter_->execute(OutputCommand{SelfInterruptionCommand{}});
  EXPECT_TRUE(spin_until(exec_, [&] { return received; }));
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
