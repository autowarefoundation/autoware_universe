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

// Tests for CommandModeSubSystemAdapter.
// execute(): verified via topic subscriber.
// on_command_mode_request / check_sub_ecu_error: verified via RecordingProcessor events.

#include "command_mode_subsystem_adapter.hpp"

#include <autoware_command_mode_types/modes.hpp>
#include <rclcpp/rclcpp.hpp>
#include <redundancy_switcher_interface/core_logic/i_processor.hpp>
#include <redundancy_switcher_interface/plugin/command_bus.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>

#include <tier4_system_msgs/msg/active_control_unit.hpp>
#include <tier4_system_msgs/msg/command_mode_availability.hpp>
#include <tier4_system_msgs/msg/command_mode_request.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

namespace autoware::redundancy_switcher
{

class RecordingProcessor : public IProcessor
{
public:
  std::vector<OutputCommand> handle(const InputEvent & event) override
  {
    received.push_back(event);
    return {};
  }
  DomainSnapshot snapshot() const override { return {}; }
  std::vector<InputEvent> received;
};

template <typename T>
static const T * find_event(const std::vector<InputEvent> & events)
{
  for (const auto & e : events) {
    if (const auto * p = std::get_if<T>(&e)) return p;
  }
  return nullptr;
}

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

// ---------------------------------------------------------------------------
// Base fixture
// ---------------------------------------------------------------------------

struct AdapterFixture
{
  explicit AdapterFixture(bool is_main_ecu)
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
      rclcpp::Parameter("is_main_ecu", is_main_ecu),
      rclcpp::Parameter("availability_timeout_milli", 500.0),
    });
    node = std::make_shared<rclcpp::Node>("test_cmd_adapter", opts);
    helper = std::make_shared<rclcpp::Node>("test_cmd_adapter_helper");
    processor = std::make_shared<RecordingProcessor>();
    bus = std::make_shared<CommandBus>();
    gateway = std::make_shared<EventGateway>(processor, bus);
    adapter = std::make_shared<CommandModeSubSystemAdapter>();
    adapter->initialize(node.get(), gateway);
    exec.add_node(node);
    exec.add_node(helper);
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Node::SharedPtr helper;
  std::shared_ptr<RecordingProcessor> processor;
  std::shared_ptr<CommandBus> bus;
  std::shared_ptr<EventGateway> gateway;
  std::shared_ptr<CommandModeSubSystemAdapter> adapter;
  rclcpp::executors::SingleThreadedExecutor exec;
};

// ---------------------------------------------------------------------------
// execute(UpdateActiveControlUnitCommand) — publishes active_control_unit
// ---------------------------------------------------------------------------

TEST(CommandModeSubSystemAdapterExecute, PublishesActiveControlUnit)
{
  AdapterFixture f(/*is_main_ecu=*/true);

  std::optional<std::vector<uint8_t>> received;
  auto sub = f.node->create_subscription<tier4_system_msgs::msg::ActiveControlUnit>(
    "~/output/active_control_unit", rclcpp::QoS(1).transient_local(),
    [&received](const tier4_system_msgs::msg::ActiveControlUnit::ConstSharedPtr msg) {
      received = msg->ids;
    });

  UpdateActiveControlUnitCommand cmd;
  cmd.value.unit_ids = {0u, 2u};
  f.adapter->execute(OutputCommand{cmd});

  EXPECT_TRUE(spin_until(f.exec, [&] { return received.has_value(); }));
  ASSERT_EQ(received->size(), 2u);
  EXPECT_EQ((*received)[0], 0u);
  EXPECT_EQ((*received)[1], 2u);
}

TEST(CommandModeSubSystemAdapterExecute, DeduplicatesIdenticalActiveControlUnit)
{
  AdapterFixture f(/*is_main_ecu=*/true);

  int publish_count = 0;
  auto sub = f.node->create_subscription<tier4_system_msgs::msg::ActiveControlUnit>(
    "~/output/active_control_unit", rclcpp::QoS(1).transient_local(),
    [&publish_count](const tier4_system_msgs::msg::ActiveControlUnit::ConstSharedPtr) {
      ++publish_count;
    });

  UpdateActiveControlUnitCommand cmd;
  cmd.value.unit_ids = {0u, 2u};
  f.adapter->execute(OutputCommand{cmd});
  f.adapter->execute(OutputCommand{cmd});  // same content — should be suppressed

  spin_until(f.exec, [&] { return publish_count > 0; });
  // Give extra time to catch a potential spurious second publication.
  f.exec.spin_some(std::chrono::milliseconds(100));
  EXPECT_EQ(publish_count, 1);
}

// ---------------------------------------------------------------------------
// on_command_mode_request — main ECU self-interruption trigger
// ---------------------------------------------------------------------------

class CommandModeRequestTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    f_ = std::make_unique<AdapterFixture>(/*is_main_ecu=*/true);
    const auto qos = rclcpp::QoS(1).reliable();
    pub_ = f_->helper->create_publisher<tier4_system_msgs::msg::CommandModeRequest>(
      "/test_cmd_adapter/input/command_mode_request", qos);
  }

  void publish_request(uint16_t mode)
  {
    tier4_system_msgs::msg::CommandModeRequest msg;
    tier4_system_msgs::msg::CommandModeRequestItem item;
    item.mode = mode;
    msg.items = {item};
    pub_->publish(msg);
    spin_until(f_->exec, [&, prev = f_->processor->received.size()] {
      return f_->processor->received.size() > prev;
    });
  }

  std::unique_ptr<AdapterFixture> f_;
  rclcpp::Publisher<tier4_system_msgs::msg::CommandModeRequest>::SharedPtr pub_;
};

TEST_F(CommandModeRequestTest, FirstMessage_NoSelfInterruptionEvent)
{
  // First message only sets prev; no event is fired regardless of mode.
  publish_request(autoware::command_mode_types::modes::sub_ecu_standby);
  EXPECT_EQ(find_event<SelfInterruptionEvent>(f_->processor->received), nullptr);
}

TEST_F(CommandModeRequestTest, ModeChangeTo_SubEcuStandby_FiresSelfInterruption)
{
  namespace modes = autoware::command_mode_types::modes;
  // First message: establish prev.
  publish_request(modes::main_ecu_in_lane_emergency_stop);
  const std::size_t before = f_->processor->received.size();

  // Second message: mode changes to sub_ecu_standby → SelfInterruptionEvent.
  publish_request(modes::sub_ecu_standby);
  EXPECT_GT(f_->processor->received.size(), before);
  EXPECT_NE(find_event<SelfInterruptionEvent>(f_->processor->received), nullptr);
}

TEST_F(CommandModeRequestTest, ModeChangeTo_SubEcuModeStop_FiresSelfInterruption)
{
  namespace modes = autoware::command_mode_types::modes;
  publish_request(modes::main_ecu_in_lane_emergency_stop);
  const std::size_t before = f_->processor->received.size();

  publish_request(modes::sub_ecu_in_lane_moderate_stop);
  EXPECT_GT(f_->processor->received.size(), before);
  EXPECT_NE(find_event<SelfInterruptionEvent>(f_->processor->received), nullptr);
}

TEST_F(CommandModeRequestTest, UnchangedMode_NoAdditionalEvent)
{
  namespace modes = autoware::command_mode_types::modes;
  publish_request(modes::sub_ecu_standby);
  const std::size_t after_first = f_->processor->received.size();

  // Publish identical message: items unchanged → no new event.
  tier4_system_msgs::msg::CommandModeRequest msg;
  tier4_system_msgs::msg::CommandModeRequestItem item;
  item.mode = modes::sub_ecu_standby;
  msg.items = {item};
  pub_->publish(msg);
  f_->exec.spin_some(std::chrono::milliseconds(100));
  EXPECT_EQ(f_->processor->received.size(), after_first);
}

TEST(CommandModeRequestSubEcu, SubEcu_CommandModeRequest_IsIgnored)
{
  AdapterFixture f(/*is_main_ecu=*/false);

  const auto qos = rclcpp::QoS(1).reliable();
  auto pub = f.helper->create_publisher<tier4_system_msgs::msg::CommandModeRequest>(
    "/test_cmd_adapter/input/command_mode_request", qos);

  namespace modes = autoware::command_mode_types::modes;
  tier4_system_msgs::msg::CommandModeRequest msg;
  tier4_system_msgs::msg::CommandModeRequestItem item;
  item.mode = modes::sub_ecu_standby;
  msg.items = {item};
  pub->publish(msg);
  f.exec.spin_some(std::chrono::milliseconds(100));

  EXPECT_EQ(find_event<SelfInterruptionEvent>(f.processor->received), nullptr);
}

// ---------------------------------------------------------------------------
// check_sub_ecu_error — sub ECU self-interruption trigger via availability
// ---------------------------------------------------------------------------

class CommandModeAvailabilityTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    f_ = std::make_unique<AdapterFixture>(/*is_main_ecu=*/false);
    const auto qos = rclcpp::QoS(1).reliable();
    pub_ = f_->helper->create_publisher<tier4_system_msgs::msg::CommandModeAvailability>(
      "/test_cmd_adapter/input/command_mode_availability", qos);
  }

  void publish_availability(uint16_t mode, bool available)
  {
    tier4_system_msgs::msg::CommandModeAvailability msg;
    tier4_system_msgs::msg::CommandModeAvailabilityItem item;
    item.mode = mode;
    item.available = available;
    msg.items = {item};
    pub_->publish(msg);
    spin_until(f_->exec, [&, prev = f_->processor->received.size()] {
      return f_->processor->received.size() > prev;
    });
  }

  std::unique_ptr<AdapterFixture> f_;
  rclcpp::Publisher<tier4_system_msgs::msg::CommandModeAvailability>::SharedPtr pub_;
};

TEST_F(CommandModeAvailabilityTest, SubEcuModeUnavailable_FiresSelfInterruption)
{
  namespace modes = autoware::command_mode_types::modes;
  publish_availability(modes::sub_ecu_in_lane_moderate_stop, /*available=*/false);
  EXPECT_NE(find_event<SelfInterruptionEvent>(f_->processor->received), nullptr);
}

TEST_F(CommandModeAvailabilityTest, SubEcuModeAvailable_NoSelfInterruption)
{
  namespace modes = autoware::command_mode_types::modes;
  publish_availability(modes::sub_ecu_in_lane_moderate_stop, /*available=*/true);
  EXPECT_EQ(find_event<SelfInterruptionEvent>(f_->processor->received), nullptr);
}

TEST(CommandModeAvailabilityMainEcu, MainEcu_SubEcuModeUnavailable_IsIgnored)
{
  AdapterFixture f(/*is_main_ecu=*/true);

  const auto qos = rclcpp::QoS(1).reliable();
  auto pub = f.helper->create_publisher<tier4_system_msgs::msg::CommandModeAvailability>(
    "/test_cmd_adapter/input/command_mode_availability", qos);

  namespace modes = autoware::command_mode_types::modes;
  tier4_system_msgs::msg::CommandModeAvailability msg;
  tier4_system_msgs::msg::CommandModeAvailabilityItem item;
  item.mode = modes::sub_ecu_in_lane_moderate_stop;
  item.available = false;
  msg.items = {item};
  pub->publish(msg);
  spin_until(f.exec, [&, prev = f.processor->received.size()] {
    return f.processor->received.size() > prev;
  });

  EXPECT_EQ(find_event<SelfInterruptionEvent>(f.processor->received), nullptr);
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
