//  Copyright 2025 The Autoware Contributors
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

// Tests for NonRedundantSwitcherAdapter.
// Requires rclcpp (node is passed to initialize() for logging only).

#include "non_redundant_adapter.hpp"

#include <redundancy_switcher_interface/core_logic/i_processor.hpp>
#include <redundancy_switcher_interface/plugin/command_bus.hpp>
#include <redundancy_switcher_interface/plugin/event_gateway.hpp>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace autoware::redundancy_switcher
{

// Minimal IProcessor mock that records every submitted InputEvent.
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

// Helper to find a specific event type in the recorded list.
template <typename T>
const T * find_event(const std::vector<InputEvent> & events)
{
  for (const auto & e : events) {
    if (const auto * p = std::get_if<T>(&e)) return p;
  }
  return nullptr;
}

class NonRedundantAdapterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("is_main_ecu", true)});
    node_ = std::make_shared<rclcpp::Node>("test_non_redundant_adapter", options);
    processor_ = std::make_shared<RecordingProcessor>();
    bus_ = std::make_shared<CommandBus>();
    gateway_ = std::make_shared<EventGateway>(processor_, bus_);
    adapter_ = std::make_shared<NonRedundantSwitcherAdapter>();
    adapter_->initialize(node_.get(), gateway_);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<RecordingProcessor> processor_;
  std::shared_ptr<CommandBus> bus_;
  std::shared_ptr<EventGateway> gateway_;
  std::shared_ptr<NonRedundantSwitcherAdapter> adapter_;
};

// initialize() must submit SetSwitcherSignalsEvent as the first event.
TEST_F(NonRedundantAdapterTest, Initialize_SubmitsSwitcherSignalsEvent)
{
  const auto * ev = find_event<SetSwitcherSignalsEvent>(processor_->received);
  ASSERT_NE(ev, nullptr);
  EXPECT_TRUE(ev->value.value.is_stable);
  EXPECT_FALSE(ev->value.value.is_self_interrupted);
  EXPECT_FALSE(ev->value.value.is_faulted);
}

// initialize() must submit SetActiveControlUnitEvent as the second event.
TEST_F(NonRedundantAdapterTest, Initialize_SubmitsActiveControlUnitEvent)
{
  const auto * ev = find_event<SetActiveControlUnitEvent>(processor_->received);
  ASSERT_NE(ev, nullptr);
  ASSERT_EQ(ev->value.value.unit_ids.size(), 2u);
  EXPECT_EQ(ev->value.value.unit_ids[0], 0u);
  EXPECT_EQ(ev->value.value.unit_ids[1], 2u);
}

// initialize() must submit exactly two events in the correct order.
TEST_F(NonRedundantAdapterTest, Initialize_SubmitsExactlyTwoEventsInOrder)
{
  ASSERT_EQ(processor_->received.size(), 2u);
  EXPECT_TRUE(std::holds_alternative<SetSwitcherSignalsEvent>(processor_->received[0]));
  EXPECT_TRUE(std::holds_alternative<SetActiveControlUnitEvent>(processor_->received[1]));
}

// execute() must not submit any further events for any command type.
TEST_F(NonRedundantAdapterTest, Execute_SelfInterruptionCommand_DoesNothing)
{
  const std::size_t before = processor_->received.size();
  adapter_->execute(OutputCommand{SelfInterruptionCommand{}});
  EXPECT_EQ(processor_->received.size(), before);
}

TEST_F(NonRedundantAdapterTest, Execute_ResetCommand_DoesNothing)
{
  const std::size_t before = processor_->received.size();
  adapter_->execute(OutputCommand{ResetCommand{}});
  EXPECT_EQ(processor_->received.size(), before);
}

TEST_F(NonRedundantAdapterTest, Execute_UpdateAutowareReadyCommand_DoesNothing)
{
  const std::size_t before = processor_->received.size();
  adapter_->execute(OutputCommand{UpdateAutowareReadyCommand{AutowareReady::True}});
  EXPECT_EQ(processor_->received.size(), before);
}

TEST_F(NonRedundantAdapterTest, Execute_UpdateAnotherEcuAvailabilityTimeoutCommand_DoesNothing)
{
  const std::size_t before = processor_->received.size();
  adapter_->execute(OutputCommand{UpdateAnotherEcuAvailabilityTimeoutCommand{true}});
  EXPECT_EQ(processor_->received.size(), before);
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
