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

#include "redundancy_switcher_interface/core_logic/i_processor.hpp"
#include "redundancy_switcher_interface/plugin/command_bus.hpp"
#include "redundancy_switcher_interface/plugin/event_gateway.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::redundancy_switcher
{

class MockProcessor : public IProcessor
{
public:
  explicit MockProcessor(std::vector<OutputCommand> effects_to_return = {})
  : effects_to_return_(std::move(effects_to_return))
  {
  }

  std::vector<OutputCommand> handle(const InputEvent & event) override
  {
    received_events_.push_back(event);
    return effects_to_return_;
  }

  DomainSnapshot snapshot() const override { return snapshot_; }

  const std::vector<InputEvent> & received_events() const { return received_events_; }

  void set_effects(std::vector<OutputCommand> commands)
  {
    effects_to_return_ = std::move(commands);
  }

private:
  std::vector<OutputCommand> effects_to_return_;
  std::vector<InputEvent> received_events_;
  DomainSnapshot snapshot_;
};

class SpyAdapter : public IAdapterPlugin
{
public:
  void initialize(rclcpp::Node *, std::shared_ptr<EventGateway>) override {}

  void execute(const OutputCommand & command) override { captured_effects_.push_back(command); }

  const std::vector<OutputCommand> & captured() const { return captured_effects_; }

  void clear() { captured_effects_.clear(); }

  template <typename T>
  const T * find_effect() const
  {
    for (const auto & e : captured_effects_) {
      if (const auto * p = std::get_if<T>(&e)) return p;
    }
    return nullptr;
  }

  template <typename T>
  bool has_effect() const
  {
    return find_effect<T>() != nullptr;
  }

private:
  std::vector<OutputCommand> captured_effects_;
};

class EventGatewayTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    mock_processor_ = std::make_shared<MockProcessor>();
    command_bus_ = std::make_shared<CommandBus>();
    gateway_ = std::make_shared<EventGateway>(mock_processor_, command_bus_);

    spy_ = std::make_shared<SpyAdapter>();
    command_bus_->add_handler(spy_);
  }

  std::shared_ptr<MockProcessor> mock_processor_;
  std::shared_ptr<CommandBus> command_bus_;
  std::shared_ptr<EventGateway> gateway_;
  std::shared_ptr<SpyAdapter> spy_;
};

TEST_F(EventGatewayTest, Submit_ForwardsEventToProcessor)
{
  gateway_->submit(InputEvent{ResetEvent{}});

  ASSERT_EQ(mock_processor_->received_events().size(), 1u);
  const auto & ev = mock_processor_->received_events().front();
  ASSERT_TRUE(std::holds_alternative<ResetEvent>(ev));
}

TEST_F(EventGatewayTest, Submit_DispatchesEffectsToAdapter)
{
  mock_processor_->set_effects({
    LogCommand{LogLevel::Info, "test"},
    SelfInterruptionCommand{},
    UpdateStatusDiagCommand{DomainSnapshot{}},
  });

  gateway_->submit(InputEvent{SelfInterruptionEvent{}});

  EXPECT_EQ(spy_->captured().size(), 3u);
  EXPECT_TRUE(spy_->has_effect<LogCommand>());
  EXPECT_TRUE(spy_->has_effect<SelfInterruptionCommand>());
  EXPECT_TRUE(spy_->has_effect<UpdateStatusDiagCommand>());
}

TEST_F(EventGatewayTest, Submit_NoEffects_AdapterReceivesNothing)
{
  mock_processor_->set_effects({});

  gateway_->submit(InputEvent{ResetEvent{}});

  EXPECT_TRUE(spy_->captured().empty());
}

TEST_F(EventGatewayTest, ExpiredAdapter_IsNotCalled)
{
  mock_processor_->set_effects({LogCommand{LogLevel::Info, "test"}});

  spy_.reset();

  EXPECT_NO_THROW(gateway_->submit(InputEvent{ResetEvent{}}));
}

TEST_F(EventGatewayTest, SubmitRequest_ReturnsCommandsAndDispatchesToAdapter)
{
  mock_processor_->set_effects({
    ResetCommand{},
    ResetResultCommand{true, {}, "Reset accepted."},
    LogCommand{LogLevel::Info, "test"},
  });

  const auto commands = gateway_->submit_request(InputEvent{ResetEvent{}});

  ASSERT_EQ(commands.size(), 3u);
  EXPECT_TRUE(std::any_of(commands.begin(), commands.end(), [](const auto & c) {
    return std::holds_alternative<ResetResultCommand>(c);
  }));

  EXPECT_EQ(spy_->captured().size(), 3u);
  EXPECT_TRUE(spy_->has_effect<ResetCommand>());
  EXPECT_TRUE(spy_->has_effect<ResetResultCommand>());
}

TEST_F(EventGatewayTest, SubmitRequest_ResetResult_ReflectsAccepted)
{
  mock_processor_->set_effects({ResetResultCommand{false, ResetRejectedReason::Error, "rejected"}});

  const auto commands = gateway_->submit_request(InputEvent{ResetEvent{}});

  const auto it = std::find_if(commands.begin(), commands.end(), [](const auto & c) {
    return std::holds_alternative<ResetResultCommand>(c);
  });
  ASSERT_NE(it, commands.end());
  EXPECT_FALSE(std::get<ResetResultCommand>(*it).accepted);
  EXPECT_EQ(std::get<ResetResultCommand>(*it).message, "rejected");
}

TEST(CommandBusTest, Dispatch_CallsAllRegisteredAdapters)
{
  auto bus = std::make_shared<CommandBus>();
  auto spy1 = std::make_shared<SpyAdapter>();
  auto spy2 = std::make_shared<SpyAdapter>();
  bus->add_handler(spy1);
  bus->add_handler(spy2);

  std::vector<OutputCommand> commands = {LogCommand{LogLevel::Info, "test"}};
  bus->dispatch(commands);

  EXPECT_EQ(spy1->captured().size(), 1u);
  EXPECT_EQ(spy2->captured().size(), 1u);
}

TEST(CommandBusTest, Dispatch_SkipsExpiredHandlers)
{
  auto bus = std::make_shared<CommandBus>();
  auto spy = std::make_shared<SpyAdapter>();
  bus->add_handler(spy);
  spy.reset();

  std::vector<OutputCommand> commands = {LogCommand{LogLevel::Info, "test"}};
  EXPECT_NO_THROW(bus->dispatch(commands));
}

}  // namespace autoware::redundancy_switcher
