// Copyright 2026 The Autoware Contributors
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

#include "core/interface.hpp"
#include "core/selector.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace autoware::trajectory_gate
{

class TestReceiver : public TrajectoryReceiver
{
public:
  void receive(const Trajectory & msg) override
  {
    ++count;
    last_frame_id = msg.header.frame_id;
  }

  size_t count{0};
  std::string last_frame_id;
};

class TestSender : public TrajectorySender
{
public:
  void emit(const Trajectory & msg) { send(msg); }
};

TEST(TrajectorySenderTest, send_without_output_is_no_operation)
{
  TestSender sender;
  TestReceiver receiver;
  Trajectory msg;
  msg.header.frame_id = "map";

  // No output is connected, so emit must be a no-op.
  sender.emit(msg);

  EXPECT_EQ(receiver.count, 0U);
}

TEST(TrajectorySenderTest, send_forwards_message_to_output)
{
  TestSender sender;
  TestReceiver receiver;
  Trajectory msg;
  msg.header.frame_id = "map";
  sender.set_output(&receiver);

  // Once connected, emit should forward the message to the receiver.
  sender.emit(msg);

  EXPECT_EQ(receiver.count, 1U);
  EXPECT_EQ(receiver.last_frame_id, "map");
}

TEST(TrajectorySelectorTest, source_defaults_to_invalid)
{
  TrajectorySelector selector;
  EXPECT_EQ(selector.source(), 0U);
}

TEST(TrajectorySelectorTest, add_input_rejects_invalid_source_id)
{
  TrajectorySelector selector;
  TestSender input;

  EXPECT_THROW(selector.add_input(&input, 0U), std::runtime_error);
}

TEST(TrajectorySelectorTest, add_input_rejects_duplicate_source_id)
{
  TrajectorySelector selector;
  TestSender input_a;
  TestSender input_b;
  selector.add_input(&input_a, 1U);

  EXPECT_THROW(selector.add_input(&input_b, 1U), std::runtime_error);
}

TEST(TrajectorySelectorTest, set_output_rejects_duplicate_output)
{
  TrajectorySelector selector;
  TestReceiver output_a;
  TestReceiver output_b;
  selector.set_output(&output_a);

  EXPECT_THROW(selector.set_output(&output_b), std::runtime_error);
}

TEST(TrajectorySelectorTest, select_unknown_source_resets_current_source)
{
  TrajectorySelector selector;
  TestSender input;
  TestReceiver output;
  selector.add_input(&input, 1U);
  selector.set_output(&output);

  // Route source 1 to output and confirm the message is delivered.
  ASSERT_TRUE(selector.select(1U));
  EXPECT_EQ(selector.source(), 1U);

  Trajectory msg;
  msg.header.frame_id = "first";
  input.emit(msg);
  EXPECT_EQ(output.count, 1U);

  // Unknown source selection resets to invalid and disconnects current input.
  EXPECT_FALSE(selector.select(999U));
  EXPECT_EQ(selector.source(), 0U);

  input.emit(msg);
  EXPECT_EQ(output.count, 1U);
}

TEST(TrajectorySelectorTest, select_switches_routing_to_selected_input)
{
  TrajectorySelector selector;
  TestSender input_a;
  TestSender input_b;
  TestReceiver output;
  selector.add_input(&input_a, 1U);
  selector.add_input(&input_b, 2U);
  selector.set_output(&output);

  // Select source 1 and verify only input_a is routed.
  ASSERT_TRUE(selector.select(1U));
  Trajectory msg_a;
  msg_a.header.frame_id = "a";
  input_a.emit(msg_a);
  EXPECT_EQ(output.count, 1U);
  EXPECT_EQ(output.last_frame_id, "a");

  // Switch to source 2: input_a should be disconnected.
  ASSERT_TRUE(selector.select(2U));
  input_a.emit(msg_a);
  EXPECT_EQ(output.count, 1U);

  // input_b is now active and should be delivered.
  Trajectory msg_b;
  msg_b.header.frame_id = "b";
  input_b.emit(msg_b);
  EXPECT_EQ(output.count, 2U);
  EXPECT_EQ(output.last_frame_id, "b");
}

}  // namespace autoware::trajectory_gate
