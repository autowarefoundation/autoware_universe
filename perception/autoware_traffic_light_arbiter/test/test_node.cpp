// Copyright 2024 TIER IV, Inc.
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

#include "autoware/traffic_light_arbiter/traffic_light_arbiter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

using autoware::traffic_light::TrafficLightArbiter;
using LaneletMapBin = autoware_map_msgs::msg::LaneletMapBin;
using TrafficSignalArray = autoware_perception_msgs::msg::TrafficLightGroupArray;
using TrafficSignal = autoware_perception_msgs::msg::TrafficLightGroup;
using TrafficElement = autoware_perception_msgs::msg::TrafficLightElement;
using PredictedTrafficLightState = autoware_perception_msgs::msg::PredictedTrafficLightState;

std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  return test_manager;
}

std::shared_ptr<TrafficLightArbiter> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto package_dir =
    ament_index_cpp::get_package_share_directory("autoware_traffic_light_arbiter");
  node_options.arguments(
    {"--ros-args", "--params-file", package_dir + "/config/traffic_light_arbiter.param.yaml"});
  return std::make_shared<TrafficLightArbiter>(node_options);
}

void generateMap(LaneletMapBin & vector_map_msg)
{
  const auto package_dir = ament_index_cpp::get_package_share_directory("autoware_test_utils");
  lanelet::LaneletMapPtr vector_map_ptr =
    autoware::test_utils::loadMap(package_dir + "/test_map/lanelet2_map.osm");
  vector_map_msg.header.frame_id = "base_link";
  lanelet::utils::conversion::toBinMsg(vector_map_ptr, &vector_map_msg);
}

void generatePerceptionMsg(
  TrafficSignalArray & perception_msg, const builtin_interfaces::msg::Time & time)
{
  perception_msg.stamp = time;
  // traffic_light_group_id 1: 1012
  {
    TrafficSignal traffic_light_groups;
    traffic_light_groups.traffic_light_group_id = 1012;
    // elements 1: red + circle
    {
      TrafficElement elements;
      elements.color = TrafficElement::RED;
      elements.shape = TrafficElement::CIRCLE;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 0.9;
      traffic_light_groups.elements.push_back(elements);
    }
    // elements 2: green + right arrow
    {
      TrafficElement elements;
      elements.color = TrafficElement::GREEN;
      elements.shape = TrafficElement::RIGHT_ARROW;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 0.8;
      traffic_light_groups.elements.push_back(elements);
    }
    // predicted state
    {
      PredictedTrafficLightState predictions;
      predictions.predicted_stamp = time;
      predictions.predicted_stamp.sec += 10;
      {
        TrafficElement elements;
        elements.color = TrafficElement::RED;
        elements.shape = TrafficElement::CIRCLE;
        elements.status = TrafficElement::SOLID_ON;
        elements.confidence = 0.7;
        predictions.simultaneous_elements.push_back(elements);
      }
      predictions.reliability = 1.0;
      predictions.information_source =
        PredictedTrafficLightState::INFORMATION_SOURCE_INTERNAL_ESTIMATION;
      traffic_light_groups.predictions.push_back(predictions);
    }
    perception_msg.traffic_light_groups.push_back(traffic_light_groups);
  }
}

void generateExternalMsg(
  TrafficSignalArray & external_msg, const builtin_interfaces::msg::Time & time)
{
  external_msg.stamp = time;
  // traffic_light_group_id 1: 1012
  {
    TrafficSignal traffic_light_groups;
    traffic_light_groups.traffic_light_group_id = 1012;
    // elements 1: green + circle
    {
      TrafficElement elements;
      elements.color = TrafficElement::GREEN;
      elements.shape = TrafficElement::CIRCLE;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 0.6;
      traffic_light_groups.elements.push_back(elements);
    }
    // elements 2: green + right arrow
    {
      TrafficElement elements;
      elements.color = TrafficElement::GREEN;
      elements.shape = TrafficElement::RIGHT_ARROW;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 0.5;
      traffic_light_groups.elements.push_back(elements);
    }
    // predicted state
    {
      PredictedTrafficLightState predictions;
      predictions.predicted_stamp = time;
      predictions.predicted_stamp.sec += 10;
      {
        TrafficElement elements;
        elements.color = TrafficElement::GREEN;
        elements.shape = TrafficElement::CIRCLE;
        elements.status = TrafficElement::SOLID_ON;
        elements.confidence = 1.0;
        predictions.simultaneous_elements.push_back(elements);
      }
      predictions.reliability = 1.0;
      predictions.information_source = PredictedTrafficLightState::INFORMATION_SOURCE_V2I;
      traffic_light_groups.predictions.push_back(predictions);
    }
    {
      PredictedTrafficLightState predictions;
      predictions.predicted_stamp = time;
      predictions.predicted_stamp.sec += 20;
      {
        TrafficElement elements;
        elements.color = TrafficElement::RED;
        elements.shape = TrafficElement::CIRCLE;
        elements.status = TrafficElement::SOLID_ON;
        elements.confidence = 1.0;
        predictions.simultaneous_elements.push_back(elements);
      }
      predictions.reliability = 1.0;
      predictions.information_source = PredictedTrafficLightState::INFORMATION_SOURCE_V2I;
      traffic_light_groups.predictions.push_back(predictions);
    }
    external_msg.traffic_light_groups.push_back(traffic_light_groups);
  }
}

bool isElementEqual(
  const std::vector<TrafficElement> & input_element, const std::vector<TrafficElement> & gt_element)
{
  // check number of elements
  if (input_element.size() != gt_element.size()) {
    return false;
  }

  for (std::size_t element_idx = 0; element_idx < input_element.size(); ++element_idx) {
    const auto & input_traffic_light_element = input_element.at(element_idx);
    const auto & gt_traffic_light_element = gt_element.at(element_idx);

    // check color
    if (input_traffic_light_element.color != gt_traffic_light_element.color) {
      return false;
    }

    // check shape
    if (input_traffic_light_element.shape != gt_traffic_light_element.shape) {
      return false;
    }

    // check status
    if (input_traffic_light_element.status != gt_traffic_light_element.status) {
      return false;
    }

    // check confidence
    constexpr float error = std::numeric_limits<float>::epsilon();
    if (
      std::fabs(input_traffic_light_element.confidence - gt_traffic_light_element.confidence) >
      error) {
      return false;
    }
  }
  return true;
}

bool isPredictedStatusEqual(
  const std::vector<PredictedTrafficLightState> & input_predicted_state,
  const std::vector<PredictedTrafficLightState> & gt_predicted_state)
{
  // check number of groups
  if (input_predicted_state.size() != gt_predicted_state.size()) {
    return false;
  }

  for (std::size_t group_idx = 0; group_idx < input_predicted_state.size(); ++group_idx) {
    const auto & input_traffic_light_group = input_predicted_state.at(group_idx);
    const auto & gt_traffic_light_group = gt_predicted_state.at(group_idx);

    // check predicted_stamp
    if (input_traffic_light_group.predicted_stamp != gt_traffic_light_group.predicted_stamp) {
      return false;
    }

    // check elements
    if (
      isElementEqual(
        input_traffic_light_group.simultaneous_elements,
        gt_traffic_light_group.simultaneous_elements) == false) {
      return false;
    }
  }
  return true;
}

bool isEqual(const TrafficSignalArray & input_msg, const TrafficSignalArray & gt_msg)
{
  // check stamp
  if (input_msg.stamp != gt_msg.stamp) {
    return false;
  }

  // check number of groups
  if (input_msg.traffic_light_groups.size() != gt_msg.traffic_light_groups.size()) {
    return false;
  }

  for (std::size_t group_idx = 0; group_idx < input_msg.traffic_light_groups.size(); ++group_idx) {
    const auto & input_traffic_light_group = input_msg.traffic_light_groups.at(group_idx);
    const auto & gt_traffic_light_group = gt_msg.traffic_light_groups.at(group_idx);

    // check traffic_light_group_id
    if (
      input_traffic_light_group.traffic_light_group_id !=
      gt_traffic_light_group.traffic_light_group_id) {
      return false;
    }

    // check elements
    if (
      isElementEqual(input_traffic_light_group.elements, gt_traffic_light_group.elements) ==
      false) {
      return false;
    }

    // check predictions
    if (
      isPredictedStatusEqual(
        input_traffic_light_group.predictions, gt_traffic_light_group.predictions) == false) {
      return false;
    }
  }
  return true;
}

TEST(TrafficLightArbiterTest, testWithoutPredictions)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_perception_topic =
    "/traffic_light_arbiter/sub/perception_traffic_signals";
  const std::string input_external_topic = "/traffic_light_arbiter/sub/external_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map preparation
  LaneletMapBin vector_map_msg;
  generateMap(vector_map_msg);

  // test callback preparation
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  // perception preparation
  TrafficSignalArray perception_msg;
  perception_msg.stamp = test_target_node->now();
  {
    TrafficSignal traffic_light_groups;
    traffic_light_groups.traffic_light_group_id = 1012;
    {
      TrafficElement elements;
      elements.color = TrafficElement::RED;
      elements.shape = TrafficElement::CIRCLE;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 0.4;
      traffic_light_groups.elements.push_back(elements);
    }
    perception_msg.traffic_light_groups.push_back(traffic_light_groups);
  }

  // external preparation
  TrafficSignalArray external_msg;
  external_msg.stamp = test_target_node->now();
  {
    TrafficSignal traffic_light_groups;
    traffic_light_groups.traffic_light_group_id = 1012;
    {
      TrafficElement elements;
      elements.color = TrafficElement::GREEN;
      elements.shape = TrafficElement::CIRCLE;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 1.0;
      traffic_light_groups.elements.push_back(elements);
    }
    {
      TrafficElement elements;
      elements.color = TrafficElement::GREEN;
      elements.shape = TrafficElement::RIGHT_ARROW;
      elements.status = TrafficElement::SOLID_ON;
      elements.confidence = 1.0;
      traffic_light_groups.elements.push_back(elements);
    }
    external_msg.traffic_light_groups.push_back(traffic_light_groups);
  }

  test_manager->test_pub_msg<LaneletMapBin>(
    test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_perception_topic, perception_msg);
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, external_msg);

  EXPECT_TRUE(isEqual(latest_msg, external_msg));
  rclcpp::shutdown();
}

TEST(TrafficLightArbiterTest, testTrafficSignalOnlyPerceptionMsg)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_perception_topic =
    "/traffic_light_arbiter/sub/perception_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map msg preparation
  LaneletMapBin vector_map_msg;
  generateMap(vector_map_msg);

  // test callback preparation
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  // perception msg preparation
  TrafficSignalArray perception_msg;
  generatePerceptionMsg(perception_msg, test_target_node->now());

  test_manager->test_pub_msg<LaneletMapBin>(
    test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_perception_topic, perception_msg);

  EXPECT_TRUE(isEqual(latest_msg, perception_msg));
  rclcpp::shutdown();
}

TEST(TrafficLightArbiterTest, testTrafficSignalOnlyExternalMsg)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_external_topic = "/traffic_light_arbiter/sub/external_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map msg preparation
  LaneletMapBin vector_map_msg;
  generateMap(vector_map_msg);

  // test callback preparation
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  // external msg preparation
  TrafficSignalArray external_msg;
  generateExternalMsg(external_msg, test_target_node->now());

  test_manager->test_pub_msg<LaneletMapBin>(
    test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, external_msg);

  EXPECT_TRUE(isEqual(latest_msg, external_msg));
  rclcpp::shutdown();
}

TEST(TrafficLightArbiterTest, testTrafficSignalBothMsg)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_perception_topic =
    "/traffic_light_arbiter/sub/perception_traffic_signals";
  const std::string input_external_topic = "/traffic_light_arbiter/sub/external_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map preparation
  LaneletMapBin vector_map_msg;
  generateMap(vector_map_msg);

  // test callback preparation
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  // perception preparation
  TrafficSignalArray perception_msg;
  generatePerceptionMsg(perception_msg, test_target_node->now());

  // external preparation
  TrafficSignalArray external_msg;
  generateExternalMsg(external_msg, test_target_node->now());

  test_manager->test_pub_msg<LaneletMapBin>(
    test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, external_msg);
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_perception_topic, perception_msg);

  // latest_msg should be equal to perception_msg without predictions because it has higher
  // confidence than external_msg
  TrafficSignalArray gt_msg = perception_msg;
  // predictions should be equal to combined predictions of external_msg and perception_msg
  for (auto & traffic_light_group : gt_msg.traffic_light_groups) {
    for (const auto & traffic_light_group_ex : external_msg.traffic_light_groups) {
      if (
        traffic_light_group_ex.traffic_light_group_id ==
        traffic_light_group.traffic_light_group_id) {
        traffic_light_group.predictions.insert(
          traffic_light_group.predictions.end(), traffic_light_group_ex.predictions.begin(),
          traffic_light_group_ex.predictions.end());
        break;
      }
    }
  }

  EXPECT_TRUE(isEqual(latest_msg, gt_msg));
  rclcpp::shutdown();
}

// Helper function to generate external message with specific traffic light ID
void generateExternalMsgWithId(
  TrafficSignalArray & external_msg, const builtin_interfaces::msg::Time & time,
  const uint64_t traffic_light_id, const TrafficElement::_color_type color = TrafficElement::GREEN)
{
  external_msg.stamp = time;
  external_msg.traffic_light_groups.clear();

  TrafficSignal traffic_light_group;
  traffic_light_group.traffic_light_group_id = traffic_light_id;

  // Add a traffic light element
  TrafficElement element;
  element.color = color;
  element.shape = TrafficElement::CIRCLE;
  element.status = TrafficElement::SOLID_ON;
  element.confidence = 0.8;
  traffic_light_group.elements.push_back(element);

  external_msg.traffic_light_groups.push_back(traffic_light_group);
}

TEST(TrafficLightArbiterTest, testMultipleExternalSources)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_external_topic = "/traffic_light_arbiter/sub/external_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map msg preparation
  LaneletMapBin vector_map_msg;
  generateMap(vector_map_msg);

  // test callback preparation
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  // Publish map first
  test_manager->test_pub_msg<LaneletMapBin>(
    test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());

  // Create external messages from different sources with different traffic light IDs
  TrafficSignalArray external_msg_source1, external_msg_source2;
  auto current_time = test_target_node->now();

  // Source 1: Traffic light ID 1012 (GREEN)
  generateExternalMsgWithId(external_msg_source1, current_time, 1012, TrafficElement::GREEN);

  // Source 2: Traffic light ID 1015 (RED) - published slightly later
  auto later_time = current_time + rclcpp::Duration::from_seconds(1.0);
  generateExternalMsgWithId(external_msg_source2, later_time, 1015, TrafficElement::RED);

  // Publish first external source
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, external_msg_source1);

  // Brief wait to ensure processing
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Verify first source data is present and GREEN
  EXPECT_EQ(latest_msg.traffic_light_groups.size(), 1)
    << "Expected 1 traffic light group from first source";
  if (!latest_msg.traffic_light_groups.empty()) {
    EXPECT_EQ(
      static_cast<uint64_t>(latest_msg.traffic_light_groups[0].traffic_light_group_id), 1012u);
    EXPECT_EQ(latest_msg.traffic_light_groups[0].elements[0].color, TrafficElement::GREEN)
      << "Traffic light 1012 should be GREEN from source 1";
  }

  // Publish second external source (different traffic light)
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, external_msg_source2);

  // Wait for processing
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Verify both traffic lights are present in the output
  EXPECT_EQ(latest_msg.traffic_light_groups.size(), 2)
    << "Expected 2 traffic light groups (one from each external source)";

  // Check that both traffic light IDs are present
  std::set<uint64_t> found_ids;
  for (const auto & group : latest_msg.traffic_light_groups) {
    found_ids.insert(static_cast<uint64_t>(group.traffic_light_group_id));
  }

  EXPECT_TRUE(found_ids.count(1012u)) << "Traffic light 1012 from source 1 should be present";
  EXPECT_TRUE(found_ids.count(1015u)) << "Traffic light 1015 from source 2 should be present";

  // Verify the colors are correct for each traffic light
  for (const auto & group : latest_msg.traffic_light_groups) {
    if (static_cast<uint64_t>(group.traffic_light_group_id) == 1012u) {
      EXPECT_EQ(group.elements[0].color, TrafficElement::GREEN)
        << "Traffic light 1012 should be GREEN";
    } else if (static_cast<uint64_t>(group.traffic_light_group_id) == 1015u) {
      EXPECT_EQ(group.elements[0].color, TrafficElement::RED) << "Traffic light 1015 should be RED";
    }
  }

  rclcpp::shutdown();
}

TEST(TrafficLightArbiterTest, testExternalSourceTimeout)
{
  rclcpp::init(0, nullptr);
  const std::string input_map_topic = "/traffic_light_arbiter/sub/vector_map";
  const std::string input_external_topic = "/traffic_light_arbiter/sub/external_traffic_signals";
  const std::string output_topic = "/traffic_light_arbiter/pub/traffic_signals";
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // map msg preparation
  LaneletMapBin vector_map_msg;
  generateMap(vector_map_msg);

  // test callback preparation
  TrafficSignalArray latest_msg;
  auto callback = [&latest_msg](const TrafficSignalArray::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrafficSignalArray>(output_topic, callback);

  // Publish map first
  test_manager->test_pub_msg<LaneletMapBin>(
    test_target_node, input_map_topic, vector_map_msg, rclcpp::QoS(1).transient_local());

  // Create old external message (should be rejected due to timeout)
  TrafficSignalArray old_external_msg;
  auto old_time =
    test_target_node->now() - rclcpp::Duration::from_seconds(
                                20.0);  // 20 seconds ago (should exceed external_delay_tolerance)
  generateExternalMsgWithId(old_external_msg, old_time, 1012, TrafficElement::GREEN);

  // Create recent external message (should be accepted)
  TrafficSignalArray recent_external_msg;
  auto recent_time = test_target_node->now();
  generateExternalMsgWithId(recent_external_msg, recent_time, 1012, TrafficElement::RED);

  // Publish old message first (should be rejected)
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, old_external_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Should have no traffic lights due to timeout rejection
  EXPECT_EQ(latest_msg.traffic_light_groups.size(), 0)
    << "Expected no traffic light groups (old message should be rejected due to timeout)";

  // Publish recent message (should be accepted)
  test_manager->test_pub_msg<TrafficSignalArray>(
    test_target_node, input_external_topic, recent_external_msg);

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // Should now have the recent traffic light
  EXPECT_EQ(latest_msg.traffic_light_groups.size(), 1)
    << "Expected 1 traffic light group (recent message should be accepted)";

  if (!latest_msg.traffic_light_groups.empty()) {
    EXPECT_EQ(latest_msg.traffic_light_groups[0].traffic_light_group_id, 1012)
      << "Should have traffic light ID 1012";
    EXPECT_EQ(latest_msg.traffic_light_groups[0].elements[0].color, TrafficElement::RED)
      << "Recent traffic light should be RED";
  }

  rclcpp::shutdown();
}
