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

#include "../src/detected_object_sorter_node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware::object_sorter::DetectedObjectSorterNode;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

namespace
{

struct Label
{
  std::string name;
  uint8_t id;
};

const std::vector<Label> kConfiguredLabels = {
  {"UNKNOWN", ObjectClassification::UNKNOWN}, {"CAR", ObjectClassification::CAR},
  {"TRUCK", ObjectClassification::TRUCK},     {"BUS", ObjectClassification::BUS},
  {"TRAILER", ObjectClassification::TRAILER}, {"MOTORCYCLE", ObjectClassification::MOTORCYCLE},
  {"BICYCLE", ObjectClassification::BICYCLE}, {"PEDESTRIAN", ObjectClassification::PEDESTRIAN},
  {"ANIMAL", ObjectClassification::ANIMAL},   {"HAZARD", ObjectClassification::HAZARD},
};

// Build NodeOptions with every parameter the sorter declares set explicitly. Each
// configured label gets `publish=true`, no velocity gate, and the same distance band.
rclcpp::NodeOptions makeNodeOptions(bool publish = true)
{
  constexpr double kMinDistance = 30.0;
  constexpr double kMaxDistance = 100.0;
  constexpr double kMinVelocityThreshold = 0.0;

  rclcpp::NodeOptions options;
  options.append_parameter_override("range_calc_frame_id", std::string("base_link"));
  options.append_parameter_override("range_calc_offset.x", 0.0);
  options.append_parameter_override("range_calc_offset.y", 0.0);
  options.append_parameter_override("range_thresholding_mode", std::string("distance"));

  for (const auto & label : kConfiguredLabels) {
    const std::string prefix = "sort_target." + label.name;
    options.append_parameter_override(prefix + ".publish", publish);
    options.append_parameter_override(prefix + ".min_velocity_threshold", kMinVelocityThreshold);
    options.append_parameter_override(prefix + ".range_threshold.max_distance", kMaxDistance);
    options.append_parameter_override(prefix + ".range_threshold.min_distance", kMinDistance);
  }
  return options;
}

std::shared_ptr<DetectedObjectSorterNode> generateNode(bool publish = true)
{
  return std::make_shared<DetectedObjectSorterNode>(makeNodeOptions(publish));
}

DetectedObject makeObject(uint8_t label, double x, double y)
{
  DetectedObject obj;
  ObjectClassification c;
  c.label = label;
  c.probability = 1.0f;
  obj.classification.push_back(c);
  obj.kinematics.pose_with_covariance.pose.position.x = x;
  obj.kinematics.pose_with_covariance.pose.position.y = y;
  obj.kinematics.pose_with_covariance.pose.position.z = 0.0;
  obj.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  return obj;
}

bool containsLabel(const DetectedObjects & msg, uint8_t label)
{
  for (const auto & obj : msg.objects) {
    for (const auto & c : obj.classification) {
      if (c.label == label) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace

// Check every class detection can pass through when the publish option is true
TEST(DetectedObjectSorterTest, AllConfiguredClassesPassThrough)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  auto sorter = generateNode();

  const std::string input_topic = "/detected_object_sorter_node/input/objects";
  const std::string output_topic = "/detected_object_sorter_node/output/objects";

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    output_topic, [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects input;
  input.header.frame_id = "base_link";
  // x = 50 m is inside the configured [kMinDistance, kMaxDistance] band for every label.
  for (const auto & label : kConfiguredLabels) {
    input.objects.push_back(makeObject(label.id, 50.0, 0.0));
  }

  test_manager->test_pub_msg<DetectedObjects>(sorter, input_topic, input);

  EXPECT_EQ(latest_msg.objects.size(), kConfiguredLabels.size());
  for (const auto & label : kConfiguredLabels) {
    EXPECT_TRUE(containsLabel(latest_msg, label.id))
      << "label '" << label.name << "' (id=" << static_cast<int>(label.id)
      << ") was filtered out unexpectedly";
  }

  rclcpp::shutdown();
}

// Check every class detection get filtered when the publish option is false
TEST(DetectedObjectSorterTest, AllConfiguredClassesFilterOut)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  auto sorter = generateNode(false);

  const std::string input_topic = "/detected_object_sorter_node/input/objects";
  const std::string output_topic = "/detected_object_sorter_node/output/objects";

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    output_topic, [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects input;
  input.header.frame_id = "base_link";
  // x = 50 m is inside the configured [kMinDistance, kMaxDistance] band for every label.
  for (const auto & label : kConfiguredLabels) {
    input.objects.push_back(makeObject(label.id, 50.0, 0.0));
  }

  test_manager->test_pub_msg<DetectedObjects>(sorter, input_topic, input);

  EXPECT_EQ(latest_msg.objects.size(), 0u);

  rclcpp::shutdown();
}

// An object outside the configured range must be filtered out
TEST(DetectedObjectSorterTest, OutOfRangeFiltered)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  auto sorter = generateNode();

  const std::string input_topic = "/detected_object_sorter_node/input/objects";
  const std::string output_topic = "/detected_object_sorter_node/output/objects";

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    output_topic, [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects input;
  input.header.frame_id = "base_link";
  // x = 5 m is below kMinDistance, so the object must be dropped.
  input.objects.push_back(makeObject(ObjectClassification::CAR, 5.0, 0.0));

  test_manager->test_pub_msg<DetectedObjects>(sorter, input_topic, input);

  EXPECT_EQ(latest_msg.objects.size(), 0u);

  rclcpp::shutdown();
}

// Labels that have no `sort_target.<NAME>` block in the YAML must fall back to UNKNOWN's
// settings rather than being silently dropped or crashing
TEST(DetectedObjectSorterTest, UnconfiguredLabelFallsBackToUnknown)
{
  rclcpp::init(0, nullptr);

  auto test_manager = std::make_shared<autoware::test_utils::AutowareTestManager>();
  auto sorter = generateNode();

  const std::string input_topic = "/detected_object_sorter_node/input/objects";
  const std::string output_topic = "/detected_object_sorter_node/output/objects";

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    output_topic, [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects input;
  input.header.frame_id = "base_link";
  // OVER_DRIVABLE is not in kConfiguredLabels, so it should fall back to UNKNOWN's
  // settings: x=50 m passes the [kMinDistance, kMaxDistance] band, x=5 m does not.
  input.objects.push_back(makeObject(ObjectClassification::OVER_DRIVABLE, 50.0, 0.0));
  input.objects.push_back(makeObject(ObjectClassification::OVER_DRIVABLE, 5.0, 0.0));

  test_manager->test_pub_msg<DetectedObjects>(sorter, input_topic, input);

  ASSERT_EQ(latest_msg.objects.size(), 1u);
  EXPECT_EQ(latest_msg.objects[0].classification[0].label, ObjectClassification::OVER_DRIVABLE);

  rclcpp::shutdown();
}
