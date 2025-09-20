// Copyright 2025 TIER IV, Inc.
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

#include "autoware/simple_object_merger/simple_detected_object_merger_node.hpp"
#include "autoware/simple_object_merger/simple_tracked_object_merger_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <vector>

using autoware::simple_object_merger::SimpleDetectedObjectMergerNode;
using autoware::simple_object_merger::SimpleTrackedObjectMergerNode;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

namespace
{

/// @brief Create a test manager (custom utility for Autoware tests)
std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  return std::make_shared<autoware::test_utils::AutowareTestManager>();
}

/// @brief Create an instance of SimpleDetectedObjectMergerNode with a parameter file
std::shared_ptr<SimpleDetectedObjectMergerNode> generateSimpleDetectedObjectMergerNode()
{
  auto node_options = rclcpp::NodeOptions{};
  const auto simple_object_merger_dir =
    ament_index_cpp::get_package_share_directory("autoware_simple_object_merger");
  node_options.arguments(
    {"--ros-args", "--params-file",
     simple_object_merger_dir + "/config/simple_object_merger.param.yaml"});
  node_options.append_parameter_override(
    "input_topics", std::vector<std::string>{"input/node_A", "input/node_B"});

  return std::make_shared<SimpleDetectedObjectMergerNode>(node_options);
}

/// @brief Create an instance of SimpleTrackedObjectMergerNode with a parameter file
std::shared_ptr<SimpleTrackedObjectMergerNode> generateSimpleTrackedObjectMergerNode(
  std::vector<std::string> input_topics = {"input/node_A", "input/node_B"},
  float uuid_mapping_cleanup_threshold = 30.0)
{
  auto node_options = rclcpp::NodeOptions{};
  const auto simple_object_merger_dir =
    ament_index_cpp::get_package_share_directory("autoware_simple_object_merger");
  node_options.arguments(
    {"--ros-args", "--params-file",
     simple_object_merger_dir + "/config/simple_tracked_object_merger.param.yaml"});
  node_options.append_parameter_override("input_topics", input_topics);
  node_options.append_parameter_override(
    "uuid_mapping_cleanup_threshold", uuid_mapping_cleanup_threshold);

  return std::make_shared<SimpleTrackedObjectMergerNode>(node_options);
}

std::shared_ptr<rclcpp::Node> createStaticTfBroadcasterNode(
  const std::string & parent_frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::Vector3 & translation, const geometry_msgs::msg::Quaternion & rotation,
  const std::string & node_name = "test_tf_broadcaster")
{
  // Create a dedicated node for broadcasting the static transform
  auto broadcaster_node = std::make_shared<rclcpp::Node>(node_name);

  // Create the static transform broadcaster
  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(broadcaster_node);

  // Prepare the transform message
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = broadcaster_node->now();
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation = translation;
  transform_stamped.transform.rotation = rotation;

  // Broadcast the transform
  static_broadcaster->sendTransform(transform_stamped);

  return broadcaster_node;
}

}  // namespace

/// @brief Test topic merging with SimpleDetectedObjectMergerNode
TEST(SimpleDetectedObjectMergerTest, testObjectMerging)
{
  rclcpp::init(0, nullptr);

  // Setup test manager and node
  auto test_manager = generateTestManager();
  auto test_target_node = generateSimpleDetectedObjectMergerNode();

  // Create a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_1");

  // Subscribe to the output topic
  const std::string output_topic = "/simple_object_merger/output/objects";
  DetectedObjects latest_msg;
  auto output_callback = [&latest_msg](const DetectedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, output_callback);

  const std::string input_object_topic_node_a = "input/node_A";
  DetectedObjects input_objects_from_node_a;
  input_objects_from_node_a.header.frame_id = "base_link";
  {
    DetectedObject obj;
    input_objects_from_node_a.objects.push_back(obj);
  }

  const std::string input_object_topic_node_b = "input/node_B";
  DetectedObjects input_objects_from_node_b;
  input_objects_from_node_b.header.frame_id = "base_link";
  {
    DetectedObject obj;
    input_objects_from_node_b.objects.push_back(obj);
    input_objects_from_node_b.objects.push_back(obj);
  }

  // Publish the objects
  test_manager->test_pub_msg<DetectedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<DetectedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);

  // Check result => all objects should remain
  EXPECT_EQ(latest_msg.objects.size(), 3U)
    << "3 objects should be contained in the published message.";

  rclcpp::shutdown();
}

/// @brief Test topic merging with SimpleTrackedObjectMergerNode
TEST(SimpleTrackedObjectMergerNodeTest, testObjectMerging)
{
  rclcpp::init(0, nullptr);

  // Setup test manager and node
  auto test_manager = generateTestManager();
  auto test_target_node = generateSimpleTrackedObjectMergerNode();

  // Create a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "base_link", "map", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_1");

  // Subscribe to the output topic
  const std::string output_topic = "/simple_tracked_object_merger/output/objects";
  TrackedObjects latest_msg;
  auto output_callback = [&latest_msg](const TrackedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrackedObjects>(output_topic, output_callback);

  const std::string input_object_topic_node_a = "input/node_A";
  TrackedObjects input_objects_from_node_a;
  input_objects_from_node_a.header.frame_id = "base_link";

  unique_identifier_msgs::msg::UUID uuid_1;
  uuid_1.uuid = {
    0x84, 0x84, 0x15, 0x60, 0xa7, 0xf3, 0xe1, 0x84, 0x84, 0xde, 0xfb, 0xab, 0xaa, 0x56, 0x0d, 0x00,
  };
  {
    TrackedObject obj;
    obj.object_id = uuid_1;
    input_objects_from_node_a.objects.push_back(obj);
  }

  const std::string input_object_topic_node_b = "input/node_B";
  TrackedObjects input_objects_from_node_b;
  input_objects_from_node_b.header.frame_id = "base_link";

  unique_identifier_msgs::msg::UUID uuid_2;
  uuid_2.uuid = {
    0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d,
  };
  {
    TrackedObject obj;
    obj.object_id = uuid_2;
    input_objects_from_node_b.objects.push_back(obj);
  }

  // Publish the objects
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);

  // Check result
  EXPECT_EQ(latest_msg.objects.size(), 2U)
    << "2 objects should be contained in the published message.";

  // Check the case when UUIDs collide
  input_objects_from_node_a.objects.clear();
  {
    TrackedObject obj;
    obj.object_id = uuid_1;  // pass through
    input_objects_from_node_a.objects.push_back(obj);
  }

  input_objects_from_node_b.objects.clear();
  {
    TrackedObject obj;
    obj.object_id = uuid_1;  // UUID collides
    input_objects_from_node_b.objects.push_back(obj);

    TrackedObject obj_2;
    obj_2.object_id = uuid_2;
    input_objects_from_node_b.objects.push_back(obj_2);
  }

  // Publish the objects
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);

  // Check result
  EXPECT_EQ(latest_msg.objects.size(), 3U)
    << "3 objects should be contained in the published message.";

  // Verify that a new UUID was issued
  bool uuid_1_exist = false;
  bool uuid_2_exist = false;
  bool new_uuid_exist = false;
  for (const auto & obj : latest_msg.objects) {
    if (obj.object_id.uuid == uuid_1.uuid)
      uuid_1_exist = true;
    else if (obj.object_id.uuid == uuid_2.uuid)
      uuid_2_exist = true;
    else
      new_uuid_exist = true;
  }

  EXPECT_EQ(uuid_1_exist, true) << "topic that match with uuid_1 must exist.";
  EXPECT_EQ(uuid_2_exist, true) << "topic that match with uuid_2 must exist.";
  EXPECT_EQ(new_uuid_exist, true) << "topic with new uuid must exist.";

  rclcpp::shutdown();
}

// Test the case that input nodes exist more than 2
/// @brief Test topic merging with SimpleTrackedObjectMergerNode
TEST(SimpleTrackedObjectMergerNodeTest, testThreeNodeObjectMerging)
{
  rclcpp::init(0, nullptr);

  // Setup test manager and node
  auto test_manager = generateTestManager();
  auto test_target_node =
    generateSimpleTrackedObjectMergerNode({"input/node_A", "input/node_B", "input/node_C"}, 30.0);

  // Create a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "base_link", "map", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_1");

  // Subscribe to the output topic
  const std::string output_topic = "/simple_tracked_object_merger/output/objects";
  TrackedObjects latest_msg;
  auto output_callback = [&latest_msg](const TrackedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrackedObjects>(output_topic, output_callback);

  unique_identifier_msgs::msg::UUID uuid;
  uuid.uuid = {
    0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d, 0x61, 0x75, 0x6d,
  };

  const std::string input_object_topic_node_a = "input/node_A";
  TrackedObjects input_objects_from_node_a;
  input_objects_from_node_a.header.frame_id = "base_link";
  {
    TrackedObject obj;
    obj.object_id = uuid;
    input_objects_from_node_a.objects.push_back(obj);
  }

  const std::string input_object_topic_node_b = "input/node_B";
  TrackedObjects input_objects_from_node_b;
  input_objects_from_node_b.header.frame_id = "base_link";
  {
    TrackedObject obj;
    obj.object_id = uuid;
    input_objects_from_node_b.objects.push_back(obj);
  }

  const std::string input_object_topic_node_c = "input/node_C";
  TrackedObjects input_objects_from_node_c;
  input_objects_from_node_c.header.frame_id = "base_link";
  {
    TrackedObject obj;
    obj.object_id = uuid;
    input_objects_from_node_c.objects.push_back(obj);
  }

  // Publish the objects
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_c, input_objects_from_node_c);

  // Check result
  EXPECT_EQ(latest_msg.objects.size(), 3U)
    << "3 objects should be contained in the published message.";

  // Verify that a new UUID was issued
  bool original_uuid_exist = false;
  int new_uuid_count = 0;
  for (const auto & obj : latest_msg.objects) {
    if (obj.object_id.uuid == uuid.uuid)
      original_uuid_exist = true;
    else
      new_uuid_count++;
  }

  EXPECT_EQ(original_uuid_exist, true) << "topic that match with original uuid must exist.";
  EXPECT_EQ(new_uuid_count, 2) << "two topics with new uuid must exist.";

  rclcpp::shutdown();
}

/// @brief Test SimpleTrackedObjectMergerNode's mapping gets cleaned up properly
TEST(SimpleTrackedObjectMergerNodeTest, testUUIDMappingCleanUp)
{
  rclcpp::init(0, nullptr);

  // Setup test manager and node
  auto test_manager = generateTestManager();
  // Create a node that forget the mapping immediately
  auto test_target_node =
    generateSimpleTrackedObjectMergerNode({"input/node_A", "input/node_B"}, 0.0);

  // Create a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "base_link", "map", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_1");

  // Subscribe to the output topic
  const std::string output_topic = "/simple_tracked_object_merger/output/objects";
  TrackedObjects latest_msg;
  auto output_callback = [&latest_msg](const TrackedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrackedObjects>(output_topic, output_callback);

  const std::string input_object_topic_node_a = "input/node_A";
  TrackedObjects input_objects_from_node_a;
  input_objects_from_node_a.header.frame_id = "base_link";

  unique_identifier_msgs::msg::UUID uuid_collide;
  uuid_collide.uuid = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
  };
  unique_identifier_msgs::msg::UUID uuid_1;
  uuid_1.uuid = {
    0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
  };
  unique_identifier_msgs::msg::UUID uuid_2;
  uuid_2.uuid = {
    0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
  };

  {
    TrackedObject obj;
    obj.object_id = uuid_collide;
    input_objects_from_node_a.objects.push_back(obj);
  }

  const std::string input_object_topic_node_b = "input/node_B";
  TrackedObjects input_objects_from_node_b;
  input_objects_from_node_b.header.frame_id = "base_link";
  {
    TrackedObject obj;
    obj.object_id = uuid_collide;
    input_objects_from_node_b.objects.push_back(obj);
  }

  // Publish the objects, UUID will be collided
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);

  // Create two topics to test a case where the input/node_B topic’s UUID collides within mapping
  input_objects_from_node_a.objects.clear();
  {
    TrackedObject obj;
    obj.object_id = uuid_1;
    input_objects_from_node_a.objects.push_back(obj);
  }

  input_objects_from_node_b.objects.clear();
  {
    TrackedObject obj;
    obj.object_id = uuid_collide;
    input_objects_from_node_b.objects.push_back(obj);
  }

  // Publish the objects
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);

  // Check result
  EXPECT_EQ(latest_msg.objects.size(), 2U)
    << "2 objects should be contained in the published message.";

  // Verify that UUID has not changed
  bool uuid_1_exist = false;
  bool uuid_collide_exist = false;
  for (const auto & obj : latest_msg.objects) {
    if (obj.object_id.uuid == uuid_1.uuid) uuid_1_exist = true;
    if (obj.object_id.uuid == uuid_collide.uuid) uuid_collide_exist = true;
  }

  EXPECT_EQ(uuid_1_exist, true) << "UUID should not get changed.";
  EXPECT_EQ(uuid_collide_exist, true) << "UUID should not get changed.";

  // Above test case should be enough, but just in case, we will test this pattern too
  // to ensure the test
  input_objects_from_node_a.objects.clear();
  {
    TrackedObject obj;
    obj.object_id = uuid_collide;
    input_objects_from_node_a.objects.push_back(obj);
  }

  input_objects_from_node_b.objects.clear();
  {
    TrackedObject obj;
    obj.object_id = uuid_2;
    input_objects_from_node_b.objects.push_back(obj);
  }

  // Publish the objects
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_a, input_objects_from_node_a);
  test_manager->test_pub_msg<TrackedObjects>(
    test_target_node, input_object_topic_node_b, input_objects_from_node_b);

  // Check result
  EXPECT_EQ(latest_msg.objects.size(), 2U)
    << "2 objects should be contained in the published message.";

  // Verify that the UUID remains unchanged.
  bool uuid_2_exist = false;
  uuid_collide_exist = false;
  for (const auto & obj : latest_msg.objects) {
    if (obj.object_id.uuid == uuid_2.uuid) uuid_2_exist = true;
    if (obj.object_id.uuid == uuid_collide.uuid) uuid_collide_exist = true;
  }

  EXPECT_EQ(uuid_2_exist, true) << "UUID should not get changed.";
  EXPECT_EQ(uuid_collide_exist, true) << "UUID should not get changed.";

  rclcpp::shutdown();
}
