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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware_test_utils/autoware_test_utils.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

// Include the filter node header with include guard

#include "../../src/lanelet_filter/lanelet_filter.hpp"
#include "../test_lanelet_utils.hpp"

using autoware::detected_object_validation::lanelet_filter::ObjectLaneletFilterNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;

namespace
{
/// @brief Create a test manager (custom utility for Autoware tests)
std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  return std::make_shared<autoware::test_utils::AutowareTestManager>();
}

/// @brief Create an instance of ObjectLaneletFilterNode with a parameter file
std::shared_ptr<ObjectLaneletFilterNode> generateNode()
{
  auto node_options = rclcpp::NodeOptions{};
  // Example parameter file path
  const auto detected_object_validation_dir =
    ament_index_cpp::get_package_share_directory("autoware_detected_object_validation");
  node_options.arguments(
    {"--ros-args", "--params-file",
     detected_object_validation_dir + "/config/object_lanelet_filter.param.yaml"});

  return std::make_shared<ObjectLaneletFilterNode>(node_options);
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

// publish sample LaneletMapBin message
void publishLaneletMapBin(
  const std::shared_ptr<autoware::test_utils::AutowareTestManager> & test_manager,
  const std::shared_ptr<rclcpp::Node> & test_target_node,
  const std::string & input_map_topic = "input/vector_map")
{
  auto qos = rclcpp::QoS(1).transient_local();
  LaneletMapBin map_msg = createSimpleLaneletMapMsg();
  test_manager->test_pub_msg<LaneletMapBin>(test_target_node, input_map_topic, map_msg, qos);
}
}  // namespace

/// @brief Test LaneletMapBin publishing + object filtering
TEST(DetectedObjectValidationTest, testObjectLaneletFilterWithMap)
{
  rclcpp::init(0, nullptr);

  // Create the test node (lanelet_filter_node)
  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Create and add a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster");

  // Subscribe to the output topic
  const std::string output_topic = "output/object";
  DetectedObjects latest_msg;
  auto output_callback = [&latest_msg](const DetectedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, output_callback);

  // 1) Publish a LaneletMapBin
  //    In a real test, you'd have an actual map message.
  publishLaneletMapBin(test_manager, test_target_node);

  // 2) Publish DetectedObjects
  const std::string input_object_topic = "input/object";
  DetectedObjects input_objects;
  input_objects.header.frame_id = "base_link";

  // Prepare two unknown objects
  {  // Object 1: not in the lanelet
    DetectedObject obj;
    obj.kinematics.pose_with_covariance.pose.position.x = 100.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 5.0;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(1.0).z(0.0));
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(-1.0).z(0.0));
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(-1.0).z(0.0));
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(1.0).z(0.0));
    input_objects.objects.push_back(obj);
  }
  {  // Object 2: in the lanelet. To be filtered out.
    DetectedObject obj;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(1.0).z(0.0));
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(-1.0).z(0.0));
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(-1.0).z(0.0));
    obj.shape.footprint.points.push_back(
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(1.0).z(0.0));
    input_objects.objects.push_back(obj);
  }

  // Publish the DetectedObjects
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_object_topic, input_objects);

  EXPECT_EQ(latest_msg.objects.size(), 1)
    << "Expected one object to pass through (or receive something).";

  rclcpp::shutdown();
}

/// @brief Test with an empty object list
TEST(DetectedObjectValidationTest, testObjectLaneletFilterEmptyObjects)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateNode();

  // Create and add a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster");

  // Publish simple LaneletMapBin
  publishLaneletMapBin(test_manager, test_target_node);

  // Subscribe to the output topic
  const std::string output_topic = "output/object";
  int callback_counter = 0;
  auto output_callback = [&callback_counter](const DetectedObjects::ConstSharedPtr msg) {
    (void)msg;
    ++callback_counter;
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, output_callback);

  // Publish empty objects (not publishing map)
  DetectedObjects input_objects;
  input_objects.header.frame_id = "base_link";

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/object", input_objects);

  // The callback should still be called at least once
  EXPECT_GE(callback_counter, 1);

  rclcpp::shutdown();
}

TEST(DetectedObjectValidationTest, testObjectLaneletFilterHeightThreshold)
{
  rclcpp::init(0, nullptr);

  // 1) Setup test manager and node with a custom param override (height filter enabled)
  auto test_manager = generateTestManager();

  auto node_options = rclcpp::NodeOptions{};
  const auto detected_object_validation_dir =
    ament_index_cpp::get_package_share_directory("autoware_detected_object_validation");
  node_options.arguments(
    {"--ros-args", "--params-file",
     detected_object_validation_dir + "/config/object_lanelet_filter.param.yaml"});
  node_options.append_parameter_override("filter_settings.use_height_threshold", true);
  node_options.append_parameter_override("filter_settings.max_height_threshold", 2.0);
  node_options.append_parameter_override("filter_settings.min_height_threshold", 0.0);

  auto test_target_node = std::make_shared<ObjectLaneletFilterNode>(node_options);

  // 2) Create a TF broadcaster node (map->base_link at z=0.0)
  auto tf_node = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_1");

  // Subscribe to the output topic
  const std::string output_topic = "output/object";
  DetectedObjects latest_msg;
  auto output_callback = [&latest_msg](const DetectedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<DetectedObjects>(output_topic, output_callback);

  // 3) Publish a simple LaneletMapBin (50m straight lane)
  publishLaneletMapBin(test_manager, test_target_node);

  // 4) Publish DetectedObjects
  const std::string input_object_topic = "input/object";
  DetectedObjects input_objects;
  input_objects.header.frame_id = "base_link";

  // (A) Object in-lane, height=1.5 => expected to remain
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 1.5;
    input_objects.objects.push_back(obj);
  }

  // (B) Object in-lane, height=3.0 => expected to be filtered out
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 5.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 3.0;
    input_objects.objects.push_back(obj);
  }

  // Publish the objects (Round 1)
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_object_topic, input_objects);

  // 5) Check result => only the first object remains
  EXPECT_EQ(latest_msg.objects.size(), 1U)
    << "Height filter is enabled => only the shorter object (1.5m) should remain.";

  // 6) Second scenario: place ego at z=1.3, effectively lowering object's relative height
  auto tf_node_after = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(1.3),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_2");

  // Publish the same objects (Round 2)
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_object_topic, input_objects);

  // 7) Check result => now both objects remain because each one's height above base_link is < 2.0
  EXPECT_EQ(latest_msg.objects.size(), 2U)
    << "With ego at z=1.3, the 3.0m object is effectively only ~1.7m above ego => remain.";

  rclcpp::shutdown();
}
