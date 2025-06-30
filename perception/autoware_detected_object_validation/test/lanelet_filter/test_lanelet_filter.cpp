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
#include <autoware_perception_msgs/msg/detected_object.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <memory>
#include <string>
#include <thread>
#include <vector>

// Include the filter node header with include guard

#include "../../src/lanelet_filter/detected_object_lanelet_filter.hpp"
#include "../../src/lanelet_filter/tracked_object_lanelet_filter.hpp"
#include "../test_lanelet_utils.hpp"

using autoware::detected_object_validation::lanelet_filter::DetectedObjectLaneletFilterNode;
using autoware::detected_object_validation::lanelet_filter::TrackedObjectLaneletFilterNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

namespace
{
/// @brief Create a test manager (custom utility for Autoware tests)
std::shared_ptr<autoware::test_utils::AutowareTestManager> generateTestManager()
{
  return std::make_shared<autoware::test_utils::AutowareTestManager>();
}

/// @brief Create an instance of DetectedObjectLaneletFilterNode with a parameter file
std::shared_ptr<DetectedObjectLaneletFilterNode> generateDetectedObjectLaneletFilterNode()
{
  auto node_options = rclcpp::NodeOptions{};
  // Example parameter file path
  const auto detected_object_validation_dir =
    ament_index_cpp::get_package_share_directory("autoware_detected_object_validation");
  node_options.arguments(
    {"--ros-args", "--params-file",
     detected_object_validation_dir + "/config/object_lanelet_filter.param.yaml"});

  return std::make_shared<DetectedObjectLaneletFilterNode>(node_options);
}

/// @brief Create an instance of TrackedObjectLaneletFilterNode with a parameter file
std::shared_ptr<TrackedObjectLaneletFilterNode> generateTrackedObjectLaneletFilterNode()
{
  auto node_options = rclcpp::NodeOptions{};
  // Example parameter file path
  const auto tracked_object_validation_dir =
    ament_index_cpp::get_package_share_directory("autoware_detected_object_validation");
  node_options.arguments(
    {"--ros-args", "--params-file",
     tracked_object_validation_dir + "/config/tracked_object_lanelet_filter.param.yaml"});

  return std::make_shared<TrackedObjectLaneletFilterNode>(node_options);
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
TEST(DetectedObjectValidationTest, testDetectedObjectLaneletFilterWithMap)
{
  rclcpp::init(0, nullptr);

  // Create the test node (lanelet_filter_node)
  auto test_manager = generateTestManager();
  auto test_target_node = generateDetectedObjectLaneletFilterNode();

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
TEST(DetectedObjectValidationTest, testDetectedObjectLaneletFilterEmptyObjects)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateDetectedObjectLaneletFilterNode();

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

/// @brief Test with lanelet object elevation filter
TEST(DetectedObjectValidationTest, testDetectedObjectLaneletObjectElevationFilter)
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
  node_options.append_parameter_override("filter_target_label.UNKNOWN", true);
  node_options.append_parameter_override(
    "filter_settings.lanelet_object_elevation_filter.enabled", true);
  node_options.append_parameter_override(
    "filter_settings.lanelet_object_elevation_filter.max_elevation_threshold", 5.0);
  node_options.append_parameter_override(
    "filter_settings.lanelet_object_elevation_filter.min_elevation_threshold", 0.0);

  auto test_target_node = std::make_shared<DetectedObjectLaneletFilterNode>(node_options);

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

  // check objects in the elevation range
  // (A) Object in-lane, and both query points are within the range
  //     z pos.:
  //       centroid: 1.0
  //       top point: 1.5
  //       bottom point: 0.5
  //  => expected to remain
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = 1.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 1.0;
    input_objects.objects.push_back(obj);
  }

  // (B) Object in-lane, and top query point is within the range
  //     z pos.:
  //       centroid: -1.0
  //       top point: 0.0
  //       bottom point: -2.0
  //  => expected to remain
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = -1.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 2.0;
    input_objects.objects.push_back(obj);
  }

  // (C) Object in-lane, and bottom query point is within the range
  //     z pos.:
  //       centroid: 6.0
  //       top point: 7.0
  //       bottom point: 5.0
  //  => expected to remain
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = 6.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 2.0;
    input_objects.objects.push_back(obj);
  }

  // Publish the objects (test objects in the range)
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_object_topic, input_objects);

  // 5) Check result => all objects should remain
  EXPECT_EQ(latest_msg.objects.size(), 3U)
    << "lanelet_object_elevation_filter is enabled "
    << "=> objects in the elevation range [0.0, 5.0] should remain.";

  input_objects.objects.clear();
  // check objects outside the elevation range
  // (D) Object in-lane, and both query points are within the range
  //     z pos.:
  //       centroid: -4.0
  //       top point: -1.0
  //       bottom point: -7.0
  //  => expected to be removed
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = -4.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 6.0;
    input_objects.objects.push_back(obj);
  }

  // (E) Object in-lane, and top query point is within the range
  //     z pos.:
  //       centroid: 7.0
  //       top point: 8.0
  //       bottom point: 6.0
  //  => expected to be removed
  {
    DetectedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = 7.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 2.0;
    input_objects.objects.push_back(obj);
  }

  // Publish the objects (test objects outside the range)
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, input_object_topic, input_objects);

  // 6) Check result => no objects remain
  EXPECT_EQ(latest_msg.objects.size(), 0U)
    << "lanelet_object_elevation_filter is enabled "
    << "=> objects not within the elevation range [0.0, 5.0] should be removed.";

  rclcpp::shutdown();
}

/*
 * do the same test for TrackedObjectLaneletFilterNode
 */
/// @brief Test LaneletMapBin publishing + object filtering
TEST(DetectedObjectValidationTest, testTrackedObjectLaneletFilterWithMap)
{
  rclcpp::init(0, nullptr);

  // Create the test node (lanelet_filter_node)
  auto test_manager = generateTestManager();
  auto test_target_node = generateTrackedObjectLaneletFilterNode();

  // Create and add a TF broadcaster node
  auto tf_node = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster");

  // Subscribe to the output topic
  const std::string output_topic = "output/object";
  TrackedObjects latest_msg;
  auto output_callback = [&latest_msg](const TrackedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrackedObjects>(output_topic, output_callback);

  // 1) Publish a LaneletMapBin
  //    In a real test, you'd have an actual map message.
  publishLaneletMapBin(test_manager, test_target_node);

  // 2) Publish TrackedObjects
  const std::string input_object_topic = "input/object";
  TrackedObjects input_objects;
  input_objects.header.frame_id = "base_link";

  // Prepare two unknown objects
  {  // Object 1: not in the lanelet
    TrackedObject obj;
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
    TrackedObject obj;
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

  // Publish the TrackedObjects
  test_manager->test_pub_msg<TrackedObjects>(test_target_node, input_object_topic, input_objects);

  EXPECT_EQ(latest_msg.objects.size(), 1)
    << "Expected one object to pass through (or receive something).";

  rclcpp::shutdown();
}

/// @brief Test with an empty object list
TEST(DetectedObjectValidationTest, testTrackedObjectLaneletFilterEmptyObjects)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generateTestManager();
  auto test_target_node = generateTrackedObjectLaneletFilterNode();

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
  auto output_callback = [&callback_counter](const TrackedObjects::ConstSharedPtr msg) {
    (void)msg;
    ++callback_counter;
  };
  test_manager->set_subscriber<TrackedObjects>(output_topic, output_callback);

  // Publish empty objects (not publishing map)
  TrackedObjects input_objects;
  input_objects.header.frame_id = "base_link";

  test_manager->test_pub_msg<TrackedObjects>(test_target_node, "input/object", input_objects);

  // The callback should still be called at least once
  EXPECT_GE(callback_counter, 1);

  rclcpp::shutdown();
}

/// @brief Test with lanelet object elevation filter
TEST(DetectedObjectValidationTest, testTrackedObjectLaneletObjectElevationFilter)
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
  node_options.append_parameter_override("filter_target_label.UNKNOWN", true);
  node_options.append_parameter_override(
    "filter_settings.lanelet_object_elevation_filter.enabled", true);
  node_options.append_parameter_override(
    "filter_settings.lanelet_object_elevation_filter.max_elevation_threshold", 5.0);
  node_options.append_parameter_override(
    "filter_settings.lanelet_object_elevation_filter.min_elevation_threshold", 0.0);

  auto test_target_node = std::make_shared<TrackedObjectLaneletFilterNode>(node_options);

  // 2) Create a TF broadcaster node (map->base_link at z=0.0)
  auto tf_node = createStaticTfBroadcasterNode(
    "map", "base_link", geometry_msgs::build<geometry_msgs::msg::Vector3>().x(0.0).y(0.0).z(0.0),
    geometry_msgs::build<geometry_msgs::msg::Quaternion>().x(0.0).y(0.0).z(0.0).w(1.0),
    "my_test_tf_broadcaster_1");

  // Subscribe to the output topic
  const std::string output_topic = "output/object";
  TrackedObjects latest_msg;
  auto output_callback = [&latest_msg](const TrackedObjects::ConstSharedPtr msg) {
    latest_msg = *msg;
  };
  test_manager->set_subscriber<TrackedObjects>(output_topic, output_callback);

  // 3) Publish a simple LaneletMapBin (50m straight lane)
  publishLaneletMapBin(test_manager, test_target_node);

  // 4) Publish TrackedObjects
  const std::string input_object_topic = "input/object";
  TrackedObjects input_objects;
  input_objects.header.frame_id = "base_link";

  // check objects in the elevation range
  // (A) Object in-lane, and both query points are within the range
  //     z pos.:
  //       centroid: 1.0
  //       top point: 1.5
  //       bottom point: 0.5
  //  => expected to remain
  {
    TrackedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = 1.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 1.0;
    input_objects.objects.push_back(obj);
  }

  // (B) Object in-lane, and top query point is within the range
  //     z pos.:
  //       centroid: -1.0
  //       top point: 0.0
  //       bottom point: -2.0
  //  => expected to remain
  {
    TrackedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = -1.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 2.0;
    input_objects.objects.push_back(obj);
  }

  // (C) Object in-lane, and bottom query point is within the range
  //     z pos.:
  //       centroid: 6.0
  //       top point: 7.0
  //       bottom point: 5.0
  //  => expected to remain
  {
    TrackedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = 6.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 2.0;
    input_objects.objects.push_back(obj);
  }

  // Publish the objects (test objects in the range)
  test_manager->test_pub_msg<TrackedObjects>(test_target_node, input_object_topic, input_objects);

  // 5) Check result => all objects should remain
  EXPECT_EQ(latest_msg.objects.size(), 3U)
    << "lanelet_object_elevation_filter is enabled "
    << "=> objects in the elevation range [0.0, 5.0] should remain.";

  input_objects.objects.clear();
  // check objects outside the elevation range
  // (D) Object in-lane, and both query points are within the range
  //     z pos.:
  //       centroid: -4.0
  //       top point: -1.0
  //       bottom point: -7.0
  //  => expected to be removed
  {
    TrackedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = -4.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 6.0;
    input_objects.objects.push_back(obj);
  }

  // (E) Object in-lane, and top query point is within the range
  //     z pos.:
  //       centroid: 7.0
  //       top point: 8.0
  //       bottom point: 6.0
  //  => expected to be removed
  {
    TrackedObject obj;
    obj.classification.resize(1);
    obj.classification[0].label = ObjectClassification::UNKNOWN;
    obj.kinematics.pose_with_covariance.pose.position.x = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.y = 0.0;
    obj.kinematics.pose_with_covariance.pose.position.z = 7.0;
    obj.shape.type = autoware_perception_msgs::msg::Shape::BOUNDING_BOX;
    obj.shape.dimensions.x = 2.0;
    obj.shape.dimensions.y = 2.0;
    obj.shape.dimensions.z = 2.0;
    input_objects.objects.push_back(obj);
  }

  // Publish the objects (test objects outside the range)
  test_manager->test_pub_msg<TrackedObjects>(test_target_node, input_object_topic, input_objects);

  // 6) Check result => no objects remain
  EXPECT_EQ(latest_msg.objects.size(), 0U)
    << "lanelet_object_elevation_filter is enabled "
    << "=> objects not within the elevation range [0.0, 5.0] should be removed.";

  rclcpp::shutdown();
}
