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

#include "autoware/object_merger/object_fusion_merger_node.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <geometry_msgs/msg/point32.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using autoware::object_merger::ObjectFusionMergerNode;
using autoware_perception_msgs::msg::DetectedObject;
using autoware_perception_msgs::msg::DetectedObjectKinematics;
using autoware_perception_msgs::msg::DetectedObjects;
using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;

namespace
{
std::shared_ptr<autoware::test_utils::AutowareTestManager> generate_test_manager()
{
  return std::make_shared<autoware::test_utils::AutowareTestManager>();
}

std::shared_ptr<ObjectFusionMergerNode> generate_node(const bool keep_input_dimensions = false)
{
  auto node_options = rclcpp::NodeOptions{};
  const auto package_dir = std::filesystem::path(__FILE__).parent_path().parent_path();
  node_options.arguments(
    {"--ros-args", "--params-file",
     (package_dir / "config" / "object_fusion_merger.param.yaml").string(), "-p",
     std::string("keep_input_dimensions:=") + (keep_input_dimensions ? "true" : "false")});
  return std::make_shared<ObjectFusionMergerNode>(node_options);
}

std::shared_ptr<rclcpp::Node> create_static_tf_broadcaster_node(
  const std::string & parent_frame_id, const std::string & child_frame_id)
{
  auto broadcaster_node = std::make_shared<rclcpp::Node>("test_tf_broadcaster");
  auto static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(broadcaster_node);
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = broadcaster_node->get_clock()->now();
  transform_stamped.header.frame_id = parent_frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.rotation.w = 1.0;
  static_broadcaster->sendTransform(transform_stamped);
  return broadcaster_node;
}

DetectedObject make_object(
  const double x, const double covariance_x, const double existence_probability,
  const double shape_x, const uint8_t label)
{
  DetectedObject object;
  object.existence_probability = static_cast<float>(existence_probability);
  object.kinematics.pose_with_covariance.pose.position.x = x;
  object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;
  object.kinematics.has_position_covariance = true;
  object.kinematics.pose_with_covariance.covariance[0] = covariance_x;
  object.kinematics.pose_with_covariance.covariance[7] = covariance_x;
  object.kinematics.pose_with_covariance.covariance[14] = covariance_x;
  object.kinematics.orientation_availability = DetectedObjectKinematics::AVAILABLE;
  object.kinematics.has_twist = true;
  object.kinematics.has_twist_covariance = true;
  object.kinematics.twist_with_covariance.twist.linear.x = x;
  object.kinematics.twist_with_covariance.covariance[0] = covariance_x;
  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = shape_x;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.0;
  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0f;
  object.classification.push_back(classification);
  return object;
}

DetectedObject make_polygon_object(
  const double x, const double covariance_x, const double existence_probability,
  const std::vector<geometry_msgs::msg::Point32> & footprint_points, const uint8_t label)
{
  auto object = make_object(x, covariance_x, existence_probability, 1.0, label);
  object.shape.type = Shape::POLYGON;
  object.shape.footprint.points = footprint_points;
  object.shape.dimensions.x = 0.0;
  object.shape.dimensions.y = 0.0;
  object.shape.dimensions.z = 1.0;
  return object;
}

DetectedObject make_cylinder_object(
  const double x, const double covariance_x, const double existence_probability,
  const double diameter, const uint8_t label)
{
  auto object = make_object(x, covariance_x, existence_probability, diameter, label);
  object.shape.type = Shape::CYLINDER;
  object.shape.dimensions.x = diameter;
  object.shape.dimensions.y = diameter;
  object.shape.dimensions.z = 1.0;
  return object;
}

double max_abs_footprint_x(const DetectedObject & object)
{
  double max_abs_x = 0.0;
  for (const auto & point : object.shape.footprint.points) {
    max_abs_x = std::max(max_abs_x, std::abs(static_cast<double>(point.x)));
  }
  return max_abs_x;
}

double max_abs_footprint_y(const DetectedObject & object)
{
  double max_abs_y = 0.0;
  for (const auto & point : object.shape.footprint.points) {
    max_abs_y = std::max(max_abs_y, std::abs(static_cast<double>(point.y)));
  }
  return max_abs_y;
}

bool footprint_has_vertex(
  const DetectedObject & object, const double expected_x, const double expected_y,
  const double epsilon = 1e-3)
{
  return std::any_of(
    object.shape.footprint.points.begin(), object.shape.footprint.points.end(),
    [expected_x, expected_y, epsilon](const auto & point) {
      return std::abs(static_cast<double>(point.x) - expected_x) < epsilon &&
             std::abs(static_cast<double>(point.y) - expected_y) < epsilon;
    });
}
}  // namespace

TEST(ObjectFusionMergerNodeTest, testMatchedObjectsAreFused)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  DetectedObjects unmatched_sub_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });
  test_manager->set_subscriber<DetectedObjects>(
    "/output/other_objects",
    [&unmatched_sub_msg](const DetectedObjects::ConstSharedPtr msg) { unmatched_sub_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(make_object(0.0, 4.0, 0.6, 4.0, ObjectClassification::CAR));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_object(1.5, 1.0, 0.5, 2.0, ObjectClassification::CAR));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_TRUE(unmatched_sub_msg.objects.empty());
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.x, 0.25, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.x, 4.5, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().existence_probability, 0.6, 1e-3);
  ASSERT_EQ(latest_msg.objects.front().classification.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().classification.front().label, ObjectClassification::CAR);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testFusedObjectHeightCoversMainAndSubZRange)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  auto main_object = make_object(0.0, 4.0, 0.6, 4.0, ObjectClassification::CAR);
  main_object.kinematics.pose_with_covariance.pose.position.z = 1.0;
  main_object.shape.dimensions.z = 2.0;
  main_objects.objects.push_back(main_object);

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  auto sub_object = make_object(1.5, 1.0, 0.5, 2.0, ObjectClassification::CAR);
  sub_object.kinematics.pose_with_covariance.pose.position.z = 2.5;
  sub_object.shape.dimensions.z = 3.0;
  sub_objects.objects.push_back(sub_object);

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.z, 2.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.z, 4.0, 1e-3);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testUnmatchedMainIsKeptAndUnmatchedSubIsPublishedSeparately)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  DetectedObjects unmatched_sub_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });
  test_manager->set_subscriber<DetectedObjects>(
    "/output/other_objects",
    [&unmatched_sub_msg](const DetectedObjects::ConstSharedPtr msg) { unmatched_sub_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(make_object(0.0, 4.0, 0.6, 4.0, ObjectClassification::CAR));
  main_objects.objects.push_back(make_object(10.0, 4.0, 0.9, 3.0, ObjectClassification::TRUCK));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_object(1.5, 1.0, 0.5, 2.0, ObjectClassification::CAR));
  sub_objects.objects.push_back(make_object(20.0, 1.0, 0.5, 6.0, ObjectClassification::BUS));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 2U);
  EXPECT_NEAR(latest_msg.objects.at(0).kinematics.pose_with_covariance.pose.position.x, 0.25, 1e-3);
  EXPECT_NEAR(latest_msg.objects.at(0).shape.dimensions.x, 4.5, 1e-3);
  EXPECT_NEAR(latest_msg.objects.at(1).kinematics.pose_with_covariance.pose.position.x, 10.0, 1e-3);
  ASSERT_EQ(latest_msg.objects.at(1).classification.size(), 1U);
  EXPECT_EQ(latest_msg.objects.at(1).classification.front().label, ObjectClassification::TRUCK);
  ASSERT_EQ(unmatched_sub_msg.objects.size(), 1U);
  EXPECT_NEAR(
    unmatched_sub_msg.objects.front().kinematics.pose_with_covariance.pose.position.x, 20.0, 1e-3);
  ASSERT_EQ(unmatched_sub_msg.objects.front().classification.size(), 1U);
  EXPECT_EQ(
    unmatched_sub_msg.objects.front().classification.front().label, ObjectClassification::BUS);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testSubObjectOverlappingMultipleMainObjectsIsIgnored)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  DetectedObjects other_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });
  test_manager->set_subscriber<DetectedObjects>(
    "/output/other_objects",
    [&other_msg](const DetectedObjects::ConstSharedPtr msg) { other_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(make_object(-1.0, 1.0, 0.6, 3.0, ObjectClassification::CAR));
  main_objects.objects.push_back(make_object(1.0, 1.0, 0.6, 3.0, ObjectClassification::CAR));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_object(0.0, 1.0, 0.5, 2.0, ObjectClassification::CAR));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 2U);
  EXPECT_NEAR(latest_msg.objects.at(0).shape.dimensions.x, 3.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.at(1).shape.dimensions.x, 3.0, 1e-3);
  EXPECT_TRUE(other_msg.objects.empty());

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testMultipleSubObjectsCanExpandOneMainObject)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(make_object(0.0, 1.0, 0.6, 2.0, ObjectClassification::CAR));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_object(-0.9, 1.0, 0.5, 1.4, ObjectClassification::CAR));
  sub_objects.objects.push_back(make_object(0.9, 1.0, 0.5, 1.4, ObjectClassification::CAR));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.x, 3.2, 1e-3);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testPolygonSubCanExpandMainBoundingBox)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  auto main_object = make_object(0.0, 4.0, 0.6, 4.0, ObjectClassification::CAR);
  main_object.kinematics.pose_with_covariance.pose.position.y = 2.0;
  main_objects.objects.push_back(main_object);

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  auto sub_object = make_polygon_object(
    1.0, 1.0, 0.5,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(3.5).y(1.5).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(3.5).y(-1.5).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-3.5).y(-1.5).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-3.5).y(1.5).z(0.0),
    },
    ObjectClassification::CAR);
  sub_object.kinematics.pose_with_covariance.pose.position.y = 2.3;
  sub_objects.objects.push_back(sub_object);

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().shape.type, Shape::BOUNDING_BOX);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.x, 1.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.x, 7.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.y, 3.0, 1e-3);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testUnionCanBeEnclosedWithMainCylinder)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(
    make_cylinder_object(0.0, 4.0, 0.6, 2.0, ObjectClassification::PEDESTRIAN));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_polygon_object(
    0.5, 1.0, 0.5,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.5).y(1.2).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.5).y(-1.2).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-0.5).y(-1.2).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-0.5).y(1.2).z(0.0),
    },
    ObjectClassification::PEDESTRIAN));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().shape.type, Shape::CYLINDER);
  EXPECT_GT(latest_msg.objects.front().shape.dimensions.x, 4.6);
  EXPECT_NEAR(
    latest_msg.objects.front().shape.dimensions.x, latest_msg.objects.front().shape.dimensions.y,
    1e-3);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testBoundingBoxCanRetainExpandedFootprintWithoutDimensionGrowth)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node(true);
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  auto main_object = make_object(0.0, 4.0, 0.6, 4.0, ObjectClassification::CAR);
  main_object.kinematics.pose_with_covariance.pose.position.y = 2.0;
  main_objects.objects.push_back(main_object);

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_polygon_object(
    1.0, 1.0, 0.5,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(3.5).y(1.5).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(3.5).y(-1.5).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-3.5).y(-1.5).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-3.5).y(1.5).z(0.0),
    },
    ObjectClassification::CAR));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  // The position and dimensions should be the same as the main object since the sub object is fully
  // enclosed and keep_input_dimensions is true, but the footprint should be expanded to cover the
  // union of main and sub objects.
  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().shape.type, Shape::BOUNDING_BOX);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.y, 2.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.x, 4.0, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.y, 2.0, 1e-3);
  EXPECT_GT(max_abs_footprint_x(latest_msg.objects.front()), 4.4);
  EXPECT_GT(max_abs_footprint_y(latest_msg.objects.front()), 1.4);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testCylinderCanRetainExpandedFootprintWithoutDiameterGrowth)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node(true);
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  auto main_object = make_cylinder_object(0.0, 4.0, 0.6, 2.0, ObjectClassification::PEDESTRIAN);
  main_object.kinematics.pose_with_covariance.pose.position.y = -1.5;
  main_objects.objects.push_back(main_object);

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  auto sub_object = make_polygon_object(
    0.5, 1.0, 0.5,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.5).y(1.2).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.5).y(-1.2).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-0.5).y(-1.2).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-0.5).y(1.2).z(0.0),
    },
    ObjectClassification::PEDESTRIAN);
  sub_object.kinematics.pose_with_covariance.pose.position.y = -1.1;
  sub_objects.objects.push_back(sub_object);

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  // The position and dimensions should be the same as the main object since the sub object is fully
  // enclosed and keep_input_dimensions is true, but the footprint should be expanded to cover the
  // union of main and sub objects.
  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().shape.type, Shape::CYLINDER);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.x, 0.0, 1e-3);
  EXPECT_NEAR(
    latest_msg.objects.front().kinematics.pose_with_covariance.pose.position.y, -1.5, 1e-3);
  EXPECT_NEAR(latest_msg.objects.front().shape.dimensions.x, 2.0, 1e-3);
  EXPECT_NEAR(
    latest_msg.objects.front().shape.dimensions.x, latest_msg.objects.front().shape.dimensions.y,
    1e-3);
  EXPECT_GT(max_abs_footprint_x(latest_msg.objects.front()), 1.9);
  EXPECT_GT(max_abs_footprint_y(latest_msg.objects.front()), 1.1);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testUnionCanBeEnclosedWithMainPolygon)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(make_polygon_object(
    0.0, 4.0, 0.6,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(-1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(-1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(1.0).z(0.0),
    },
    ObjectClassification::CAR));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_polygon_object(
    0.6, 1.0, 0.5,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.6).y(1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.6).y(-1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-0.6).y(-1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-0.6).y(1.0).z(0.0),
    },
    ObjectClassification::CAR));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().shape.type, Shape::POLYGON);
  EXPECT_GT(latest_msg.objects.front().shape.footprint.points.size(), 3U);
  EXPECT_GT(max_abs_footprint_x(latest_msg.objects.front()), 2.1);
  EXPECT_GT(max_abs_footprint_y(latest_msg.objects.front()), 0.9);

  rclcpp::shutdown();
}

TEST(ObjectFusionMergerNodeTest, testPolygonUnionKeepsConcaveBoundary)
{
  rclcpp::init(0, nullptr);

  auto test_manager = generate_test_manager();
  auto test_target_node = generate_node();
  auto tf_node = create_static_tf_broadcaster_node("map", "base_link");

  DetectedObjects latest_msg;
  test_manager->set_subscriber<DetectedObjects>(
    "/output/objects",
    [&latest_msg](const DetectedObjects::ConstSharedPtr msg) { latest_msg = *msg; });

  DetectedObjects main_objects;
  main_objects.header.frame_id = "base_link";
  main_objects.objects.push_back(make_polygon_object(
    0.0, 4.0, 0.6,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(1.0).y(-1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(-1.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(-1.0).y(1.0).z(0.0),
    },
    ObjectClassification::CAR));

  DetectedObjects sub_objects;
  sub_objects.header.frame_id = "base_link";
  sub_objects.objects.push_back(make_polygon_object(
    0.0, 1.0, 0.5,
    {
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(2.0).y(2.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(2.0).y(0.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(0.0).y(0.0).z(0.0),
      geometry_msgs::build<geometry_msgs::msg::Point32>().x(0.0).y(2.0).z(0.0),
    },
    ObjectClassification::CAR));

  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/main_objects", main_objects);
  test_manager->test_pub_msg<DetectedObjects>(test_target_node, "input/sub_objects", sub_objects);

  ASSERT_EQ(latest_msg.objects.size(), 1U);
  EXPECT_EQ(latest_msg.objects.front().shape.type, Shape::POLYGON);
  EXPECT_TRUE(footprint_has_vertex(latest_msg.objects.front(), 1.0, 0.0));
  EXPECT_TRUE(footprint_has_vertex(latest_msg.objects.front(), 0.0, 1.0));

  rclcpp::shutdown();
}
