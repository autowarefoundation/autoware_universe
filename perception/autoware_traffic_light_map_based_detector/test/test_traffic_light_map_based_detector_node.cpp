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

#include "../src/traffic_light_map_based_detector_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <thread>

namespace
{

using autoware::traffic_light::MapBasedDetector;
using autoware_map_msgs::msg::LaneletMapBin;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

rclcpp::NodeOptions make_node_options()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("max_vibration_pitch", 0.01745);
  options.append_parameter_override("max_vibration_yaw", 0.01745);
  options.append_parameter_override("max_vibration_height", 0.5);
  options.append_parameter_override("max_vibration_width", 0.5);
  options.append_parameter_override("max_vibration_depth", 0.5);
  options.append_parameter_override("max_detection_range", 200.0);
  options.append_parameter_override("min_timestamp_offset", -0.01);
  options.append_parameter_override("max_timestamp_offset", 0.0);
  options.append_parameter_override("car_traffic_light_max_angle_range", 40.0);
  options.append_parameter_override("pedestrian_traffic_light_max_angle_range", 80.0);
  return options;
}

CameraInfo make_camera_info(const rclcpp::Time & stamp)
{
  CameraInfo info;
  info.header.frame_id = "camera_optical_link";
  info.header.stamp = stamp;
  info.width = 640;
  info.height = 480;
  info.p = {400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  info.distortion_model = "plumb_bob";
  info.k = {400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return info;
}

/// Camera at map origin, looking along +x in map frame.
/// Optical convention: z=forward(map +x), x=right(map -y), y=down(map -z).
geometry_msgs::msg::TransformStamped make_map_to_camera_transform(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_optical_link";
  transform.transform.rotation.w = 0.5;
  transform.transform.rotation.x = -0.5;
  transform.transform.rotation.y = 0.5;
  transform.transform.rotation.z = -0.5;
  return transform;
}

/// Builds a minimal lanelet map with one road lanelet and one traffic light.
///
/// Coordinates are in the map frame (REP-103): +x = forward, +y = left, +z = up.
/// The camera (see make_map_to_camera_transform) sits at the origin and looks
/// along +x.
///
/// Top-down view (X-Y plane, looking from +z toward -z):
///
///                      +y (left)
///                       ^
///                       |
///          y=+2    vl1 *-----------+------------* vl2      <- left lane edge
///                       |          |            |
///                       |          | TL bar     |          <- traffic-light line at x=20
///                       |          | (y=+/-0.5) |             z = 3.5 (bottom)
///                       |          |            |             z = 4.5 (top, via height=1)
///                       |          |            |
///          y=-2    vr1 *-----------+------------* vr2      <- right lane edge
///                       |          |            |
///                       C          |            |            ----> +x (forward)
///                  camera(0,0,0)  x=20         x=30
///
LaneletMapBin make_test_map()
{
  using lanelet::AttributeName;
  using lanelet::AttributeValueString;
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::Point3d;
  using lanelet::utils::getId;

  Point3d vl1(getId(), 0.0, 2.0, 0.0);
  Point3d vl2(getId(), 30.0, 2.0, 0.0);
  Point3d vr1(getId(), 0.0, -2.0, 0.0);
  Point3d vr2(getId(), 30.0, -2.0, 0.0);
  LineString3d vehicle_left(getId(), {vl1, vl2});
  LineString3d vehicle_right(getId(), {vr1, vr2});

  auto road_lanelet = Lanelet(getId(), vehicle_left, vehicle_right);
  road_lanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;

  Point3d tl_front(getId(), 20.0, 0.5, 3.5);
  Point3d tl_back(getId(), 20.0, -0.5, 3.5);
  LineString3d traffic_light_ls(getId(), {tl_front, tl_back});
  traffic_light_ls.attributes()["subtype"] = "red_yellow_green";
  traffic_light_ls.attributes()["height"] = "1.0";

  auto traffic_light_reg_elem = lanelet::autoware::AutowareTrafficLight::make(
    getId(), lanelet::AttributeMap(), {traffic_light_ls});
  road_lanelet.addRegulatoryElement(traffic_light_reg_elem);

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(road_lanelet);

  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

void spin_for(rclcpp::Executor & executor, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

template <typename Predicate>
bool spin_until(rclcpp::Executor & executor, std::chrono::milliseconds timeout, Predicate condition)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!condition() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return condition();
}

}  // namespace

TEST(MapBasedDetectorNodeTest, PublishesRoiWhenAllInputsAreReceived)
{
  // Arrange
  rclcpp::init(0, nullptr);

  auto node = std::make_shared<MapBasedDetector>(make_node_options());
  auto test_node = std::make_shared<rclcpp::Node>("test_node");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(test_node);

  const auto map_pub = test_node->create_publisher<LaneletMapBin>(
    "/traffic_light_map_based_detector/input/vector_map", rclcpp::QoS(1).transient_local());
  const auto camera_info_pub = test_node->create_publisher<CameraInfo>(
    "/traffic_light_map_based_detector/input/camera_info", rclcpp::SensorDataQoS());

  TrafficLightRoiArray::SharedPtr received_rois;
  TrafficLightRoiArray::SharedPtr received_expect_rois;
  const auto rois_sub = test_node->create_subscription<TrafficLightRoiArray>(
    "/traffic_light_map_based_detector/output/rois", rclcpp::QoS(1),
    [&](const TrafficLightRoiArray::SharedPtr msg) { received_rois = msg; });
  const auto expect_rois_sub = test_node->create_subscription<TrafficLightRoiArray>(
    "/traffic_light_map_based_detector/expect/rois", rclcpp::QoS(1),
    [&](const TrafficLightRoiArray::SharedPtr msg) { received_expect_rois = msg; });

  tf2_ros::StaticTransformBroadcaster tf_broadcaster(test_node);
  const auto tf_map_to_camera = make_map_to_camera_transform(node->now());
  const auto camera_info = make_camera_info(node->now());

  // Act
  tf_broadcaster.sendTransform(tf_map_to_camera);
  map_pub->publish(make_test_map());
  spin_for(executor, std::chrono::milliseconds(100));

  camera_info_pub->publish(camera_info);

  const bool received_all = spin_until(
    executor, std::chrono::seconds(3), [&] { return received_rois && received_expect_rois; });

  // Assert
  ASSERT_TRUE(received_all);
  EXPECT_EQ(received_rois->rois.size(), 1u);
  EXPECT_EQ(received_expect_rois->rois.size(), 1u);

  rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
