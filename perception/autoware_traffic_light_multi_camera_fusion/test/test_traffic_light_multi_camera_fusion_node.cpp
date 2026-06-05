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

#include "../src/traffic_light_multi_camera_fusion_node.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/traffic_light_group_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using autoware::traffic_light::MultiCameraFusionNode;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::TrafficLightGroupArray;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::TrafficLight;
using tier4_perception_msgs::msg::TrafficLightArray;
using tier4_perception_msgs::msg::TrafficLightElement;
using tier4_perception_msgs::msg::TrafficLightRoi;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

// IDs assigned to map elements. The two traffic-light line strings (*_TRAFFIC_LIGHT_ID) are
// bound to the same regulatory element (REGULATORY_ELEMENT_ID); the node groups them into a
// single output.
constexpr lanelet::Id LEFT_TRAFFIC_LIGHT_ID = 11;
constexpr lanelet::Id RIGHT_TRAFFIC_LIGHT_ID = 12;
constexpr lanelet::Id REGULATORY_ELEMENT_ID = 100;

struct FusionNodeOptions
{
  std::vector<std::string> camera_namespaces{"camera0", "camera1"};
  bool approximate_sync = true;
  double message_lifespan = 1.0;
  double prior_log_odds = 0.0;
  bool consistency_check_enable = false;
  bool publish_partial_matched_signal = false;
};

class MultiCameraFusionIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    map_publisher_ = test_node_->create_publisher<LaneletMapBin>(
      "/traffic_light_multi_camera_fusion/input/vector_map", rclcpp::QoS(1).transient_local());

    const std::vector<std::string> camera_namespaces = {"camera0", "camera1"};
    for (const auto & camera_namespace : camera_namespaces) {
      camera_info_publishers_.push_back(test_node_->create_publisher<CameraInfo>(
        "/" + camera_namespace + "/camera_info", rclcpp::SensorDataQoS()));
      roi_publishers_.push_back(test_node_->create_publisher<TrafficLightRoiArray>(
        "/" + camera_namespace + "/detection/rois", rclcpp::QoS(1)));
      signal_publishers_.push_back(test_node_->create_publisher<TrafficLightArray>(
        "/" + camera_namespace + "/classification/traffic_signals", rclcpp::QoS(1)));
    }

    output_subscription_ = test_node_->create_subscription<TrafficLightGroupArray>(
      "/traffic_light_multi_camera_fusion/output/traffic_signals", rclcpp::QoS(1),
      [this](const TrafficLightGroupArray::SharedPtr message) {
        received_message_ = message;
        message_received_ = true;
      });
  }

  void initialize_fusion_node(const FusionNodeOptions & options)
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("camera_namespaces", options.camera_namespaces);
    node_options.append_parameter_override("approximate_sync", options.approximate_sync);
    node_options.append_parameter_override("message_lifespan", options.message_lifespan);
    node_options.append_parameter_override("prior_log_odds", options.prior_log_odds);
    node_options.append_parameter_override(
      "signal_consistency_check.enable", options.consistency_check_enable);
    node_options.append_parameter_override(
      "signal_consistency_check.publish_partial_matched_signal",
      options.publish_partial_matched_signal);

    node_ = std::make_shared<MultiCameraFusionNode>(node_options);
    executor_->add_node(node_);
  }

  void TearDown() override
  {
    executor_.reset();
    output_subscription_.reset();
    camera_info_publishers_.clear();
    roi_publishers_.clear();
    signal_publishers_.clear();
    map_publisher_.reset();
    test_node_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  bool wait_for_message(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    message_received_ = false;
    while (!message_received_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  void publish_map(const LaneletMapBin & map_bin)
  {
    map_publisher_->publish(map_bin);
    spin_for(std::chrono::milliseconds(100));
  }

  void publish_camera_info(
    size_t camera_index, const rclcpp::Time & stamp, const std::string & frame_id)
  {
    CameraInfo camera_info;
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id;
    camera_info.width = 1920;
    camera_info.height = 1080;

    camera_info_publishers_[camera_index]->publish(camera_info);
  }

  void publish_roi(
    size_t camera_index, const rclcpp::Time & stamp, const std::string & frame_id,
    lanelet::Id traffic_light_id)
  {
    TrafficLightRoi roi;
    roi.traffic_light_id = static_cast<TrafficLightRoi::_traffic_light_id_type>(traffic_light_id);
    roi.traffic_light_type = 0;
    // ROI placed well inside the image so the visibility score is maximal.
    roi.roi.x_offset = 100;
    roi.roi.y_offset = 100;
    roi.roi.width = 50;
    roi.roi.height = 50;

    TrafficLightRoiArray roi_array;
    roi_array.header.stamp = stamp;
    roi_array.header.frame_id = frame_id;
    roi_array.rois.push_back(roi);

    roi_publishers_[camera_index]->publish(roi_array);
  }

  void publish_signal(
    size_t camera_index, const rclcpp::Time & stamp, const std::string & frame_id,
    lanelet::Id traffic_light_id, uint8_t color, float confidence)
  {
    TrafficLightElement element;
    element.color = color;
    element.shape = TrafficLightElement::CIRCLE;
    element.status = TrafficLightElement::SOLID_ON;
    element.confidence = confidence;

    TrafficLight signal;
    signal.traffic_light_id = static_cast<TrafficLight::_traffic_light_id_type>(traffic_light_id);
    signal.elements.push_back(element);

    TrafficLightArray signal_array;
    signal_array.header.stamp = stamp;
    signal_array.header.frame_id = frame_id;
    signal_array.signals.push_back(signal);

    signal_publishers_[camera_index]->publish(signal_array);
  }

  void publish_camera_detection(
    size_t camera_index, lanelet::Id traffic_light_id, uint8_t color, float confidence)
  {
    const rclcpp::Time stamp(1, 0, RCL_ROS_TIME);
    const std::string frame_id = "camera" + std::to_string(camera_index);

    publish_camera_info(camera_index, stamp, frame_id);
    publish_roi(camera_index, stamp, frame_id, traffic_light_id);
    publish_signal(camera_index, stamp, frame_id, traffic_light_id, color, confidence);
  }

  std::shared_ptr<MultiCameraFusionNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_publisher_;
  std::vector<rclcpp::Publisher<CameraInfo>::SharedPtr> camera_info_publishers_;
  std::vector<rclcpp::Publisher<TrafficLightRoiArray>::SharedPtr> roi_publishers_;
  std::vector<rclcpp::Publisher<TrafficLightArray>::SharedPtr> signal_publishers_;
  rclcpp::Subscription<TrafficLightGroupArray>::SharedPtr output_subscription_;

  TrafficLightGroupArray::SharedPtr received_message_;
  bool message_received_ = false;
};

/// @brief Create a lanelet map with two traffic-light line strings sharing a single
///        regulatory element.
///
///   y
///   2 +-------------------+    <- road left bound
///     |                   |
///   0 +    [left_traffic_light] [right_traffic_light] (height=1, z=3.5..4.5, at x=20)
///     |        AutowareTrafficLight reg-elem (id=100)
///  -2 +-------------------+    <- road right bound
///     0                  30   x
///
///   Both `left_traffic_light` (id=11) and `right_traffic_light` (id=12) are registered
///   as `traffic_lights` on a single regulatory element (id=100), so the node maps both
///   traffic_light_id → regulatory_element_id.
LaneletMapBin create_map()
{
  using lanelet::AttributeName;
  using lanelet::AttributeValueString;
  using lanelet::Lanelet;
  using lanelet::LineString3d;
  using lanelet::Point3d;

  Point3d road_left_start(1, 0.0, 2.0, 0.0);
  Point3d road_left_end(2, 30.0, 2.0, 0.0);
  Point3d road_right_start(3, 0.0, -2.0, 0.0);
  Point3d road_right_end(4, 30.0, -2.0, 0.0);
  LineString3d road_left(5, {road_left_start, road_left_end});
  LineString3d road_right(6, {road_right_start, road_right_end});

  auto road_lanelet = Lanelet(1000, road_left, road_right);
  road_lanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;

  Point3d left_traffic_light_left_end(7, 19.5, 0.5, 3.5);
  Point3d left_traffic_light_right_end(8, 19.5, -0.5, 3.5);
  LineString3d left_traffic_light(
    LEFT_TRAFFIC_LIGHT_ID, {left_traffic_light_left_end, left_traffic_light_right_end});
  left_traffic_light.attributes()["subtype"] = "red_yellow_green";
  left_traffic_light.attributes()["height"] = "1.0";

  Point3d right_traffic_light_left_end(9, 20.5, 0.5, 3.5);
  Point3d right_traffic_light_right_end(10, 20.5, -0.5, 3.5);
  LineString3d right_traffic_light(
    RIGHT_TRAFFIC_LIGHT_ID, {right_traffic_light_left_end, right_traffic_light_right_end});
  right_traffic_light.attributes()["subtype"] = "red_yellow_green";
  right_traffic_light.attributes()["height"] = "1.0";

  auto traffic_light_regulatory_element = lanelet::autoware::AutowareTrafficLight::make(
    REGULATORY_ELEMENT_ID, lanelet::AttributeMap(), {left_traffic_light, right_traffic_light});
  road_lanelet.addRegulatoryElement(traffic_light_regulatory_element);

  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(road_lanelet);

  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

// Two cameras observe two distinct traffic lights that belong to the same regulatory element.
// Per-camera fusion picks the only record per traffic-light id, and group fusion accumulates
// evidence across the two ids and outputs a single GREEN group.
TEST_F(MultiCameraFusionIntegrationTest, TwoCamerasOneRegulatoryElementOutputsGreenGroup)
{
  // Arrange
  initialize_fusion_node(FusionNodeOptions{});
  publish_map(create_map());

  // Act
  publish_camera_detection(0, LEFT_TRAFFIC_LIGHT_ID, TrafficLightElement::GREEN, 0.9f);
  publish_camera_detection(1, RIGHT_TRAFFIC_LIGHT_ID, TrafficLightElement::GREEN, 0.9f);

  // Assert
  ASSERT_TRUE(wait_for_message());
  ASSERT_EQ(received_message_->traffic_light_groups.size(), 1u);
  const auto & group = received_message_->traffic_light_groups.front();
  EXPECT_EQ(group.traffic_light_group_id, REGULATORY_ELEMENT_ID);
  ASSERT_EQ(group.elements.size(), 1u);
  EXPECT_EQ(
    group.elements.front().color, autoware_perception_msgs::msg::TrafficLightElement::GREEN);
  EXPECT_EQ(
    group.elements.front().shape, autoware_perception_msgs::msg::TrafficLightElement::CIRCLE);
}

// Two cameras observe different traffic lights in the same regulatory element with
// conflicting colors and different confidences. With consistency check disabled,
// the node selects the color from the higher-confidence observation.
TEST_F(MultiCameraFusionIntegrationTest, HigherConfidenceColorWinsAmongConflictingObservations)
{
  // Arrange
  initialize_fusion_node(FusionNodeOptions{});
  publish_map(create_map());

  // Act
  publish_camera_detection(0, LEFT_TRAFFIC_LIGHT_ID, TrafficLightElement::RED, 0.6f);
  publish_camera_detection(1, RIGHT_TRAFFIC_LIGHT_ID, TrafficLightElement::GREEN, 0.9f);

  // Assert
  ASSERT_TRUE(wait_for_message());
  ASSERT_EQ(received_message_->traffic_light_groups.size(), 1u);
  const auto & group = received_message_->traffic_light_groups.front();
  EXPECT_EQ(group.traffic_light_group_id, REGULATORY_ELEMENT_ID);
  ASSERT_EQ(group.elements.size(), 1u);
  EXPECT_EQ(
    group.elements.front().color, autoware_perception_msgs::msg::TrafficLightElement::GREEN);
}

// Two cameras observe the same regulatory element with conflicting colors (RED vs GREEN).
// With consistency check enabled and partial-match publishing disabled, the node detects
// a CONFLICT between the two state keys and emits a fail-safe UNKNOWN group.
TEST_F(MultiCameraFusionIntegrationTest, TwoCamerasConflictingColorsOutputsFailsafeUnknown)
{
  // Arrange
  FusionNodeOptions options;
  options.consistency_check_enable = true;
  initialize_fusion_node(options);
  publish_map(create_map());

  // Act
  publish_camera_detection(0, LEFT_TRAFFIC_LIGHT_ID, TrafficLightElement::RED, 0.9f);
  publish_camera_detection(1, RIGHT_TRAFFIC_LIGHT_ID, TrafficLightElement::GREEN, 0.9f);

  // Assert
  ASSERT_TRUE(wait_for_message());
  ASSERT_EQ(received_message_->traffic_light_groups.size(), 1u);
  const auto & group = received_message_->traffic_light_groups.front();
  EXPECT_EQ(group.traffic_light_group_id, REGULATORY_ELEMENT_ID);
  ASSERT_EQ(group.elements.size(), 1u);
  EXPECT_EQ(
    group.elements.front().color, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);
  EXPECT_EQ(
    group.elements.front().shape, autoware_perception_msgs::msg::TrafficLightElement::UNKNOWN);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
