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
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <gtest/gtest.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using autoware::traffic_light::MapBasedDetector;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletRoute;
using sensor_msgs::msg::CameraInfo;
using tier4_perception_msgs::msg::TrafficLightRoiArray;

class MapBasedDetectorIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

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

    node_ = std::make_shared<MapBasedDetector>(options);
    test_node_ = std::make_shared<rclcpp::Node>("test_node");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(test_node_);

    map_pub_ = test_node_->create_publisher<LaneletMapBin>(
      "/traffic_light_map_based_detector/input/vector_map", rclcpp::QoS(1).transient_local());
    camera_info_pub_ = test_node_->create_publisher<CameraInfo>(
      "/traffic_light_map_based_detector/input/camera_info", rclcpp::SensorDataQoS());
    route_pub_ = test_node_->create_publisher<LaneletRoute>(
      "/traffic_light_map_based_detector/input/route", rclcpp::QoS(1).transient_local());

    roi_received_ = false;
    roi_sub_ = test_node_->create_subscription<TrafficLightRoiArray>(
      "/traffic_light_map_based_detector/output/rois", rclcpp::QoS(1),
      [this](const TrafficLightRoiArray::SharedPtr msg) {
        received_roi_msg_ = msg;
        roi_received_ = true;
      });

    expect_roi_received_ = false;
    expect_roi_sub_ = test_node_->create_subscription<TrafficLightRoiArray>(
      "/traffic_light_map_based_detector/expect/rois", rclcpp::QoS(1),
      [this](const TrafficLightRoiArray::SharedPtr msg) {
        received_expect_roi_msg_ = msg;
        expect_roi_received_ = true;
      });

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(test_node_);
  }

  void TearDown() override
  {
    executor_.reset();
    node_.reset();
    test_node_.reset();
    rclcpp::shutdown();
  }

  // --- Low-level helpers ---

  bool waitForRoiMessage(std::chrono::milliseconds timeout = std::chrono::milliseconds(3000))
  {
    auto start = std::chrono::steady_clock::now();
    roi_received_ = false;
    expect_roi_received_ = false;
    while (!roi_received_ || !expect_roi_received_) {
      if (std::chrono::steady_clock::now() - start > timeout) {
        return false;
      }
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return true;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start < duration) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // --- High-level helpers ---

  /// @brief Create a lanelet map with one road lanelet and one car traffic light
  ///
  ///   y
  ///   2 + vl1 ------------- vl2       (vehicle left bound)
  ///     |         (TL)
  ///  -2 + vr1 ------------- vr2       (vehicle right bound)
  ///     +---+-------+-------+--> x
  ///     0          20      30
  ///
  ///   Traffic light at x=20: linestring (20, 0.5, 3.5) → (20, -0.5, 3.5)
  ///     subtype="red_yellow_green", height=1.0
  ///   Vehicle lanelet: subtype="road"
  LaneletMapBin createMap()
  {
    using lanelet::AttributeName;
    using lanelet::AttributeValueString;
    using lanelet::Lanelet;
    using lanelet::LineString3d;
    using lanelet::Point3d;
    using lanelet::utils::getId;

    // Vehicle lanelet bounds
    Point3d vl1(getId(), 0.0, 2.0, 0.0);
    Point3d vl2(getId(), 30.0, 2.0, 0.0);
    Point3d vr1(getId(), 0.0, -2.0, 0.0);
    Point3d vr2(getId(), 30.0, -2.0, 0.0);
    LineString3d vehicle_left(getId(), {vl1, vl2});
    LineString3d vehicle_right(getId(), {vr1, vr2});

    auto road_lanelet = Lanelet(getId(), vehicle_left, vehicle_right);
    road_lanelet.attributes()[AttributeName::Subtype] = AttributeValueString::Road;
    road_lanelet_id_ = road_lanelet.id();

    // Traffic light linestring: front → back ordered so that
    // tl_yaw + π/2 ≈ 0 (camera_yaw), passing the angle range check
    Point3d tl_front(getId(), 20.0, 0.5, 3.5);
    Point3d tl_back(getId(), 20.0, -0.5, 3.5);
    LineString3d traffic_light_ls(getId(), {tl_front, tl_back});
    traffic_light_ls.attributes()["subtype"] = "red_yellow_green";
    traffic_light_ls.attributes()["height"] = "1.0";
    traffic_light_linestring_id_ = traffic_light_ls.id();

    auto traffic_light_reg_elem = lanelet::autoware::AutowareTrafficLight::make(
      getId(), lanelet::AttributeMap(), {traffic_light_ls});

    road_lanelet.addRegulatoryElement(traffic_light_reg_elem);

    auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
    lanelet_map->add(road_lanelet);

    auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
    map_bin.header.frame_id = "map";
    return map_bin;
  }

  void publishMap()
  {
    map_pub_->publish(createMap());
    spinFor(std::chrono::milliseconds(500));
  }

  void publishTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "camera_optical_link";

    // Camera at origin, looking along +x in map frame
    // Camera optical frame: z=forward(map +x), x=right(map -y), y=down(map -z)
    // Rotation matrix (camera_optical → map):
    //   | 0   0   1 |
    //   | -1  0   0 |
    //   | 0  -1   0 |
    // Quaternion: w=0.5, x=-0.5, y=0.5, z=-0.5
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.w = 0.5;
    transform.transform.rotation.x = -0.5;
    transform.transform.rotation.y = 0.5;
    transform.transform.rotation.z = -0.5;

    tf_broadcaster_->sendTransform(transform);
    spinFor(std::chrono::milliseconds(500));
  }

  void publishCameraInfo()
  {
    CameraInfo camera_info;
    camera_info.header.frame_id = "camera_optical_link";
    camera_info.header.stamp = node_->now();
    // 640x480 ideal pinhole camera: fx=fy=400, cx=320, cy=240, no distortion.
    // The node uses P for 3D-to-pixel projection and width/height for bounds checking.
    // k, r, d are required by image_geometry but have no effect with zero distortion.
    camera_info.width = 640;
    camera_info.height = 480;
    camera_info.p = {400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
    camera_info.distortion_model = "plumb_bob";
    camera_info.k = {400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0};
    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info_pub_->publish(camera_info);
  }

  void publishRoute()
  {
    LaneletRoute route;
    autoware_planning_msgs::msg::LaneletSegment segment;
    autoware_planning_msgs::msg::LaneletPrimitive primitive;
    primitive.id = road_lanelet_id_;
    segment.primitives.push_back(primitive);
    route.segments.push_back(segment);
    route_pub_->publish(route);
    spinFor(std::chrono::milliseconds(500));
  }

  std::vector<tier4_perception_msgs::msg::TrafficLightRoi> getOutputRois() const
  {
    return received_roi_msg_->rois;
  }

  std::vector<tier4_perception_msgs::msg::TrafficLightRoi> getExpectRois() const
  {
    return received_expect_roi_msg_->rois;
  }

  // --- Member variables ---

  std::shared_ptr<MapBasedDetector> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<LaneletMapBin>::SharedPtr map_pub_;
  rclcpp::Publisher<CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;

  rclcpp::Subscription<TrafficLightRoiArray>::SharedPtr roi_sub_;
  rclcpp::Subscription<TrafficLightRoiArray>::SharedPtr expect_roi_sub_;

  TrafficLightRoiArray::SharedPtr received_roi_msg_;
  TrafficLightRoiArray::SharedPtr received_expect_roi_msg_;
  bool roi_received_ = false;
  bool expect_roi_received_ = false;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

  int64_t road_lanelet_id_ = 0;
  int64_t traffic_light_linestring_id_ = 0;
};

// Test case based on the following geometry and camera setup:
//
// coordinate transform: world (x, y, z) -> camera optical (x_opt, y_opt, z_opt)
//   world x (forward)  -> z_opt (depth)
//   world y (left)     -> -x_opt (right is positive in camera)
//   world z (up)       -> -y_opt (down is positive in camera)
//
// top left    : world(20.0,  0.5, 4.5) -> camera(-0.5, -4.5, 20.0)
// right bottom: world(20.0, -0.5, 3.5) -> camera( 0.5, -3.5, 20.0)
//
// for ideal camera
// c_x = 320
// c_y = 240
//
// u = f_x * (x_opt / z_opt) + c_x
// v = f_y * (y_opt / z_opt) + c_y
//
// ------------------------------------------------------------------------------
// without margin (expect ROI)
//
// top left point:
//     u_top_left = f_x * (x_opt / z_opt) + c_x = 400 * (-0.5 / 20) + 320 = 310
//     v_top_left = f_y * (y_opt / z_opt) + c_y = 400 * (-4.5 / 20) + 240 = 150
//
// bottom right point
//     u_bottom_right = f_x * (x_opt / z_opt) + c_x = 400 * (0.5 / 20) + 320 = 330
//     v_bottom_right = f_y * (y_opt / z_opt) + c_y = 400 * (-3.5 / 20) + 240 = 170
//
// (x, y, w, h) -> (310, 150, 20, 20)
//
// ------------------------------------------------------------------------------
// with margin (rough ROI)
//
// margin def.:
//     margin_x = (margin_yaw / 2) * depth + margin_width / 2
//     margin_y = (margin_pitch / 2) * depth + margin_height / 2
//     margin_z = margin_depth / 2
//
// margin_x = (0.01745 / 2) * 20.0 + (0.5 / 2) = 0.4245
// margin_y = (0.01745 / 2) * 20.0 + (0.5 / 2) = 0.4245
// margin_z = 0.5 / 2 = 0.25
//
// top left point (tl_camera_optical - margin):
//     u_top_left = 400 * ((-0.5 - 0.4245) / (20 - 0.25)) + 320 ≒ 301.276
//     v_top_left = 400 * ((-4.5 - 0.4245) / (20 - 0.25)) + 240 ≒ 140.263
//
// bottom right point (tl_camera_optical + margin):
//     u_bottom_right = 400 * ((0.5 + 0.4245) / (20 + 0.25)) + 320 ≒ 338.262
//     v_bottom_right = 400 * ((-3.5 + 0.4245) / (20 + 0.25)) + 240 ≒ 179.249
//
// (x, y, w, h) -> (301.276, 140.263, 36.986 38.986) ≒ (301, 140, 37, 39)
TEST_F(MapBasedDetectorIntegrationTest, MapRouteAndCameraInfoWithVisibleTrafficLightProducesRois)
{
  // Arrange
  publishTransform();
  publishMap();
  publishRoute();

  // Act
  publishCameraInfo();
  ASSERT_TRUE(waitForRoiMessage());

  // Assert

  // output/rois includes vibration margin
  auto rois = getOutputRois();
  ASSERT_EQ(rois.size(), 1u);
  EXPECT_EQ(rois[0].traffic_light_id, traffic_light_linestring_id_);
  EXPECT_NEAR(rois[0].roi.x_offset, 301, 1);
  EXPECT_NEAR(rois[0].roi.y_offset, 140, 1);
  EXPECT_NEAR(rois[0].roi.width, 37, 1);
  EXPECT_NEAR(rois[0].roi.height, 39, 1);

  // expect/rois does not include vibration margin
  auto expect_rois = getExpectRois();
  ASSERT_EQ(expect_rois.size(), 1u);
  EXPECT_EQ(expect_rois[0].traffic_light_id, traffic_light_linestring_id_);
  EXPECT_EQ(expect_rois[0].roi.x_offset, 310u);
  EXPECT_EQ(expect_rois[0].roi.y_offset, 150u);
  EXPECT_EQ(expect_rois[0].roi.width, 20u);
  EXPECT_EQ(expect_rois[0].roi.height, 20u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
