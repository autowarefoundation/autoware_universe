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

#include "../src/traffic_light_map_based_detector.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <rclcpp/time.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using autoware::traffic_light::StampedTransform;
using autoware::traffic_light::TrafficLightMapBasedDetector;
using autoware::traffic_light::TrafficLightMapBasedDetectorConfig;
using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using sensor_msgs::msg::CameraInfo;

/// Default detector config matching the integration test's parameter set.
TrafficLightMapBasedDetectorConfig make_default_config()
{
  TrafficLightMapBasedDetectorConfig config;
  config.max_vibration_pitch = 0.01745;
  config.max_vibration_yaw = 0.01745;
  config.max_vibration_height = 0.5;
  config.max_vibration_width = 0.5;
  config.max_vibration_depth = 0.5;
  config.max_detection_range = 200.0;
  config.car_traffic_light_max_angle_range = 40.0;
  config.pedestrian_traffic_light_max_angle_range = 80.0;
  return config;
}

/// 640x480 ideal pinhole camera (fx=fy=400, cx=320, cy=240, no distortion).
CameraInfo make_default_camera_info()
{
  CameraInfo camera_info;
  camera_info.header.frame_id = "camera_optical_link";
  camera_info.header.stamp = rclcpp::Time(1000, 0);
  camera_info.width = 640;
  camera_info.height = 480;
  camera_info.p = {400.0, 0.0, 320.0, 0.0, 0.0, 400.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info.distortion_model = "plumb_bob";
  camera_info.k = {400.0, 0.0, 320.0, 0.0, 400.0, 240.0, 0.0, 0.0, 1.0};
  camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return camera_info;
}

/// Camera at map origin, looking along +x in map.
/// Optical convention: z_opt=+x_map (forward), x_opt=-y_map (right), y_opt=-z_map (down).
tf2::Transform make_default_camera_pose()
{
  tf2::Transform pose;
  pose.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  pose.setRotation(tf2::Quaternion(-0.5, 0.5, -0.5, 0.5));
  return pose;
}

std::vector<StampedTransform> make_tf_samples(
  const rclcpp::Time & stamp, const tf2::Transform & pose)
{
  return {{stamp, pose}};
}

/// Create a lanelet map with one road lanelet and one traffic light linestring.
/// Geometry matches the integration test so the traffic light is visible from
/// the default camera pose with default config.
LaneletMapBin make_test_map(const std::string & traffic_light_subtype = "red_yellow_green")
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
  traffic_light_ls.attributes()["subtype"] = traffic_light_subtype;
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

/// Extract all road lanelet IDs from a LaneletMapBin.
std::vector<int64_t> get_road_lanelet_ids(const LaneletMapBin & map_bin)
{
  const auto lanelet_map = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin);
  std::vector<int64_t> ids;
  for (const auto & lanelet : lanelet::utils::query::laneletLayer(lanelet_map)) {
    ids.push_back(lanelet.id());
  }
  return ids;
}

/// Extract all traffic light linestring IDs from a LaneletMapBin.
std::vector<int64_t> get_traffic_light_ids(const LaneletMapBin & map_bin)
{
  const auto lanelet_map = autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_bin);
  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  std::vector<int64_t> ids;
  for (const auto & traffic_light_reg_elem :
       lanelet::utils::query::autowareTrafficLights(all_lanelets)) {
    for (const auto & light_string_primitive : traffic_light_reg_elem->trafficLights()) {
      if (light_string_primitive.isLineString()) {
        ids.push_back(static_cast<lanelet::ConstLineString3d>(light_string_primitive).id());
      }
    }
  }
  return ids;
}

LaneletRoute make_route(int64_t lanelet_id)
{
  LaneletRoute route;
  LaneletSegment segment;
  LaneletPrimitive primitive;
  primitive.id = lanelet_id;
  segment.primitives.push_back(primitive);
  route.segments.push_back(segment);
  return route;
}

}  // namespace

TEST(TrafficLightMapBasedDetectorTest, ConstructorThrowsWhenMaxDetectionRangeIsZero)
{
  // Arrange
  auto config = make_default_config();
  config.max_detection_range = 0.0;
  const auto map = make_test_map();

  // Act & Assert
  EXPECT_THROW(TrafficLightMapBasedDetector(config, map), std::invalid_argument);
}

TEST(TrafficLightMapBasedDetectorTest, ConstructorThrowsWhenMaxDetectionRangeIsNegative)
{
  // Arrange
  auto config = make_default_config();
  config.max_detection_range = -1.0;
  const auto map = make_test_map();

  // Act & Assert
  EXPECT_THROW(TrafficLightMapBasedDetector(config, map), std::invalid_argument);
}

TEST(TrafficLightMapBasedDetectorTest, DetectWithoutSetRouteUsesAllMapTrafficLights)
{
  // Arrange
  const auto config = make_default_config();
  const auto map = make_test_map();
  const auto camera_info = make_default_camera_info();
  const auto tf_samples = make_tf_samples(camera_info.header.stamp, make_default_camera_pose());
  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto result = detector.detect(tf_samples, camera_info);

  // Assert
  ASSERT_EQ(result.rough_rois.rois.size(), 1u);
  ASSERT_EQ(result.expect_rois.rois.size(), 1u);
  EXPECT_EQ(result.markers.markers.size(), 1u);
}

TEST(TrafficLightMapBasedDetectorTest, DetectWithEmptyTransformSamplesReturnsEmpty)
{
  // Arrange
  const auto config = make_default_config();
  const auto map = make_test_map();
  const auto camera_info = make_default_camera_info();
  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto result = detector.detect({}, camera_info);

  // Assert
  EXPECT_TRUE(result.rough_rois.rois.empty());
  EXPECT_TRUE(result.expect_rois.rois.empty());
  EXPECT_TRUE(result.markers.markers.empty());
}

TEST(TrafficLightMapBasedDetectorTest, DetectFiltersOutSolidSubtypeTrafficLight)
{
  // Arrange: subtype "solid" represents static signage and must be excluded.
  const auto config = make_default_config();
  const auto map = make_test_map("solid");
  const auto camera_info = make_default_camera_info();
  const auto tf_samples = make_tf_samples(camera_info.header.stamp, make_default_camera_pose());
  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto result = detector.detect(tf_samples, camera_info);

  // Assert
  EXPECT_TRUE(result.rough_rois.rois.empty());
  EXPECT_TRUE(result.expect_rois.rois.empty());
}

TEST(TrafficLightMapBasedDetectorTest, DetectFiltersOutTrafficLightOutsideDistanceRange)
{
  // Arrange: traffic light is ~20 m away, set the detection range below it.
  auto config = make_default_config();
  config.max_detection_range = 5.0;
  const auto map = make_test_map();
  const auto camera_info = make_default_camera_info();
  const auto tf_samples = make_tf_samples(camera_info.header.stamp, make_default_camera_pose());
  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto result = detector.detect(tf_samples, camera_info);

  // Assert
  EXPECT_TRUE(result.rough_rois.rois.empty());
}

TEST(TrafficLightMapBasedDetectorTest, DetectFiltersOutTrafficLightOutsideAngleRange)
{
  // Arrange: rotate the camera 90 deg around the map z axis so its yaw differs
  // from the traffic light yaw by pi/2, beyond the 40-deg car angle range.
  const auto config = make_default_config();
  const auto map = make_test_map();
  const auto camera_info = make_default_camera_info();

  tf2::Quaternion z_rotation;
  z_rotation.setRPY(0.0, 0.0, M_PI_2);
  const auto default_pose = make_default_camera_pose();
  tf2::Transform rotated_pose;
  rotated_pose.setOrigin(default_pose.getOrigin());
  rotated_pose.setRotation(z_rotation * default_pose.getRotation());
  const auto tf_samples = make_tf_samples(camera_info.header.stamp, rotated_pose);

  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto result = detector.detect(tf_samples, camera_info);

  // Assert
  EXPECT_TRUE(result.rough_rois.rois.empty());
}

TEST(TrafficLightMapBasedDetectorTest, SetRouteWithUnknownLaneletIdReturnsError)
{
  // Arrange
  const auto config = make_default_config();
  const auto map = make_test_map();
  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto error = detector.setRoute(make_route(99999999));

  // Assert
  ASSERT_TRUE(error.has_value());
}

TEST(TrafficLightMapBasedDetectorTest, SetRouteWithKnownLaneletIdSucceedsAndDetectFindsRoi)
{
  // Arrange
  const auto config = make_default_config();
  const auto map = make_test_map();
  const auto camera_info = make_default_camera_info();
  const auto tf_samples = make_tf_samples(camera_info.header.stamp, make_default_camera_pose());
  const auto traffic_light_id = get_traffic_light_ids(map)[0];
  const auto route = make_route(get_road_lanelet_ids(map)[0]);
  TrafficLightMapBasedDetector detector(config, map);

  // Act
  const auto error = detector.setRoute(route);
  const auto result = detector.detect(tf_samples, camera_info);

  // Assert
  EXPECT_FALSE(error.has_value());
  ASSERT_EQ(result.rough_rois.rois.size(), 1u);
  EXPECT_EQ(result.rough_rois.rois[0].traffic_light_id, traffic_light_id);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
