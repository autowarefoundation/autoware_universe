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

//
// Integration test for TrafficLightRecognitionNode.
//
// Stands up the real node, publishes an empty vector map (transient_local) and a synchronized
// (black Image, CameraInfo) pair plus the map->camera tf, and asserts that the node responds with
// an empty traffic-signal recognition result -- there is nothing to recognize when the map names
// no traffic lights.
//
// The node's constructor builds three TensorRT engines, so this suite needs a GPU + TensorRT + the
// ONNX models under autoware_data. It is gated in CMakeLists.txt behind TRT_AVAIL AND CUDA_AVAIL
// and self-skips (GTEST_SKIP) when no usable GPU is found at runtime.
//

#include "traffic_light_recognition/traffic_light_recognition_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{
namespace traffic_light = autoware::traffic_light;

constexpr char kCameraFrame[] = "camera_optical_link";

std::string ml_model_path()
{
  if (const char * override_dir = std::getenv("TL_PIPELINE_TEST_DATA_DIR")) {
    return override_dir;
  }
  const char * home = std::getenv("HOME");
  return home ? std::string(home) + "/autoware_data" : "";
}

std::string config_file_path()
{
  return ament_index_cpp::get_package_share_directory("autoware_traffic_light_pipeline") +
         "/config/traffic_light_recognition.param.yaml";
}

rclcpp::NodeOptions make_node_options()
{
  const auto yolox_roi_remap =
    ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") +
    "/config/traffic_light_roi_label_remap.csv";

  rclcpp::NodeOptions options;
  options.arguments({
    "--ros-args",
    "--params-file",
    config_file_path(),
    "-p",
    "ml_model_path:=" + ml_model_path(),
    "-p",
    "whole_image_detector.roi_remap_path:=" + yolox_roi_remap,
  });
  return options;
}

autoware_map_msgs::msg::LaneletMapBin make_empty_map()
{
  const auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  auto map_bin = autoware::experimental::lanelet2_utils::to_autoware_map_msgs(lanelet_map);
  map_bin.header.frame_id = "map";
  return map_bin;
}

sensor_msgs::msg::Image make_black_image(int width, int height, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::Image image;
  image.header.frame_id = kCameraFrame;
  image.header.stamp = stamp;
  image.height = static_cast<uint32_t>(height);
  image.width = static_cast<uint32_t>(width);
  image.encoding = "bgr8";
  image.is_bigendian = 0;
  image.step = image.width * 3;
  image.data.assign(static_cast<size_t>(image.step) * image.height, 0);
  return image;
}

sensor_msgs::msg::CameraInfo make_camera_info(int width, int height, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header.frame_id = kCameraFrame;
  camera_info.header.stamp = stamp;
  camera_info.width = static_cast<uint32_t>(width);
  camera_info.height = static_cast<uint32_t>(height);
  const double fx = width;
  const double fy = width;
  const double cx = width / 2.0;
  const double cy = height / 2.0;
  camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info.distortion_model = "plumb_bob";
  camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return camera_info;
}

// An empty map names no traffic lights, so the node reports an empty recognition result.
TEST(TrafficLightRecognitionNodeTest, EmptyMapYieldsEmptySignals)
{
  if (!autoware::cuda_utils::is_cuda_runtime_available()) {
    GTEST_SKIP() << "CUDA runtime / GPU not available";
  }

  // Arrange
  auto node = std::make_shared<traffic_light::TrafficLightRecognitionNode>(make_node_options());
  auto tester = std::make_shared<rclcpp::Node>("integration_tester");

  tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr received_signals;
  auto signals_sub = tester->create_subscription<tier4_perception_msgs::msg::TrafficLightArray>(
    "/traffic_light_recognition/output/traffic_signals", rclcpp::QoS{1},
    [&received_signals](tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr msg) {
      received_signals = msg;
    });

  auto map_pub = tester->create_publisher<autoware_map_msgs::msg::LaneletMapBin>(
    "/traffic_light_recognition/input/vector_map", rclcpp::QoS{1}.transient_local());
  auto image_pub = tester->create_publisher<sensor_msgs::msg::Image>(
    "/traffic_light_recognition/input/image", rclcpp::SensorDataQoS());
  auto camera_info_pub = tester->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/traffic_light_recognition/input/camera_info", rclcpp::SensorDataQoS());
  tf2_ros::StaticTransformBroadcaster tf_broadcaster(tester);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(tester);

  map_pub->publish(make_empty_map());

  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.header.stamp = tester->now();
  transform.child_frame_id = kCameraFrame;
  transform.transform.rotation.w = 1.0;
  tf_broadcaster.sendTransform(transform);

  const rclcpp::Time stamp(20, 0);
  const auto image = make_black_image(640, 480, stamp);
  const auto camera_info = make_camera_info(640, 480, stamp);

  // Act
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
  while (!received_signals && std::chrono::steady_clock::now() < deadline) {
    image_pub->publish(image);
    camera_info_pub->publish(camera_info);
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Assert
  ASSERT_TRUE(received_signals);
  EXPECT_TRUE(received_signals->signals.empty());
}

}  // namespace

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
