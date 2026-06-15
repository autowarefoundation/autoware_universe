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

#include "autoware/tensorrt_yolox/tensorrt_yolox_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <opencv2/opencv.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// cspell: ignore semseg

// Characterization test for TrtYoloXNode driven end-to-end through ROS topics: an image is
// published on ~/in/image and the resulting detections / segmentation masks are verified on
// ~/out/objects, ~/out/mask and ~/out/color_mask.
//
// Two model configurations are exercised:
//   * traffic-light detector (yolox_s_car_ped_tl_detector): detection only, no segmentation head.
//   * general detector + segmentation (yolox-sPlus-opt ... seg16cls): a "multitask" model, so the
//     node additionally publishes the segmentation mask and (optionally) a colorized mask.
//
// The node requires an NVIDIA GPU, TensorRT and the ONNX models (downloaded under autoware_data).
// When any of these are missing the test self-skips (GTEST_SKIP), so it is a no-op on CI machines
// without a GPU. In CMake it is additionally gated behind `TRT_AVAIL AND CUDA_AVAIL`.
//
// Detection outputs (box coordinates / scores) are not bit-reproducible across GPU / TensorRT
// versions, so the assertions are intentionally coarse: presence, class, ROI within bounds and
// score above threshold. Exact pixel coordinates are deliberately not asserted.

namespace
{
using autoware::tensorrt_yolox::TrtYoloXNode;
using tier4_perception_msgs::msg::DetectedObjectsWithFeature;

// The node hard-codes its name to "tensorrt_yolox", so its relative topics resolve to these.
constexpr char input_image_topic[] = "/tensorrt_yolox/in/image";
constexpr char output_objects_topic[] = "/tensorrt_yolox/out/objects";
constexpr char output_mask_topic[] = "/tensorrt_yolox/out/mask";
constexpr char output_color_mask_topic[] = "/tensorrt_yolox/out/color_mask";

// Class ID after remapping by traffic_light_roi_label_remap.csv.
constexpr uint8_t vehicle_traffic_light_label = 1;

// Detection score threshold configured for the node (see the *.param.yaml files).
constexpr float score_threshold = 0.35f;

// Resolve a file shipped under autoware_data. The directory layout differs between setups
// (`autoware_data/tensorrt_yolox` vs `autoware_data/ml_models/tensorrt_yolox`), and an explicit
// override via YOLOX_TEST_DATA_DIR takes precedence.
std::string resolve_autoware_data_file(const std::string & filename)
{
  std::vector<std::string> candidate_dirs;
  if (const char * override_dir = std::getenv("YOLOX_TEST_DATA_DIR")) {
    candidate_dirs.emplace_back(override_dir);
  }
  if (const char * home = std::getenv("HOME")) {
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/tensorrt_yolox");
    candidate_dirs.emplace_back(std::string(home) + "/autoware_data/ml_models/tensorrt_yolox");
  }
  for (const auto & dir : candidate_dirs) {
    const std::string candidate = dir + "/" + filename;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return "";
}

std::string config_file(const std::string & filename)
{
  return ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") + "/config/" +
         filename;
}

// Parameters shared by every configuration (TensorRT builder settings and disabled features).
void append_common_overrides(rclcpp::NodeOptions & options)
{
  options.append_parameter_override("score_threshold", static_cast<double>(score_threshold));
  options.append_parameter_override("nms_threshold", 0.7);
  options.append_parameter_override("calibration_algorithm", std::string("Entropy"));
  options.append_parameter_override("dla_core_id", -1);
  options.append_parameter_override("quantize_first_layer", false);
  options.append_parameter_override("quantize_last_layer", false);
  options.append_parameter_override("profile_per_layer", false);
  options.append_parameter_override("clip_value", 6.0);
  options.append_parameter_override("preprocess_on_gpu", true);
  options.append_parameter_override("calibration_image_list_path", std::string(""));
  options.append_parameter_override("gpu_id", 0);
}

// Build the parameters for the traffic-light detector model (detection only, no segmentation).
rclcpp::NodeOptions make_traffic_light_detector_options(
  const std::string & model_path, const std::string & label_path)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_path", model_path);
  options.append_parameter_override("label_path", label_path);
  // Without a remap file every class maps to "unmapped" and all detections are dropped, so this
  // path is mandatory to exercise the detection output.
  options.append_parameter_override(
    "roi_remap_path", config_file("traffic_light_roi_label_remap.csv"));
  options.append_parameter_override("precision", std::string("fp16"));

  options.append_parameter_override("is_roi_overlap_segmentation", false);
  options.append_parameter_override("is_publish_color_mask", false);
  options.append_parameter_override("overlap_roi_score_threshold", 0.3);
  options.append_parameter_override("roi_overlay_segmentation_label.UNKNOWN", false);
  options.append_parameter_override("roi_overlay_segmentation_label.CAR", false);
  options.append_parameter_override("roi_overlay_segmentation_label.TRUCK", false);
  options.append_parameter_override("roi_overlay_segmentation_label.BUS", false);
  options.append_parameter_override("roi_overlay_segmentation_label.MOTORCYCLE", false);
  options.append_parameter_override("roi_overlay_segmentation_label.BICYCLE", false);
  options.append_parameter_override("roi_overlay_segmentation_label.PEDESTRIAN", false);
  options.append_parameter_override("roi_overlay_segmentation_label.ANIMAL", false);
  append_common_overrides(options);
  return options;
}

// Build the parameters for the multitask model (general detection + semantic segmentation), with
// both the segmentation mask and the colorized mask enabled. fp16 is used instead of the launch's
// int8 to avoid the int8 calibration-cache dependency; the exercised node code is precision
// independent. Settings mirror config/yolox_s_plus_opt.param.yaml.
rclcpp::NodeOptions make_segmentation_options(
  const std::string & model_path, const std::string & label_path,
  const std::string & semantic_segmentation_color_map_path)
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("model_path", model_path);
  options.append_parameter_override("label_path", label_path);
  options.append_parameter_override("roi_remap_path", config_file("roi_label_remap.csv"));
  options.append_parameter_override(
    "semantic_segmentation_color_map_path", semantic_segmentation_color_map_path);
  options.append_parameter_override(
    "roi_to_semantic_segmentation_remap_path", config_file("roi_to_semseg_label_remap.csv"));
  options.append_parameter_override("precision", std::string("fp16"));

  options.append_parameter_override("is_roi_overlap_segmentation", true);
  options.append_parameter_override("is_publish_color_mask", true);
  options.append_parameter_override("overlap_roi_score_threshold", 0.3);
  options.append_parameter_override("roi_overlay_segmentation_label.UNKNOWN", true);
  options.append_parameter_override("roi_overlay_segmentation_label.CAR", false);
  options.append_parameter_override("roi_overlay_segmentation_label.TRUCK", false);
  options.append_parameter_override("roi_overlay_segmentation_label.BUS", false);
  options.append_parameter_override("roi_overlay_segmentation_label.MOTORCYCLE", true);
  options.append_parameter_override("roi_overlay_segmentation_label.BICYCLE", true);
  options.append_parameter_override("roi_overlay_segmentation_label.PEDESTRIAN", true);
  options.append_parameter_override("roi_overlay_segmentation_label.ANIMAL", true);
  append_common_overrides(options);
  return options;
}

sensor_msgs::msg::Image to_image_msg(const cv::Mat & bgr_image, const rclcpp::Time & stamp)
{
  std_msgs::msg::Header header;
  header.stamp = stamp;
  header.frame_id = "camera";
  return *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, bgr_image).toImageMsg();
}

cv::Mat make_black_image(int width, int height)
{
  return cv::Mat::zeros(height, width, CV_8UC3);
}

cv::Mat load_test_image()
{
  const std::string path = ament_index_cpp::get_package_share_directory("autoware_tensorrt_yolox") +
                           "/test/test_image.jpg";
  const cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.empty()) {
    throw std::runtime_error("failed to load test image: " + path);
  }
  return image;
}

size_t count_objects_with_label(const DetectedObjectsWithFeature & objects, uint8_t label)
{
  size_t count = 0;
  for (const auto & feature_object : objects.feature_objects) {
    if (
      !feature_object.object.classification.empty() &&
      feature_object.object.classification.front().label == label) {
      ++count;
    }
  }
  return count;
}

bool all_rois_within_image(
  const DetectedObjectsWithFeature & objects, uint32_t width, uint32_t height)
{
  for (const auto & feature_object : objects.feature_objects) {
    const auto & roi = feature_object.feature.roi;
    if (roi.x_offset + roi.width > width || roi.y_offset + roi.height > height) {
      return false;
    }
  }
  return true;
}

// Asserts a non-empty set of detections was published with every ROI inside the image bounds.
// This is the common, lenient validation shared by the real-image tests; per-test specifics
// (e.g. an expected class) are asserted separately by the caller.
void expect_objects_detected(
  const DetectedObjectsWithFeature::ConstSharedPtr & objects, const cv::Mat & image)
{
  ASSERT_NE(objects, nullptr) << "node did not publish any objects message";
  EXPECT_FALSE(objects->feature_objects.empty());
  EXPECT_TRUE(all_rois_within_image(
    *objects, static_cast<uint32_t>(image.cols), static_cast<uint32_t>(image.rows)));
}

// Asserts the multitask model published a non-empty segmentation mask (class-id map).
void expect_segmentation_mask_published(const sensor_msgs::msg::Image::ConstSharedPtr & mask)
{
  ASSERT_NE(mask, nullptr) << "multitask model did not publish a segmentation mask";
  EXPECT_GT(mask->width, 0u);
  EXPECT_GT(mask->height, 0u);
}

// Asserts a non-empty colorized mask was published as a BGR8 image.
void expect_color_mask_published(const sensor_msgs::msg::Image::ConstSharedPtr & color_mask)
{
  ASSERT_NE(color_mask, nullptr) << "color mask was not published";
  EXPECT_EQ(color_mask->encoding, sensor_msgs::image_encodings::BGR8);
  EXPECT_GT(color_mask->width, 0u);
  EXPECT_GT(color_mask->height, 0u);
}
}  // namespace

class TrtYoloXNodeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    test_node_ = std::make_shared<rclcpp::Node>("yolox_integration_test_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);

    objects_subscription_ = test_node_->create_subscription<DetectedObjectsWithFeature>(
      output_objects_topic, rclcpp::QoS(10),
      [this](const DetectedObjectsWithFeature::ConstSharedPtr message) {
        received_objects_ = message;
      });
    mask_subscription_ = test_node_->create_subscription<sensor_msgs::msg::Image>(
      output_mask_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) { received_mask_ = message; });
    color_mask_subscription_ = test_node_->create_subscription<sensor_msgs::msg::Image>(
      output_color_mask_topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
        received_color_mask_ = message;
      });
    image_publisher_ = test_node_->create_publisher<sensor_msgs::msg::Image>(
      input_image_topic, rclcpp::SensorDataQoS());
  }

  void TearDown() override
  {
    executor_.reset();
    image_publisher_.reset();
    objects_subscription_.reset();
    mask_subscription_.reset();
    color_mask_subscription_.reset();
    test_node_.reset();
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  // Construct the node under test. Returns false when GPU initialization fails: the node
  // constructor calls rclcpp::shutdown() in that case, which we detect via rclcpp::ok().
  // The first call for a given model builds the TensorRT engine (a few minutes) before returning.
  bool initialize_yolox_node(const rclcpp::NodeOptions & options)
  {
    node_ = std::make_shared<TrtYoloXNode>(options);
    if (!rclcpp::ok()) {
      return false;
    }
    executor_->add_node(node_);
    return true;
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // Publish the image repeatedly (best-effort QoS may drop early messages, and the node only
  // subscribes lazily once it sees our output subscriber) while spinning, until the node publishes
  // a detected-objects message or the timeout elapses.
  void publish_until_detected_objects_received(
    const sensor_msgs::msg::Image & image,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(30000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!received_objects_ && std::chrono::steady_clock::now() < deadline) {
      image_publisher_->publish(image);
      spin_for(std::chrono::milliseconds(100));
    }
  }

  // Wait for the segmentation and color masks. The multitask model publishes them from the same
  // onImage call that produced the detections, so once objects have arrived we only need to keep
  // spinning until those (reliable) messages are delivered.
  void wait_for_mask_messages(std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while ((!received_mask_ || !received_color_mask_) &&
           std::chrono::steady_clock::now() < deadline) {
      spin_for(std::chrono::milliseconds(20));
    }
  }

  std::shared_ptr<TrtYoloXNode> node_;
  std::shared_ptr<rclcpp::Node> test_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Subscription<DetectedObjectsWithFeature>::SharedPtr objects_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_mask_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  DetectedObjectsWithFeature::ConstSharedPtr received_objects_;
  sensor_msgs::msg::Image::ConstSharedPtr received_mask_;
  sensor_msgs::msg::Image::ConstSharedPtr received_color_mask_;
};

// A blank (all-black) image yields an inference pass with no detections above the threshold,
// so the node still publishes a DetectedObjectsWithFeature message with an empty object list.
TEST_F(TrtYoloXNodeIntegrationTest, PublishesEmptyObjectsForBlankImage)
{
  // Arrange
  const std::string model_path =
    resolve_autoware_data_file("yolox_s_car_ped_tl_detector_960_960_batch_1.onnx");
  const std::string label_path = resolve_autoware_data_file("car_ped_tl_detector_labels.txt");
  if (model_path.empty() || label_path.empty()) {
    GTEST_SKIP() << "traffic-light detector model/label not found under autoware_data.";
  }
  if (!initialize_yolox_node(make_traffic_light_detector_options(model_path, label_path))) {
    GTEST_SKIP() << "GPU is not available for inference.";
  }
  const auto blank_image = to_image_msg(make_black_image(1280, 827), test_node_->now());

  // Act
  publish_until_detected_objects_received(blank_image);

  // Assert
  ASSERT_NE(received_objects_, nullptr) << "node did not publish any objects message";
  EXPECT_TRUE(received_objects_->feature_objects.empty());
}

// Longest detection path: a real camera image containing traffic lights drives the full onImage
// loop (inference -> per-object classification remap -> publish a non-empty object list).
TEST_F(TrtYoloXNodeIntegrationTest, DetectsTrafficLightInRealImage)
{
  // Arrange
  const std::string model_path =
    resolve_autoware_data_file("yolox_s_car_ped_tl_detector_960_960_batch_1.onnx");
  const std::string label_path = resolve_autoware_data_file("car_ped_tl_detector_labels.txt");
  if (model_path.empty() || label_path.empty()) {
    GTEST_SKIP() << "traffic-light detector model/label not found under autoware_data.";
  }
  if (!initialize_yolox_node(make_traffic_light_detector_options(model_path, label_path))) {
    GTEST_SKIP() << "GPU is not available for inference.";
  }
  const cv::Mat image = load_test_image();
  const auto image_msg = to_image_msg(image, test_node_->now());

  // Act
  publish_until_detected_objects_received(image_msg);

  // Assert
  expect_objects_detected(received_objects_, image);
  EXPECT_GE(count_objects_with_label(*received_objects_, vehicle_traffic_light_label), 1u);
}

// Longest segmentation path: the multitask model additionally publishes a segmentation mask and a
// colorized mask, exercising the segmentation branches of onImage (mask publish, ROI overlay and
// getColorizedMask) that the detection-only model never reaches.
TEST_F(TrtYoloXNodeIntegrationTest, PublishesSegmentationMaskForMultitaskModel)
{
  // Arrange
  const std::string model_path =
    resolve_autoware_data_file("yolox-sPlus-opt-pseudoV2-T4-960x960-T4-seg16cls.onnx");
  const std::string label_path = resolve_autoware_data_file("label.txt");
  const std::string color_map_path = resolve_autoware_data_file("semseg_color_map.csv");
  if (model_path.empty() || label_path.empty() || color_map_path.empty()) {
    GTEST_SKIP() << "segmentation model/label/color-map not found under autoware_data.";
  }
  if (!initialize_yolox_node(make_segmentation_options(model_path, label_path, color_map_path))) {
    GTEST_SKIP() << "GPU is not available for inference.";
  }
  const cv::Mat image = load_test_image();
  const auto image_msg = to_image_msg(image, test_node_->now());

  // Act
  publish_until_detected_objects_received(image_msg);
  wait_for_mask_messages();

  // Assert
  expect_objects_detected(received_objects_, image);
  expect_segmentation_mask_published(received_mask_);
  expect_color_mask_published(received_color_mask_);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
