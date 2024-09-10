// Copyright 2024 TIER IV, Inc.
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

#include "tensorrt_rtmdet/tensorrt_rtmdet_node.hpp"

#include <dlfcn.h>

#include <fstream>

namespace autoware::tensorrt_rtmdet
{
TrtRTMDetNode::TrtRTMDetNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_rtmdet", node_options),
  is_publish_color_mask_(declare_parameter<bool>("is_publish_color_mask")),
  is_publish_debug_image_(declare_parameter<bool>("is_publish_debug_image"))
{
  {
    stop_watch_ptr_ =
      std::make_unique<autoware::universe_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ =
      std::make_unique<autoware::universe_utils::DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  std::string model_path = declare_parameter<std::string>("model_path");
  std::string color_map_path = declare_parameter<std::string>("color_map_path");
  std::string precision = declare_parameter<std::string>("precision");
  std::vector<double> mean = declare_parameter<std::vector<double>>("input_image_mean");
  std::vector<double> std = declare_parameter<std::vector<double>>("input_image_std_dev");
  double score_threshold = declare_parameter<double>("score_threshold");
  double nms_threshold = declare_parameter<double>("nms_threshold");
  double mask_threshold = declare_parameter<double>("mask_threshold");
  std::string calibration_algorithm = declare_parameter<std::string>("calibration_algorithm");
  int dla_core_id = declare_parameter<int>("dla_core_id");
  bool quantize_first_layer = declare_parameter<bool>("quantize_first_layer");
  bool quantize_last_layer = declare_parameter<bool>("quantize_last_layer");
  bool profile_per_layer = declare_parameter<bool>("profile_per_layer");
  double clip_value = declare_parameter<double>("clip_value");
  bool preprocess_on_gpu = declare_parameter<bool>("preprocess_on_gpu");
  std::string calibration_image_list_path =
    declare_parameter<std::string>("calibration_image_list_path");
  std::vector<std::string> plugin_paths =
    declare_parameter<std::vector<std::string>>("plugin_paths");

  color_map_ = readLabelFile(color_map_path);

  tensorrt_common::BuildConfig build_config(
    calibration_algorithm, dla_core_id, quantize_first_layer, quantize_last_layer,
    profile_per_layer, clip_value);

  const double norm_factor = 1.0;
  const std::string cache_dir = "";
  const tensorrt_common::BatchConfig batch_config{1, 1, 1};
  const size_t max_workspace_size = (1 << 30);

  mean_ = std::vector<float>(mean.begin(), mean.end());
  std_ = std::vector<float>(std.begin(), std.end());

  trt_rtmdet_ = std::make_unique<tensorrt_rtmdet::TrtRTMDet>(
    model_path, precision, color_map_, score_threshold, nms_threshold, mask_threshold, build_config,
    preprocess_on_gpu, calibration_image_list_path, norm_factor, mean_, std_, cache_dir,
    batch_config, max_workspace_size, plugin_paths);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtRTMDetNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  mask_pub_ =
    this->create_publisher<autoware_internal_msgs::msg::SegmentationMask>("~/out/mask", 1);

  color_mask_pub_ = image_transport::create_publisher(this, "~/out/color_mask");
  debug_image_pub_ = image_transport::create_publisher(this, "~/out/debug_image");

  if (declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void TrtRTMDetNode::onConnect()
{
  using std::placeholders::_1;
  if (
    debug_image_pub_.getNumSubscribers() == 0 &&
    mask_pub_->get_subscription_count() == 0 && mask_pub_->get_intra_process_subscription_count() == 0 &&
    color_mask_pub_.getNumSubscribers() == 0 && objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0) {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "~/in/image", std::bind(&TrtRTMDetNode::onImage, this, _1), "raw",
      rmw_qos_profile_sensor_data);
  }
}

void TrtRTMDetNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  stop_watch_ptr_->toc("processing_time", true);

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  tensorrt_rtmdet::ObjectArrays objects;
  cv::Mat mask;
  std::vector<uint8_t> class_ids;
  if (!trt_rtmdet_->doInference({in_image_ptr->image}, objects, mask, class_ids)) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    return;
  }

  tier4_perception_msgs::msg::DetectedObjectsWithFeature detected_objects_with_feature;
  for (const auto & object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature detected_object_with_feature;
    detected_object_with_feature.feature.roi.width = object.x2 - object.x1;
    detected_object_with_feature.feature.roi.height = object.y2 - object.y1;
    detected_object_with_feature.feature.roi.x_offset = object.x1;
    detected_object_with_feature.feature.roi.y_offset = object.y1;
    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = color_map_[object.class_id].label_id;
    classification.probability = object.score;
    detected_object_with_feature.object.classification =
      std::vector<autoware_perception_msgs::msg::ObjectClassification>{classification};

    detected_objects_with_feature.feature_objects.push_back(detected_object_with_feature);
  }
  detected_objects_with_feature.header = msg->header;
  objects_pub_->publish(detected_objects_with_feature);

  if (!mask.empty() && !class_ids.empty()) {
    sensor_msgs::msg::Image::SharedPtr mask_image =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, mask)
        .toImageMsg();
    autoware_internal_msgs::msg::SegmentationConfig mask_config;
    std::vector<autoware_perception_msgs::msg::ObjectClassification> classification;
    for (unsigned char class_id : class_ids) {
      autoware_perception_msgs::msg::ObjectClassification object_classification;
      object_classification.label = class_id;
      object_classification.probability = 1.0;
      classification.push_back(object_classification);
    }
    mask_config.classification = classification;
    autoware_internal_msgs::msg::SegmentationMask mask_msg;
    mask_msg.config = mask_config;
    mask_msg.image = *mask_image;
    mask_msg.header = msg->header;
    mask_pub_->publish(mask_msg);
  }

  if (is_publish_color_mask_) {
    cv::Mat color_mask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    trt_rtmdet_->getColorizedMask(trt_rtmdet_->getColorMap(), mask, color_mask);
    sensor_msgs::msg::Image::SharedPtr color_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, color_mask)
        .toImageMsg();
    color_mask_msg->header = msg->header;
    color_mask_pub_.publish(color_mask_msg);
  }

  if (is_publish_debug_image_) {
    cv::Mat debug_image = in_image_ptr->image.clone();
    drawDebugImage(debug_image, mask, objects, color_map_);
    sensor_msgs::msg::Image::SharedPtr debug_image_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, debug_image)
        .toImageMsg();
    debug_image_msg->header = msg->header;
    debug_image_pub_.publish(debug_image_msg);
  }

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<tier4_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }
}

ColorMap TrtRTMDetNode::readLabelFile(const std::string & label_path)
{
  ColorMap color_map;

  std::vector<std::string> color_list;
  {
    assert(fileExists(label_path, true));

    std::ifstream f(label_path);
    if (!f) {
      RCLCPP_ERROR(this->get_logger(), "failed to open %s", label_path.c_str());
      assert(0);
    }

    std::string line;
    while (std::getline(f, line)) {
      if (line.empty()) {
        continue;
      } else {
        line.erase(
          line.begin(), find_if(line.begin(), line.end(), [](int ch) { return !isspace(ch); }));
        line.erase(
          find_if(line.rbegin(), line.rend(), [](int ch) { return !isspace(ch); }).base(),
          line.end());
        color_list.push_back(line);
      }
    }
  }

  for (int i = 1; i < static_cast<int>(color_list.size()); i++) {
    auto splitString = [](const std::string & str, char delimiter) -> std::vector<std::string> {
      std::vector<std::string> result;
      std::stringstream ss(str);
      std::string item;

      while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
      }

      return result;
    };
    std::vector<std::string> tokens = splitString(color_list.at(i), ',');

    LabelColor label_color;
    label_color.class_name = tokens.at(1);
    label_color.color[0] = std::stoi(tokens.at(2));
    label_color.color[1] = std::stoi(tokens.at(3));
    label_color.color[2] = std::stoi(tokens.at(4));
    label_color.label_id = std::stoi(tokens.at(5));
    color_map[std::stoi(tokens.at(0))] = label_color;
  }

  return color_map;
}

void TrtRTMDetNode::drawDebugImage(
  cv::Mat & image, [[maybe_unused]] const cv::Mat & mask,
  const tensorrt_rtmdet::ObjectArrays & objects,
  [[maybe_unused]] const tensorrt_rtmdet::ColorMap & color_map)
{
  // TODO(StepTurtle): add mask to debug image

  for (const auto & object : objects.at(0)) {
    cv::rectangle(
      image, cv::Point(static_cast<int>(object.x1), static_cast<int>(object.y1)),
      cv::Point(static_cast<int>(object.x2), static_cast<int>(object.y2)),
      color_map_[object.class_id].color, 2);
    // Write the class name
    cv::putText(
      image, color_map_[object.class_id].class_name,
      cv::Point(static_cast<int>(object.x1), static_cast<int>(object.y1)), cv::FONT_HERSHEY_SIMPLEX,
      1, color_map_[object.class_id].color, 2);
  }
}
}  // namespace autoware::tensorrt_rtmdet

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_rtmdet::TrtRTMDetNode)
