// Copyright 2022 TIER IV, Inc.
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

#include "autoware/object_recognition_utils/object_classification.hpp"
#include "autoware/tensorrt_yolox/label.hpp"
#include "perception_utils/run_length_encoder.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_yolox
{
TrtYoloXNode::TrtYoloXNode(const rclcpp::NodeOptions & node_options)
: Node("tensorrt_yolox", node_options)
{
  {
    stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }
  using std::placeholders::_1;
  using std::chrono_literals::operator""ms;

  const std::string model_path = this->declare_parameter<std::string>("model_path");
  const std::string precision = this->declare_parameter<std::string>("precision");
  const float score_threshold =
    static_cast<float>(this->declare_parameter<double>("score_threshold"));
  const float nms_threshold = static_cast<float>(this->declare_parameter<double>("nms_threshold"));
  const std::string calibration_algorithm =
    this->declare_parameter<std::string>("calibration_algorithm");
  const int dla_core_id = this->declare_parameter<int>("dla_core_id");
  const bool quantize_first_layer = this->declare_parameter<bool>("quantize_first_layer");
  const bool quantize_last_layer = this->declare_parameter<bool>("quantize_last_layer");
  const bool profile_per_layer = this->declare_parameter<bool>("profile_per_layer");
  const double clip_value = this->declare_parameter<double>("clip_value");
  const bool preprocess_on_gpu = this->declare_parameter<bool>("preprocess_on_gpu");
  const std::string calibration_image_list_path =
    this->declare_parameter<std::string>("calibration_image_list_path");
  const uint8_t gpu_id = this->declare_parameter<uint8_t>("gpu_id");

  const std::string label_path = this->declare_parameter<std::string>("label_path");
  const std::string semseg_color_map_path =
    this->declare_parameter<std::string>("semantic_segmentation_color_map_path", "");

  // if the remap file path is an empty string, it will not do remap the labels
  const std::string roi_remap_path = this->declare_parameter<std::string>("roi_remap_path", "");
  const std::string roi_to_semseg_remap_path =
    this->declare_parameter<std::string>("semantic_segmentation_remap_path", "");

  is_roi_overlap_semseg_ = declare_parameter<bool>("is_roi_overlap_segmentation");
  is_publish_color_mask_ = declare_parameter<bool>("is_publish_color_mask");
  overlap_roi_score_threshold_ = declare_parameter<float>("overlap_roi_score_threshold");
  roi_overlay_semseg_labels_.UNKNOWN =
    declare_parameter<bool>("roi_overlay_segmentation_label.UNKNOWN");
  roi_overlay_semseg_labels_.CAR = declare_parameter<bool>("roi_overlay_segmentation_label.CAR");
  roi_overlay_semseg_labels_.TRUCK =
    declare_parameter<bool>("roi_overlay_segmentation_label.TRUCK");
  roi_overlay_semseg_labels_.BUS = declare_parameter<bool>("roi_overlay_segmentation_label.BUS");
  roi_overlay_semseg_labels_.MOTORCYCLE =
    declare_parameter<bool>("roi_overlay_segmentation_label.MOTORCYCLE");
  roi_overlay_semseg_labels_.BICYCLE =
    declare_parameter<bool>("roi_overlay_segmentation_label.BICYCLE");
  roi_overlay_semseg_labels_.PEDESTRIAN =
    declare_parameter<bool>("roi_overlay_segmentation_label.PEDESTRIAN");
  roi_overlay_semseg_labels_.ANIMAL =
    declare_parameter<bool>("roi_overlay_segmentation_label.ANIMAL");

  if (is_publish_color_mask_ && semseg_color_map_path.empty()) {
    std::stringstream error_msg;
    error_msg << "semseg_color_map_path must be specified"
              << "when `is_publish_color_mask_` is true.";
    throw std::runtime_error{error_msg.str()};
  }

  if (is_roi_overlap_semseg_ && roi_to_semseg_remap_path.empty()) {
    std::stringstream error_msg;
    error_msg << "semantic_segmentation_remap_path must be specified"
              << "when `is_roi_overlap_segmentation` is true.";
    throw std::runtime_error{error_msg.str()};
  }

  // setup the label information and process the remappings
  setupLabel(label_path, semseg_color_map_path, roi_remap_path, roi_to_semseg_remap_path);

  TrtCommonConfig trt_config(
    model_path, precision, "", (1ULL << 30U), dla_core_id, profile_per_layer);

  CalibrationConfig calib_config(
    calibration_algorithm, quantize_first_layer, quantize_last_layer, clip_value);

  const double norm_factor = 1.0;
  const std::string cache_dir = "";

  trt_yolox_ = std::make_unique<tensorrt_yolox::TrtYoloX>(
    trt_config, roi_id_to_name_map_.size(), score_threshold, nms_threshold, preprocess_on_gpu,
    gpu_id, calibration_image_list_path, norm_factor, cache_dir, calib_config);

  if (!trt_yolox_->isGPUInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "GPU %d does not exist or is not suitable.", gpu_id);
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "GPU %d is selected for the inference!", gpu_id);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtYoloXNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  mask_pub_ = image_transport::create_publisher(this, "~/out/mask");
  color_mask_pub_ = image_transport::create_publisher(this, "~/out/color_mask");
  image_pub_ = image_transport::create_publisher(this, "~/out/image");

  if (declare_parameter("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine file is built and exit.");
    rclcpp::shutdown();
  }
}

void TrtYoloXNode::onConnect()
{
  using std::placeholders::_1;
  if (
    objects_pub_->get_subscription_count() == 0 &&
    objects_pub_->get_intra_process_subscription_count() == 0 &&
    image_pub_.getNumSubscribers() == 0 && mask_pub_.getNumSubscribers() == 0 &&
    color_mask_pub_.getNumSubscribers() == 0) {
    image_sub_.shutdown();
  } else if (!image_sub_) {
    image_sub_ = image_transport::create_subscription(
      this, "~/in/image", std::bind(&TrtYoloXNode::onImage, this, _1), "raw",
      rmw_qos_profile_sensor_data);
  }
}

void TrtYoloXNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  tier4_perception_msgs::msg::DetectedObjectsWithFeature out_objects;

  cv_bridge::CvImagePtr in_image_ptr;
  try {
    in_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  const auto width = in_image_ptr->image.cols;
  const auto height = in_image_ptr->image.rows;

  tensorrt_yolox::ObjectArrays objects;
  std::vector<cv::Mat> masks = {cv::Mat(cv::Size(height, width), CV_8UC1, cv::Scalar(0))};
  std::vector<cv::Mat> color_masks = {
    cv::Mat(cv::Size(height, width), CV_8UC3, cv::Scalar(0, 0, 0))};

  if (!trt_yolox_->doInference({in_image_ptr->image}, objects, masks, color_masks)) {
    RCLCPP_WARN(this->get_logger(), "Fail to inference");
    return;
  }
  auto & mask = masks.at(0);

  for (const auto & yolox_object : objects.at(0)) {
    tier4_perception_msgs::msg::DetectedObjectWithFeature object;
    object.feature.roi.x_offset = yolox_object.x_offset;
    object.feature.roi.y_offset = yolox_object.y_offset;
    object.feature.roi.width = yolox_object.width;
    object.feature.roi.height = yolox_object.height;
    object.object.existence_probability = yolox_object.score;

    // direct mapping from YOLOX ID to class ID
    const int target_class_id = roi_id_to_class_id_map_[yolox_object.type];

    // drop the object if it is marked as ignore
    if (target_class_id == unmapped_class_id_) continue;

    autoware_perception_msgs::msg::ObjectClassification classification;
    classification.label = static_cast<uint8_t>(target_class_id);
    classification.probability = 1.0f;

    object.object.classification.push_back(classification);

    out_objects.feature_objects.push_back(object);

    const auto left = std::max(0, static_cast<int>(object.feature.roi.x_offset));
    const auto top = std::max(0, static_cast<int>(object.feature.roi.y_offset));
    const auto right =
      std::min(static_cast<int>(object.feature.roi.x_offset + object.feature.roi.width), width);
    const auto bottom =
      std::min(static_cast<int>(object.feature.roi.y_offset + object.feature.roi.height), height);
    cv::rectangle(
      in_image_ptr->image, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(0, 0, 255), 3,
      8, 0);
    // Refine mask: replacing segmentation mask by roi class
    // This should remove when the segmentation accuracy is high
    if (is_roi_overlap_semseg_ && trt_yolox_->getMultitaskNum() > 0) {
      overlapSegmentByRoi(yolox_object, mask, width, height);
    }
  }

  if (trt_yolox_->getMultitaskNum() > 0) {
    sensor_msgs::msg::Image::SharedPtr out_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::MONO8, mask)
        .toImageMsg();
    out_mask_msg->header = msg->header;

    std::vector<std::pair<uint8_t, int>> compressed_data = perception_utils::runLengthEncoder(mask);
    int step = sizeof(uint8_t) + sizeof(int);
    out_mask_msg->data.resize(static_cast<int>(compressed_data.size()) * step);
    for (size_t i = 0; i < compressed_data.size(); ++i) {
      std::memcpy(&out_mask_msg->data[i * step], &compressed_data.at(i).first, sizeof(uint8_t));
      std::memcpy(&out_mask_msg->data[i * step + 1], &compressed_data.at(i).second, sizeof(int));
    }

    mask_pub_.publish(out_mask_msg);
  }

  image_pub_.publish(in_image_ptr->toImageMsg());
  out_objects.header = msg->header;
  objects_pub_->publish(out_objects);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - out_objects.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  if (is_publish_color_mask_ && trt_yolox_->getMultitaskNum() > 0) {
    cv::Mat color_mask = cv::Mat::zeros(mask.rows, mask.cols, CV_8UC3);
    getColorizedMask(mask, color_mask);

    sensor_msgs::msg::Image::SharedPtr output_color_mask_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, color_mask)
        .toImageMsg();
    output_color_mask_msg->header = msg->header;
    color_mask_pub_.publish(output_color_mask_msg);
  }
}

/**
 * @brief Read label files and remap files. Then remap the labels based on the remap information.
 *
 * This method will process label and remap data in the following order:
 *
 *  1. Read the label and remap files for ROI output.
 *  2. Remap the ROI label based on the remap information.
 *  3. Read the color map and remap files for semantic segmentation output.
 *  4. Create a remap from ROI to segmentation label based on the remap information.
 *
 * you still need to use the original label name even you remap the ROI label for
 * remapping segmentation label.
 *
 * @param[in] roi_label_path file path of label file for ROI
 * @param[in] semseg_color_map_path file path of color map file for segmentation
 * @param[in] roi_label_remap_path file path of remap file for ROI
 * @param[in] roi_to_semseg_remap_path file path of remap file for segmentation
 */
void TrtYoloXNode::setupLabel(
  const std::string & roi_label_path, const std::string & semseg_color_map_path,
  const std::string & roi_label_remap_path, const std::string & roi_to_semseg_remap_path)
{
  try {
    std::unordered_map<std::string, int> roi_name_to_id_map;
    // read label file and store to roi_id_to_name_map_
    read_label_file(roi_label_path, roi_id_to_name_map_, roi_name_to_id_map);

    roi_id_to_class_id_map_.assign(roi_id_to_name_map_.size(), unmapped_class_id_);

    if (!roi_label_remap_path.empty()) {
      std::unordered_map<std::string, int> roi_label_to_new_id_remap;
      constexpr uint32_t skip_header_lines = 1;
      // load remapping of ROI to autoware interface class types
      // e.g. MOTORBIKE -> 5 (MOTORCYCLE)
      load_label_id_remap_file(roi_label_remap_path, roi_label_to_new_id_remap, skip_header_lines);

      // map original YOLOX ID directly to class ID
      for (size_t i = 0; i < roi_id_to_name_map_.size(); ++i) {
        const std::string & original_name = roi_id_to_name_map_[i];

        if (roi_label_to_new_id_remap.count(original_name) > 0) {
          roi_id_to_class_id_map_[i] = roi_label_to_new_id_remap.at(original_name);
        } else {
          // if there is no label name in the original YOLOX class, we will consider as an error
          // since it might using the wrong model
          std::stringstream error_msg;
          error_msg << "ROI label " << original_name << " not found in remap file.";
          throw std::runtime_error{error_msg.str()};
        }
      }
    }

    std::unordered_map<std::string, int> semseg_name_to_id_map;
    if (!semseg_color_map_path.empty()) {
      constexpr uint32_t skip_header_lines = 1;
      // load semantic segmentation label information (label, label name, r, g, b)
      load_segmentation_colormap(
        semseg_color_map_path, semseg_color_map_, semseg_name_to_id_map, skip_header_lines);
    }

    roi_id_to_semseg_id_map_.assign(roi_id_to_name_map_.size(), unmapped_class_id_);
    if (!roi_to_semseg_remap_path.empty()) {
      std::unordered_map<std::string, int> roi_name_to_semseg_id_remap;
      constexpr uint32_t skip_header_lines = 1;
      // load remapping of ROI to semantic segmentation label
      // e.g. PEDESTRIAN -> 6 (PEDESTRIAN)
      load_label_id_remap_file(
        roi_to_semseg_remap_path, roi_name_to_semseg_id_remap, skip_header_lines);

      // map original YOLOX ID directly to semantic segmentation ID
      for (size_t i = 0; i < roi_id_to_name_map_.size(); ++i) {
        const std::string & original_name = roi_id_to_name_map_[i];

        if (roi_name_to_semseg_id_remap.count(original_name) > 0) {
          roi_id_to_semseg_id_map_[i] = roi_name_to_semseg_id_remap.at(original_name);
        } else {
          // if there is no label name in the original YOLOX class, we will consider as an error
          // since it might using the wrong model
          std::stringstream error_msg;
          error_msg << "ROI label " << original_name << " not found in remap file.";
          throw std::runtime_error{error_msg.str()};
        }
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Label initialization failed: %s", e.what());
    throw;
  }
}

int TrtYoloXNode::mapRoiLabel2SegLabel(const int32_t roi_label_index)
{
  if (roi_overlay_semseg_labels_.isOverlay(static_cast<uint8_t>(roi_label_index))) {
    return roi_id_to_semseg_id_map_[roi_label_index];
  }
  return -1;
}

void TrtYoloXNode::overlapSegmentByRoi(
  const tensorrt_yolox::Object & roi_object, cv::Mat & mask, const int orig_width,
  const int orig_height)
{
  if (roi_object.score < overlap_roi_score_threshold_) return;
  int seg_class_index = mapRoiLabel2SegLabel(roi_object.type);
  if (seg_class_index < 0) return;

  const float scale_x = static_cast<float>(mask.cols) / static_cast<float>(orig_width);
  const float scale_y = static_cast<float>(mask.rows) / static_cast<float>(orig_height);
  const int roi_width = static_cast<int>(roi_object.width * scale_x);
  const int roi_height = static_cast<int>(roi_object.height * scale_y);
  const int roi_x_offset = static_cast<int>(roi_object.x_offset * scale_x);
  const int roi_y_offset = static_cast<int>(roi_object.y_offset * scale_y);

  cv::Mat replace_roi(
    cv::Size(roi_width, roi_height), mask.type(), static_cast<uint8_t>(seg_class_index));
  replace_roi.copyTo(mask.colRange(roi_x_offset, roi_x_offset + roi_width)
                       .rowRange(roi_y_offset, roi_y_offset + roi_height));
}

/**
 * @brief get colorized masks from index using specific colormap
 * @param[out] cmask colorized mask
 * @param[in] index multitask index
 * @param[in] colormap colormap for masks
 */
void TrtYoloXNode::getColorizedMask(const cv::Mat & mask, cv::Mat & cmask)
{
  int width = mask.cols;
  int height = mask.rows;
  if ((cmask.cols != mask.cols) || (cmask.rows != mask.rows)) {
    throw std::runtime_error("input and output image have difference size.");
  }

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      unsigned char id = mask.at<unsigned char>(y, x);
      cmask.at<cv::Vec3b>(y, x)[0] = semseg_color_map_[id].color[2];
      cmask.at<cv::Vec3b>(y, x)[1] = semseg_color_map_[id].color[1];
      cmask.at<cv::Vec3b>(y, x)[2] = semseg_color_map_[id].color[0];
    }
  }
}

}  // namespace autoware::tensorrt_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_yolox::TrtYoloXNode)
