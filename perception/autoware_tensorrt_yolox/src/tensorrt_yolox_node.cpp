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

#include "autoware/tensorrt_yolox/parameter_loader.hpp"

#include <memory>
#include <string>

// cspell: ignore semseg

namespace autoware::tensorrt_yolox
{

namespace
{

struct DetectorParameters
{
  TrtYoloXDetectorConfig config;
  std::string label_path;
  std::string semseg_color_map_path;
  std::string roi_remap_path;
  std::string roi_to_semantic_segmentation_remap_path;
  // kept so the ctor can construct a `ParameterReader` in the same mode when it
  // reads follow-up parameters (e.g. `build_only`).
  bool use_manifest{};
};

// Reads every parameter this node needs, in whichever mode is active.
//
// Manifest mode (`config_dir` set): pulls values from the yaml files listed
// in `<config_dir>/parameters.manifest.yaml`, and validates that path values are
// relative to `config_dir` or `model_dir`.
//
// Classic mode (`config_dir` empty): declares each parameter individually so
// the launcher's `<param>` overrides drive the values. Kept during the
// migration window so old launch files continue to work.
DetectorParameters read_detector_parameters(rclcpp::Node & node)
{
  const auto config_dir = node.declare_parameter<std::string>("config_dir", "");
  DetectorParameters parameters;
  parameters.use_manifest = !config_dir.empty();
  std::string model_dir;
  if (parameters.use_manifest) {
    model_dir = node.declare_parameter<std::string>("model_dir");
    load_parameters_from_manifest(node, config_dir);
    load_parameters_from_manifest(node, model_dir);
  } else {
    RCLCPP_WARN(
      node.get_logger(),
      "Running in classic parameter mode. Set `config_dir` (and `model_dir`) "
      "to switch to the manifest-driven layout.");
  }

  ParameterReader p(node, parameters.use_manifest);
  auto & config = parameters.config;

  config.precision = p.required("precision").as_string();
  config.score_threshold = static_cast<float>(p.required("score_threshold").as_double());
  config.nms_threshold = static_cast<float>(p.required("nms_threshold").as_double());
  config.calibration_algorithm = p.required("calibration_algorithm").as_string();
  config.dla_core_id = static_cast<int>(p.required("dla_core_id").as_int());
  config.quantize_first_layer = p.required("quantize_first_layer").as_bool();
  config.quantize_last_layer = p.required("quantize_last_layer").as_bool();
  config.profile_per_layer = p.required("profile_per_layer").as_bool();
  config.clip_value = p.required("clip_value").as_double();
  config.calibration_image_list_path =
    p.optional("calibration_image_list_path", rclcpp::ParameterValue(std::string{})).as_string();
  config.gpu_id = static_cast<uint8_t>(p.required("gpu_id").as_int());
  config.is_roi_overlap_semseg = p.required("is_roi_overlap_segmentation").as_bool();
  config.is_publish_color_mask = p.required("is_publish_color_mask").as_bool();
  config.overlap_roi_score_threshold =
    static_cast<float>(p.required("overlap_roi_score_threshold").as_double());

  if (parameters.use_manifest) {
    // empty optional path will disable this feature; skip the join so `""` propagates.
    const auto join_or_empty = [](const std::filesystem::path & root, const std::string & rel) {
      return rel.empty() ? std::string{} : join_relative(root, rel);
    };
    config.model_path = join_relative(model_dir, p.required("model_file").as_string());
    parameters.label_path = join_relative(model_dir, p.required("label_file").as_string());
    parameters.semseg_color_map_path = join_or_empty(
      model_dir, p.optional("colormap_file", rclcpp::ParameterValue(std::string{})).as_string());
    parameters.roi_remap_path = join_or_empty(
      model_dir, p.optional("roi_remap_file", rclcpp::ParameterValue(std::string{})).as_string());
    parameters.roi_to_semantic_segmentation_remap_path = join_or_empty(
      model_dir,
      p.optional("roi_to_semantic_segmentation_remap_file", rclcpp::ParameterValue(std::string{}))
        .as_string());
  } else {
    config.model_path = node.declare_parameter<std::string>("model_path");
    parameters.label_path = node.declare_parameter<std::string>("label_path");
    parameters.semseg_color_map_path =
      node.declare_parameter<std::string>("semantic_segmentation_color_map_path", "");
    // an empty remap path disables label remapping
    parameters.roi_remap_path = node.declare_parameter<std::string>("roi_remap_path");
    parameters.roi_to_semantic_segmentation_remap_path =
      node.declare_parameter<std::string>("roi_to_semantic_segmentation_remap_path", "");
  }

  return parameters;
}

}  // namespace

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

  auto parameters = read_detector_parameters(*this);
  auto & config = parameters.config;

  // read the label / remap / color-map files into structured data outside the detector
  try {
    config.roi_labels = load_label_maps(
      parameters.label_path, parameters.roi_remap_path,
      parameters.roi_to_semantic_segmentation_remap_path);
    config.semseg_color_map = load_segmentation_colormap(parameters.semseg_color_map_path);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load label files: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  try {
    detector_ = std::make_unique<TrtYoloXDetector>(config);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "GPU %d is selected for the inference!", config.gpu_id);

  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&TrtYoloXNode::onConnect, this));

  objects_pub_ = this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
    "~/out/objects", 1);
  mask_pub_ = image_transport::create_publisher(this, "~/out/mask");
  color_mask_pub_ = image_transport::create_publisher(this, "~/out/color_mask");
  image_pub_ = image_transport::create_publisher(this, "~/out/image");

  ParameterReader p(*this, parameters.use_manifest);
  if (p.optional("build_only", rclcpp::ParameterValue(false)).as_bool()) {
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

  const auto result = detector_->detect(*msg);
  if (!result) {
    RCLCPP_ERROR(this->get_logger(), "detection failed: %s", result.error().c_str());
    return;
  }

  if (result->mask) {
    mask_pub_.publish(*result->mask);
  }

  image_pub_.publish(result->image);

  objects_pub_->publish(result->objects);

  if (debug_publisher_) {
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - result->objects.header.stamp).nanoseconds()))
        .count();
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  if (result->color_mask) {
    color_mask_pub_.publish(*result->color_mask);
  }
}

}  // namespace autoware::tensorrt_yolox

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::tensorrt_yolox::TrtYoloXNode)
