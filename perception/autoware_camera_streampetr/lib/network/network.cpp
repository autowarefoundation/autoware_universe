// Copyright 2025 TIER IV
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

#include "autoware/camera_streampetr/network/network.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <algorithm>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

using autoware::tensorrt_common::TrtCommonConfig;
using Label = autoware_perception_msgs::msg::ObjectClassification;

std::uint8_t getSemanticType(const std::string & class_name)
{
  static const std::unordered_map<std::string, std::uint8_t> class_mapping = {
    {"CAR", Label::CAR},
    {"TRUCK", Label::TRUCK},
    {"BUS", Label::BUS},
    {"TRAILER", Label::TRAILER},
    {"MOTORCYCLE", Label::MOTORCYCLE},
    {"BICYCLE", Label::BICYCLE},
    {"PEDESTRIAN", Label::PEDESTRIAN}};

  const auto it = class_mapping.find(class_name);
  return (it != class_mapping.end()) ? it->second : Label::UNKNOWN;
}

autoware_perception_msgs::msg::DetectedObject StreamPetrNetwork::bbox_to_ros_msg(const Box3D & bbox)
{
  // cx, cy, cz, w, l, h, rot, vx, vy
  autoware_perception_msgs::msg::DetectedObject object;
  object.kinematics.pose_with_covariance.pose.position.x = bbox.x;
  object.kinematics.pose_with_covariance.pose.position.y = bbox.y;
  object.kinematics.pose_with_covariance.pose.position.z = bbox.z;
  object.shape.dimensions.x = bbox.width;
  object.shape.dimensions.y = bbox.length;
  object.shape.dimensions.z = bbox.height;
  const double yaw = bbox.yaw;
  object.kinematics.pose_with_covariance.pose.orientation.w = cos(yaw * 0.5);
  object.kinematics.pose_with_covariance.pose.orientation.x = 0;
  object.kinematics.pose_with_covariance.pose.orientation.y = 0;
  object.kinematics.pose_with_covariance.pose.orientation.z = sin(yaw * 0.5);

  object.existence_probability = bbox.score;
  object.kinematics.has_position_covariance = false;
  object.kinematics.has_twist = false;
  object.shape.type = 0;

  autoware_perception_msgs::msg::ObjectClassification classification;
  classification.probability = 1.0f;
  classification.label = getSemanticType(config_.class_names[bbox.label]);
  object.classification.push_back(classification);
  return object;
}

namespace
{

bool shouldSetLayerToFP32(nvinfer1::ILayer * layer, const std::string & layer_name_lower)
{
  // Check layer name for sigmoid/softmax keywords
  if (
    layer_name_lower.find("sigmoid") != std::string::npos ||
    layer_name_lower.find("softmax") != std::string::npos) {
    return true;
  }

  // Check layer type for activation or softmax layers
  if (layer->getType() == nvinfer1::LayerType::kACTIVATION) {
    auto * activation_layer = static_cast<nvinfer1::IActivationLayer *>(layer);
    return activation_layer->getActivationType() == nvinfer1::ActivationType::kSIGMOID;
  }

  return layer->getType() == nvinfer1::LayerType::kSOFTMAX;
}

}  // anonymous namespace

void setSigmoidAndSoftmaxLayersToFP32(std::shared_ptr<nvinfer1::INetworkDefinition> network)
{
  for (int i = 0; i < network->getNbLayers(); ++i) {
    auto * layer = network->getLayer(i);
    std::string layer_name = layer->getName();

    // Convert to lowercase for case-insensitive comparison
    std::transform(layer_name.begin(), layer_name.end(), layer_name.begin(), ::tolower);

    if (shouldSetLayerToFP32(layer, layer_name)) {
      layer->setPrecision(nvinfer1::DataType::kFLOAT);
    }
  }
}

StreamPetrNetwork::StreamPetrNetwork(const NetworkConfig & config) : config_(config)
{
  cudaStreamCreate(&stream_);

  initializeNetworks();
  setupEngines();
  setupBindings();
  initializeMemoryAndProfiling();
  configureNMSIfNeeded();
}

void StreamPetrNetwork::initializeNetworks()
{
  // Initialize TrtCommon configurations
  auto backbone_config = tensorrt_common::TrtCommonConfig(
    config_.onnx_backbone_path, config_.trt_precision, config_.engine_backbone_path,
    config_.workspace_size);
  auto pts_head_config = tensorrt_common::TrtCommonConfig(
    config_.onnx_head_path, config_.trt_precision, config_.engine_head_path,
    config_.workspace_size);
  auto pos_embed_config = tensorrt_common::TrtCommonConfig(
    config_.onnx_position_embedding_path, config_.trt_precision,
    config_.engine_position_embedding_path, config_.workspace_size);

  // Initialize TrtCommon instances
  backbone_ = std::make_unique<SubNetwork>(backbone_config, profiler_);
  pts_head_ = std::make_unique<SubNetwork>(pts_head_config, profiler_);
  pos_embed_ = std::make_unique<SubNetwork>(pos_embed_config, profiler_);

  // Apply FP32 precision for stability if using fp16
  if (config_.trt_precision == "fp16") {
    auto logger = rclcpp::get_logger(config_.logger_name.c_str());
    RCLCPP_INFO(logger, "Setting sigmoid and softmax layers to FP32 precision for stability");
    setSigmoidAndSoftmaxLayersToFP32(pts_head_->getNetwork());
  }
}

void StreamPetrNetwork::setupEngines()
{
  if (!backbone_->setup()) {
    throw std::runtime_error("Failed to setup backbone TRT engine.");
  }
  if (!pts_head_->setup()) {
    throw std::runtime_error("Failed to setup pts_head TRT engine.");
  }
  if (!pos_embed_->setup()) {
    throw std::runtime_error("Failed to setup pos_embed TRT engine.");
  }
}

void StreamPetrNetwork::setupBindings()
{
  auto logger = rclcpp::get_logger(config_.logger_name.c_str());

  if (!backbone_->setBindings(logger)) {
    throw std::runtime_error("Failed to setup backbone TRT bindings.");
  }
  if (!pts_head_->setBindings(logger)) {
    throw std::runtime_error("Failed to setup pts_head TRT bindings.");
  }
  if (!pos_embed_->setBindings(logger)) {
    throw std::runtime_error("Failed to setup pos_embed TRT bindings.");
  }
}

void StreamPetrNetwork::initializeMemoryAndProfiling()
{
  mem_.Init(stream_, config_.pre_memory_length, config_.post_memory_length);
  mem_.pre_buf = static_cast<float *>(pts_head_->bindings["pre_memory_timestamp"]->ptr);
  mem_.post_buf = static_cast<float *>(pts_head_->bindings["post_memory_timestamp"]->ptr);

  // events for measurement - pass profiler to Duration objects
  dur_backbone_ = std::make_unique<Duration>("backbone", profiler_);
  dur_ptshead_ = std::make_unique<Duration>("ptshead", profiler_);
  dur_pos_embed_ = std::make_unique<Duration>("pos_embed", profiler_);
  dur_postprocess_ = std::make_unique<Duration>("postprocess", profiler_);

  postprocess_cuda_ = std::make_unique<PostprocessCuda>(
    PostProcessingConfig(
      config_.class_names.size(), config_.circle_nms_dist_threshold, config_.confidence_thresholds,
      config_.yaw_norm_thresholds, config_.num_proposals, config_.detection_range),
    stream_);
}

void StreamPetrNetwork::configureNMSIfNeeded()
{
  if (config_.iou_threshold > 0.0) {
    NMSParams p;
    p.search_distance_2d_ = config_.search_distance_2d;
    p.iou_threshold_ = config_.iou_threshold;
    iou_bev_nms_.setParameters(p);
  }
}

void StreamPetrNetwork::wipe_memory()
{
  if (is_inference_initialized_) {
    // Reset the memory buffers to zeros
    pts_head_->bindings["pre_memory_embedding"]->initialize_to_zeros(stream_);
    pts_head_->bindings["pre_memory_reference_point"]->initialize_to_zeros(stream_);
    pts_head_->bindings["pre_memory_egopose"]->initialize_to_zeros(stream_);
    pts_head_->bindings["pre_memory_velo"]->initialize_to_zeros(stream_);
  }
}

void StreamPetrNetwork::inference_detector(
  const InferenceInputs & inputs,
  std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects,
  std::vector<float> & forward_time_ms)
{
  if (!is_inference_initialized_) {
    initializePositionEmbedding(inputs);
    is_inference_initialized_ = true;
  }

  executeBackbone(inputs);
  executePtsHead(inputs);

  cudaStreamSynchronize(stream_);

  executePostprocessing(output_objects);

  forward_time_ms.push_back(dur_backbone_->Elapsed());
  forward_time_ms.push_back(dur_ptshead_->Elapsed());
  forward_time_ms.push_back(dur_pos_embed_->Elapsed());
  forward_time_ms.push_back(dur_postprocess_->Elapsed());
}

void StreamPetrNetwork::initializePositionEmbedding(const InferenceInputs & inputs)
{
  pos_embed_->bindings["img_metas_pad"]->load_from_vector(inputs.img_metas_pad);
  pos_embed_->bindings["intrinsics"]->load_from_vector(inputs.intrinsics);
  pos_embed_->bindings["img2lidar"]->load_from_vector(inputs.img2lidar);

  dur_pos_embed_->MarkBegin(stream_);
  pos_embed_->enqueueV3(stream_);
  dur_pos_embed_->MarkEnd(stream_);

  cudaMemcpyAsync(
    pts_head_->bindings["pos_embed"]->ptr, pos_embed_->bindings["pos_embed"]->ptr,
    pos_embed_->bindings["pos_embed"]->nbytes(), cudaMemcpyDeviceToDevice, stream_);
  cudaMemcpyAsync(
    pts_head_->bindings["cone"]->ptr, pos_embed_->bindings["cone"]->ptr,
    pos_embed_->bindings["cone"]->nbytes(), cudaMemcpyDeviceToDevice, stream_);
}

void StreamPetrNetwork::executeBackbone(const InferenceInputs & inputs)
{
  cudaMemcpyAsync(
    backbone_->bindings["img"]->ptr, inputs.imgs->ptr, inputs.imgs->nbytes(),
    cudaMemcpyDeviceToDevice, stream_);

  dur_backbone_->MarkBegin(stream_);
  backbone_->enqueueV3(stream_);

  cudaMemcpyAsync(
    pts_head_->bindings["x"]->ptr, backbone_->bindings["img_feats"]->ptr,
    backbone_->bindings["img_feats"]->nbytes(), cudaMemcpyDeviceToDevice, stream_);
  dur_backbone_->MarkEnd(stream_);
}

void StreamPetrNetwork::executePtsHead(const InferenceInputs & inputs)
{
  pts_head_->bindings["data_ego_pose"]->load_from_vector(inputs.ego_pose);
  pts_head_->bindings["data_ego_pose_inv"]->load_from_vector(inputs.ego_pose_inv);

  dur_ptshead_->MarkBegin(stream_);

  mem_.StepPre(inputs.stamp);
  pts_head_->enqueueV3(stream_);
  mem_.StepPost(inputs.stamp);

  if (config_.use_temporal) {
    pts_head_->bindings["pre_memory_embedding"]->copy(
      pts_head_->bindings["post_memory_embedding"], stream_);
    pts_head_->bindings["pre_memory_reference_point"]->copy(
      pts_head_->bindings["post_memory_reference_point"], stream_);
    pts_head_->bindings["pre_memory_egopose"]->copy(
      pts_head_->bindings["post_memory_egopose"], stream_);
    pts_head_->bindings["pre_memory_velo"]->copy(pts_head_->bindings["post_memory_velo"], stream_);
  } else {
    wipe_memory();
  }
  dur_ptshead_->MarkEnd(stream_);
}

void StreamPetrNetwork::executePostprocessing(
  std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects)
{
  std::vector<Box3D> det_boxes3d;
  dur_postprocess_->MarkBegin(stream_);
  postprocess_cuda_->generateDetectedBoxes3D_launch(
    static_cast<const float *>(pts_head_->bindings["all_cls_scores"]->ptr),
    static_cast<const float *>(pts_head_->bindings["all_bbox_preds"]->ptr), det_boxes3d, stream_);
  cudaStreamSynchronize(stream_);

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;
  for (size_t i = 0; i < det_boxes3d.size(); ++i) {
    raw_objects.push_back(this->bbox_to_ros_msg(det_boxes3d[i]));
  }

  if (config_.iou_threshold > 0.0) {
    iou_bev_nms_.apply(raw_objects, output_objects);
  } else {
    output_objects = std::move(raw_objects);
  }
  dur_postprocess_->MarkEnd(stream_);
}

StreamPetrNetwork::~StreamPetrNetwork()
{
  if (stream_) {
    cudaStreamDestroy(stream_);
  }
}

}  // namespace autoware::camera_streampetr
