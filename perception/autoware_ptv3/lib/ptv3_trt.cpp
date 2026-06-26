// Copyright 2025 TIER IV, Inc.
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

#include "autoware/ptv3/ptv3_trt.hpp"

#include "autoware/ptv3/execution_context.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::ptv3
{
PTv3TRT::PTv3TRT(
  const tensorrt_common::TrtCommonConfig & backbone_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & seg3d_head_trt_config,
  const std::optional<tensorrt_common::TrtCommonConfig> & det3d_head_trt_config,
  const PTv3Config & config)
: config_(config)
{
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing/inner");

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));

  backbone_preprocessor_ =
    std::make_unique<BackbonePreprocessor>(config_.backbone_preprocess_config_);
  backbone_engine_ = std::make_unique<BackboneEngine>(
    backbone_trt_config, config_.backbone_config_, backbone_preprocessor_->gridCoord(),
    backbone_preprocessor_->features(), backbone_preprocessor_->serializedCode());
  backbone_engine_->bindSerializedPoolingAddresses(
    backbone_preprocessor_->serializedPoolingStageViews());
  const auto backbone_output = backbone_engine_->output();
  if (config_.use_seg3d_head_) {
    if (!seg3d_head_trt_config.has_value()) {
      throw std::runtime_error("seg3d_head_trt_config is required when segmentation3d.use_head.");
    }
    semseg_module_ = std::make_unique<SemsegModule>(
      *seg3d_head_trt_config, config_.semseg_config_, backbone_output.point_feat,
      backbone_preprocessor_->features(), backbone_preprocessor_->compactPoints());
  }
  if (config_.use_det3d_head_) {
    if (!det3d_head_trt_config.has_value()) {
      throw std::runtime_error("det3d_head_trt_config is required when detection3d.use_head.");
    }
    detection3d_module_ = std::make_unique<Detection3DModule>(
      *det3d_head_trt_config, config_.detection3d_config_, backbone_output.point_feat,
      backbone_output.point_grid_coord);
  }
}

void PTv3TRT::setPublishSegmentedPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  semseg_module_->setPublishSegmentedPointcloud(std::move(func));
}

void PTv3TRT::setPublishVisualizationPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  semseg_module_->setPublishVisualizationPointcloud(std::move(func));
}

void PTv3TRT::setPublishFilteredPointcloud(
  std::function<void(std::unique_ptr<const cuda_blackboard::CudaPointCloud2>)> func)
{
  semseg_module_->setPublishFilteredPointcloud(std::move(func));
}

PTv3TRT::~PTv3TRT()
{
  if (stream_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    CHECK_CUDA_ERROR(cudaStreamDestroy(stream_));
  }
}

bool PTv3TRT::infer(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  bool should_publish_segmented_pointcloud, bool should_publish_visualization_pointcloud,
  bool should_publish_filtered_pointcloud, bool should_detect_objects,
  std::optional<std::vector<Box3D>> & det_boxes3d,
  std::unordered_map<std::string, double> & proc_timing)
{
  det_boxes3d.reset();

  const bool should_run_seg3d = semseg_module_ && (should_publish_segmented_pointcloud ||
                                                   should_publish_visualization_pointcloud ||
                                                   should_publish_filtered_pointcloud);
  const bool should_run_det3d = detection3d_module_ && should_detect_objects;
  const PTv3ExecutionContext context{stream_};

  stop_watch_ptr_->toc("processing/inner", true);
  backbone_preprocessor_->prepareCloudFormat(*msg_ptr);
  if (semseg_module_) {
    semseg_module_->preparePreprocess(
      *msg_ptr, context, backbone_preprocessor_->inputFormat(), should_run_seg3d);
  }
  if (detection3d_module_) {
    detection3d_module_->preparePreprocess();
  }
  if (!backbone_preprocessor_->run(*msg_ptr, context)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Pre-process failed. Skipping inference.");
    return false;
  }
  if (semseg_module_) {
    semseg_module_->preprocessSourceReconstruction(
      *msg_ptr, context, backbone_preprocessor_->cudaPreprocessor(),
      backbone_preprocessor_->numCroppedPoints(), should_run_seg3d);
  }
  proc_timing.emplace(
    "debug/processing_time/preprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!prepareInferenceShapes(should_run_seg3d, should_run_det3d)) {
    return false;
  }

  if (!backbone_engine_->enqueue(context)) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Backbone inference failed.");
    return false;
  }

  bool seg_ok = !should_run_seg3d;
  bool det_ok = !should_run_det3d;
  bool seg_post_ok = !should_run_seg3d;
  bool det_post_ok = !should_run_det3d;

  if (should_run_seg3d) {
    seg_ok = semseg_module_->enqueue(context);
    if (!seg_ok) {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Seg head inference failed.");
    }
  }
  if (should_run_det3d) {
    det_ok = detection3d_module_->enqueue(context);
    if (!det_ok) {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Det head inference failed.");
    }
  }

  CHECK_CUDA_ERROR(cudaStreamSynchronize(context.stream()));
  proc_timing.emplace(
    "debug/processing_time/inference_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (seg_ok && should_run_seg3d) {
    if (semseg_module_->postProcess(
          msg_ptr->header, context, backbone_preprocessor_->cudaPreprocessor(),
          backbone_preprocessor_->inputFormat(), should_publish_segmented_pointcloud,
          should_publish_visualization_pointcloud, should_publish_filtered_pointcloud,
          backbone_preprocessor_->numVoxels())) {
      seg_post_ok = true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Seg post-process failed.");
    }
  }

  if (det_ok && should_run_det3d) {
    std::vector<Box3D> detected_boxes;
    if (detection3d_module_->postProcess(context, detected_boxes)) {
      det_boxes3d = std::move(detected_boxes);
      det_post_ok = true;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Det post-process failed.");
    }
  }

  proc_timing.emplace(
    "debug/processing_time/postprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  return (should_run_seg3d && seg_ok && seg_post_ok) || (should_run_det3d && det_ok && det_post_ok);
}

bool PTv3TRT::prepareInferenceShapes(const bool should_run_seg3d, const bool should_run_det3d)
{
  if (!backbone_engine_->setInputShapes(
        backbone_preprocessor_->numVoxels(),
        backbone_preprocessor_->serializedPoolingNumVoxels())) {
    RCLCPP_ERROR(rclcpp::get_logger("ptv3"), "Failed to set serialized pooling input shapes.");
    return false;
  }

  if (should_run_seg3d) {
    semseg_module_->setInputShape(backbone_preprocessor_->numVoxels());
  }
  if (should_run_det3d) {
    detection3d_module_->setInputShapes(backbone_preprocessor_->numVoxels());
  }

  return true;
}

}  //  namespace autoware::ptv3
