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

#include <NvInfer.h>
#include <NvOnnxParser.h>

#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::camera_streampetr
{

using Label = autoware_perception_msgs::msg::ObjectClassification;

std::uint8_t getSemanticType(const std::string & class_name)
{
  if (class_name == "CAR") {
    return Label::CAR;
  } else if (class_name == "TRUCK") {
    return Label::TRUCK;
  } else if (class_name == "BUS") {
    return Label::BUS;
  } else if (class_name == "TRAILER") {
    return Label::TRAILER;
  } else if (class_name == "MOTORCYCLE") {
    return Label::MOTORCYCLE;
  } else if (class_name == "BICYCLE") {
    return Label::BICYCLE;
  } else if (class_name == "PEDESTRIAN") {
    return Label::PEDESTRIAN;
  } else {  // CONSTRUCTION_VEHICLE, BARRIER, TRAFFIC_CONE
    return Label::UNKNOWN;
  }
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
  classification.label = getSemanticType(class_names_[bbox.label]);
  object.classification.push_back(classification);
  return object;
}

StreamPetrNetwork::StreamPetrNetwork(
  const std::string & engine_backbone_path, const std::string & engine_head_path,
  const std::string & engine_position_embedding_path, const bool use_temporal,
  const double search_distance_2d, const double circle_nms_dist_threshold,
  const double iou_threshold, const double confidence_threshold,
  const std::vector<std::string> class_names, const int32_t num_proposals,
  const std::vector<double> yaw_norm_thresholds, const std::vector<float> detection_range)
: use_temporal_(use_temporal),
  confidence_threshold_(confidence_threshold),
  class_names_(class_names)
{
  // Initialize TensorRT runtime
  runtime_ = std::unique_ptr<IRuntime>{nvinfer1::createInferRuntime(gLogger)};
  backbone_ = std::make_unique<SubNetwork>(engine_backbone_path, runtime_.get());
  pts_head_ = std::make_unique<SubNetwork>(engine_head_path, runtime_.get());
  pos_embed_ = std::make_unique<SubNetwork>(engine_position_embedding_path, runtime_.get());

  cudaStreamCreate(&stream_);
  backbone_->EnableCudaGraph(stream_);
  pts_head_->EnableCudaGraph(stream_);
  pos_embed_->EnableCudaGraph(stream_);

  pts_head_->bindings["pre_memory_embedding"]->initialize_to_zeros(stream_);
  pts_head_->bindings["pre_memory_reference_point"]->initialize_to_zeros(stream_);
  pts_head_->bindings["pre_memory_egopose"]->initialize_to_zeros(stream_);
  pts_head_->bindings["pre_memory_velo"]->initialize_to_zeros(stream_);
  pts_head_->bindings["pre_memory_timestamp"]->initialize_to_zeros(stream_);

  mem_.mem_stream = stream_;
  mem_.pre_buf = static_cast<float *>(pts_head_->bindings["pre_memory_timestamp"]->ptr);
  mem_.post_buf = static_cast<float *>(pts_head_->bindings["post_memory_timestamp"]->ptr);

  // events for measurement
  dur_backbone_ = std::make_unique<Duration>("backbone");
  dur_ptshead_ = std::make_unique<Duration>("ptshead");
  dur_pos_embed_ = std::make_unique<Duration>("pos_embed");
  dur_postprocess_ = std::make_unique<Duration>("postprocess");

  if (iou_threshold > 0.0) {
    NMSParams p;
    p.search_distance_2d_ = search_distance_2d;
    p.iou_threshold_ = iou_threshold;
    iou_bev_nms_.setParameters(p);
    use_iou_bev_nms_ = true;
  }

  postprocess_cuda_ = std::make_unique<PostprocessCuda>(
    PostProcessingConfig(
      class_names.size(), circle_nms_dist_threshold, confidence_threshold, yaw_norm_thresholds,
      num_proposals, detection_range),
    stream_);
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
  const std::shared_ptr<Tensor> imgs, const std::vector<float> & ego_pose,
  const std::vector<float> & ego_pose_inv, const std::vector<float> & img_metas_pad,
  const std::vector<float> & intrinsics, const std::vector<float> & img2lidar, const float stamp,
  std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects,
  std::vector<float> & forward_time_ms)
{
  if (!is_inference_initialized_) {
    pos_embed_->bindings["img_metas_pad"]->load_from_vector(img_metas_pad);
    pos_embed_->bindings["intrinsics"]->load_from_vector(intrinsics);
    pos_embed_->bindings["img2lidar"]->load_from_vector(img2lidar);

    dur_pos_embed_->MarkBegin(stream_);
    pos_embed_->Enqueue(stream_);
    dur_pos_embed_->MarkEnd(stream_);

    cudaMemcpyAsync(
      pts_head_->bindings["pos_embed"]->ptr, pos_embed_->bindings["pos_embed"]->ptr,
      pos_embed_->bindings["pos_embed"]->nbytes(), cudaMemcpyDeviceToDevice, stream_);
    cudaMemcpyAsync(
      pts_head_->bindings["cone"]->ptr, pos_embed_->bindings["cone"]->ptr,
      pos_embed_->bindings["cone"]->nbytes(), cudaMemcpyDeviceToDevice, stream_);
    is_inference_initialized_ = true;
  }

  cudaMemcpyAsync(
    backbone_->bindings["img"]->ptr, imgs->ptr, imgs->nbytes(), cudaMemcpyDeviceToDevice, stream_);

  {  // feature extraction execution
    dur_backbone_->MarkBegin(stream_);
    // inference
    backbone_->Enqueue(stream_);

    cudaMemcpyAsync(
      pts_head_->bindings["x"]->ptr, backbone_->bindings["img_feats"]->ptr,
      backbone_->bindings["img_feats"]->nbytes(), cudaMemcpyDeviceToDevice, stream_);
    dur_backbone_->MarkEnd(stream_);
  }

  pts_head_->bindings["data_ego_pose"]->load_from_vector(ego_pose);
  pts_head_->bindings["data_ego_pose_inv"]->load_from_vector(ego_pose_inv);

  {
    dur_ptshead_->MarkBegin(stream_);

    mem_.StepPre(stamp);
    // inference
    pts_head_->Enqueue(stream_);
    mem_.StepPost(stamp);

    if (use_temporal_) {
      pts_head_->bindings["pre_memory_embedding"]->mov(
        pts_head_->bindings["post_memory_embedding"], stream_);
      pts_head_->bindings["pre_memory_reference_point"]->mov(
        pts_head_->bindings["post_memory_reference_point"], stream_);
      pts_head_->bindings["pre_memory_egopose"]->mov(
        pts_head_->bindings["post_memory_egopose"], stream_);
      pts_head_->bindings["pre_memory_velo"]->mov(pts_head_->bindings["post_memory_velo"], stream_);
    } else {
      wipe_memory();
    }
    dur_ptshead_->MarkEnd(stream_);
  }

  cudaStreamSynchronize(stream_);

  std::vector<Box3D> det_boxes3d;
  dur_postprocess_->MarkBegin(stream_);
  postprocess_cuda_->generateDetectedBoxes3D_launch(
    static_cast<const float *>(pts_head_->bindings["all_cls_scores"]->ptr),
    static_cast<const float *>(pts_head_->bindings["all_bbox_preds"]->ptr), det_boxes3d, stream_);

  std::vector<autoware_perception_msgs::msg::DetectedObject> raw_objects;

  for (size_t i = 0; i < det_boxes3d.size(); ++i) {
    raw_objects.push_back(this->bbox_to_ros_msg(det_boxes3d[i]));
  }

  if (use_iou_bev_nms_)
    iou_bev_nms_.apply(raw_objects, output_objects);
  else
    output_objects = std::move(raw_objects);
  dur_postprocess_->MarkEnd(stream_);

  forward_time_ms.push_back(dur_backbone_->Elapsed());
  forward_time_ms.push_back(dur_ptshead_->Elapsed());
  forward_time_ms.push_back(dur_pos_embed_->Elapsed());
  forward_time_ms.push_back(dur_postprocess_->Elapsed());
}

}  // namespace autoware::camera_streampetr
