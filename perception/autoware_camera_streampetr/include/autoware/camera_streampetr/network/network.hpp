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

#ifndef AUTOWARE__CAMERA_STREAMPETR__NETWORK__NETWORK_HPP_
#define AUTOWARE__CAMERA_STREAMPETR__NETWORK__NETWORK_HPP_

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>

// From NVIDIA/DL4AGX
#include "autoware/camera_streampetr/network/memory.cuh"

#include <NvInferRuntime.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
// From NVIDIA/DL4AGX

#include "autoware/camera_streampetr/cuda_utils.hpp"
#include "autoware/camera_streampetr/postprocess/non_maximum_suppression.hpp"
#include "autoware/camera_streampetr/postprocess/postprocess_kernel.hpp"
#include "autoware/camera_streampetr/utils.hpp"

// TensorRT Common
#include "autoware/tensorrt_common/tensorrt_common.hpp"
#include "autoware/tensorrt_common/utils.hpp"

// CUDA utilities
#include <autoware/cuda_utils/cuda_utils.hpp>

namespace autoware::camera_streampetr
{

using cuda::Tensor;
using nvinfer1::DataType;
using nvinfer1::Dims;

// Use tensorrt_common components
using autoware::tensorrt_common::Logger;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;
using autoware::tensorrt_common::TrtCommonConfig;

class SubNetwork : public TrtCommon
{
public:
  std::unordered_map<std::string, std::shared_ptr<Tensor>> bindings;

  using TrtCommon::TrtCommon;
  virtual ~SubNetwork() = default;
  bool setBindings(const rclcpp::Logger & logger)
  {
    for (int n = 0; n < getNbIOTensors(); n++) {
      std::string name = getIOTensorName(n);
      Dims d = getTensorShape(name.c_str());
      auto dtype_opt = getTensorDataType(name.c_str());
      if (!dtype_opt.has_value()) {
        RCLCPP_WARN(logger, "Warning: Could not get data type for tensor: %s", name.c_str());
        return false;
      }
      DataType dtype = dtype_opt.value();
      bindings[name] = std::make_shared<Tensor>(name, d, dtype);
      bindings[name]->iomode = getTensorIOMode(name.c_str());

      std::stringstream ss;
      ss << *(bindings[name]);
      RCLCPP_INFO(logger, "%s", ss.str().c_str());

      setTensorAddress(name.c_str(), bindings[name]->ptr);
    }
    return true;
  }
};

class Duration
{
  // CUDA events for timing
  cudaEvent_t begin_event_, end_event_;
  std::string layer_name_;
  std::shared_ptr<Profiler> profiler_;

public:
  explicit Duration(const std::string & name, std::shared_ptr<Profiler> profiler = nullptr)
  : layer_name_(name), profiler_(profiler)
  {
    cudaEventCreate(&begin_event_);
    cudaEventCreate(&end_event_);
  }

  ~Duration()
  {
    cudaEventDestroy(begin_event_);
    cudaEventDestroy(end_event_);
  }

  void MarkBegin(cudaStream_t stream) { cudaEventRecord(begin_event_, stream); }

  void MarkEnd(cudaStream_t stream) { cudaEventRecord(end_event_, stream); }

  float Elapsed()
  {
    float elapsed_ms;
    cudaEventElapsedTime(&elapsed_ms, begin_event_, end_event_);

    // Report to profiler if available
    if (profiler_) {
      profiler_->reportLayerTime(layer_name_.c_str(), elapsed_ms);
    }

    return elapsed_ms;
  }
};  // class Duration

struct NetworkConfig
{
  // Logging
  std::string logger_name = "camera_streampetr";

  // Model parameters
  bool use_temporal = true;
  double search_distance_2d = 0.0;
  double circle_nms_dist_threshold = 0.0;
  double iou_threshold = 0.0;
  double confidence_threshold = 0.0;
  std::vector<std::string> class_names;
  int32_t num_proposals = 0;
  std::vector<double> yaw_norm_thresholds;
  std::vector<float> detection_range;
  int pre_memory_length = 0;
  int post_memory_length = 0;

  int image_height = 0;
  int image_width = 0;
  int image_num = 0;

  uint64_t workspace_size = 0;
  std::string trt_precision;

  // Engine paths
  std::string onnx_backbone_path;
  std::string onnx_head_path;
  std::string onnx_position_embedding_path;

  std::string engine_backbone_path = "";
  std::string engine_head_path = "";
  std::string engine_position_embedding_path = "";
};

struct InferenceInputs
{
  std::shared_ptr<Tensor> imgs;
  std::vector<float> ego_pose;
  std::vector<float> ego_pose_inv;
  std::vector<float> img_metas_pad;
  std::vector<float> intrinsics;
  std::vector<float> img2lidar;
  float stamp;
};

class StreamPetrNetwork
{
public:
  explicit StreamPetrNetwork(const NetworkConfig & config);

  ~StreamPetrNetwork();
  void inference_detector(
    const InferenceInputs & inputs,
    std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects,
    std::vector<float> & forward_time_ms);

  void wipe_memory();

private:
  autoware_perception_msgs::msg::DetectedObject bbox_to_ros_msg(const Box3D & bbox);

  // Helper methods for constructor
  void initializeNetworks();
  void setupEngines();
  void setupBindings();
  void initializeMemoryAndProfiling();
  void configureNMSIfNeeded();

  // Helper methods for inference_detector
  void initializePositionEmbedding(const InferenceInputs & inputs);
  void executeBackbone(const InferenceInputs & inputs);
  void executePtsHead(const InferenceInputs & inputs);
  void executePostprocessing(
    std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects);

  NetworkConfig config_;
  std::shared_ptr<Logger> logger_;
  std::shared_ptr<Profiler> profiler_;
  std::unique_ptr<SubNetwork> backbone_;
  std::unique_ptr<SubNetwork> pts_head_;
  std::unique_ptr<SubNetwork> pos_embed_;

  std::unique_ptr<Duration> dur_backbone_;
  std::unique_ptr<Duration> dur_ptshead_;
  std::unique_ptr<Duration> dur_pos_embed_;
  std::unique_ptr<Duration> dur_postprocess_;

  std::unique_ptr<PostprocessCuda> postprocess_cuda_;
  NonMaximumSuppression iou_bev_nms_;

  bool is_inference_initialized_ = false;
  Memory mem_;
  cudaStream_t stream_;
};

}  // namespace autoware::camera_streampetr

#endif  // AUTOWARE__CAMERA_STREAMPETR__NETWORK__NETWORK_HPP_
