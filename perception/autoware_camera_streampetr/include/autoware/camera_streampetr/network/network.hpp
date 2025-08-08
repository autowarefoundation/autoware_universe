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

namespace autoware::camera_streampetr
{

using cuda::Tensor;
using nvinfer1::DataType;
using nvinfer1::Dims;
using nvinfer1::ICudaEngine;
using nvinfer1::IExecutionContext;
using nvinfer1::ILogger;
using nvinfer1::IRuntime;

// Use tensorrt_common components
using autoware::tensorrt_common::Logger;
using autoware::tensorrt_common::Profiler;
using autoware::tensorrt_common::TrtCommon;

class SubNetwork
{
private:
  std::unique_ptr<TrtCommon> trt_common_;
  cudaStream_t stream_;

public:
  std::unordered_map<std::string, std::shared_ptr<Tensor>> bindings;
  bool use_cuda_graph = false;
  cudaGraph_t graph;
  cudaGraphExec_t graph_exec;

  SubNetwork(const std::string & engine_path, IRuntime * runtime, cudaStream_t stream)
  {
    stream_ = stream;

    // Load the engine using TrtCommon's runtime approach
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file) {
      throw std::runtime_error("Error opening engine file: " + engine_path);
    }
    engine_file.seekg(0, engine_file.end);
    int64_t fsize = engine_file.tellg();
    engine_file.seekg(0, engine_file.beg);

    // Read the engine file into a buffer
    std::vector<char> engineData(fsize);
    engine_file.read(engineData.data(), fsize);

    auto engine = runtime->deserializeCudaEngine(engineData.data(), fsize);
    if (!engine) {
      throw std::runtime_error("Failed to deserialize engine from: " + engine_path);
    }

    auto context = engine->createExecutionContext();
    if (!context) {
      throw std::runtime_error("Failed to create execution context");
    }

    int nb = engine->getNbIOTensors();
    for (int n = 0; n < nb; n++) {
      std::string name = engine->getIOTensorName(n);
      Dims d = engine->getTensorShape(name.c_str());
      DataType dtype = engine->getTensorDataType(name.c_str());
      bindings[name] = std::make_shared<Tensor>(name, d, dtype);
      bindings[name]->iomode = engine->getTensorIOMode(name.c_str());
      std::cout << *(bindings[name]) << std::endl;
      context->setTensorAddress(name.c_str(), bindings[name]->ptr);
    }

    // Store the raw pointers for direct access
    engine_ = engine;
    context_ = context;
  }

  void Enqueue()
  {
    if (this->use_cuda_graph) {
      cudaGraphLaunch(graph_exec, stream_);
    } else {
      context_->enqueueV3(stream_);
    }
  }

  ~SubNetwork()
  {
    if (context_) {
      delete context_;
    }
    if (engine_) {
      delete engine_;
    }
  }

  void EnableCudaGraph()
  {
    // run first time to avoid allocation
    this->Enqueue();
    cudaStreamSynchronize(stream_);

    cudaStreamBeginCapture(stream_, cudaStreamCaptureModeGlobal);
    this->Enqueue();
    cudaStreamEndCapture(stream_, &graph);
    this->use_cuda_graph = true;
#if CUDART_VERSION < 12000
    cudaGraphInstantiate(&graph_exec, graph, NULL, NULL, 0);
#else
    cudaGraphInstantiate(&graph_exec, graph, 0);
#endif
  }

private:
  ICudaEngine * engine_;
  IExecutionContext * context_;
};  // class SubNetwork

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
  // Engine paths
  std::string engine_backbone_path;
  std::string engine_head_path;
  std::string engine_position_embedding_path;

  // Model parameters
  bool use_temporal;
  double search_distance_2d;
  double circle_nms_dist_threshold;
  double iou_threshold;
  double confidence_threshold;
  std::vector<std::string> class_names;
  int32_t num_proposals;
  std::vector<double> yaw_norm_thresholds;
  std::vector<float> detection_range;
  int pre_memory_length;
  int post_memory_length;

  // Constructor with default initialization
  NetworkConfig(
    const std::string & engine_backbone_path_, const std::string & engine_head_path_,
    const std::string & engine_position_embedding_path_, const bool use_temporal_,
    const double search_distance_2d_, const double circle_nms_dist_threshold_,
    const double iou_threshold_, const double confidence_threshold_,
    const std::vector<std::string> & class_names_, const int32_t num_proposals_,
    const std::vector<double> & yaw_norm_thresholds_, const std::vector<float> & detection_range_,
    const int pre_memory_length_, const int post_memory_length_)
  : engine_backbone_path(engine_backbone_path_),
    engine_head_path(engine_head_path_),
    engine_position_embedding_path(engine_position_embedding_path_),
    use_temporal(use_temporal_),
    search_distance_2d(search_distance_2d_),
    circle_nms_dist_threshold(circle_nms_dist_threshold_),
    iou_threshold(iou_threshold_),
    confidence_threshold(confidence_threshold_),
    class_names(class_names_),
    num_proposals(num_proposals_),
    yaw_norm_thresholds(yaw_norm_thresholds_),
    detection_range(detection_range_),
    pre_memory_length(pre_memory_length_),
    post_memory_length(post_memory_length_)
  {
  }
};

class StreamPetrNetwork
{
public:
  explicit StreamPetrNetwork(const NetworkConfig & config);

  ~StreamPetrNetwork();
  void inference_detector(
    const std::shared_ptr<Tensor> imgs, const std::vector<float> & ego_pose,
    const std::vector<float> & ego_pose_inv, const std::vector<float> & img_metas_pad,
    const std::vector<float> & intrinsics, const std::vector<float> & img2lidar, const float stamp,
    std::vector<autoware_perception_msgs::msg::DetectedObject> & output_objects,
    std::vector<float> & forward_time_ms);

  void printBindingInfo();
  void wipe_memory();
  void printProfiling() const;

private:
  autoware_perception_msgs::msg::DetectedObject bbox_to_ros_msg(const Box3D & bbox);

  NetworkConfig config_;
  std::shared_ptr<Logger> logger_;
  std::shared_ptr<Profiler> profiler_;
  std::unique_ptr<IRuntime> runtime_;
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
