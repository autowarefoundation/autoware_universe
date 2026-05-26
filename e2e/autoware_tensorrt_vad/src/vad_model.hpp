// Copyright 2025 TIER IV.
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

#ifndef VAD_MODEL_HPP_
#define VAD_MODEL_HPP_

#include "data_types.hpp"
#include "networks/backbone.hpp"
#include "networks/head.hpp"
#include "networks/net.hpp"
#include "networks/postprocess/map_postprocess.hpp"
#include "networks/postprocess/object_postprocess.hpp"
#include "networks/preprocess/multi_camera_preprocess.hpp"
#include "ros_vad_logger.hpp"
#include "vad_config.hpp"

#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <opencv2/core/mat.hpp>

#include <NvInfer.h>
#include <cuda_runtime.h>
#include <dlfcn.h>

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::tensorrt_vad
{

// Helper function to parse external input configuration
inline std::pair<std::string, std::string> parse_external_inputs(
  const std::pair<std::string, std::map<std::string, std::string>> & input_pair)
{
  const auto & ext_map = input_pair.second;
  return {ext_map.at("net"), ext_map.at("name")};
}

// VAD model class - Handles inference using CUDA/TensorRT
template <typename LoggerType>
class VadModel
{
public:
  VadModel(
    const VadConfig & vad_config,
    const autoware::tensorrt_common::TrtCommonConfig & backbone_config,
    const autoware::tensorrt_common::TrtCommonConfig & head_config,
    const autoware::tensorrt_common::TrtCommonConfig & head_no_prev_config,
    std::shared_ptr<LoggerType> logger)
  : stream_(nullptr),
    is_first_frame_(true),
    vad_config_(vad_config),
    logger_(std::move(logger)),
    head_trt_config_(head_config)
  {
    // Logger accepts only classes that inherit from VadLogger
    static_assert(
      std::is_base_of_v<VadLogger, LoggerType>,
      "LoggerType must be VadLogger or derive from VadLogger.");

    cudaStreamCreate(&stream_);

    nets_ = init_engines(
      vad_config_.nets_config, vad_config_, backbone_config, head_config, head_no_prev_config);

    // Initialize MultiCameraPreprocessor
    MultiCameraPreprocessConfig preprocess_config =
      vad_config_.create_multi_camera_preprocess_config();
    preprocessor_ = std::make_unique<MultiCameraPreprocessor>(preprocess_config, logger_);

    // Initialize MapPostprocessor
    MapPostprocessConfig map_postprocess_config = vad_config_.create_map_postprocess_config();
    map_postprocessor_ = std::make_unique<MapPostprocessor>(map_postprocess_config, logger_);

    // Initialize ObjectPostprocessor
    ObjectPostprocessConfig object_postprocess_config =
      vad_config_.create_object_postprocess_config();
    object_postprocessor_ =
      std::make_unique<ObjectPostprocessor>(object_postprocess_config, logger_);
  }

  // Destructor
  ~VadModel()
  {
    if (stream_) {
      cudaStreamDestroy(stream_);
      stream_ = nullptr;
    }

    // Cleanup nets
    nets_.clear();
  }

  // Main inference API
  [[nodiscard]] std::optional<VadOutputData> infer(const VadInputData & vad_input_data)
  {
    // Change head name based on whether it's the first frame
    const std::string head_name = is_first_frame_ ? "head_no_prev" : "head";

    // Load to bindings. If preprocessing fails the head bindings (including the ping-pong
    // setTensorAddress) are not refreshed, so we must skip enqueue/postprocess and leave
    // prev_is_a_ untouched to keep the software ping-pong state in sync with TRT.
    if (!load_inputs(vad_input_data, head_name)) {
      return std::nullopt;
    }

    // Enqueue backbone and head
    enqueue(head_name);

    // Convert output to VadOutputData
    VadOutputData output = postprocess(head_name, vad_input_data.command);

    if (is_first_frame_) {
      // Transition from "head_no_prev" to "head" with persistent ping-pong buffers.
      // Capture head_no_prev's BEV output ptr before its bindings get cleared.
      void * hnp_bev_ptr = nets_["head_no_prev"]->bindings["out.bev_embed"]->ptr;
      bev_nbytes_ = nets_["head_no_prev"]->bindings["out.bev_embed"]->nbytes();

      // Build "head" engine bindings (this cudaMalloc's prev_bev and out.bev_embed buffers).
      load_head();

      // Reuse the two auto-allocated buffers as ping-pong (no extra cudaMalloc).
      bev_buf_a_ = nets_["head"]->bindings["prev_bev"]->ptr;
      bev_buf_b_ = nets_["head"]->bindings["out.bev_embed"]->ptr;

      // One-time D->D copy: seed buffer A with the first-frame BEV from head_no_prev.
      cudaMemcpyAsync(bev_buf_a_, hnp_bev_ptr, bev_nbytes_, cudaMemcpyDeviceToDevice, stream_);

      release_network("head_no_prev");
      prev_is_a_ = true;  // buffer A holds the latest BEV, used as prev_bev next frame
      is_first_frame_ = false;
    } else {
      // Steady state: the buffer we just wrote to becomes next frame's prev_bev input.
      prev_is_a_ = !prev_is_a_;
    }

    return output;
  }

  // Member variables
  cudaStream_t stream_;
  std::unordered_map<std::string, std::shared_ptr<Net>> nets_;

  // Ping-pong buffers for prev_bev (input) and out.bev_embed (output) of the "head" engine.
  // Each frame they swap roles via setTensorAddress; the engine writes a fresh BEV to one
  // buffer while reading the previous BEV from the other, eliminating per-frame D->D copies.
  void * bev_buf_a_{nullptr};
  void * bev_buf_b_{nullptr};
  bool prev_is_a_{true};
  size_t bev_nbytes_{0};
  bool is_first_frame_;

  // Configuration information storage
  VadConfig vad_config_;

  std::shared_ptr<VadLogger> logger_;

private:
  autoware::tensorrt_common::TrtCommonConfig head_trt_config_;
  std::unique_ptr<MultiCameraPreprocessor> preprocessor_;
  std::unique_ptr<MapPostprocessor> map_postprocessor_;
  std::unique_ptr<ObjectPostprocessor> object_postprocessor_;

  std::unordered_map<std::string, std::shared_ptr<Net>> init_engines(
    const std::vector<NetConfig> & nets_config, const VadConfig & vad_config,
    const autoware::tensorrt_common::TrtCommonConfig & backbone_config,
    const autoware::tensorrt_common::TrtCommonConfig & head_config,
    const autoware::tensorrt_common::TrtCommonConfig & head_no_prev_config)
  {
    std::unordered_map<std::string, std::shared_ptr<Net>> nets;

    for (const auto & engine : nets_config) {
      std::unordered_map<std::string, std::shared_ptr<Tensor>> external_bindings;
      // reuse memory
      for (const auto & input_pair : engine.inputs) {
        const std::string & k = input_pair.first;
        auto [external_network, external_input_name] = parse_external_inputs(input_pair);
        logger_->info(k + " <- " + external_network + "[" + external_input_name + "]");
        external_bindings[k] = nets[external_network]->bindings[external_input_name];
      }

      if (engine.name == "backbone") {
        nets[engine.name] = std::make_shared<Backbone>(
          vad_config, backbone_config, vad_config_.plugins_path, logger_);
        nets[engine.name]->set_input_tensor(external_bindings);
      } else if (engine.name == "head_no_prev") {
        nets[engine.name] = std::make_shared<Head>(
          vad_config, head_no_prev_config, NetworkType::HEAD_NO_PREV, vad_config_.plugins_path,
          logger_);
        nets[engine.name]->set_input_tensor(external_bindings);
      } else if (engine.name == "head") {
        nets[engine.name] = std::make_shared<Head>(
          vad_config, head_config, NetworkType::HEAD, vad_config_.plugins_path, logger_);
      }
    }

    return nets;
  }

  // Helper functions used in infer function. Returns false if any input preparation step
  // failed; the caller must then abort inference for this frame and leave the ping-pong
  // selector (prev_is_a_) unchanged, otherwise the software state would drift away from
  // the TensorRT binding state.
  [[nodiscard]] bool load_inputs(const VadInputData & vad_input_data, const std::string & head_name)
  {
    // Use MultiCameraPreprocessor to process camera images
    cudaError_t preprocess_result = preprocessor_->preprocess_images(
      vad_input_data.camera_images, static_cast<float *>(nets_["backbone"]->bindings["img"]->ptr),
      stream_);

    if (preprocess_result != cudaSuccess) {
      logger_->error(
        "CUDA preprocessing failed: " + std::string(cudaGetErrorString(preprocess_result)));
      return false;
    }

    nets_[head_name]->bindings["img_metas.0[shift]"]->load(vad_input_data.shift, stream_);
    nets_[head_name]->bindings["img_metas.0[lidar2img]"]->load(
      vad_input_data.vad_base2img, stream_);
    nets_[head_name]->bindings["img_metas.0[can_bus]"]->load(vad_input_data.can_bus, stream_);

    if (head_name == "head") {
      // Ping-pong: route TensorRT bindings to the buffer holding the latest BEV (prev_bev)
      // and the buffer that will receive the new BEV (out.bev_embed). Without this, TRT
      // would keep reading whichever buffer it was bound to at engine setup time.
      void * prev_ptr = prev_is_a_ ? bev_buf_a_ : bev_buf_b_;
      void * out_ptr = prev_is_a_ ? bev_buf_b_ : bev_buf_a_;
      nets_["head"]->trt_common->setTensorAddress("prev_bev", prev_ptr);
      nets_["head"]->trt_common->setTensorAddress("out.bev_embed", out_ptr);
    }
    return true;
  }

  void enqueue(const std::string & head_name)
  {
    nets_["backbone"]->enqueue(stream_);
    nets_[head_name]->enqueue(stream_);
    cudaStreamSynchronize(stream_);
  }

  void release_network(const std::string & network_name)
  {
    if (nets_.find(network_name) != nets_.end()) {
      // First clear bindings
      nets_[network_name]->bindings.clear();
      cudaStreamSynchronize(stream_);

      // Then release Net object
      nets_[network_name].reset();
      nets_.erase(network_name);
      cudaStreamSynchronize(stream_);
    }
  }

  void load_head()
  {
    auto head_engine = std::find_if(
      vad_config_.nets_config.begin(), vad_config_.nets_config.end(),
      [](const NetConfig & engine) { return engine.name == "head"; });

    if (head_engine == vad_config_.nets_config.end()) {
      logger_->error("Head engine configuration not found");
      return;
    }

    std::unordered_map<std::string, std::shared_ptr<Tensor>> external_bindings;
    for (const auto & input_pair : head_engine->inputs) {
      const std::string & k = input_pair.first;
      auto [external_network, external_input_name] = parse_external_inputs(input_pair);
      logger_->info(k + " <- " + external_network + "[" + external_input_name + "]");
      external_bindings[k] = nets_[external_network]->bindings[external_input_name];
    }

    nets_["head"]->set_input_tensor(external_bindings);
  }

  VadOutputData postprocess(const std::string & head_name, int32_t cmd)
  {
    std::vector<float> ego_fut_preds =
      nets_[head_name]->bindings["out.ego_fut_preds"]->cpu<float>();

    // Process detected objects using CUDA postprocessor
    ObjectPostprocessor::InferenceInputs inference_inputs{
      static_cast<const float *>(nets_[head_name]->bindings["out.all_cls_scores"]->ptr),
      static_cast<const float *>(nets_[head_name]->bindings["out.all_traj_preds"]->ptr),
      static_cast<const float *>(nets_[head_name]->bindings["out.all_traj_cls_scores"]->ptr),
      static_cast<const float *>(nets_[head_name]->bindings["out.all_bbox_preds"]->ptr)};

    std::vector<BBox> filtered_bboxes =
      object_postprocessor_->postprocess_objects(inference_inputs, stream_);

    // Process map polylines using CUDA postprocessor
    std::vector<MapPolyline> map_polylines = map_postprocessor_->postprocess_map_preds(
      static_cast<const float *>(nets_[head_name]->bindings["out.map_all_cls_scores"]->ptr),
      static_cast<const float *>(nets_[head_name]->bindings["out.map_all_pts_preds"]->ptr),
      stream_);

    // Extract planning trajectories (use config parameters)
    const int32_t points_per_trajectory = vad_config_.planning_timesteps * 2;  // x,y per timestep
    const int32_t num_commands = vad_config_.planning_ego_commands;

    // Extract planning for the given command
    std::vector<float> planning(
      ego_fut_preds.begin() + cmd * points_per_trajectory,
      ego_fut_preds.begin() + (cmd + 1) * points_per_trajectory);

    // cumsum to build trajectory in 3d space
    for (int32_t i = 1; i < vad_config_.planning_timesteps; i++) {
      planning[i * 2] += planning[(i - 1) * 2];
      planning[i * 2 + 1] += planning[(i - 1) * 2 + 1];
    }

    // Extract all trajectories for all commands
    std::map<int32_t, std::vector<float>> all_trajectories;

    for (int32_t command_idx = 0; command_idx < num_commands; command_idx++) {
      std::vector<float> trajectory(
        ego_fut_preds.begin() + command_idx * points_per_trajectory,
        ego_fut_preds.begin() + (command_idx + 1) * points_per_trajectory);

      // cumsum to build trajectory in 3d space
      for (int32_t i = 1; i < vad_config_.planning_timesteps; i++) {
        trajectory[i * 2] += trajectory[(i - 1) * 2];
        trajectory[i * 2 + 1] += trajectory[(i - 1) * 2 + 1];
      }

      all_trajectories[command_idx] = trajectory;
    }
    return VadOutputData{planning, all_trajectories, map_polylines, filtered_bboxes};
  }
};

}  // namespace autoware::tensorrt_vad

#endif  // VAD_MODEL_HPP_
