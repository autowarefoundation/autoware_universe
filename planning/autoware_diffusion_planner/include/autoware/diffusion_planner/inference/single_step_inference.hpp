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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__SINGLE_STEP_INFERENCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__SINGLE_STEP_INFERENCE_HPP_

#include "autoware/diffusion_planner/inference/inference.hpp"
#include "autoware/diffusion_planner/inference/utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
/**
 * @class TensorrtInference
 * @brief TensorRT inference wrapper for diffusion planner models.
 */
class SingleStepInference : public Inference
{
public:
  using InferenceResult = autoware::diffusion_planner::InferenceResult;

  /**
   * @brief Initialize TensorRT engine and device buffers for the batch size.
   * @param model_path Path to the TensorRT model file.
   * @param plugins_path Path to TensorRT plugin shared libraries.
   * @param batch_size Batch size for inference buffers and shapes.
   */
  SingleStepInference(
    const std::string & model_path, const std::string & plugins_path, int batch_size,
    const std::string & precision = "fp32", bool use_cuda_graph = false);
  ~SingleStepInference() override;

  /**
   * @brief Run inference and return prediction and turn indicator logits.
   * @param input_data_map Input tensor data keyed by input names.
   * @return InferenceResult containing outputs or error message.
   */
  InferenceResult infer(const preprocess::InputDataMap & input_data_map) override;
  /**
   * @brief Load or rebuild the TensorRT engine from the given model path.
   * @param model_path Path to the TensorRT model file.
   */
  void load_engine(const std::string & model_path);

private:
  int batch_size_{1};
  std::string plugins_path_;
  std::string precision_{"fp32"};
  bool use_cuda_graph_{false};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_{nullptr};
  CudaGraphExecutor network_cuda_graph_;
  /**
   * @brief Device buffers for model inputs/outputs.
   */
  autoware::cuda_utils::CudaUniquePtr<float[]> sampled_trajectories_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> ego_history_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> ego_current_state_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> neighbor_agents_past_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> static_objects_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lanes_d_;
  autoware::cuda_utils::CudaUniquePtr<bool[]> lanes_has_speed_limit_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> lanes_speed_limit_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_lanes_d_;
  autoware::cuda_utils::CudaUniquePtr<bool[]> route_lanes_has_speed_limit_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> route_lanes_speed_limit_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> polygons_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> line_strings_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> goal_pose_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> ego_shape_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> turn_indicators_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> delay_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> output_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> turn_indicator_logit_d_;

  // Pinned host buffers for fast D2H transfers
  autoware::cuda_utils::CudaUniquePtrHost<float[]> output_pinned_;
  autoware::cuda_utils::CudaUniquePtrHost<float[]> logit_pinned_;
  size_t output_num_elements_{0};
  size_t logit_num_elements_{0};

  cudaStream_t stream_{nullptr};

  void bindBuffers();
  void transferInputsToDevice(const preprocess::InputDataMap & input_data_map);
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__SINGLE_STEP_INFERENCE_HPP_
