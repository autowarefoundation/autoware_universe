// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__TENSORRT_ONEPLANNER__PLANNER_INFERENCE_HPP_
#define AUTOWARE__TENSORRT_ONEPLANNER__PLANNER_INFERENCE_HPP_

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_runtime_api.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::tensorrt_oneplanner
{

using InputDataMap = autoware::diffusion_planner::preprocess::InputDataMap;

/**
 * @class PlannerInference
 * @brief TensorRT inference for the OnePlanner head.
 *
 * The engine consumes the standard diffusion-planner input tensors plus a BEV feature map
 * produced on-device by the BevEncoder. The BEV feature map never leaves the GPU: its
 * device pointer is bound directly as an engine input.
 *
 * The model is ego-only (P=1): the `prediction` output is [B, 1, OUTPUT_T, 4].
 */
class PlannerInference
{
public:
  struct InferenceResult
  {
    /// (prediction, turn_indicator_logit) when inference succeeds
    std::optional<std::pair<std::vector<float>, std::vector<float>>> outputs;
    std::string error_msg;
  };

  /**
   * @param model_path Path to the planner ONNX file.
   * @param plugins_path Path to the TensorRT plugin library.
   * @param batch_size Batch size (must be 1 in the current integration).
   * @param bev_feature_d Device pointer to the BEV feature map [1, C, H, W].
   * @param bev_channels BEV feature channel count C.
   * @param bev_size BEV feature map side length (H == W).
   */
  PlannerInference(
    const std::string & model_path, const std::string & plugins_path, int batch_size,
    const float * bev_feature_d, int64_t bev_channels, int64_t bev_size, cudaStream_t stream);

  InferenceResult infer(const InputDataMap & input_data_map);

private:
  void load_engine(const std::string & model_path);
  void bind_buffers();
  void transfer_inputs_to_device(const InputDataMap & input_data_map);

  int batch_size_{1};
  std::string plugins_path_;
  const float * bev_feature_d_{nullptr};
  int64_t bev_channels_{0};
  int64_t bev_size_{0};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> network_trt_ptr_{nullptr};

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

  autoware::cuda_utils::CudaUniquePtrHost<float[]> output_pinned_;
  autoware::cuda_utils::CudaUniquePtrHost<float[]> logit_pinned_;
  size_t output_num_elements_{0};
  size_t logit_num_elements_{0};

  cudaStream_t stream_{nullptr};
};

}  // namespace autoware::tensorrt_oneplanner

#endif  // AUTOWARE__TENSORRT_ONEPLANNER__PLANNER_INFERENCE_HPP_
