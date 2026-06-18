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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__MULTI_STEP_INFERENCE_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__MULTI_STEP_INFERENCE_HPP_

#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"
#include "autoware/diffusion_planner/inference/inference.hpp"
#include "autoware/diffusion_planner/inference/solver/dpm_solver.hpp"
#include "autoware/diffusion_planner/inference/utils.hpp"
#include "autoware/diffusion_planner/preprocessing/preprocessing_utils.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <cuda_runtime_api.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diffusion_planner
{

class MultiStepInference : public Inference
{
public:
  using InferenceResult = autoware::diffusion_planner::InferenceResult;

  MultiStepInference(
    const std::string & encoder_model_path, const std::string & decoder_model_path,
    const std::string & turn_indicator_model_path, const std::string & plugins_path, int batch_size,
    const std::string & precision = "fp32", bool use_cuda_graph = false, int dpm_solver_steps = 10,
    std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances = {});
  ~MultiStepInference() override;

  void load_engines(
    const std::string & encoder_model_path, const std::string & decoder_model_path,
    const std::string & turn_indicator_model_path);
  InferenceResult infer(const preprocess::InputDataMap & input_data_map) override;

private:
  int batch_size_{1};
  int dpm_solver_steps_{10};
  std::string plugins_path_;
  std::string precision_{"fp32"};
  bool use_cuda_graph_{false};
  std::unordered_map<std::string, std::shared_ptr<Guidance>> guidances_;

  std::unique_ptr<autoware::tensorrt_common::TrtCommon> encoder_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> decoder_trt_ptr_{nullptr};
  std::unique_ptr<autoware::tensorrt_common::TrtCommon> turn_indicator_trt_ptr_{nullptr};
  CudaGraphExecutor encoder_cuda_graph_;
  CudaGraphExecutor decoder_cuda_graph_;
  CudaGraphExecutor turn_indicator_cuda_graph_;

  autoware::cuda_utils::CudaUniquePtr<float[]> sampled_trajectories_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> diffusion_time_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> ego_history_d_;
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

  autoware::cuda_utils::CudaUniquePtr<float[]> encoding_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> model_output_d_;
  autoware::cuda_utils::CudaUniquePtr<float[]> turn_indicator_logit_d_;

  autoware::cuda_utils::CudaUniquePtrHost<float[]> model_output_pinned_;
  autoware::cuda_utils::CudaUniquePtrHost<float[]> logit_pinned_;
  size_t encoding_num_elements_{0};
  size_t model_output_num_elements_{0};
  size_t logit_num_elements_{0};

  cudaStream_t stream_{nullptr};

  void bind_encoder_buffers();
  void bind_decoder_buffers();
  void bind_turn_indicator_buffers();
  void transfer_inputs_to_device(const preprocess::InputDataMap & input_data_map);
  DpmSolver::SampleResult run_dpm_solver(const preprocess::InputDataMap & input_data_map);
  std::vector<float> evaluate_decoder(const std::vector<float> & x, float t);
  std::vector<float> create_diffusion_time(float t) const;
  std::vector<float> create_current_states(const preprocess::InputDataMap & input_data_map) const;
  void apply_prefix_constraint(
    std::vector<float> & x, const std::vector<float> & current_states) const;
};

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__MULTI_STEP_INFERENCE_HPP_
