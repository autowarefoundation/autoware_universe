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

#include "autoware/simpl_prediction/trt_simpl.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_utils.hpp>

#include <NvInferRuntimeBase.h>

#include <functional>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

namespace autoware::simpl_prediction
{
TrtSimpl::TrtSimpl(const tensorrt_common::TrtCommonConfig & config)
{
  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(config);

  // TODO(ktro2828): add support of dynamic shape inference
  auto profile_dims = std::make_unique<std::vector<tensorrt_common::ProfileDims>>();
  auto network_io = std::make_unique<std::vector<tensorrt_common::NetworkIO>>();
  {
    constexpr size_t num_input = 3;   // (agent, map, rpe)
    constexpr size_t num_output = 2;  // (score, trajectory)
    // input profiles
    for (size_t i = 0; i < num_input; ++i) {
      const auto dims = trt_common_->getInputDims(i);
      profile_dims->emplace_back(i, dims, dims, dims);

      const auto name = trt_common_->getIOTensorName(i);
      network_io->emplace_back(name, dims);
    }

    // output profiles
    for (size_t i = 0; i < num_output; ++i) {
      const auto dims = trt_common_->getOutputDims(i);
      const auto name = trt_common_->getIOTensorName(i + num_input);
      network_io->emplace_back(name, dims);
    }
  }

  if (!trt_common_->setup(std::move(profile_dims), std::move(network_io))) {
    throw archetype::SimplException(
      archetype::SimplError_t::TENSORRT, "Failed to setup TensorRT engine.");
  }

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

archetype::Result<TrtSimpl::output_type> TrtSimpl::do_inference(
  const archetype::AgentTensor & agent_tensor, const archetype::MapTensor & map_tensor,
  const std::vector<float> & rpe_tensor) noexcept
{
  // Copy inputs from host to device
  try {
    init_cuda_ptr(agent_tensor, map_tensor, rpe_tensor);
  } catch (const std::runtime_error & e) {
    return archetype::Err<output_type>(archetype::SimplError_t::CUDA, e.what());
  }

  // function to compute the size of elements
  auto compute_dim_size = [](const nvinfer1::Dims & dims) -> size_t {
    return std::accumulate(dims.d, dims.d + dims.nbDims, 1, std::multiplies<size_t>());
  };

  const auto score_size = compute_dim_size(trt_common_->getOutputDims(0));
  const auto trajectory_size = compute_dim_size(trt_common_->getOutputDims(1));
  if (!out_score_d_) {
    out_score_d_ = cuda_utils::make_unique<float[]>(score_size);
  } else {
    cuda_utils::clear_async(out_score_d_.get(), score_size, stream_);
  }
  if (!out_trajectory_d_) {
    out_trajectory_d_ = cuda_utils::make_unique<float[]>(trajectory_size);
  } else {
    cuda_utils::clear_async(out_trajectory_d_.get(), trajectory_size, stream_);
  }

  if (cudaStreamSynchronize(stream_) != cudaSuccess) {
    return archetype::Err<output_type>(
      archetype::SimplError_t::CUDA, "Failed to synchronize stream");
  }

  // Set tensors addresses
  std::vector<void *> tensors{
    in_agent_d_.get(), in_map_d_.get(), in_rpe_d_.get(), out_score_d_.get(),
    out_trajectory_d_.get()};
  if (!trt_common_->setTensorsAddresses(tensors)) {
    return archetype::Err<output_type>(
      archetype::SimplError_t::TENSORRT, "Failed to set tensor addresses");
  }

  // Execute inference
  if (!trt_common_->enqueueV3(stream_)) {
    return archetype::Err<output_type>(archetype::SimplError_t::TENSORRT, "Failed to enqueue.");
  }

  if (cudaStreamSynchronize(stream_) != cudaSuccess) {
    return archetype::Err<output_type>(
      archetype::SimplError_t::CUDA, "Failed to synchronize stream");
  }

  // Copy outputs from device to host
  score_type score_h(score_size);
  trajectory_type trajectory_h(trajectory_size);

  try {
    CHECK_CUDA_ERROR(cudaMemcpy(
      score_h.data(), out_score_d_.get(), sizeof(float) * score_size, cudaMemcpyDeviceToHost));
    CHECK_CUDA_ERROR(cudaMemcpy(
      trajectory_h.data(), out_trajectory_d_.get(), sizeof(float) * trajectory_size,
      cudaMemcpyDeviceToHost));
  } catch (const std::runtime_error & e) {
    return archetype::Err<output_type>(archetype::SimplError_t::CUDA, e.what());
  }

  return archetype::Ok<output_type>({score_h, trajectory_h});
}

void TrtSimpl::init_cuda_ptr(
  const archetype::AgentTensor & agent_tensor, const archetype::MapTensor & map_tensor,
  const std::vector<float> & rpe_tensor)
{
  if (!in_agent_d_) {
    in_agent_d_ = cuda_utils::make_unique<float[]>(agent_tensor.size());
  } else {
    cuda_utils::clear_async(in_agent_d_.get(), agent_tensor.size(), stream_);
  }
  if (!in_map_d_) {
    in_map_d_ = cuda_utils::make_unique<float[]>(map_tensor.size());
  } else {
    cuda_utils::clear_async(in_map_d_.get(), map_tensor.size(), stream_);
  }
  if (!in_rpe_d_) {
    in_rpe_d_ = cuda_utils::make_unique<float[]>(rpe_tensor.size());
  } else {
    cuda_utils::clear_async(in_rpe_d_.get(), rpe_tensor.size(), stream_);
  }

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    in_agent_d_.get(), agent_tensor.data(), sizeof(float) * agent_tensor.size(),
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    in_map_d_.get(), map_tensor.data(), sizeof(float) * map_tensor.size(), cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    in_rpe_d_.get(), rpe_tensor.data(), sizeof(float) * rpe_tensor.size(), cudaMemcpyHostToDevice,
    stream_));
}
}  // namespace autoware::simpl_prediction
