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

#include "autoware/simpl/trt_simpl.hpp"

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/exception.hpp"
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/archetype/result.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>

#include <NvInferRuntimeBase.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::simpl
{
namespace
{
/**
 * @brief Calculate the size of tensor dimensions.
 *
 * @param dims Tensor dimensions.
 */
size_t calculate_dim_size(const nvinfer1::Dims & dims)
{
  return std::accumulate(dims.d, dims.d + dims.nbDims, 1, std::multiplies<size_t>());
}
}  // namespace

using output_type = TrtSimpl::output_type;

TrtSimpl::TrtSimpl(const tensorrt_common::TrtCommonConfig & config)
{
  trt_common_ = std::make_unique<tensorrt_common::TrtCommon>(config);

  using ProfileDims = tensorrt_common::ProfileDims;

  // NOTE: Only static inference is supported
  constexpr size_t num_input = 3;
  std::vector<ProfileDims> profile_dims;
  for (size_t i = 0; i < num_input; ++i) {
    const auto dims = trt_common_->getInputDims(i);
    profile_dims.emplace_back(i, dims, dims, dims);
  }

  auto profile_dims_ptr = std::make_unique<std::vector<ProfileDims>>(profile_dims);
  if (!trt_common_->setup(std::move(profile_dims_ptr))) {
    throw archetype::SimplException(
      archetype::SimplError_t::TensorRT, "Failed to setup TensorRT engine.");
  }

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
}

archetype::Result<output_type> TrtSimpl::do_inference(
  const archetype::AgentTensor & agent_tensor, const archetype::MapTensor & map_tensor,
  const std::vector<float> & rpe_tensor) noexcept
{
  // Copy inputs from host to device
  try {
    init_cuda_ptr(agent_tensor, map_tensor, rpe_tensor);
  } catch (const std::runtime_error & e) {
    return archetype::Err<output_type>(archetype::SimplError_t::Cuda, e.what());
  }

  // Execute inference
  std::vector<void *> buffers{
    in_agent_d_.get(), in_map_d_.get(), in_rpe_d_.get(), out_score_d_.get(),
    out_trajectory_d_.get()};
  if (!trt_common_->enqueueV3(stream_)) {
    return archetype::Err<output_type>(archetype::SimplError_t::TensorRT, "Failed to enqueue.");
  }

  // Copy outputs from device to host
  const auto score_dims = trt_common_->getOutputDims(0);
  const auto score_size = calculate_dim_size(score_dims);
  score_type score_h(score_size);

  const auto trajectory_dims = trt_common_->getOutputDims(1);
  const auto trajectory_size = calculate_dim_size(trajectory_dims);
  trajectory_type trajectory_h(trajectory_size);

  try {
    CHECK_CUDA_ERROR(
      cudaMemcpy(score_h.data(), out_score_d_.get(), score_size, cudaMemcpyDeviceToHost));
    CHECK_CUDA_ERROR(cudaMemcpy(
      trajectory_h.data(), out_trajectory_d_.get(), trajectory_size, cudaMemcpyDeviceToHost));
  } catch (const std::runtime_error & e) {
    return archetype::Err<output_type>(archetype::SimplError_t::Cuda, e.what());
  }

  return archetype::Ok<output_type>({score_h, trajectory_h});
}

void TrtSimpl::init_cuda_ptr(
  const archetype::AgentTensor & agent_tensor, const archetype::MapTensor & map_tensor,
  const std::vector<float> & rpe_tensor)
{
  const auto in_agent_dims = trt_common_->getInputDims(0);
  const auto in_map_dims = trt_common_->getInputDims(1);
  const auto in_rpe_dims = trt_common_->getInputDims(2);

  const auto in_agent_size = calculate_dim_size(in_agent_dims);
  if (in_agent_size != agent_tensor.size()) {
    throw archetype::SimplException(
      archetype::SimplError_t::InvalidValue, "Invalid input agent size");
  }
  const auto in_map_size = calculate_dim_size(in_map_dims);
  if (in_map_size != map_tensor.size()) {
    throw archetype::SimplException(
      archetype::SimplError_t::InvalidValue, "Invalid input map size");
  }
  const auto in_rpe_size = calculate_dim_size(in_rpe_dims);
  if (in_rpe_size != rpe_tensor.size()) {
    throw archetype::SimplException(
      archetype::SimplError_t::InvalidValue, "Invalid input RPE size");
  }

  in_agent_d_ = cuda_utils::make_unique<float[]>(in_agent_size);
  in_map_d_ = cuda_utils::make_unique<float[]>(in_map_size);
  in_rpe_d_ = cuda_utils::make_unique<float[]>(in_rpe_size);

  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    in_agent_d_.get(), agent_tensor.data_ptr(), sizeof(float) * in_agent_size,
    cudaMemcpyHostToDevice, stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    in_map_d_.get(), map_tensor.data_ptr(), sizeof(float) * in_map_size, cudaMemcpyHostToDevice,
    stream_));
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    in_rpe_d_.get(), rpe_tensor.data(), sizeof(float) * in_rpe_size, cudaMemcpyHostToDevice,
    stream_));
}
}  // namespace autoware::simpl
