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

#ifndef AUTOWARE__DIFFUSION_PLANNER__INFERENCE__UTILS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__INFERENCE__UTILS_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

namespace autoware::diffusion_planner
{

class CudaGraphExecutor
{
public:
  CudaGraphExecutor() = default;
  ~CudaGraphExecutor();

  CudaGraphExecutor(const CudaGraphExecutor &) = delete;
  CudaGraphExecutor & operator=(const CudaGraphExecutor &) = delete;

  bool is_captured() const;
  void reset();
  bool capture(cudaStream_t stream, const std::function<bool(cudaStream_t)> & enqueue);
  bool launch(cudaStream_t stream) const;

private:
  cudaGraph_t graph_{nullptr};
  cudaGraphExec_t graph_exec_{nullptr};
};

template <class Container>
size_t num_elements_without_batch(const Container & shape)
{
  return std::accumulate(shape.begin() + 1, shape.end(), size_t{1}, std::multiplies<>());
}

size_t num_elements(const std::vector<int64_t> & shape);

template <class Container>
nvinfer1::Dims to_dynamic_dims(const Container & shape, const int batch_size)
{
  nvinfer1::Dims dims;
  dims.nbDims = static_cast<int>(shape.size());
  dims.d[0] = (batch_size == 1 ? 1 : -1);
  for (size_t i = 1; i < shape.size(); ++i) {
    dims.d[i] = static_cast<int>(shape[i]);
  }
  return dims;
}

template <class Container>
nvinfer1::Dims to_dims_with_batch(const Container & shape, const int batch_size)
{
  nvinfer1::Dims dims;
  dims.nbDims = static_cast<int>(shape.size());
  dims.d[0] = batch_size;
  for (size_t i = 1; i < shape.size(); ++i) {
    dims.d[i] = static_cast<int>(shape[i]);
  }
  return dims;
}

autoware::tensorrt_common::ProfileDims make_profile_dims(
  const std::string & name, const nvinfer1::Dims & dims, int batch_size);

std::string engine_file_path(
  const std::string & model_path, int batch_size, const std::string & precision);

std::unique_ptr<autoware::tensorrt_common::TrtCommon> setup_engine(
  const std::string & model_path, const std::string & plugins_path, int batch_size,
  const std::string & precision,
  const std::vector<autoware::tensorrt_common::NetworkIO> & network_io,
  const std::vector<autoware::tensorrt_common::ProfileDims> & profile_dims);

bool enqueue_trt(
  autoware::tensorrt_common::TrtCommon & trt, CudaGraphExecutor & cuda_graph, cudaStream_t stream,
  bool use_cuda_graph);

template <class DevicePtr>
void transfer_float_input(
  const std::vector<float> & host_vec, const DevicePtr & device_ptr, cudaStream_t stream)
{
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_ptr.get(), host_vec.data(), host_vec.size() * sizeof(float), cudaMemcpyHostToDevice,
    stream));
}

template <class DevicePtr>
void transfer_speed_mask(
  const std::vector<float> & speed_limit, const DevicePtr & device_ptr, const size_t count,
  cudaStream_t stream)
{
  std::vector<uint8_t> bool_array(count);
  for (size_t i = 0; i < count; ++i) {
    bool_array[i] = speed_limit[i] > std::numeric_limits<float>::epsilon();
  }
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    device_ptr.get(), bool_array.data(), count * sizeof(uint8_t), cudaMemcpyHostToDevice, stream));
}

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__INFERENCE__UTILS_HPP_
