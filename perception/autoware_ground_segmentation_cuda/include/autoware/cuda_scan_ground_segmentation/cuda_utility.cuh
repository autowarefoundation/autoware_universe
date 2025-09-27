// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION_CUDA_UTILITIES_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION_CUDA_UTILITIES_HPP_

#include "cuda_common.hpp"
#include "cuda_mempool_wrapper.hpp"
#include "cuda_stream_wrapper.hpp"
#include "device_vector.hpp"

#include <cub/cub.cuh>

#include <cuda_runtime.h>

namespace autoware::cuda
{

template <int block_size, typename... Args>
inline cudaError_t launchAsync(
  int thread_num, int shared_size, cudaStream_t & stream, void (*f)(Args...), Args... args)
{
  int block_x = (thread_num > block_size) ? block_size : thread_num;

  if (block_x <= 0) {
    return cudaErrorLaunchFailure;
  }

  int grid_x = (thread_num + block_x - 1) / block_x;

  f<<<grid_x, block_x, shared_size, stream>>>(args...);

  return cudaGetLastError();
}

template <typename T>
cudaError_t ExclusiveScan(
  T * input, T * output, int ele_num, std::shared_ptr<CudaStream> stream,
  std::shared_ptr<CudaMempool> mempool)
{
  if (ele_num == 0) {
    return cudaSuccess;
  }

  if (ele_num < 0 || !stream) {
    return cudaErrorInvalidValue;
  }

  device_vector<int> d_temp_storage(stream, mempool);
  size_t temp_storage_bytes = 0;

  cub::DeviceScan::ExclusiveSum(
    (void *)(d_temp_storage.data()), temp_storage_bytes, input, output, ele_num, stream->get());

  int temp_ele_num = (temp_storage_bytes + sizeof(int) - 1) / sizeof(int);
  d_temp_storage.resize(temp_ele_num);

  cub::DeviceScan::ExclusiveSum(
    (void *)(d_temp_storage.data()), temp_storage_bytes, input, output, ele_num, stream->get());

  return cudaGetLastError();
}

template <typename T>
cudaError_t ExclusiveScan(device_vector<T> & input, device_vector<T> & output)
{
  if (input.empty()) {
    return cudaSuccess;
  }

  output.resize(input.size());

  return ExclusiveScan(
    input.data(), output.data(), (int)(input.size()), input.get_stream(), input.get_mempool());
}

template <typename T>
cudaError_t ExclusiveScan(device_vector<T> & input)
{
  return ExclusiveScan(input, input);
}

template <typename T>
__global__ void fillVector(T * vec, int ele_num, T init_val)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < ele_num; i += stride) {
    vec[i] = init_val;
  }
}

template <typename T>
cudaError_t fill(T * input, int ele_num, T val, std::shared_ptr<CudaStream> stream)
{
  if (ele_num == 0) {
    return cudaSuccess;
  }

  if (ele_num < 0 || !stream) {
    return cudaErrorInvalidValue;
  }

  return launchAsync<BLOCK_SIZE_X>(ele_num, 0, stream->get(), fillVector, input, ele_num, val);
}

template <typename T>
cudaError_t fill(device_vector<T> & input, T val)
{
  return fill(input.data(), (int)(input.size()), val, input.get_stream());
}

CUDAH void memcpy(uint8_t * dst, const uint8_t * src, int size)
{
  for (int i = 0; i < size; ++i) {
    dst[i] = src[i];
  }
}

inline void copyPointCloud2Metadata(
  cuda_blackboard::CudaPointCloud2 & dst, const cuda_blackboard::CudaPointCloud2 & src)
{
  dst.header = src.header;
  dst.height = src.height;
  dst.width = src.width;
  dst.fields = src.fields;
  dst.is_bigendian = src.is_bigendian;
  dst.point_step = src.point_step;
  dst.row_step = src.row_step;
  dst.is_dense = src.is_dense;
}

}  // namespace autoware::cuda

#endif
