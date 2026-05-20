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

#pragma once

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace autoware::tensorrt_plugins::test
{

class CudaStreamGuard
{
public:
  CudaStreamGuard()
  {
    const cudaError_t status = cudaStreamCreate(&stream_);
    if (status != cudaSuccess) {
      throw std::runtime_error(cudaGetErrorString(status));
    }
  }

  ~CudaStreamGuard()
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  CudaStreamGuard(const CudaStreamGuard &) = delete;
  CudaStreamGuard & operator=(const CudaStreamGuard &) = delete;
  CudaStreamGuard(CudaStreamGuard &&) = delete;
  CudaStreamGuard & operator=(CudaStreamGuard &&) = delete;

  [[nodiscard]] cudaStream_t get() const { return stream_; }

private:
  cudaStream_t stream_{nullptr};
};

template <typename T>
class DeviceBuffer
{
public:
  explicit DeviceBuffer(std::size_t element_count) : element_count_(element_count)
  {
    const cudaError_t status = cudaMalloc(&data_, sizeof(T) * element_count_);
    if (status != cudaSuccess) {
      throw std::runtime_error(cudaGetErrorString(status));
    }
  }

  ~DeviceBuffer()
  {
    if (data_ != nullptr) {
      cudaFree(data_);
    }
  }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;
  DeviceBuffer(DeviceBuffer &&) = delete;
  DeviceBuffer & operator=(DeviceBuffer &&) = delete;

  [[nodiscard]] T * get() const { return static_cast<T *>(data_); }

private:
  void * data_{nullptr};
  std::size_t element_count_{0U};
};

template <typename T>
void copy_to_device(T * device_ptr, const std::vector<T> & host_values)
{
  const cudaError_t status = cudaMemcpy(
    device_ptr, host_values.data(), sizeof(T) * host_values.size(), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

template <typename T>
std::vector<T> copy_to_host(const T * device_ptr, const std::size_t element_count)
{
  std::vector<T> host_values(element_count);
  const cudaError_t status =
    cudaMemcpy(host_values.data(), device_ptr, sizeof(T) * element_count, cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
  return host_values;
}

}  // namespace autoware::tensorrt_plugins::test
