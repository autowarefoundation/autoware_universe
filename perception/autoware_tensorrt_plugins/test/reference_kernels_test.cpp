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

#include "autoware/argsort_ops/argsort.hpp"
#include "autoware/unique_ops/unique.hpp"

#include <gtest/gtest.h>

#include <cuda_runtime_api.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <vector>

namespace
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

  cudaStream_t get() const { return stream_; }

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

  T * get() const { return static_cast<T *>(data_); }

private:
  void * data_{nullptr};
  std::size_t element_count_{0U};
};

int getCudaDeviceCount()
{
  int device_count = 0;
  const cudaError_t status = cudaGetDeviceCount(&device_count);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
  return device_count;
}

template <typename T>
void copyToDevice(T * device_ptr, const std::vector<T> & host_values)
{
  const cudaError_t status = cudaMemcpy(
    device_ptr, host_values.data(), sizeof(T) * host_values.size(), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

template <typename T>
std::vector<T> copyToHost(const T * device_ptr, const std::size_t element_count)
{
  std::vector<T> host_values(element_count);
  const cudaError_t status = cudaMemcpy(
    host_values.data(), device_ptr, sizeof(T) * element_count, cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
  return host_values;
}

std::size_t getArgsortTotalWorkspaceSize(const std::size_t num_elements)
{
  const auto temp_size = get_argsort_workspace_size(num_elements);
  const auto scratch_offset =
    ((temp_size + alignof(std::int64_t) - 1U) / alignof(std::int64_t)) * alignof(std::int64_t);
  return scratch_offset + sizeof(std::int64_t) * 2U * num_elements;
}

std::vector<std::int64_t> makeArgsortReference(const std::vector<std::int64_t> & input)
{
  std::vector<std::int64_t> indices(input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    indices[index] = static_cast<std::int64_t>(index);
  }

  std::stable_sort(
    indices.begin(), indices.end(),
    [&input](const std::int64_t lhs, const std::int64_t rhs) { return input[lhs] < input[rhs]; });

  return indices;
}

struct UniqueReference
{
  std::vector<std::int64_t> unique_values;
  std::vector<std::int64_t> inverse_indices;
  std::vector<std::int64_t> counts;
};

UniqueReference makeUniqueReference(const std::vector<std::int64_t> & input)
{
  UniqueReference reference;
  reference.unique_values = input;
  std::sort(reference.unique_values.begin(), reference.unique_values.end());
  reference.unique_values.erase(
    std::unique(reference.unique_values.begin(), reference.unique_values.end()),
    reference.unique_values.end());

  std::map<std::int64_t, std::int64_t> value_to_index;
  for (std::size_t index = 0; index < reference.unique_values.size(); ++index) {
    value_to_index.emplace(reference.unique_values[index], static_cast<std::int64_t>(index));
  }
  reference.counts.resize(reference.unique_values.size(), 0);

  reference.inverse_indices.reserve(input.size());
  for (const auto value : input) {
    const auto index = value_to_index.at(value);
    reference.inverse_indices.push_back(index);
    reference.counts[static_cast<std::size_t>(index)] += 1;
  }

  return reference;
}

TEST(ReferenceKernelsTest, ArgsortMatchesCpuReference)
{
  if (getCudaDeviceCount() == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }

  const std::vector<std::int64_t> input{7, 3, 7, 5, 3, 3, 9, 5, 11, 7};
  const auto reference = makeArgsortReference(input);

  CudaStreamGuard stream;
  DeviceBuffer<std::int64_t> input_d(input.size());
  DeviceBuffer<std::int64_t> output_d(input.size());
  DeviceBuffer<std::uint8_t> workspace_d(getArgsortTotalWorkspaceSize(input.size()));

  copyToDevice(input_d.get(), input);

  ASSERT_EQ(
    argsort(
      input_d.get(), output_d.get(), workspace_d.get(), input.size(),
      get_argsort_workspace_size(input.size()), stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  EXPECT_EQ(copyToHost(output_d.get(), input.size()), reference);
}

TEST(ReferenceKernelsTest, UniqueMatchesCpuReference)
{
  if (getCudaDeviceCount() == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }

  const std::vector<std::int64_t> input{7, 3, 7, 5, 3, 3, 9, 5, 11, 7};
  const UniqueReference reference = makeUniqueReference(input);

  CudaStreamGuard stream;
  DeviceBuffer<std::int64_t> input_d(input.size());
  DeviceBuffer<std::int64_t> unique_d(input.size());
  DeviceBuffer<std::int64_t> inverse_d(input.size());
  DeviceBuffer<std::int64_t> counts_d(input.size());
  DeviceBuffer<std::uint8_t> workspace_d(get_unique_workspace_size(input.size()));

  copyToDevice(input_d.get(), input);

  const auto num_unique = unique(
    input_d.get(), unique_d.get(), inverse_d.get(), counts_d.get(), workspace_d.get(), input.size(),
    get_unique_workspace_size(input.size()), stream.get());
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  const auto unique_values = copyToHost(unique_d.get(), static_cast<std::size_t>(num_unique));
  const auto inverse_indices = copyToHost(inverse_d.get(), input.size());
  const auto counts = copyToHost(counts_d.get(), static_cast<std::size_t>(num_unique));

  EXPECT_EQ(unique_values, reference.unique_values);
  EXPECT_EQ(inverse_indices, reference.inverse_indices);
  EXPECT_EQ(counts, reference.counts);
}

}  // namespace
