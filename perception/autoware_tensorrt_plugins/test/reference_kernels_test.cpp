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
#include "autoware/scatter_ops/segment_csr.h"
#include "autoware/unique_ops/unique.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <tuple>
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
  const cudaError_t status =
    cudaMemcpy(host_values.data(), device_ptr, sizeof(T) * element_count, cudaMemcpyDeviceToHost);
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

std::vector<float> makeSegmentReferenceMean(
  const std::vector<float> & src, const std::size_t rows, const std::size_t cols,
  const std::vector<std::int64_t> & indptr)
{
  std::vector<float> out((indptr.size() - 1U) * cols, 0.0F);
  for (std::size_t segment = 0; segment + 1U < indptr.size(); ++segment) {
    const auto start = static_cast<std::size_t>(indptr[segment]);
    const auto end = static_cast<std::size_t>(indptr[segment + 1U]);
    const auto count = std::max<std::size_t>(end - start, 1U);
    for (std::size_t col = 0; col < cols; ++col) {
      float accum = 0.0F;
      for (std::size_t row = start; row < end; ++row) {
        accum += src[row * cols + col];
      }
      out[segment * cols + col] = accum / static_cast<float>(count);
    }
  }
  (void)rows;
  return out;
}

std::vector<float> makeSegmentReferenceMax(
  const std::vector<float> & src, const std::size_t rows, const std::size_t cols,
  const std::vector<std::int64_t> & indptr)
{
  std::vector<float> out((indptr.size() - 1U) * cols, 0.0F);
  for (std::size_t segment = 0; segment + 1U < indptr.size(); ++segment) {
    const auto start = static_cast<std::size_t>(indptr[segment]);
    const auto end = static_cast<std::size_t>(indptr[segment + 1U]);
    for (std::size_t col = 0; col < cols; ++col) {
      float best = 0.0F;
      if (start < end) {
        best = -std::numeric_limits<float>::infinity();
        for (std::size_t row = start; row < end; ++row) {
          best = std::max(best, src[row * cols + col]);
        }
      }
      out[segment * cols + col] = best;
    }
  }
  (void)rows;
  return out;
}

void expectFloatVectorsEqual(const std::vector<float> & actual, const std::vector<float> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    EXPECT_FLOAT_EQ(actual[index], expected[index]) << "index=" << index;
  }
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
  DeviceBuffer<std::int64_t> num_unique_d(1U);
  DeviceBuffer<std::uint8_t> workspace_d(get_unique_workspace_size(input.size()));

  copyToDevice(input_d.get(), input);

  ASSERT_EQ(
    unique(
      input_d.get(), unique_d.get(), inverse_d.get(), counts_d.get(), num_unique_d.get(),
      workspace_d.get(), input.size(), get_unique_temp_storage_size(input.size()), stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  const auto num_unique = copyToHost(num_unique_d.get(), 1U).front();
  const auto unique_values = copyToHost(unique_d.get(), static_cast<std::size_t>(num_unique));
  const auto inverse_indices = copyToHost(inverse_d.get(), input.size());
  const auto counts = copyToHost(counts_d.get(), static_cast<std::size_t>(num_unique));

  EXPECT_EQ(unique_values, reference.unique_values);
  EXPECT_EQ(inverse_indices, reference.inverse_indices);
  EXPECT_EQ(counts, reference.counts);
}

TEST(ReferenceKernelsTest, SegmentCsrMeanMatchesCpuReference)
{
  if (getCudaDeviceCount() == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }

  const std::size_t rows = 6U;
  const std::size_t cols = 2U;
  const std::vector<float> src{1.0F, 2.0F, 3.0F, 4.0F,  5.0F,  6.0F,
                               7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F};
  const std::vector<std::int64_t> indptr{0, 2, 5, 6};
  const auto reference = makeSegmentReferenceMean(src, rows, cols, indptr);

  CudaStreamGuard stream;
  DeviceBuffer<float> src_d(src.size());
  DeviceBuffer<std::int64_t> indptr_d(indptr.size());
  DeviceBuffer<float> out_d(reference.size());

  copyToDevice(src_d.get(), src);
  copyToDevice(indptr_d.get(), indptr);

  ASSERT_EQ(
    (segment_csr_launch<float, MEAN>(
      src_d.get(), static_cast<std::int32_t>(rows), static_cast<std::int32_t>(cols), indptr_d.get(),
      static_cast<std::int32_t>(indptr.size()),
      std::make_tuple(out_d.get(), static_cast<std::int64_t *>(nullptr)), stream.get())),
    0);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  expectFloatVectorsEqual(copyToHost(out_d.get(), reference.size()), reference);
}

TEST(ReferenceKernelsTest, SegmentCsrMaxMatchesCpuReference)
{
  if (getCudaDeviceCount() == 0) {
    GTEST_SKIP() << "CUDA device not available";
  }

  const std::size_t rows = 6U;
  const std::size_t cols = 2U;
  const std::vector<float> src{1.0F, 6.0F, 3.0F, 4.0F,  5.0F,  9.0F,
                               7.0F, 8.0F, 2.0F, 10.0F, 11.0F, 12.0F};
  const std::vector<std::int64_t> indptr{0, 2, 5, 6};
  const auto reference = makeSegmentReferenceMax(src, rows, cols, indptr);

  CudaStreamGuard stream;
  DeviceBuffer<float> src_d(src.size());
  DeviceBuffer<std::int64_t> indptr_d(indptr.size());
  DeviceBuffer<float> out_d(reference.size());

  copyToDevice(src_d.get(), src);
  copyToDevice(indptr_d.get(), indptr);

  ASSERT_EQ(
    (segment_csr_launch<float, MAX>(
      src_d.get(), static_cast<std::int32_t>(rows), static_cast<std::int32_t>(cols), indptr_d.get(),
      static_cast<std::int32_t>(indptr.size()),
      std::make_tuple(out_d.get(), static_cast<std::int64_t *>(nullptr)), stream.get())),
    0);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  expectFloatVectorsEqual(copyToHost(out_d.get(), reference.size()), reference);
}

}  // namespace
