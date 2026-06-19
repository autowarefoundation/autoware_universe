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

#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using autoware::ptv3::PreprocessCuda;
using autoware::ptv3::PTv3Config;
using autoware::ptv3::SerializedPoolingDeviceStageView;

class CudaStreamGuard
{
public:
  CudaStreamGuard()
  {
    const auto status = cudaStreamCreate(&stream_);
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

  [[nodiscard]] cudaStream_t get() const { return stream_; }

private:
  cudaStream_t stream_{nullptr};
};

template <typename T>
class DeviceBuffer
{
public:
  explicit DeviceBuffer(const std::size_t element_count)
  {
    const auto status = cudaMalloc(&data_, sizeof(T) * element_count);
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
  DeviceBuffer(DeviceBuffer && other) noexcept : data_(other.data_) { other.data_ = nullptr; }
  DeviceBuffer & operator=(DeviceBuffer && other) noexcept
  {
    if (this != &other) {
      if (data_ != nullptr) {
        cudaFree(data_);
      }
      data_ = other.data_;
      other.data_ = nullptr;
    }
    return *this;
  }

  [[nodiscard]] T * get() const { return static_cast<T *>(data_); }

private:
  void * data_{nullptr};
};

template <typename T>
void copy_to_device(T * device_ptr, const std::vector<T> & values)
{
  const auto status =
    cudaMemcpy(device_ptr, values.data(), sizeof(T) * values.size(), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

template <typename T>
std::vector<T> copy_to_host(const T * device_ptr, const std::size_t count)
{
  std::vector<T> values(count);
  const auto status =
    cudaMemcpy(values.data(), device_ptr, sizeof(T) * count, cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
  return values;
}

struct DeviceStage
{
  explicit DeviceStage(const std::size_t capacity, const std::size_t num_orders)
  : indices(capacity),
    indptr(capacity + 1),
    head_indices(capacity),
    cluster(capacity),
    grid_coord(capacity * 3),
    serialized_code(capacity * num_orders),
    serialized_order(capacity * num_orders),
    serialized_inverse(capacity * num_orders)
  {
  }

  DeviceBuffer<std::uint32_t> indices;
  DeviceBuffer<std::uint32_t> indptr;
  DeviceBuffer<std::uint32_t> head_indices;
  DeviceBuffer<std::uint32_t> cluster;
  DeviceBuffer<std::int32_t> grid_coord;
  DeviceBuffer<std::int64_t> serialized_code;
  DeviceBuffer<std::uint32_t> serialized_order;
  DeviceBuffer<std::uint32_t> serialized_inverse;
};

struct CpuStage
{
  std::vector<std::uint32_t> indices;
  std::vector<std::uint32_t> indptr;
  std::vector<std::uint32_t> head_indices;
  std::vector<std::uint32_t> cluster;
  std::vector<std::int32_t> grid_coord;
  std::vector<std::int64_t> serialized_code;
  std::vector<std::uint32_t> serialized_order;
  std::vector<std::uint32_t> serialized_inverse;
};

std::int32_t pooling_depth(const std::int64_t stride)
{
  std::int32_t depth = 0;
  for (auto value = stride; value > 1; value >>= 1) {
    ++depth;
  }
  return depth;
}

std::int64_t serialize_coord(
  const std::int32_t x, const std::int32_t y, const std::int32_t z, const std::int32_t depth,
  const bool transposed)
{
  std::uint64_t code = 0;
  for (std::int32_t bit = 0; bit < depth; ++bit) {
    const auto mask = static_cast<std::uint64_t>(1ULL << bit);
    if (transposed) {
      code |= ((y & mask) << (2 * bit + 2));
      code |= ((x & mask) << (2 * bit + 1));
    } else {
      code |= ((x & mask) << (2 * bit + 2));
      code |= ((y & mask) << (2 * bit + 1));
    }
    code |= ((z & mask) << (2 * bit));
  }
  return static_cast<std::int64_t>(code);
}

std::vector<std::int64_t> make_serialized_code(
  const std::vector<std::int32_t> & grid_coord, const std::int32_t depth)
{
  const auto count = grid_coord.size() / 3;
  std::vector<std::int64_t> code(2 * count);
  for (std::size_t index = 0; index < count; ++index) {
    const auto x = grid_coord[index * 3 + 0];
    const auto y = grid_coord[index * 3 + 1];
    const auto z = grid_coord[index * 3 + 2];
    code[index] = serialize_coord(x, y, z, depth, false);
    code[count + index] = serialize_coord(x, y, z, depth, true);
  }
  return code;
}

template <typename T>
std::vector<std::uint32_t> stable_argsort(const std::vector<T> & values)
{
  std::vector<std::uint32_t> order(values.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&values](const auto lhs, const auto rhs) {
    return values[static_cast<std::size_t>(lhs)] < values[static_cast<std::size_t>(rhs)];
  });
  return order;
}

CpuStage make_stage_reference(
  const std::vector<std::int32_t> & grid_coord_in,
  const std::vector<std::int64_t> & serialized_code_in, const std::size_t num_orders,
  const std::int64_t stride)
{
  const auto input_count = grid_coord_in.size() / 3;
  const auto depth = pooling_depth(stride);
  std::vector<std::int64_t> pooled_keys(input_count);
  for (std::size_t index = 0; index < input_count; ++index) {
    pooled_keys[index] = serialized_code_in[index] >> (depth * 3);
  }

  std::vector<std::int64_t> unique_keys = pooled_keys;
  std::sort(unique_keys.begin(), unique_keys.end());
  unique_keys.erase(std::unique(unique_keys.begin(), unique_keys.end()), unique_keys.end());

  std::map<std::int64_t, std::uint32_t> key_to_cluster;
  for (std::size_t index = 0; index < unique_keys.size(); ++index) {
    key_to_cluster.emplace(unique_keys[index], static_cast<std::uint32_t>(index));
  }

  CpuStage stage;
  stage.cluster.resize(input_count);
  stage.indptr.assign(unique_keys.size() + 1, 0);
  for (std::size_t index = 0; index < input_count; ++index) {
    const auto cluster = key_to_cluster.at(pooled_keys[index]);
    stage.cluster[index] = cluster;
    ++stage.indptr[static_cast<std::size_t>(cluster + 1U)];
  }
  for (std::size_t index = 1; index < stage.indptr.size(); ++index) {
    stage.indptr[index] += stage.indptr[index - 1];
  }

  stage.indices = stable_argsort(stage.cluster);
  stage.head_indices.resize(unique_keys.size());
  for (std::size_t segment = 0; segment < unique_keys.size(); ++segment) {
    stage.head_indices[segment] = stage.indices[static_cast<std::size_t>(stage.indptr[segment])];
  }

  stage.grid_coord.resize(unique_keys.size() * 3);
  stage.serialized_code.resize(num_orders * unique_keys.size());
  for (std::size_t segment = 0; segment < unique_keys.size(); ++segment) {
    const auto source = static_cast<std::size_t>(stage.head_indices[segment]);
    for (std::size_t coord = 0; coord < 3; ++coord) {
      stage.grid_coord[segment * 3 + coord] = grid_coord_in[source * 3 + coord] >> depth;
    }
    for (std::size_t order = 0; order < num_orders; ++order) {
      stage.serialized_code[order * unique_keys.size() + segment] =
        serialized_code_in[order * input_count + source] >> (depth * 3);
    }
  }

  stage.serialized_order.resize(num_orders * unique_keys.size());
  stage.serialized_inverse.resize(num_orders * unique_keys.size());
  for (std::size_t order = 0; order < num_orders; ++order) {
    std::vector<std::int64_t> order_codes(unique_keys.size());
    for (std::size_t index = 0; index < unique_keys.size(); ++index) {
      order_codes[index] = stage.serialized_code[order * unique_keys.size() + index];
    }
    const auto sorted_order = stable_argsort(order_codes);
    for (std::size_t rank = 0; rank < sorted_order.size(); ++rank) {
      const auto input_index = sorted_order[rank];
      stage.serialized_order[order * unique_keys.size() + rank] = input_index;
      stage.serialized_inverse[order * unique_keys.size() + static_cast<std::size_t>(input_index)] =
        static_cast<std::uint32_t>(rank);
    }
  }
  return stage;
}

PTv3Config make_test_config()
{
  return PTv3Config(
    "", 64, {1, 16, 32}, {0.0F, 0.0F, 0.0F, 64.0F, 64.0F, 64.0F}, {1.0F, 1.0F, 1.0F}, {"class"},
    {"z", "z-trans"}, {2, 2}, {0, 0, 0}, 0.0F, {}, "XYZI", "none", true);
}

template <typename T>
void expect_equal(
  const std::vector<T> & actual, const std::vector<T> & expected, const std::string & name)
{
  EXPECT_EQ(actual, expected) << name;
}

TEST(SerializedPoolingMetadataTest, MatchesCpuReferenceForOnnxFacingInputs)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  const auto config = make_test_config();
  constexpr std::size_t kNumOrders = 2;
  const std::vector<std::int32_t> grid_coord{5, 0, 0, 0, 0, 0, 1, 0, 0, 2, 0, 0, 3,  0, 1,
                                             4, 4, 0, 5, 4, 1, 8, 0, 0, 9, 0, 0, 10, 2, 0};
  const auto serialized_code = make_serialized_code(grid_coord, config.serialization_depth_);
  const auto num_voxels = static_cast<std::uint32_t>(grid_coord.size() / 3);

  CudaStreamGuard stream;
  PreprocessCuda preprocess(config, stream.get());
  DeviceBuffer<std::int32_t> grid_coord_d(grid_coord.size());
  DeviceBuffer<std::int64_t> serialized_code_d(serialized_code.size());
  DeviceBuffer<std::uint32_t> serialized_order_d(serialized_code.size());
  DeviceBuffer<std::uint32_t> serialized_inverse_d(serialized_code.size());
  DeviceBuffer<std::uint32_t> stage_counts_d(config.pooling_strides_.size() + 1);
  std::vector<DeviceStage> device_stages;
  std::vector<SerializedPoolingDeviceStageView> stage_views;
  for (std::size_t stage = 0; stage < config.pooling_strides_.size(); ++stage) {
    device_stages.emplace_back(config.max_num_voxels_, kNumOrders);
  }
  for (auto & stage : device_stages) {
    stage_views.push_back(SerializedPoolingDeviceStageView{
      stage.indices.get(), stage.indptr.get(), stage.head_indices.get(), stage.cluster.get(),
      stage.grid_coord.get(), stage.serialized_code.get(), stage.serialized_order.get(),
      stage.serialized_inverse.get()});
  }

  copy_to_device(grid_coord_d.get(), grid_coord);
  copy_to_device(serialized_code_d.get(), serialized_code);

  preprocess.generateSerializedPoolingMetadata(
    grid_coord_d.get(), serialized_code_d.get(), num_voxels, serialized_order_d.get(),
    serialized_inverse_d.get(), stage_views, stage_counts_d.get());
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  std::vector<std::uint32_t> expected_serialized_order(serialized_code.size());
  std::vector<std::uint32_t> expected_serialized_inverse(serialized_code.size());
  for (std::size_t order = 0; order < kNumOrders; ++order) {
    std::vector<std::int64_t> order_codes(num_voxels);
    for (std::size_t index = 0; index < num_voxels; ++index) {
      order_codes[index] = serialized_code[order * num_voxels + index];
    }
    const auto sorted_order = stable_argsort(order_codes);
    for (std::size_t rank = 0; rank < sorted_order.size(); ++rank) {
      const auto input_index = sorted_order[rank];
      expected_serialized_order[order * num_voxels + rank] = input_index;
      expected_serialized_inverse[order * num_voxels + input_index] =
        static_cast<std::uint32_t>(rank);
    }
  }
  expect_equal(
    copy_to_host(serialized_order_d.get(), serialized_code.size()), expected_serialized_order,
    "root serialized_order");
  expect_equal(
    copy_to_host(serialized_inverse_d.get(), serialized_code.size()), expected_serialized_inverse,
    "root serialized_inverse");

  std::vector<CpuStage> references;
  references.push_back(
    make_stage_reference(grid_coord, serialized_code, kNumOrders, config.pooling_strides_[0]));
  references.push_back(make_stage_reference(
    references[0].grid_coord, references[0].serialized_code, kNumOrders,
    config.pooling_strides_[1]));

  const auto stage_counts = copy_to_host(stage_counts_d.get(), config.pooling_strides_.size() + 1);
  ASSERT_EQ(stage_counts[0], num_voxels);
  ASSERT_EQ(stage_counts[1], static_cast<std::uint32_t>(references[0].head_indices.size()));
  ASSERT_EQ(stage_counts[2], static_cast<std::uint32_t>(references[1].head_indices.size()));

  for (std::size_t stage_index = 0; stage_index < references.size(); ++stage_index) {
    const auto & expected = references[stage_index];
    const auto & actual = device_stages[stage_index];
    const auto in_count = static_cast<std::size_t>(stage_counts[stage_index]);
    const auto out_count = static_cast<std::size_t>(stage_counts[stage_index + 1]);
    const auto prefix = "stage " + std::to_string(stage_index) + " ";

    expect_equal(
      copy_to_host(actual.indices.get(), in_count), expected.indices, prefix + "indices");
    expect_equal(
      copy_to_host(actual.indptr.get(), out_count + 1), expected.indptr, prefix + "indptr");
    expect_equal(
      copy_to_host(actual.head_indices.get(), out_count), expected.head_indices,
      prefix + "head_indices");
    expect_equal(
      copy_to_host(actual.cluster.get(), in_count), expected.cluster, prefix + "cluster");
    expect_equal(
      copy_to_host(actual.grid_coord.get(), out_count * 3), expected.grid_coord,
      prefix + "grid_coord");
    expect_equal(
      copy_to_host(actual.serialized_code.get(), out_count * kNumOrders), expected.serialized_code,
      prefix + "serialized_code");
    expect_equal(
      copy_to_host(actual.serialized_order.get(), out_count * kNumOrders),
      expected.serialized_order, prefix + "serialized_order");
    expect_equal(
      copy_to_host(actual.serialized_inverse.get(), out_count * kNumOrders),
      expected.serialized_inverse, prefix + "serialized_inverse");
  }
}

}  // namespace
