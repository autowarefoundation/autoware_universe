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

#include "autoware/scatter_ops/gather_segment_csr.hpp"
#include "test_utils.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

using autoware::scatter_ops::gather_segment_csr_float;
using autoware::scatter_ops::GatherSegmentCSRReduce;
using autoware::tensorrt_plugins::test::copy_to_device;
using autoware::tensorrt_plugins::test::copy_to_host;
using autoware::tensorrt_plugins::test::CudaStreamGuard;
using autoware::tensorrt_plugins::test::DeviceBuffer;

/// Return the neutral CPU accumulator value matching the CUDA reducer.
float initial_value(const GatherSegmentCSRReduce reduce)
{
  if (reduce == GatherSegmentCSRReduce::kMin) {
    return std::numeric_limits<float>::max();
  }
  if (reduce == GatherSegmentCSRReduce::kMax) {
    return -std::numeric_limits<float>::max();
  }
  return 0.0F;
}

/// Apply one CPU reduction step matching the CUDA reducer.
float update_value(const float current, const float next, const GatherSegmentCSRReduce reduce)
{
  if (reduce == GatherSegmentCSRReduce::kMin) {
    return std::min(current, next);
  }
  if (reduce == GatherSegmentCSRReduce::kMax) {
    return std::max(current, next);
  }
  return current + next;
}

/// Build the expected pooled feature tensor from CSR groups.
std::vector<float> make_feature_reference(
  const std::vector<float> & features, const std::vector<std::int64_t> & indices,
  const std::vector<std::int64_t> & indptr, const std::int32_t num_channels,
  const GatherSegmentCSRReduce reduce)
{
  const auto num_segments = static_cast<std::int32_t>(indptr.size() - 1);
  std::vector<float> output(static_cast<std::size_t>(num_segments * num_channels), 0.0F);
  for (std::int32_t segment = 0; segment < num_segments; ++segment) {
    const auto begin = indptr[segment];
    const auto end = indptr[segment + 1];
    const auto count = end - begin;
    for (std::int32_t channel = 0; channel < num_channels; ++channel) {
      float value = initial_value(reduce);
      for (auto cursor = begin; cursor < end; ++cursor) {
        const auto source = indices[static_cast<std::size_t>(cursor)];
        value = update_value(
          value, features[static_cast<std::size_t>(source * num_channels + channel)], reduce);
      }
      if (count == 0) {
        value = 0.0F;
      } else if (reduce == GatherSegmentCSRReduce::kMean) {
        value /= static_cast<float>(count);
      }
      output[static_cast<std::size_t>(segment * num_channels + channel)] = value;
    }
  }
  return output;
}

/// Build the expected pooled coordinate tensor from the same CSR groups as features.
std::vector<float> make_coord_reference(
  const std::vector<float> & coords, const std::vector<std::int64_t> & indices,
  const std::vector<std::int64_t> & indptr)
{
  constexpr std::int32_t kCoordChannels = 3;
  const auto num_segments = static_cast<std::int32_t>(indptr.size() - 1);
  std::vector<float> output(static_cast<std::size_t>(num_segments * kCoordChannels), 0.0F);
  for (std::int32_t segment = 0; segment < num_segments; ++segment) {
    const auto begin = indptr[segment];
    const auto end = indptr[segment + 1];
    const auto count = end - begin;
    for (std::int32_t channel = 0; channel < kCoordChannels; ++channel) {
      float value = 0.0F;
      for (auto cursor = begin; cursor < end; ++cursor) {
        const auto source = indices[static_cast<std::size_t>(cursor)];
        value += coords[static_cast<std::size_t>(source * kCoordChannels + channel)];
      }
      output[static_cast<std::size_t>(segment * kCoordChannels + channel)] =
        count == 0 ? 0.0F : value / static_cast<float>(count);
    }
  }
  return output;
}

/// Compare flat tensors and report the failing linear index.
void expect_near_vector(
  const std::vector<float> & actual, const std::vector<float> & expected, const float tolerance)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    EXPECT_NEAR(actual[index], expected[index], tolerance) << "index=" << index;
  }
}

class GatherSegmentCSRKernelTest
: public ::testing::TestWithParam<std::pair<std::string, GatherSegmentCSRReduce>>
{
};

TEST_P(GatherSegmentCSRKernelTest, MatchesCpuCsrReference)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  constexpr std::int32_t kNumInputs = 6;
  constexpr std::int32_t kNumChannels = 3;
  const auto reduce = GetParam().second;

  const std::vector<float> features{1.0F, 10.0F, -1.0F, 2.0F, 20.0F, -2.0F, 3.0F, 30.0F, -3.0F,
                                    4.0F, 40.0F, -4.0F, 5.0F, 50.0F, -5.0F, 6.0F, 60.0F, -6.0F};
  const std::vector<float> coords{0.0F, 0.0F, 0.0F, 1.0F, 2.0F, 3.0F,  2.0F, 4.0F,  6.0F,
                                  3.0F, 6.0F, 9.0F, 4.0F, 8.0F, 12.0F, 5.0F, 10.0F, 15.0F};

  // Non-identity ordering is essential: it catches accidental pre-gathered plugin inputs.
  const std::vector<std::int64_t> indices{4, 1, 5, 0, 3, 2};
  const std::vector<std::int64_t> indptr{0, 2, 3, 6};
  const std::int32_t num_segments = static_cast<std::int32_t>(indptr.size() - 1);

  const auto expected_features =
    make_feature_reference(features, indices, indptr, kNumChannels, reduce);
  const auto expected_coords = make_coord_reference(coords, indices, indptr);

  CudaStreamGuard stream;
  DeviceBuffer<float> features_d(features.size());
  DeviceBuffer<float> coords_d(coords.size());
  DeviceBuffer<std::int64_t> indices_d(indices.size());
  DeviceBuffer<std::int64_t> indptr_d(indptr.size());
  DeviceBuffer<float> feature_out_d(expected_features.size());
  DeviceBuffer<float> coord_out_d(expected_coords.size());

  copy_to_device(features_d.get(), features);
  copy_to_device(coords_d.get(), coords);
  copy_to_device(indices_d.get(), indices);
  copy_to_device(indptr_d.get(), indptr);

  ASSERT_EQ(
    gather_segment_csr_float(
      features_d.get(), coords_d.get(), indices_d.get(), indptr_d.get(), feature_out_d.get(),
      coord_out_d.get(), num_segments, kNumChannels, reduce, stream.get()),
    cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  expect_near_vector(
    copy_to_host(feature_out_d.get(), expected_features.size()), expected_features, 1.0e-5F);
  expect_near_vector(
    copy_to_host(coord_out_d.get(), expected_coords.size()), expected_coords, 1.0e-5F);

  (void)kNumInputs;
}

INSTANTIATE_TEST_SUITE_P(
  ReduceModes, GatherSegmentCSRKernelTest,
  ::testing::Values(
    std::make_pair("sum", GatherSegmentCSRReduce::kSum),
    std::make_pair("mean", GatherSegmentCSRReduce::kMean),
    std::make_pair("min", GatherSegmentCSRReduce::kMin),
    std::make_pair("max", GatherSegmentCSRReduce::kMax)));

}  // namespace
