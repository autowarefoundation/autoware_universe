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

#include "autoware/scatter_ops/segment_csr.h"
#include "test_utils.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

namespace
{

using autoware::tensorrt_plugins::test::copy_to_device;
using autoware::tensorrt_plugins::test::copy_to_host;
using autoware::tensorrt_plugins::test::CudaStreamGuard;
using autoware::tensorrt_plugins::test::DeviceBuffer;

struct SegmentCsrCase
{
  std::string name;
  std::size_t rows;
  std::size_t cols;
  std::vector<float> src;
  std::vector<std::int64_t> indptr;
};

std::vector<float> make_segment_reference_mean(
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

std::vector<float> make_segment_reference_max(
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

void expect_float_vectors_equal(
  const std::vector<float> & actual, const std::vector<float> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t index = 0; index < actual.size(); ++index) {
    EXPECT_FLOAT_EQ(actual[index], expected[index]) << "index=" << index;
  }
}

template <ReductionType REDUCE>
void run_segment_csr_case(const SegmentCsrCase & test_case, const std::vector<float> & reference)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  CudaStreamGuard stream;
  DeviceBuffer<float> src_d(std::max<std::size_t>(test_case.src.size(), 1U));
  DeviceBuffer<std::int64_t> indptr_d(test_case.indptr.size());
  DeviceBuffer<float> out_d(std::max<std::size_t>(reference.size(), 1U));
  const std::vector<std::int32_t> src_size{
    static_cast<std::int32_t>(test_case.rows), static_cast<std::int32_t>(test_case.cols)};
  const std::vector<std::int32_t> indptr_size{static_cast<std::int32_t>(test_case.indptr.size())};

  if (!test_case.src.empty()) {
    copy_to_device(src_d.get(), test_case.src);
  }
  copy_to_device(indptr_d.get(), test_case.indptr);

  ASSERT_EQ(
    (segment_csr_launch<float, REDUCE>(
      src_d.get(), src_size, indptr_d.get(), indptr_size,
      std::make_tuple(out_d.get(), static_cast<std::int64_t *>(nullptr)), stream.get())),
    0);
  ASSERT_EQ(cudaStreamSynchronize(stream.get()), cudaSuccess);

  if (!reference.empty()) {
    expect_float_vectors_equal(copy_to_host(out_d.get(), reference.size()), reference);
  }
}

class SegmentCsrTest : public ::testing::TestWithParam<SegmentCsrCase>
{
};

TEST_P(SegmentCsrTest, MeanMatchesCpuReference)
{
  const auto & test_case = GetParam();
  const auto reference =
    make_segment_reference_mean(test_case.src, test_case.rows, test_case.cols, test_case.indptr);
  run_segment_csr_case<MEAN>(test_case, reference);
}

TEST_P(SegmentCsrTest, MaxMatchesCpuReference)
{
  const auto & test_case = GetParam();
  const auto reference =
    make_segment_reference_max(test_case.src, test_case.rows, test_case.cols, test_case.indptr);
  run_segment_csr_case<MAX>(test_case, reference);
}

std::string segment_csr_case_name(const testing::TestParamInfo<SegmentCsrCase> & info)
{
  return info.param.name;
}

INSTANTIATE_TEST_SUITE_P(
  ReferenceKernelsTest, SegmentCsrTest,
  testing::Values(
    SegmentCsrCase{
      "NonEmptySegments",
      6U,
      2U,
      {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F},
      {0, 2, 5, 6}},
    SegmentCsrCase{
      "EmptyMiddleSegment",
      6U,
      2U,
      {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F},
      {0, 2, 2, 5, 6}},
    SegmentCsrCase{
      "EmptyBoundarySegments",
      6U,
      2U,
      {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F},
      {0, 0, 3, 6, 6}},
    SegmentCsrCase{
      "NoSegmentsWithRows",
      6U,
      2U,
      {1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F},
      {0}},
    SegmentCsrCase{"EmptyInputNoSegments", 0U, 2U, {}, {0}},
    SegmentCsrCase{"EmptyInputEmptySegments", 0U, 2U, {}, {0, 0, 0}},
    SegmentCsrCase{"ZeroColumns", 3U, 0U, {}, {0, 1, 3}}),
  segment_csr_case_name);

}  // namespace
