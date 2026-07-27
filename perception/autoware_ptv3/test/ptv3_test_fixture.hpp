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

#ifndef PTV3_TEST_FIXTURE_HPP_
#define PTV3_TEST_FIXTURE_HPP_

#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>
#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace test
{
using autoware::cuda_utils::CudaUniquePtr;

// Shared PTv3Config defaults; each test overrides only the fields it needs.
struct PTv3ConfigParams
{
  bool use_seg3d_head = true;
  bool use_det3d_head = false;
  std::string plugins_path = "";
  std::int64_t cloud_capacity = 8;
  std::vector<std::int64_t> voxels_num = {1, 4, 8};
  std::vector<float> point_cloud_range = {-1.0F, -1.0F, -1.0F, 3.0F, 3.0F, 3.0F};
  std::vector<float> voxel_size = {1.0F, 1.0F, 1.0F};
  std::vector<std::string> segmentation_class_names = {"background", "car"};
  std::vector<std::string> serialization_orders = {"z", "z-trans"};
  std::vector<std::int64_t> pooling_strides = {2, 2};
  std::vector<std::int64_t> enc_channels = {8, 16, 32};
  std::vector<std::int64_t> palette = {0, 0, 0, 255, 0, 0};
  float filter_class_probability_threshold = 0.5F;
  std::vector<std::string> filter_classes = {};
  std::string filter_output_format = "xyzi";
  std::string source_reconstruction = "partial";
  std::vector<std::int64_t> dec_depths = {0, 0};
};

inline PTv3Config makeConfig(const PTv3ConfigParams & params = {})
{
  return PTv3Config(
    params.use_seg3d_head, params.use_det3d_head, params.plugins_path, params.cloud_capacity,
    params.voxels_num, params.point_cloud_range, params.voxel_size, params.segmentation_class_names,
    params.serialization_orders, params.pooling_strides, params.enc_channels, params.palette,
    params.filter_class_probability_threshold, params.filter_classes, params.filter_output_format,
    params.source_reconstruction, params.dec_depths);
}

// Base fixture for all autoware_ptv3 CUDA unit tests: owns a CUDA stream and the
// device-buffer / host<->device copy operations shared across kernels.
// Inherits cuda_utils::CudaTest so the suite is skipped when CUDA is unavailable.
class PTv3CudaTest : public autoware::cuda_utils::CudaTest
{
protected:
  void SetUp() override
  {
    SKIP_TEST_IF_CUDA_UNAVAILABLE();  // returns before stream creation when no CUDA
    ASSERT_EQ(cudaStreamCreate(&stream_), cudaSuccess);
  }

  void TearDown() override
  {
    if (stream_ != nullptr) {
      EXPECT_EQ(cudaStreamDestroy(stream_), cudaSuccess);
    }
  }

  // Allocate a device buffer of `count` elements. Delegates to cuda_utils::make_unique,
  // which owns its own error checking, so no assertion is added here.
  template <typename T>
  CudaUniquePtr<T[]> makeDeviceBuffer(const std::size_t count)
  {
    return autoware::cuda_utils::make_unique<T[]>(count);
  }

  // Host<->device copies. No assertion: cudaMemcpy correctness is covered by the
  // cuda_utils package, per review discussion.
  template <typename T>
  void copyToDevice(T * device, const std::vector<T> & host)
  {
    cudaMemcpy(device, host.data(), host.size() * sizeof(T), cudaMemcpyHostToDevice);
  }

  template <typename T>
  std::vector<T> copyToHost(const T * device, const std::size_t count)
  {
    std::vector<T> host(count);
    cudaMemcpy(host.data(), device, count * sizeof(T), cudaMemcpyDeviceToHost);
    return host;
  }

  cudaStream_t stream_{nullptr};
};

}  // namespace test
}  // namespace autoware::ptv3

#endif  // PTV3_TEST_FIXTURE_HPP_
