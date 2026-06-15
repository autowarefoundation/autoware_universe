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

#include "autoware/ptv3/preprocess/point_type.hpp"
#include "autoware/ptv3/preprocess/preprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"

#include <autoware/cuda_utils/cuda_gtest_utils.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

template <typename T>
struct CudaDeleter
{
  void operator()(T * ptr) const
  {
    if (ptr != nullptr) {
      cudaFree(ptr);
    }
  }
};

template <typename T>
using CudaPtr = std::unique_ptr<T, CudaDeleter<T>>;

template <typename T>
CudaPtr<T> make_device_buffer(const std::size_t count)
{
  T * ptr = nullptr;
  EXPECT_EQ(cudaMalloc(reinterpret_cast<void **>(&ptr), count * sizeof(T)), cudaSuccess);
  return CudaPtr<T>(ptr);
}

template <typename T>
void copy_to_device(T * device, const std::vector<T> & host)
{
  ASSERT_EQ(
    cudaMemcpy(device, host.data(), host.size() * sizeof(T), cudaMemcpyHostToDevice), cudaSuccess);
}

template <typename T>
std::vector<T> copy_to_host(const T * device, const std::size_t count)
{
  std::vector<T> host(count);
  EXPECT_EQ(
    cudaMemcpy(host.data(), device, count * sizeof(T), cudaMemcpyDeviceToHost), cudaSuccess);
  return host;
}

PTv3Config make_config(const std::string & source_reconstruction = "partial")
{
  return PTv3Config(
    /* plugins_path */ "",
    /* cloud_capacity */ 8,
    /* voxel_size [min, opt, max]*/ {1, 4, 8},
    /* point_cloud_range [x_min, y_min, z_min, x_max, y_max, z_max] */
    {-1.0F, -1.0F, -1.0F, 3.0F, 3.0F, 3.0F},
    /*voxel_size [x, y, z]*/ {1.0F, 1.0F, 1.0F},
    /* class_name */ {"background", "car"},
    /* serialization_orders */ {"z", "z-trans"},
    /* pooling_strides */ {2, 2},
    /* palette  */ {0, 0, 0, 255, 0, 0},
    /* filter_class_probability_threshold  */ 0.5F,
    /* filter_classes  */ {},
    /* filter_output_format */ "xyzi",
    /* source_reconstruction */ source_reconstruction,
    /* use_seg3d_head */ true);
}

TEST(PreprocessKernelTest, GenerateFeaturesCropsVoxelsAndBuildsInverseMap)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  cudaStream_t stream{};
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);

  const auto config = make_config("partial");
  PreprocessCuda preprocess(config, stream);

  const std::vector<CloudPointTypeXYZI> host_points{
    {0.10F, 0.20F, 0.30F, 1.5F},
    {0.80F, 0.20F, 0.30F, 2.5F},  // Same voxel as the previous point.
    {1.10F, 1.20F, 1.30F, 3.5F},
    {4.00F, 0.00F, 0.00F, 4.5F},  // Out of range.
    {-1.00F, -1.00F, -1.00F, 5.5F},
  };

  auto input_points_d = make_device_buffer<CloudPointTypeXYZI>(host_points.size());
  copy_to_device(input_points_d.get(), host_points);

  auto voxel_features_d =
    make_device_buffer<float>(config.cloud_capacity_ * config.num_point_feature_size_);
  auto reconstruction_features_d =
    make_device_buffer<float>(config.cloud_capacity_ * config.num_point_feature_size_);
  auto voxel_coords_d = make_device_buffer<std::int32_t>(config.cloud_capacity_ * 3);
  auto voxel_hashes_d = make_device_buffer<std::int64_t>(config.cloud_capacity_ * 2);
  auto compact_points_d = make_device_buffer<CloudPointTypeXYZI>(config.cloud_capacity_);
  auto cropped_source_points_d = make_device_buffer<CloudPointTypeXYZI>(config.cloud_capacity_);
  auto inverse_map_d = make_device_buffer<std::int64_t>(config.cloud_capacity_);

  std::size_t num_cropped_points = 0;
  const auto num_voxels = preprocess.generateFeatures(
    input_points_d.get(), CloudFormat::XYZI, host_points.size(), voxel_features_d.get(),
    voxel_coords_d.get(), voxel_hashes_d.get(), compact_points_d.get(),
    reconstruction_features_d.get(), cropped_source_points_d.get(), inverse_map_d.get(),
    &num_cropped_points);

  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  EXPECT_EQ(num_cropped_points, 4U);
  EXPECT_EQ(num_voxels, 3U);

  const auto crop_mask = copy_to_host(preprocess.cropMask(), host_points.size());
  const auto crop_indices = copy_to_host(preprocess.cropIndices(), host_points.size());
  EXPECT_EQ(crop_mask, (std::vector<std::uint32_t>{1, 1, 1, 0, 1}));
  EXPECT_EQ(crop_indices, (std::vector<std::uint32_t>{1, 2, 3, 3, 4}));

  const auto reconstruction_features =
    copy_to_host(reconstruction_features_d.get(), num_cropped_points * 4);
  const std::vector<float> expected_reconstruction_features{
    0.10F, 0.20F, 0.30F, 1.5F, 0.80F,  0.20F,  0.30F,  2.5F,
    1.10F, 1.20F, 1.30F, 3.5F, -1.00F, -1.00F, -1.00F, 5.5F};
  EXPECT_EQ(reconstruction_features, expected_reconstruction_features);

  const auto cropped_source_points =
    copy_to_host(cropped_source_points_d.get(), num_cropped_points);
  auto is_xyzi_identical = [](auto & p1, auto & p2) {
    EXPECT_EQ(p1.x, p2.x);
    EXPECT_EQ(p1.y, p2.y);
    EXPECT_EQ(p1.z, p2.z);
    EXPECT_EQ(p1.intensity, p2.intensity);
  };
  is_xyzi_identical(cropped_source_points[0], host_points[0]);
  is_xyzi_identical(cropped_source_points[1], host_points[1]);
  is_xyzi_identical(cropped_source_points[2], host_points[2]);
  is_xyzi_identical(cropped_source_points[3], host_points[4]);

  const auto inverse_map = copy_to_host(inverse_map_d.get(), num_cropped_points);
  EXPECT_EQ(inverse_map[0], inverse_map[1]);
  EXPECT_NE(inverse_map[0], inverse_map[2]);
  EXPECT_NE(inverse_map[0], inverse_map[3]);
  EXPECT_NE(inverse_map[2], inverse_map[3]);
  EXPECT_TRUE(std::all_of(inverse_map.begin(), inverse_map.end(), [num_voxels](const auto value) {
    return value >= 0 && static_cast<std::size_t>(value) < num_voxels;
  }));

  const auto voxel_coords = copy_to_host(voxel_coords_d.get(), num_voxels * 3);
  for (std::size_t voxel_idx = 0; voxel_idx < num_voxels; ++voxel_idx) {
    const auto x = voxel_coords[voxel_idx * 3 + 0];
    const auto y = voxel_coords[voxel_idx * 3 + 1];
    const auto z = voxel_coords[voxel_idx * 3 + 2];
    EXPECT_TRUE(
      (x == 0 && y == 0 && z == 0) || (x == 1 && y == 1 && z == 1) || (x == 2 && y == 2 && z == 2));
  }

  ASSERT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}

TEST(PreprocessKernelTest, FullReconstructionKeepsAllInputFeaturesBeforeCrop)
{
  SKIP_TEST_IF_CUDA_UNAVAILABLE();

  cudaStream_t stream{};
  ASSERT_EQ(cudaStreamCreate(&stream), cudaSuccess);

  const auto config = make_config("full");
  PreprocessCuda preprocess(config, stream);

  const std::vector<CloudPointTypeXYZI> host_points{
    {0.0F, 0.0F, 0.0F, 1.0F},
    {4.0F, 0.0F, 0.0F, 2.0F},  // Out of range
    {1.0F, 1.0F, 1.0F, 3.0F},
  };

  auto input_points_d = make_device_buffer<CloudPointTypeXYZI>(host_points.size());
  copy_to_device(input_points_d.get(), host_points);

  auto voxel_features_d =
    make_device_buffer<float>(config.cloud_capacity_ * config.num_point_feature_size_);
  auto reconstruction_features_d =
    make_device_buffer<float>(config.cloud_capacity_ * config.num_point_feature_size_);
  auto voxel_coords_d = make_device_buffer<std::int32_t>(config.cloud_capacity_ * 3);
  auto voxel_hashes_d = make_device_buffer<std::int64_t>(config.cloud_capacity_ * 2);
  auto compact_points_d = make_device_buffer<CloudPointTypeXYZI>(config.cloud_capacity_);

  std::size_t num_cropped_points = 0;
  const auto num_voxels = preprocess.generateFeatures(
    input_points_d.get(), CloudFormat::XYZI, host_points.size(), voxel_features_d.get(),
    voxel_coords_d.get(), voxel_hashes_d.get(), compact_points_d.get(),
    reconstruction_features_d.get(), nullptr, nullptr, &num_cropped_points);

  ASSERT_EQ(cudaStreamSynchronize(stream), cudaSuccess);
  EXPECT_EQ(num_cropped_points, 2U);
  EXPECT_EQ(num_voxels, 2U);

  const auto reconstruction_features =
    copy_to_host(reconstruction_features_d.get(), host_points.size() * 4);
  const std::vector<float> expected_reconstruction_features{0.0F, 0.0F, 0.0F, 1.0F, 4.0F, 0.0F,
                                                            0.0F, 2.0F, 1.0F, 1.0F, 1.0F, 3.0F};
  EXPECT_EQ(reconstruction_features, expected_reconstruction_features);

  ASSERT_EQ(cudaStreamDestroy(stream), cudaSuccess);
}

}  // namespace test
}  // namespace autoware::ptv3
