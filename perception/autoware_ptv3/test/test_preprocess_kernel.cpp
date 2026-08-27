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
#include "ptv3_test_fixture.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

class PreprocessKernelTest : public PTv3CudaTest
{
protected:
  static inline const std::vector<CloudPointTypeXYZI> kPartialReconstructionPoints{
    {0.10F, 0.20F, 0.30F, 1.5F},
    {0.80F, 0.20F, 0.30F, 2.5F},  // Same voxel as the previous point.
    {1.10F, 1.20F, 1.30F, 3.5F},
    {4.00F, 0.00F, 0.00F, 4.5F},  // Out of range.
    {-1.00F, -1.00F, -1.00F, 5.5F},
  };

  struct GenerateFeaturesResult
  {
    CudaUniquePtr<float[]> reconstruction_features_d;
    CudaUniquePtr<std::int32_t[]> voxel_coords_d;
    CudaUniquePtr<std::int64_t[]> serialized_code_d;
    CudaUniquePtr<CloudPointTypeXYZI[]> cropped_source_points_d;
    CudaUniquePtr<std::int64_t[]> inverse_map_d;
    std::size_t num_cropped_points{};
    std::size_t num_voxels{};
  };

  void TearDown() override
  {
    preprocess_.reset();
    PTv3CudaTest::TearDown();
  }

  void initializePreprocess(const std::string & source_reconstruction)
  {
    PTv3ConfigParams params;
    params.source_reconstruction = source_reconstruction;
    initializePreprocess(params);
  }

  void initializePreprocess(const PTv3ConfigParams & params)
  {
    config_.emplace(makeConfig(params));
    preprocess_ = std::make_unique<PreprocessCuda>(*config_, stream_);
  }

  void expectFloatVectorEq(
    const std::vector<float> & actual, const std::vector<float> & expected) const
  {
    // NOTE: Since EXPECT_FLOAT_EQ does not have overload for std::vector<float>, this function
    // checks the size and elements one-by-one
    ASSERT_EQ(actual.size(), expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
      EXPECT_FLOAT_EQ(actual[i], expected[i]) << "at index " << i;
    }
  }

  void expectPointEq(const CloudPointTypeXYZI & actual, const CloudPointTypeXYZI & expected) const
  {
    EXPECT_FLOAT_EQ(actual.x, expected.x);
    EXPECT_FLOAT_EQ(actual.y, expected.y);
    EXPECT_FLOAT_EQ(actual.z, expected.z);
    EXPECT_FLOAT_EQ(actual.intensity, expected.intensity);
  }

  GenerateFeaturesResult runGenerateFeatures(
    const std::string & source_reconstruction, const std::vector<CloudPointTypeXYZI> & host_points,
    const bool with_source_outputs)
  {
    PTv3ConfigParams params;
    params.source_reconstruction = source_reconstruction;
    return runGenerateFeatures(params, host_points, with_source_outputs);
  }

  GenerateFeaturesResult runGenerateFeatures(
    const PTv3ConfigParams & params, const std::vector<CloudPointTypeXYZI> & host_points,
    const bool with_source_outputs)
  {
    initializePreprocess(params);
    const auto & config = *config_;

    auto input_points_d = makeDeviceBuffer<CloudPointTypeXYZI>(host_points.size());
    copyToDevice(input_points_d.get(), host_points);

    auto voxel_features_d =
      makeDeviceBuffer<float>(config.cloud_capacity_ * config.num_point_feature_size_);
    auto reconstruction_features_d =
      makeDeviceBuffer<float>(config.cloud_capacity_ * config.num_point_feature_size_);
    auto voxel_coords_d = makeDeviceBuffer<std::int32_t>(config.cloud_capacity_ * 3);
    auto serialized_code_d = makeDeviceBuffer<std::int64_t>(config.cloud_capacity_ * 2);
    auto compact_points_d = makeDeviceBuffer<CloudPointTypeXYZI>(config.cloud_capacity_);
    auto cropped_source_points_d = makeDeviceBuffer<CloudPointTypeXYZI>(config.cloud_capacity_);
    auto inverse_map_d = makeDeviceBuffer<std::int64_t>(config.cloud_capacity_);

    std::size_t num_cropped_points = 0;
    const auto num_voxels = preprocess_->generateFeatures(
      input_points_d.get(), CloudFormat::XYZI, host_points.size(), voxel_features_d.get(),
      voxel_coords_d.get(), serialized_code_d.get(), compact_points_d.get(),
      reconstruction_features_d.get(),
      with_source_outputs ? cropped_source_points_d.get() : nullptr,
      with_source_outputs ? inverse_map_d.get() : nullptr, &num_cropped_points);

    EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

    return GenerateFeaturesResult{
      std::move(reconstruction_features_d),
      std::move(voxel_coords_d),
      std::move(serialized_code_d),
      std::move(cropped_source_points_d),
      std::move(inverse_map_d),
      num_cropped_points,
      num_voxels};
  }

  cudaStream_t stream_{nullptr};
  std::optional<PTv3Config> config_;
  std::unique_ptr<PreprocessCuda> preprocess_;
};

TEST_F(PreprocessKernelTest, PartialReconstructionBuildsCropMaskAndIndices)
{
  const auto result = runGenerateFeatures("partial", kPartialReconstructionPoints, true);
  EXPECT_EQ(result.num_cropped_points, 4U);
  EXPECT_EQ(result.num_voxels, 3U);

  const auto crop_mask = copyToHost(preprocess_->cropMask(), kPartialReconstructionPoints.size());
  const auto crop_indices =
    copyToHost(preprocess_->cropIndices(), kPartialReconstructionPoints.size());
  EXPECT_EQ(crop_mask, (std::vector<std::uint32_t>{1, 1, 1, 0, 1}));
  EXPECT_EQ(crop_indices, (std::vector<std::uint32_t>{1, 2, 3, 3, 4}));
}

TEST_F(PreprocessKernelTest, PartialReconstructionKeepsCroppedFeatures)
{
  const auto result = runGenerateFeatures("partial", kPartialReconstructionPoints, true);
  EXPECT_EQ(result.num_cropped_points, 4U);

  const auto reconstruction_features =
    copyToHost(result.reconstruction_features_d.get(), result.num_cropped_points * 4);
  const std::vector<float> expected_reconstruction_features{
    0.10F, 0.20F, 0.30F, 1.5F, 0.80F,  0.20F,  0.30F,  2.5F,
    1.10F, 1.20F, 1.30F, 3.5F, -1.00F, -1.00F, -1.00F, 5.5F};
  expectFloatVectorEq(reconstruction_features, expected_reconstruction_features);
}

TEST_F(PreprocessKernelTest, PartialReconstructionStoresCroppedSourcePoints)
{
  const auto result = runGenerateFeatures("partial", kPartialReconstructionPoints, true);
  EXPECT_EQ(result.num_cropped_points, 4U);

  const auto cropped_source_points =
    copyToHost(result.cropped_source_points_d.get(), result.num_cropped_points);
  expectPointEq(cropped_source_points[0], kPartialReconstructionPoints[0]);
  expectPointEq(cropped_source_points[1], kPartialReconstructionPoints[1]);
  expectPointEq(cropped_source_points[2], kPartialReconstructionPoints[2]);
  expectPointEq(cropped_source_points[3], kPartialReconstructionPoints[4]);
}

TEST_F(PreprocessKernelTest, PartialReconstructionBuildsInverseMap)
{
  const auto result = runGenerateFeatures("partial", kPartialReconstructionPoints, true);
  EXPECT_EQ(result.num_cropped_points, 4U);
  EXPECT_EQ(result.num_voxels, 3U);

  const auto inverse_map = copyToHost(result.inverse_map_d.get(), result.num_cropped_points);
  EXPECT_EQ(inverse_map[0], inverse_map[1]);
  EXPECT_NE(inverse_map[0], inverse_map[2]);
  EXPECT_NE(inverse_map[0], inverse_map[3]);
  EXPECT_NE(inverse_map[2], inverse_map[3]);
  EXPECT_TRUE(std::all_of(inverse_map.begin(), inverse_map.end(), [&result](const auto value) {
    return value >= 0 && static_cast<std::size_t>(value) < result.num_voxels;
  }));
}

TEST_F(PreprocessKernelTest, PartialReconstructionBuildsVoxelCoords)
{
  const auto result = runGenerateFeatures("partial", kPartialReconstructionPoints, true);
  EXPECT_EQ(result.num_voxels, 3U);

  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  for (std::size_t voxel_idx = 0; voxel_idx < result.num_voxels; ++voxel_idx) {
    const auto x = voxel_coords[voxel_idx * 3 + 0];
    const auto y = voxel_coords[voxel_idx * 3 + 1];
    const auto z = voxel_coords[voxel_idx * 3 + 2];
    EXPECT_TRUE(
      (x == 0 && y == 0 && z == 0) || (x == 1 && y == 1 && z == 1) || (x == 2 && y == 2 && z == 2));
  }
}

TEST_F(PreprocessKernelTest, CroppedVoxelCoordsStayInsideGridBounds)
{
  PTv3ConfigParams params;
  params.source_reconstruction = "partial";
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 4.0F, 4.0F, 4.0F};

  const std::vector<CloudPointTypeXYZI> host_points{
    {0.0F, 0.0F, 0.0F, 1.0F}, {3.999F, 3.999F, 3.999F, 2.0F}, {4.0F, 0.0F, 0.0F, 3.0F},
    {0.0F, 4.0F, 0.0F, 4.0F}, {0.0F, 0.0F, 4.0F, 5.0F},       {-0.001F, 0.0F, 0.0F, 6.0F},
  };

  const auto result = runGenerateFeatures(params, host_points, true);
  EXPECT_EQ(result.num_cropped_points, 2U);
  EXPECT_EQ(result.num_voxels, 2U);

  const auto crop_mask = copyToHost(preprocess_->cropMask(), host_points.size());
  EXPECT_EQ(crop_mask, (std::vector<std::uint32_t>{1, 1, 0, 0, 0, 0}));

  const auto & config = *config_;
  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  for (std::size_t voxel_idx = 0; voxel_idx < result.num_voxels; ++voxel_idx) {
    const auto x = voxel_coords[voxel_idx * 3 + 0];
    const auto y = voxel_coords[voxel_idx * 3 + 1];
    const auto z = voxel_coords[voxel_idx * 3 + 2];
    EXPECT_GE(x, 0);
    EXPECT_GE(y, 0);
    EXPECT_GE(z, 0);
    EXPECT_LT(x, config.grid_x_size_);
    EXPECT_LT(y, config.grid_y_size_);
    EXPECT_LT(z, config.grid_z_size_);
  }
}

// generateSerializedPoolingMetadata requires voxels ordered by their order-0 serialized code; this
// pins that contract of generateFeatures down at its source.
TEST_F(PreprocessKernelTest, VoxelsAreOrderedByOrder0SerializedCode)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 64;
  params.voxels_num = {1, 32, 64};
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 8.0F, 8.0F, 8.0F};
  params.voxel_size = {1.0F, 1.0F, 1.0F};

  // Scrambled relative to the serialization curve, with one duplicated voxel, so that neither
  // input order nor deduplication can accidentally satisfy the assertions below.
  const std::vector<CloudPointTypeXYZI> host_points{
    {7.5F, 7.5F, 7.5F, 1.0F}, {0.5F, 0.5F, 0.5F, 2.0F}, {3.5F, 1.5F, 0.5F, 3.0F},
    {0.5F, 6.5F, 2.5F, 4.0F}, {3.5F, 1.5F, 0.5F, 5.0F}, {6.5F, 0.5F, 4.5F, 6.0F},
  };

  const auto result = runGenerateFeatures(params, host_points, true);
  ASSERT_EQ(result.num_cropped_points, 6U);
  ASSERT_EQ(result.num_voxels, 5U);

  const auto depth = config_->serialization_depth_;
  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  const auto serialized_code = copyToHost(result.serialized_code_d.get(), result.num_voxels * 2);

  for (std::size_t voxel_idx = 1; voxel_idx < result.num_voxels; ++voxel_idx) {
    EXPECT_LT(serialized_code[voxel_idx - 1], serialized_code[voxel_idx])
      << "at voxel " << voxel_idx;
  }

  // The codes must describe the grid coordinates that were actually emitted.
  for (std::size_t voxel_idx = 0; voxel_idx < result.num_voxels; ++voxel_idx) {
    const auto x = voxel_coords[voxel_idx * 3 + 0];
    const auto y = voxel_coords[voxel_idx * 3 + 1];
    const auto z = voxel_coords[voxel_idx * 3 + 2];
    EXPECT_EQ(serialized_code[voxel_idx], serialize_coord(x, y, z, depth, false))
      << "order 0 at voxel " << voxel_idx;
    EXPECT_EQ(serialized_code[result.num_voxels + voxel_idx], serialize_coord(x, y, z, depth, true))
      << "order 1 at voxel " << voxel_idx;
  }
}

// End-to-end check of the unaligned-boundary depth handling: the corner voxel (16, 16, 16) needs
// a fifth coordinate bit, without which it would deduplicate into (0, 0, 0).
TEST_F(PreprocessKernelTest, UnalignedRangeBoundaryKeepsCornerVoxelsDistinct)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 64;
  params.voxels_num = {1, 32, 64};
  params.point_cloud_range = {0.5F, 0.5F, 0.5F, 16.5F, 16.5F, 16.5F};
  params.voxel_size = {1.0F, 1.0F, 1.0F};

  const std::vector<CloudPointTypeXYZI> host_points{
    {0.6F, 0.6F, 0.6F, 1.0F},     // grid coord (0, 0, 0)
    {16.4F, 16.4F, 16.4F, 2.0F},  // grid coord (16, 16, 16): needs a fifth coordinate bit
  };

  const auto result = runGenerateFeatures(params, host_points, true);
  ASSERT_EQ(result.num_cropped_points, 2U);
  ASSERT_EQ(result.num_voxels, 2U);

  const auto depth = config_->serialization_depth_;
  EXPECT_EQ(depth, 5);

  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  const auto serialized_code = copyToHost(result.serialized_code_d.get(), result.num_voxels * 2);
  EXPECT_EQ(voxel_coords, (std::vector<std::int32_t>{0, 0, 0, 16, 16, 16}));
  EXPECT_EQ(serialized_code[0], serialize_coord(0, 0, 0, depth, false));
  EXPECT_EQ(serialized_code[1], serialize_coord(16, 16, 16, depth, false));
}

// Decimal-aligned borders (neither 102.4 nor 0.1 is a binary float) rely on the config using the
// same float arithmetic as the device mapping: depth stays 11, a point one ULP below the border
// lands in the last cell, and a point exactly on the border is cropped.
TEST_F(PreprocessKernelTest, Base10AlignedBordersStayWithinSerializationDepth)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 64;
  params.voxels_num = {1, 32, 64};
  params.point_cloud_range = {-102.4F, -102.4F, -102.4F, 102.4F, 102.4F, 102.4F};
  params.voxel_size = {0.1F, 0.1F, 0.1F};

  const float below_max = std::nextafter(102.4F, 0.0F);
  const std::vector<CloudPointTypeXYZI> host_points{
    {-102.4F, -102.4F, -102.4F, 1.0F},        // exactly on the min border: grid coord (0, 0, 0)
    {below_max, below_max, below_max, 2.0F},  // last cell: grid coord (2047, 2047, 2047)
    {102.4F, 102.4F, 102.4F, 3.0F},           // exactly on the max border: cropped
  };

  const auto result = runGenerateFeatures(params, host_points, true);
  ASSERT_EQ(result.num_cropped_points, 2U);
  ASSERT_EQ(result.num_voxels, 2U);

  const auto depth = config_->serialization_depth_;
  EXPECT_EQ(depth, 11);

  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  const auto serialized_code = copyToHost(result.serialized_code_d.get(), result.num_voxels * 2);
  EXPECT_EQ(voxel_coords, (std::vector<std::int32_t>{0, 0, 0, 2047, 2047, 2047}));
  EXPECT_EQ(serialized_code[0], serialize_coord(0, 0, 0, depth, false));
  EXPECT_EQ(serialized_code[1], serialize_coord(2047, 2047, 2047, depth, false));
}

TEST_F(PreprocessKernelTest, FullReconstructionKeepsAllInputFeaturesBeforeCrop)
{
  const std::vector<CloudPointTypeXYZI> host_points{
    {0.0F, 0.0F, 0.0F, 1.0F},
    {4.0F, 0.0F, 0.0F, 2.0F},  // Out of range
    {1.0F, 1.0F, 1.0F, 3.0F},
  };

  const auto result = runGenerateFeatures("full", host_points, false);
  EXPECT_EQ(result.num_cropped_points, 2U);
  EXPECT_EQ(result.num_voxels, 2U);

  const auto reconstruction_features =
    copyToHost(result.reconstruction_features_d.get(), host_points.size() * 4);
  const std::vector<float> expected_reconstruction_features{0.0F, 0.0F, 0.0F, 1.0F, 4.0F, 0.0F,
                                                            0.0F, 2.0F, 1.0F, 1.0F, 1.0F, 3.0F};
  expectFloatVectorEq(reconstruction_features, expected_reconstruction_features);
}

}  // namespace test
}  // namespace autoware::ptv3
