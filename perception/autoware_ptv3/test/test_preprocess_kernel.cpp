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
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

// Densified input rows (x, y, z, intensity, time_lag): the current frame (zero lag) forms the
// leading block, sweep rows follow with a positive lag.
constexpr std::size_t kNumFeatures = 5;
constexpr std::size_t kNumCurrentPoints = 5;
// Ego ghosts arrive poisoned by the sweep kernel: NaN coordinates that fail the range crop.
static const float kNaN = std::numeric_limits<float>::quiet_NaN();

class PreprocessKernelTest : public PTv3CudaTest
{
protected:
  static inline const std::vector<float> kDensifiedPoints{
    // current frame
    0.10F,
    0.20F,
    0.30F,
    1.5F,
    0.0F,  //
    0.80F,
    0.20F,
    0.30F,
    2.5F,
    0.0F,  // Same voxel as the previous point.
    1.10F,
    1.20F,
    1.30F,
    3.5F,
    0.0F,  //
    4.00F,
    0.00F,
    0.00F,
    4.5F,
    0.0F,  // Out of range.
    -1.00F,
    -1.00F,
    -1.00F,
    5.5F,
    0.0F,  //
    // sweeps
    kNaN,
    kNaN,
    kNaN,
    0.0F,
    0.1F,  // Ego ghost poisoned by the sweep kernel.
    2.10F,
    2.20F,
    2.30F,
    7.5F,
    0.1F,  //
  };

  // The original current-frame message points matching the leading block above.
  static inline const std::vector<CloudPointTypeXYZI> kCurrentSourcePoints{
    {0.10F, 0.20F, 0.30F, 1.5F}, {0.80F, 0.20F, 0.30F, 2.5F},    {1.10F, 1.20F, 1.30F, 3.5F},
    {4.00F, 0.00F, 0.00F, 4.5F}, {-1.00F, -1.00F, -1.00F, 5.5F},
  };

  struct GenerateVoxelsResult
  {
    CudaUniquePtr<float[]> voxels_d;
    CudaUniquePtr<std::int32_t[]> num_points_per_voxel_d;
    CudaUniquePtr<std::int32_t[]> voxel_coords_d;
    CudaUniquePtr<std::int64_t[]> serialized_code_d;
    CudaUniquePtr<std::int64_t[]> inverse_map_d;
    std::size_t num_cropped_points{};
    std::size_t num_cropped_current_points{};
    std::size_t num_voxels{};
  };

  void TearDown() override
  {
    preprocess_.reset();
    PTv3CudaTest::TearDown();
  }

  void initializePreprocess(const PTv3ConfigParams & params)
  {
    config_.emplace(makeConfig(params));
    preprocess_ = std::make_unique<PreprocessCuda>(*config_, stream_);
  }

  void expectFloatVectorEq(
    const std::vector<float> & actual, const std::vector<float> & expected) const
  {
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

  GenerateVoxelsResult runGenerateVoxels(
    const std::vector<float> & host_points, const std::size_t num_current_points)
  {
    return runGenerateVoxels(PTv3ConfigParams{}, host_points, num_current_points);
  }

  GenerateVoxelsResult runGenerateVoxels(
    const PTv3ConfigParams & params, const std::vector<float> & host_points,
    const std::size_t num_current_points)
  {
    initializePreprocess(params);
    const auto & config = *config_;
    const auto num_points = host_points.size() / kNumFeatures;
    auto input_points_d = makeDeviceBuffer<float>(host_points.size());
    copyToDevice(input_points_d.get(), host_points);

    GenerateVoxelsResult result;
    result.voxels_d =
      makeDeviceBuffer<float>(config.max_num_voxels_ * config.max_points_per_voxel_ * kNumFeatures);
    result.num_points_per_voxel_d = makeDeviceBuffer<std::int32_t>(config.max_num_voxels_);
    result.voxel_coords_d = makeDeviceBuffer<std::int32_t>(config.max_num_voxels_ * 3);
    result.serialized_code_d = makeDeviceBuffer<std::int64_t>(config.max_num_voxels_ * 2);
    result.inverse_map_d = makeDeviceBuffer<std::int64_t>(config.densified_cloud_capacity_);

    result.num_voxels = preprocess_->generateVoxels(
      input_points_d.get(), num_points, num_current_points, result.voxels_d.get(),
      result.num_points_per_voxel_d.get(), result.voxel_coords_d.get(),
      result.serialized_code_d.get(), result.inverse_map_d.get(), &result.num_cropped_points,
      &result.num_cropped_current_points);
    EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
    return result;
  }

  // Returns the (x, y, z, intensity, time_lag) row of one padded voxel slot.
  static std::vector<float> voxelSlot(
    const std::vector<float> & voxels, const std::size_t voxel_idx, const std::size_t slot,
    const std::size_t max_points_per_voxel)
  {
    const auto begin = (voxel_idx * max_points_per_voxel + slot) * kNumFeatures;
    return std::vector<float>(voxels.begin() + begin, voxels.begin() + begin + kNumFeatures);
  }

  std::optional<PTv3Config> config_;
  std::unique_ptr<PreprocessCuda> preprocess_;
};

TEST_F(PreprocessKernelTest, CropDropsOutOfRangePointsAndSweepEgoGhosts)
{
  const auto result = runGenerateVoxels(kDensifiedPoints, kNumCurrentPoints);
  EXPECT_EQ(result.num_cropped_points, 5U);
  EXPECT_EQ(result.num_cropped_current_points, 4U);
  EXPECT_EQ(result.num_voxels, 4U);

  const auto num_points = kDensifiedPoints.size() / kNumFeatures;
  const auto crop_mask = copyToHost(preprocess_->cropMask(), num_points);
  const auto crop_indices = copyToHost(preprocess_->cropIndices(), num_points);
  EXPECT_EQ(crop_mask, (std::vector<std::uint32_t>{1, 1, 1, 0, 1, 0, 1}));
  EXPECT_EQ(crop_indices, (std::vector<std::uint32_t>{1, 2, 3, 3, 4, 4, 5}));
}

TEST_F(PreprocessKernelTest, CroppedFeaturesKeepCurrentFrameAsLeadingBlock)
{
  const auto result = runGenerateVoxels(kDensifiedPoints, kNumCurrentPoints);
  EXPECT_EQ(result.num_cropped_points, 5U);
  EXPECT_EQ(result.num_cropped_current_points, 4U);

  const auto cropped_features =
    copyToHost(preprocess_->croppedFeatures(), result.num_cropped_points * kNumFeatures);
  const std::vector<float> expected_cropped_features{
    0.10F,  0.20F,  0.30F,  1.5F, 0.0F,  //
    0.80F,  0.20F,  0.30F,  2.5F, 0.0F,  //
    1.10F,  1.20F,  1.30F,  3.5F, 0.0F,  //
    -1.00F, -1.00F, -1.00F, 5.5F, 0.0F,  //
    2.10F,  2.20F,  2.30F,  7.5F, 0.1F,  //
  };
  expectFloatVectorEq(cropped_features, expected_cropped_features);
}

TEST_F(PreprocessKernelTest, ExtractCurrentSourcePointsCompactsInRangeCurrentPoints)
{
  const auto result = runGenerateVoxels(kDensifiedPoints, kNumCurrentPoints);
  EXPECT_EQ(result.num_cropped_current_points, 4U);

  auto current_points_d = makeDeviceBuffer<CloudPointTypeXYZI>(kCurrentSourcePoints.size());
  copyToDevice(current_points_d.get(), kCurrentSourcePoints);
  auto cropped_source_points_d = makeDeviceBuffer<CloudPointTypeXYZI>(config_->cloud_capacity_);
  preprocess_->extractCurrentSourcePoints(
    current_points_d.get(), CloudFormat::XYZI, kCurrentSourcePoints.size(),
    cropped_source_points_d.get());
  EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

  const auto cropped_source_points =
    copyToHost(cropped_source_points_d.get(), result.num_cropped_current_points);
  expectPointEq(cropped_source_points[0], kCurrentSourcePoints[0]);
  expectPointEq(cropped_source_points[1], kCurrentSourcePoints[1]);
  expectPointEq(cropped_source_points[2], kCurrentSourcePoints[2]);
  expectPointEq(cropped_source_points[3], kCurrentSourcePoints[4]);
}

TEST_F(PreprocessKernelTest, InverseMapCoversCroppedDensifiedPoints)
{
  const auto result = runGenerateVoxels(kDensifiedPoints, kNumCurrentPoints);
  EXPECT_EQ(result.num_cropped_points, 5U);
  EXPECT_EQ(result.num_voxels, 4U);

  const auto inverse_map = copyToHost(result.inverse_map_d.get(), result.num_cropped_points);
  EXPECT_EQ(inverse_map[0], inverse_map[1]);
  EXPECT_NE(inverse_map[0], inverse_map[2]);
  EXPECT_NE(inverse_map[0], inverse_map[3]);
  EXPECT_NE(inverse_map[2], inverse_map[3]);
  EXPECT_NE(inverse_map[3], inverse_map[4]);
  EXPECT_TRUE(std::all_of(inverse_map.begin(), inverse_map.end(), [&result](const auto value) {
    return value >= 0 && static_cast<std::size_t>(value) < result.num_voxels;
  }));
}

// The voxel shared by two current-frame points holds both rows in cropped order; every other
// voxel holds its single row in slot 0 with zero padding behind it.
TEST_F(PreprocessKernelTest, PaddedVoxelsHoldPointsInCroppedOrder)
{
  const auto result = runGenerateVoxels(kDensifiedPoints, kNumCurrentPoints);
  ASSERT_EQ(result.num_voxels, 4U);
  const auto max_points_per_voxel = static_cast<std::size_t>(config_->max_points_per_voxel_);
  ASSERT_EQ(max_points_per_voxel, 2U);

  const auto voxels =
    copyToHost(result.voxels_d.get(), result.num_voxels * max_points_per_voxel * kNumFeatures);
  const auto num_points_per_voxel =
    copyToHost(result.num_points_per_voxel_d.get(), result.num_voxels);
  const auto inverse_map = copyToHost(result.inverse_map_d.get(), result.num_cropped_points);

  const auto shared_voxel = static_cast<std::size_t>(inverse_map[0]);
  EXPECT_EQ(num_points_per_voxel[shared_voxel], 2);
  expectFloatVectorEq(
    voxelSlot(voxels, shared_voxel, 0, max_points_per_voxel), {0.10F, 0.20F, 0.30F, 1.5F, 0.0F});
  expectFloatVectorEq(
    voxelSlot(voxels, shared_voxel, 1, max_points_per_voxel), {0.80F, 0.20F, 0.30F, 2.5F, 0.0F});

  const std::vector<std::vector<float>> single_rows{
    {1.10F, 1.20F, 1.30F, 3.5F, 0.0F},
    {-1.00F, -1.00F, -1.00F, 5.5F, 0.0F},
    {2.10F, 2.20F, 2.30F, 7.5F, 0.1F},
  };
  for (std::size_t voxel_idx = 0; voxel_idx < result.num_voxels; ++voxel_idx) {
    if (voxel_idx == shared_voxel) {
      continue;
    }
    EXPECT_EQ(num_points_per_voxel[voxel_idx], 1) << "voxel " << voxel_idx;
    const auto first = voxelSlot(voxels, voxel_idx, 0, max_points_per_voxel);
    EXPECT_NE(std::find(single_rows.begin(), single_rows.end(), first), single_rows.end())
      << "voxel " << voxel_idx;
    expectFloatVectorEq(
      voxelSlot(voxels, voxel_idx, 1, max_points_per_voxel), {0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
  }
}

// A voxel with more points than slots keeps its first points (current frame before sweeps) and
// reports the saturated count, like the training-time voxelizer.
TEST_F(PreprocessKernelTest, PaddedVoxelsSaturateAtMaxPointsPerVoxel)
{
  PTv3ConfigParams params;
  params.max_points_per_voxel = 2;
  const std::vector<float> host_points{
    0.10F, 0.10F, 0.10F, 1.0F, 0.0F,  // current
    0.20F, 0.20F, 0.20F, 2.0F, 0.0F,  // current, same voxel
    1.50F, 1.50F, 1.50F, 3.0F, 0.0F,  // current, another voxel
    kNaN,  kNaN,  kNaN,  0.0F, 0.1F,  // sweep: ego ghost poisoned upstream, dropped
    0.90F, 0.90F, 0.90F, 5.0F, 0.1F,  // sweep, first voxel: beyond the two slots
  };

  const auto result = runGenerateVoxels(params, host_points, 3);
  ASSERT_EQ(result.num_cropped_points, 4U);
  ASSERT_EQ(result.num_voxels, 2U);

  const auto inverse_map = copyToHost(result.inverse_map_d.get(), result.num_cropped_points);
  const auto crowded_voxel = static_cast<std::size_t>(inverse_map[0]);
  EXPECT_EQ(inverse_map[3], inverse_map[0]);
  const auto num_points_per_voxel =
    copyToHost(result.num_points_per_voxel_d.get(), result.num_voxels);
  EXPECT_EQ(num_points_per_voxel[crowded_voxel], 2);
  EXPECT_EQ(num_points_per_voxel[1 - crowded_voxel], 1);

  const auto voxels = copyToHost(result.voxels_d.get(), result.num_voxels * 2 * kNumFeatures);
  expectFloatVectorEq(voxelSlot(voxels, crowded_voxel, 0, 2), {0.10F, 0.10F, 0.10F, 1.0F, 0.0F});
  expectFloatVectorEq(voxelSlot(voxels, crowded_voxel, 1, 2), {0.20F, 0.20F, 0.20F, 2.0F, 0.0F});
}

TEST_F(PreprocessKernelTest, VoxelCoordsMatchTheOccupiedCells)
{
  const auto result = runGenerateVoxels(kDensifiedPoints, kNumCurrentPoints);
  EXPECT_EQ(result.num_voxels, 4U);

  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  for (std::size_t voxel_idx = 0; voxel_idx < result.num_voxels; ++voxel_idx) {
    const auto x = voxel_coords[voxel_idx * 3 + 0];
    const auto y = voxel_coords[voxel_idx * 3 + 1];
    const auto z = voxel_coords[voxel_idx * 3 + 2];
    EXPECT_TRUE(
      (x == 0 && y == 0 && z == 0) || (x == 1 && y == 1 && z == 1) ||
      (x == 2 && y == 2 && z == 2) || (x == 3 && y == 3 && z == 3));
  }
}

TEST_F(PreprocessKernelTest, CroppedVoxelCoordsStayInsideGridBounds)
{
  PTv3ConfigParams params;
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 4.0F, 4.0F, 4.0F};
  const std::vector<float> host_points{
    0.000F,  0.000F, 0.000F, 1.0F, 0.0F,  //
    3.999F,  3.999F, 3.999F, 2.0F, 0.0F,  //
    4.000F,  0.000F, 0.000F, 3.0F, 0.0F,  //
    0.000F,  4.000F, 0.000F, 4.0F, 0.0F,  //
    0.000F,  0.000F, 4.000F, 5.0F, 0.0F,  //
    -0.001F, 0.000F, 0.000F, 6.0F, 0.0F,  //
  };

  const auto result = runGenerateVoxels(params, host_points, 6);
  EXPECT_EQ(result.num_cropped_points, 2U);
  EXPECT_EQ(result.num_voxels, 2U);

  const auto crop_mask = copyToHost(preprocess_->cropMask(), 6);
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

// generateSerializedPoolingMetadata requires the order-0 serialized-code ordering produced by
// generateVoxels; this pins that postcondition down at its source.
TEST_F(PreprocessKernelTest, VoxelsAreOrderedByOrder0SerializedCode)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 64;
  params.voxels_num = {1, 32, 64};
  params.point_cloud_range = {0.0F, 0.0F, 0.0F, 8.0F, 8.0F, 8.0F};
  params.voxel_size = {1.0F, 1.0F, 1.0F};

  // Scrambled relative to the serialization curve, with one duplicated voxel, so that neither
  // input order nor deduplication can accidentally satisfy the assertions below.
  const std::vector<float> host_points{
    7.5F, 7.5F, 7.5F, 1.0F, 0.0F,  //
    0.5F, 0.5F, 0.5F, 2.0F, 0.0F,  //
    3.5F, 1.5F, 0.5F, 3.0F, 0.0F,  //
    0.5F, 6.5F, 2.5F, 4.0F, 0.0F,  //
    3.5F, 1.5F, 0.5F, 5.0F, 0.0F,  //
    6.5F, 0.5F, 4.5F, 6.0F, 0.0F,  //
  };

  const auto result = runGenerateVoxels(params, host_points, 6);
  ASSERT_EQ(result.num_cropped_points, 6U);
  ASSERT_EQ(result.num_voxels, 5U);

  const auto depth = config_->serialization_depth_;
  const auto voxel_coords = copyToHost(result.voxel_coords_d.get(), result.num_voxels * 3);
  const auto serialized_code = copyToHost(result.serialized_code_d.get(), result.num_voxels * 2);

  for (std::size_t voxel_idx = 1; voxel_idx < result.num_voxels; ++voxel_idx) {
    EXPECT_LT(serialized_code[voxel_idx - 1], serialized_code[voxel_idx]) << "voxel " << voxel_idx;
  }
  for (std::size_t voxel_idx = 0; voxel_idx < result.num_voxels; ++voxel_idx) {
    const std::int64_t x = voxel_coords[voxel_idx * 3 + 0];
    const std::int64_t y = voxel_coords[voxel_idx * 3 + 1];
    const std::int64_t z = voxel_coords[voxel_idx * 3 + 2];
    EXPECT_EQ(serialized_code[voxel_idx], serialize_coord(x, y, z, depth, false));
    EXPECT_EQ(
      serialized_code[result.num_voxels + voxel_idx], serialize_coord(x, y, z, depth, true));
  }
}

TEST_F(PreprocessKernelTest, UnalignedRangeBoundaryKeepsCornerVoxelsDistinct)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 64;
  params.voxels_num = {1, 32, 64};
  params.point_cloud_range = {0.5F, 0.5F, 0.5F, 16.5F, 16.5F, 16.5F};
  params.voxel_size = {1.0F, 1.0F, 1.0F};
  const std::vector<float> host_points{
    0.6F,  0.6F,  0.6F,  1.0F, 0.0F,  // grid coord (0, 0, 0)
    16.4F, 16.4F, 16.4F, 2.0F, 0.0F,  // grid coord (16, 16, 16): needs a fifth coordinate bit
  };

  const auto result = runGenerateVoxels(params, host_points, 2);
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
  const std::vector<float> host_points{
    -102.4F,   -102.4F,   -102.4F,   1.0F, 0.0F,  // exactly on the min border: grid coord (0, 0, 0)
    below_max, below_max, below_max, 2.0F, 0.0F,  // last cell: grid coord (2047, 2047, 2047)
    102.4F,    102.4F,    102.4F,    3.0F, 0.0F,  // exactly on the max border: cropped
  };

  const auto result = runGenerateVoxels(params, host_points, 3);
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

TEST_F(PreprocessKernelTest, RejectsEmptyOrOversizedCurrentFrame)
{
  initializePreprocess(PTv3ConfigParams{});
  auto input_points_d = makeDeviceBuffer<float>(kDensifiedPoints.size());
  copyToDevice(input_points_d.get(), kDensifiedPoints);
  auto voxels_d = makeDeviceBuffer<float>(
    config_->max_num_voxels_ * config_->max_points_per_voxel_ * kNumFeatures);
  auto num_points_per_voxel_d = makeDeviceBuffer<std::int32_t>(config_->max_num_voxels_);
  auto voxel_coords_d = makeDeviceBuffer<std::int32_t>(config_->max_num_voxels_ * 3);
  auto serialized_code_d = makeDeviceBuffer<std::int64_t>(config_->max_num_voxels_ * 2);
  const auto num_points = kDensifiedPoints.size() / kNumFeatures;
  std::size_t num_cropped_points = 0;
  std::size_t num_cropped_current_points = 0;

  EXPECT_THROW(
    preprocess_->generateVoxels(
      input_points_d.get(), num_points, 0, voxels_d.get(), num_points_per_voxel_d.get(),
      voxel_coords_d.get(), serialized_code_d.get(), nullptr, &num_cropped_points,
      &num_cropped_current_points),
    std::runtime_error);
  EXPECT_THROW(
    preprocess_->generateVoxels(
      input_points_d.get(), num_points, num_points + 1, voxels_d.get(),
      num_points_per_voxel_d.get(), voxel_coords_d.get(), serialized_code_d.get(), nullptr,
      &num_cropped_points, &num_cropped_current_points),
    std::runtime_error);
}

}  // namespace test
}  // namespace autoware::ptv3
