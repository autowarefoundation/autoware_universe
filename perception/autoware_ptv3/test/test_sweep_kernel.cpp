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
#include "autoware/ptv3/preprocess/sweep_kernel.hpp"
#include "ptv3_test_fixture.hpp"

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

constexpr std::size_t kNumFeatures = 5;
constexpr std::uint32_t kThreadsPerBlock = 256;

class SweepKernelTest : public PTv3CudaTest
{
protected:
  // Column-major 4x4 translation by (1, 2, 3).
  static inline const std::vector<float> kTranslationTransform{
    1.0F, 0.0F, 0.0F, 0.0F,  //
    0.0F, 1.0F, 0.0F, 0.0F,  //
    0.0F, 0.0F, 1.0F, 0.0F,  //
    1.0F, 2.0F, 3.0F, 1.0F,  //
  };

  std::vector<float> runSweepFeatures(
    const void * input_points_d, const CloudFormat format, const std::size_t num_points,
    const float time_lag, const float close_radius = 1.0F)
  {
    auto transform_d = makeDeviceBuffer<float>(kTranslationTransform.size());
    copyToDevice(transform_d.get(), kTranslationTransform);
    auto output_d = makeDeviceBuffer<float>(num_points * kNumFeatures);

    generateSweepFeaturesLaunch(
      input_points_d, format, num_points, time_lag, close_radius, transform_d.get(), kNumFeatures,
      output_d.get(), kThreadsPerBlock, stream_);
    EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);

    return copyToHost(output_d.get(), num_points * kNumFeatures);
  }
};

TEST_F(SweepKernelTest, TransformsCoordinatesAndWritesTimeLag)
{
  const std::vector<CloudPointTypeXYZI> points{
    {1.0F, 0.0F, 0.0F, 10.0F},
    {0.0F, 2.0F, 0.0F, 20.0F},
  };
  auto points_d = makeDeviceBuffer<CloudPointTypeXYZI>(points.size());
  copyToDevice(points_d.get(), points);

  const auto output = runSweepFeatures(points_d.get(), CloudFormat::XYZI, points.size(), 0.25F);

  // XYZI carries float intensity, consumed raw.
  const std::vector<float> expected{
    2.0F, 2.0F, 3.0F, 10.0F, 0.25F,  //
    1.0F, 4.0F, 3.0F, 20.0F, 0.25F,  //
  };
  ASSERT_EQ(output.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_FLOAT_EQ(output[i], expected[i]) << "at index " << i;
  }
}

TEST_F(SweepKernelTest, NormalizesEightBitIntensity)
{
  const std::vector<CloudPointTypeXYZIRC> points{
    {1.0F, 0.0F, 0.0F, 255U, 0U, 0U},
    {0.0F, 2.0F, 0.0F, 51U, 0U, 0U},
  };
  auto points_d = makeDeviceBuffer<CloudPointTypeXYZIRC>(points.size());
  copyToDevice(points_d.get(), points);

  const auto output = runSweepFeatures(points_d.get(), CloudFormat::XYZIRC, points.size(), 0.0F);

  // XYZIRC carries 8-bit intensity, normalized to [0, 1] like the training strength.
  EXPECT_FLOAT_EQ(output[3], 1.0F);
  EXPECT_FLOAT_EQ(output[4], 0.0F);
  EXPECT_FLOAT_EQ(output[kNumFeatures + 3], 0.2F);
}

TEST_F(SweepKernelTest, PoisonsEgoGhostsOfPastSweepsInTheirOwnFrame)
{
  const std::vector<CloudPointTypeXYZI> points{
    // Inside the |x|,|y| < 1 box in the sweep's own frame but outside the unit
    // circle: only the training-matching box semantics poison it.
    {0.9F, 0.9F, 0.5F, 10.0F},
    // On the box border, kept and transformed.
    {1.0F, 0.0F, 0.0F, 20.0F},
  };
  auto points_d = makeDeviceBuffer<CloudPointTypeXYZI>(points.size());
  copyToDevice(points_d.get(), points);

  const auto output = runSweepFeatures(points_d.get(), CloudFormat::XYZI, points.size(), 0.25F);

  EXPECT_TRUE(std::isnan(output[0]));
  EXPECT_TRUE(std::isnan(output[1]));
  EXPECT_TRUE(std::isnan(output[2]));
  EXPECT_FLOAT_EQ(output[4], 0.25F);
  EXPECT_FLOAT_EQ(output[kNumFeatures + 0], 2.0F);
  EXPECT_FLOAT_EQ(output[kNumFeatures + 4], 0.25F);
}

TEST_F(SweepKernelTest, KeepsCurrentFramePointsNearTheOrigin)
{
  const std::vector<CloudPointTypeXYZI> points{
    {0.0F, 0.0F, 0.0F, 10.0F},
  };
  auto points_d = makeDeviceBuffer<CloudPointTypeXYZI>(points.size());
  copyToDevice(points_d.get(), points);

  const auto output = runSweepFeatures(points_d.get(), CloudFormat::XYZI, points.size(), 0.0F);

  // The current frame carries lag 0 and is never poisoned, whatever its position.
  EXPECT_FLOAT_EQ(output[0], 1.0F);
  EXPECT_FLOAT_EQ(output[1], 2.0F);
  EXPECT_FLOAT_EQ(output[2], 3.0F);
  EXPECT_FLOAT_EQ(output[4], 0.0F);
}

TEST_F(SweepKernelTest, SkipsEmptyFrames)
{
  generateSweepFeaturesLaunch(
    nullptr, CloudFormat::XYZI, 0, 0.25F, 1.0F, nullptr, kNumFeatures, nullptr, kThreadsPerBlock,
    stream_);

  EXPECT_EQ(cudaStreamSynchronize(stream_), cudaSuccess);
  EXPECT_EQ(cudaGetLastError(), cudaSuccess);
}

}  // namespace test
}  // namespace autoware::ptv3
