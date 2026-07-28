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

#include "autoware/ptv3/postprocess/postprocess_kernel.hpp"
#include "autoware/ptv3/ptv3_config.hpp"
#include "ptv3_test_fixture.hpp"

#include <autoware/cuda_utils/cuda_unique_ptr.hpp>
#include <autoware/point_types/types.hpp>

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

class PostprocessKernelTest : public PTv3CudaTest
{
protected:
  using PointCloudClassification = autoware::point_types::PointCloudClassification;

  static constexpr std::size_t kNumClasses = 2;

  PTv3Config makeSegmentationConfig() const
  {
    PTv3ConfigParams params;
    params.segmentation_class_names = {"car", "vegetation"};
    params.palette = {255, 0, 0, 0, 255, 0};
    return makeConfig(params);
  }

  // Normalized Shannon entropy expected from the kernel for a two-class distribution.
  static float expectedEntropy(const float p0, const float p1)
  {
    float entropy = 0.0F;
    for (const auto probability : {p0, p1}) {
      if (probability > 0.0F) {
        entropy -= probability * std::log(probability);
      }
    }
    return entropy / std::log(static_cast<float>(kNumClasses));
  }
};

TEST_F(PostprocessKernelTest, CreateSegmentationPointcloud)
{
  const auto config = makeSegmentationConfig();
  PostprocessCuda postprocess(config, stream_);

  // Each feature is a float4; only the leading xyz components are copied to the output point.
  const std::vector<float> features{
    0.0F, 1.0F,  2.0F,  0.0F,  // point 0: confident car
    3.0F, 4.0F,  5.0F,  0.0F,  // point 1: ambiguous vegetation
    6.0F, 7.0F,  8.0F,  0.0F,  // point 2: negative (invalid) label
    9.0F, 10.0F, 11.0F, 0.0F   // point 3: out-of-range (invalid) label
  };
  const std::vector<std::int64_t> labels{0, 1, -1, static_cast<std::int64_t>(kNumClasses)};
  const std::vector<float> probs{
    0.9F, 0.1F,  // point 0
    0.4F, 0.6F,  // point 1
    0.3F, 0.7F,  // point 2: ignored because the label is invalid
    0.2F, 0.8F   // point 3: ignored because the label is invalid
  };
  const auto num_points = labels.size();

  auto features_d = makeDeviceBuffer<float>(features.size());
  auto labels_d = makeDeviceBuffer<std::int64_t>(labels.size());
  auto probs_d = makeDeviceBuffer<float>(probs.size());
  auto output_points_d = makeDeviceBuffer<point_types::PointXYZCPE>(num_points);
  copyToDevice(features_d.get(), features);
  copyToDevice(labels_d.get(), labels);
  copyToDevice(probs_d.get(), probs);

  postprocess.createSegmentationPointcloud(
    features_d.get(), labels_d.get(), probs_d.get(), output_points_d.get(), kNumClasses,
    num_points);

  const auto output_points = copyToHost(output_points_d.get(), num_points);

  for (std::size_t i = 0; i < num_points; ++i) {
    EXPECT_FLOAT_EQ(output_points[i].x, features[i * 4]);
    EXPECT_FLOAT_EQ(output_points[i].y, features[i * 4 + 1]);
    EXPECT_FLOAT_EQ(output_points[i].z, features[i * 4 + 2]);
  }

  EXPECT_EQ(output_points[0].class_id, static_cast<std::uint8_t>(PointCloudClassification::CAR));
  EXPECT_FLOAT_EQ(output_points[0].probability, 0.9F);
  EXPECT_FLOAT_EQ(output_points[0].entropy, expectedEntropy(0.9F, 0.1F));

  EXPECT_EQ(
    output_points[1].class_id, static_cast<std::uint8_t>(PointCloudClassification::VEGETATION));
  EXPECT_FLOAT_EQ(output_points[1].probability, 0.6F);
  EXPECT_FLOAT_EQ(output_points[1].entropy, expectedEntropy(0.4F, 0.6F));

  // Invalid labels carry no class distribution, so the entropy keeps the NaN default of
  // point_types::PointXYZCPE.
  for (const std::size_t i : {std::size_t{2}, std::size_t{3}}) {
    EXPECT_EQ(
      output_points[i].class_id, static_cast<std::uint8_t>(PointCloudClassification::INVALID));
    EXPECT_FLOAT_EQ(output_points[i].probability, 0.0F);
    EXPECT_TRUE(std::isnan(output_points[i].entropy));
  }
}

}  // namespace test
}  // namespace autoware::ptv3
