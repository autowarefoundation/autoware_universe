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

#include "ptv3_test_fixture.hpp"

#include <autoware/point_types/types.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

PTv3Config make_test_config(const bool filter_apply_to_segmentation = false)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 128;
  params.voxels_num = {16, 32, 64};
  params.point_cloud_range = {-10.0F, -10.0F, -3.0F, 10.0F, 10.0F, 3.0F};
  params.voxel_size = {0.2F, 0.2F, 0.2F};
  params.segmentation_class_names = {"car", "truck", "drivable_flat"};
  params.segmentation_class_mapping = {
    {"car", "CAR"}, {"truck", "TRUCK"}, {"drivable_flat", "FLAT_SURFACE"}};
  params.palette = {
    255, 0,
    0,  // car
    0,   255,
    0,  // truck
    0,   0,
    255,  // drivable_flat
  };
  params.filter_classes = {"truck"};
  params.filter_output_format = "xyzi";
  params.filter_apply_to_segmentation = filter_apply_to_segmentation;
  params.source_reconstruction = "none";
  return makeConfig(params);
}

class PostprocessKernelTest : public PTv3CudaTest
{
protected:
  using PointXYZCPE = autoware::point_types::PointXYZCPE;
  using PointCloudClassification = autoware::point_types::PointCloudClassification;

  static constexpr std::size_t kNumClasses = 3;

  PTv3Config makeTestConfig(const bool filter_apply_to_segmentation = false) const
  {
    PTv3ConfigParams params;
    params.segmentation_class_names = {"car", "truck", "drivable_flat"};
    params.segmentation_class_mapping = {
      {"car", "CAR"}, {"truck", "TRUCK"}, {"drivable_flat", "FLAT_SURFACE"}};
    params.palette = {
      255, 0,
      0,  // car
      0,   255,
      0,  // truck
      0,   0,
      255,  // drivable_flat
    };
    params.filter_classes = {"truck"};
    params.filter_apply_to_segmentation = filter_apply_to_segmentation;

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

TEST_F(PostprocessKernelTest, SegmentationPointcloudDoesNotFilterConfiguredClassIndicesByDefault)
{
  const auto config = makeTestConfig();
  PostprocessCuda postprocess(config, stream_);

  // XYZ + padding for float4 input layout used by kernel.
  const std::vector<float> features = {
    1.0f, 10.0f, 100.0f, 0.0f,  // label 0: car
    2.0f, 20.0f, 200.0f, 0.0f,  // label 1: truck (filtered)
    3.0f, 30.0f, 300.0f, 0.0f,  // label 2: drivable_flat
    4.0f, 40.0f, 400.0f, 0.0f,  // invalid label
  };
  const std::vector<std::int64_t> labels = {0, 1, 2, -1};
  const std::vector<float> probs = {
    0.9f, 0.1f, 0.0f, 0.2f, 0.8f, 0.0f, 0.1f, 0.0f, 0.9f, 0.3f, 0.3f, 0.4f,
  };
  const auto num_points = labels.size();

  auto features_d = makeDeviceBuffer<float>(features.size());
  auto labels_d = makeDeviceBuffer<std::int64_t>(labels.size());
  auto probs_d = makeDeviceBuffer<float>(probs.size());
  auto output_points_d = makeDeviceBuffer<PointXYZCPE>(num_points);

  copyToDevice(features_d.get(), features);
  copyToDevice(labels_d.get(), labels);
  copyToDevice(probs_d.get(), probs);

  const auto num_segmented_points = postprocess.createSegmentationPointcloud(
    features_d.get(), labels_d.get(), probs_d.get(), output_points_d.get(), kNumClasses,
    num_points);

  EXPECT_EQ(num_segmented_points, 4U);

  auto output_points = copyToHost(output_points_d.get(), num_segmented_points);
  std::sort(output_points.begin(), output_points.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.x < rhs.x;
  });

  // Check that the output points have the expected xyz coordinates.
  for (std::size_t i = 0; i < num_points; ++i) {
    EXPECT_FLOAT_EQ(output_points[i].x, features[i * 4]);
    EXPECT_FLOAT_EQ(output_points[i].y, features[i * 4 + 1]);
    EXPECT_FLOAT_EQ(output_points[i].z, features[i * 4 + 2]);
  }

  // Check that the output points have the expected class IDs, probabilities and entropies.
  const auto car_label = static_cast<std::uint8_t>(PointCloudClassification::CAR);
  const auto truck_label = static_cast<std::uint8_t>(PointCloudClassification::TRUCK);
  const auto flat_surface_label = static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE);
  const auto invalid_label = static_cast<std::uint8_t>(PointCloudClassification::INVALID);

  // [0]: car
  EXPECT_EQ(output_points[0].class_id, car_label);
  EXPECT_FLOAT_EQ(output_points[0].probability, 0.9f);
  EXPECT_FLOAT_EQ(output_points[0].entropy, expectedEntropy(0.9F, 0.1F));
  // [1]: truck
  EXPECT_EQ(output_points[1].class_id, truck_label);
  EXPECT_FLOAT_EQ(output_points[1].probability, 0.8f);
  EXPECT_FLOAT_EQ(output_points[1].entropy, expectedEntropy(0.8F, 0.2F));
  // [2]: drivable_flat
  EXPECT_EQ(output_points[2].class_id, flat_surface_label);
  EXPECT_FLOAT_EQ(output_points[2].probability, 0.9f);
  EXPECT_FLOAT_EQ(output_points[2].entropy, expectedEntropy(0.9F, 0.1F));
  // [3]: invalid label -> probability=0.0, entropy=NaN
  EXPECT_EQ(output_points[3].class_id, invalid_label);
  EXPECT_FLOAT_EQ(output_points[3].probability, 0.0f);
  EXPECT_TRUE(std::isnan(output_points[3].entropy));
}

TEST_F(PostprocessKernelTest, SegmentationPointcloudFiltersConfiguredClassIndicesWhenEnabled)
{
  const auto config = makeTestConfig(true);
  PostprocessCuda postprocess(config, stream_);

  // XYZ + padding for float4 input layout used by kernel.
  const std::vector<float> features = {
    1.0f, 10.0f, 100.0f, 0.0f,  // label 0: car
    2.0f, 20.0f, 200.0f, 0.0f,  // label 1: truck (filtered)
    3.0f, 30.0f, 300.0f, 0.0f,  // label 2: drivable_flat
    4.0f, 40.0f, 400.0f, 0.0f,  // invalid label
  };
  const std::vector<std::int64_t> labels = {0, 1, 2, -1};
  const std::vector<float> probs = {
    0.9f, 0.1f, 0.0f, 0.2f, 0.8f, 0.0f, 0.1f, 0.0f, 0.9f, 0.3f, 0.3f, 0.4f,
  };
  const auto num_points = labels.size();

  auto features_d = makeDeviceBuffer<float>(features.size());
  auto labels_d = makeDeviceBuffer<std::int64_t>(labels.size());
  auto probs_d = makeDeviceBuffer<float>(probs.size());
  auto output_points_d = makeDeviceBuffer<PointXYZCPE>(num_points);

  copyToDevice(features_d.get(), features);
  copyToDevice(labels_d.get(), labels);
  copyToDevice(probs_d.get(), probs);

  const auto num_segmented_points = postprocess.createSegmentationPointcloud(
    features_d.get(), labels_d.get(), probs_d.get(), output_points_d.get(), kNumClasses,
    num_points);

  EXPECT_EQ(num_segmented_points, 3U);

  auto output_points = copyToHost(output_points_d.get(), num_segmented_points);
  std::sort(output_points.begin(), output_points.end(), [](const auto & lhs, const auto & rhs) {
    return lhs.x < rhs.x;
  });

  // Truck label (features[1]) is filtered out.
  // Check that the output points have the expected xyz coordinates.
  const std::array<std::size_t, 3> feature_indices = {0, 2, 3};
  for (std::size_t i = 0; i < output_points.size(); ++i) {
    const auto feature_index = feature_indices[i];
    EXPECT_FLOAT_EQ(output_points[i].x, features[feature_index * 4]);
    EXPECT_FLOAT_EQ(output_points[i].y, features[feature_index * 4 + 1]);
    EXPECT_FLOAT_EQ(output_points[i].z, features[feature_index * 4 + 2]);
  }

  const auto car_label = static_cast<std::uint8_t>(PointCloudClassification::CAR);
  const auto flat_surface_label = static_cast<std::uint8_t>(PointCloudClassification::FLAT_SURFACE);
  const auto invalid_label = static_cast<std::uint8_t>(PointCloudClassification::INVALID);

  // [0]: car
  EXPECT_EQ(output_points[0].class_id, car_label);
  EXPECT_FLOAT_EQ(output_points[0].probability, 0.9f);
  EXPECT_FLOAT_EQ(output_points[0].entropy, expectedEntropy(0.9F, 0.1F));
  // [1]: drivable_flat
  EXPECT_EQ(output_points[1].class_id, flat_surface_label);
  EXPECT_FLOAT_EQ(output_points[1].probability, 0.9f);
  EXPECT_FLOAT_EQ(output_points[1].entropy, expectedEntropy(0.9F, 0.1F));
  // [2]: invalid label -> probability=0.0, entropy=NaN
  EXPECT_EQ(output_points[2].class_id, invalid_label);
  EXPECT_FLOAT_EQ(output_points[2].probability, 0.0f);
  EXPECT_TRUE(std::isnan(output_points[2].entropy));
}

TEST_F(PostprocessKernelTest, FilteredPointcloudFiltersOnlyArgmaxClass)
{
  const auto config = makeTestConfig();
  PostprocessCuda postprocess(config, stream_);

  constexpr std::size_t num_points = 3;
  constexpr std::size_t num_classes = 3;

  const std::vector<CloudPointTypeXYZI> input_points = {
    {1.0f, 10.0f, 100.0f, 0.1f},  // car argmax: kept despite truck probability
    {2.0f, 20.0f, 200.0f, 0.2f},  // truck argmax: filtered
    {3.0f, 30.0f, 300.0f, 0.3f},  // drivable_flat argmax: kept despite truck probability
  };
  const std::vector<float> pred_probs = {
    0.7f, 0.2f, 0.1f, 0.2f, 0.6f, 0.2f, 0.1f, 0.3f, 0.6f,
  };

  auto input_points_d = this->template makeDeviceBuffer<CloudPointTypeXYZI>(num_points);
  auto pred_probs_d = this->template makeDeviceBuffer<float>(num_points * num_classes);
  auto output_points_d = this->template makeDeviceBuffer<CloudPointTypeXYZI>(num_points);

  copyToDevice(input_points_d.get(), input_points);
  copyToDevice(pred_probs_d.get(), pred_probs);

  const auto num_filtered_points = postprocess.createFilteredPointcloud(
    input_points_d.get(), CloudFormat::XYZI, CloudFormat::XYZI, pred_probs_d.get(),
    output_points_d.get(), num_classes, num_points);

  EXPECT_EQ(num_filtered_points, 2U);

  const auto output_points = copyToHost(output_points_d.get(), num_filtered_points);
  std::array<float, 2> x_values{};
  for (std::size_t i = 0; i < output_points.size(); ++i) {
    x_values[i] = output_points[i].x;
  }

  std::sort(x_values.begin(), x_values.end());
  EXPECT_EQ(x_values[0], 1.0f);
  EXPECT_EQ(x_values[1], 3.0f);
}

}  // namespace test
}  // namespace autoware::ptv3
