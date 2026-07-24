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

#include "autoware/ptv3/experimental/semantic_label.hpp"
#include "ptv3_test_fixture.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace autoware::ptv3
{
namespace test
{

struct OutputSegmentationPointType
{
  float x;
  float y;
  float z;
  std::uint8_t class_id;
  float probability;
  float entropy;
} __attribute__((packed));

PTv3Config make_test_config(const bool filter_apply_to_segmentation = false)
{
  PTv3ConfigParams params;
  params.cloud_capacity = 128;
  params.voxels_num = {16, 32, 64};
  params.point_cloud_range = {-10.0F, -10.0F, -3.0F, 10.0F, 10.0F, 3.0F};
  params.voxel_size = {0.2F, 0.2F, 0.2F};
  params.segmentation_class_names = {"car", "truck", "drivable_flat"};
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
};

TEST_F(PostprocessKernelTest, SegmentationPointcloudDoesNotFilterConfiguredClassIndicesByDefault)
{
  const auto config = make_test_config();
  PostprocessCuda postprocess(config, stream_);

  constexpr std::size_t num_points = 4;
  constexpr std::size_t num_classes = 3;

  // XYZ + padding for float4 input layout used by kernel.
  const std::vector<float> input_features = {
    1.0f, 10.0f, 100.0f, 0.0f,  // label 0: car
    2.0f, 20.0f, 200.0f, 0.0f,  // label 1: truck (filtered)
    3.0f, 30.0f, 300.0f, 0.0f,  // label 2: drivable_flat
    4.0f, 40.0f, 400.0f, 0.0f,  // invalid label
  };
  const std::vector<std::int64_t> pred_labels = {0, 1, 2, -1};
  const std::vector<float> pred_probs = {
    0.9f, 0.1f, 0.0f, 0.2f, 0.8f, 0.0f, 0.1f, 0.0f, 0.9f, 0.3f, 0.3f, 0.4f,
  };

  auto input_features_d = this->template makeDeviceBuffer<float>(num_points * 4);
  auto pred_labels_d = this->template makeDeviceBuffer<std::int64_t>(num_points);
  auto pred_probs_d = this->template makeDeviceBuffer<float>(num_points * num_classes);
  auto output_points_d = this->template makeDeviceBuffer<OutputSegmentationPointType>(num_points);

  copyToDevice(input_features_d.get(), input_features);
  copyToDevice(pred_labels_d.get(), pred_labels);
  copyToDevice(pred_probs_d.get(), pred_probs);

  const auto num_segmented_points = postprocess.createSegmentationPointcloud(
    input_features_d.get(), pred_labels_d.get(), pred_probs_d.get(),
    reinterpret_cast<std::uint8_t *>(output_points_d.get()), num_classes, num_points);

  EXPECT_EQ(num_segmented_points, 4U);

  const auto output_points = copyToHost(output_points_d.get(), num_segmented_points);

  std::array<float, 4> x_values{};
  std::array<std::uint8_t, 4> class_ids{};
  for (std::size_t i = 0; i < output_points.size(); ++i) {
    x_values[i] = output_points[i].x;
    class_ids[i] = output_points[i].class_id;
  }

  std::sort(x_values.begin(), x_values.end());
  EXPECT_EQ(x_values[0], 1.0f);
  EXPECT_EQ(x_values[1], 2.0f);
  EXPECT_EQ(x_values[2], 3.0f);
  EXPECT_EQ(x_values[3], 4.0f);

  const auto car_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::CAR);
  const auto truck_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::TRUCK);
  const auto ground_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::FLAT_SURFACE);

  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), car_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), truck_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), ground_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), 255U), class_ids.end());

  for (const auto & output_point : output_points) {
    if (output_point.x == 4.0f) {
      EXPECT_EQ(output_point.class_id, 255U);
      EXPECT_EQ(output_point.probability, 0.0f);
    }
  }
}

TEST_F(PostprocessKernelTest, SegmentationPointcloudFiltersConfiguredClassIndicesWhenEnabled)
{
  const auto config = make_test_config(true);
  PostprocessCuda postprocess(config, stream_);

  constexpr std::size_t num_points = 4;
  constexpr std::size_t num_classes = 3;

  // XYZ + padding for float4 input layout used by kernel.
  const std::vector<float> input_features = {
    1.0f, 10.0f, 100.0f, 0.0f,  // label 0: car
    2.0f, 20.0f, 200.0f, 0.0f,  // label 1: truck (filtered)
    3.0f, 30.0f, 300.0f, 0.0f,  // label 2: drivable_flat
    4.0f, 40.0f, 400.0f, 0.0f,  // invalid label
  };
  const std::vector<std::int64_t> pred_labels = {0, 1, 2, -1};
  const std::vector<float> pred_probs = {
    0.9f, 0.1f, 0.0f, 0.2f, 0.8f, 0.0f, 0.1f, 0.0f, 0.9f, 0.3f, 0.3f, 0.4f,
  };

  auto input_features_d = this->template makeDeviceBuffer<float>(num_points * 4);
  auto pred_labels_d = this->template makeDeviceBuffer<std::int64_t>(num_points);
  auto pred_probs_d = this->template makeDeviceBuffer<float>(num_points * num_classes);
  auto output_points_d = this->template makeDeviceBuffer<OutputSegmentationPointType>(num_points);

  copyToDevice(input_features_d.get(), input_features);
  copyToDevice(pred_labels_d.get(), pred_labels);
  copyToDevice(pred_probs_d.get(), pred_probs);

  const auto num_segmented_points = postprocess.createSegmentationPointcloud(
    input_features_d.get(), pred_labels_d.get(), pred_probs_d.get(),
    reinterpret_cast<std::uint8_t *>(output_points_d.get()), num_classes, num_points);

  EXPECT_EQ(num_segmented_points, 3U);

  const auto output_points = copyToHost(output_points_d.get(), num_segmented_points);

  std::array<float, 3> x_values{};
  std::array<std::uint8_t, 3> class_ids{};
  for (std::size_t i = 0; i < output_points.size(); ++i) {
    x_values[i] = output_points[i].x;
    class_ids[i] = output_points[i].class_id;
  }

  std::sort(x_values.begin(), x_values.end());
  EXPECT_EQ(x_values[0], 1.0f);
  EXPECT_EQ(x_values[1], 3.0f);
  EXPECT_EQ(x_values[2], 4.0f);

  const auto car_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::CAR);
  const auto truck_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::TRUCK);
  const auto ground_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::FLAT_SURFACE);

  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), car_label), class_ids.end());
  EXPECT_EQ(std::find(class_ids.begin(), class_ids.end(), truck_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), ground_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), 255U), class_ids.end());
}

TEST_F(PostprocessKernelTest, FilteredPointcloudFiltersOnlyArgmaxClass)
{
  const auto config = make_test_config();
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
