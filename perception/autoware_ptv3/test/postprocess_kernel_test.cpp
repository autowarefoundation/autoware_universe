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

#include <cuda_runtime_api.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using autoware::ptv3::PostprocessCuda;
using autoware::ptv3::PTv3Config;

class CudaStreamGuard
{
public:
  CudaStreamGuard()
  {
    const auto status = cudaStreamCreate(&stream_);
    if (status != cudaSuccess) {
      throw std::runtime_error(cudaGetErrorString(status));
    }
  }

  ~CudaStreamGuard()
  {
    if (stream_ != nullptr) {
      cudaStreamDestroy(stream_);
    }
  }

  CudaStreamGuard(const CudaStreamGuard &) = delete;
  CudaStreamGuard & operator=(const CudaStreamGuard &) = delete;

  [[nodiscard]] cudaStream_t get() const { return stream_; }

private:
  cudaStream_t stream_{nullptr};
};

template <typename T>
class DeviceBuffer
{
public:
  explicit DeviceBuffer(const std::size_t element_count)
  {
    const auto status = cudaMalloc(&data_, sizeof(T) * element_count);
    if (status != cudaSuccess) {
      throw std::runtime_error(cudaGetErrorString(status));
    }
  }

  ~DeviceBuffer()
  {
    if (data_ != nullptr) {
      cudaFree(data_);
    }
  }

  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;

  [[nodiscard]] T * get() const { return static_cast<T *>(data_); }

private:
  void * data_{nullptr};
};

template <typename T>
void copy_to_device(T * device_ptr, const std::vector<T> & values)
{
  const auto status =
    cudaMemcpy(device_ptr, values.data(), sizeof(T) * values.size(), cudaMemcpyHostToDevice);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
}

template <typename T>
std::vector<T> copy_to_host(const T * device_ptr, const std::size_t count)
{
  std::vector<T> values(count);
  const auto status =
    cudaMemcpy(values.data(), device_ptr, sizeof(T) * count, cudaMemcpyDeviceToHost);
  if (status != cudaSuccess) {
    throw std::runtime_error(cudaGetErrorString(status));
  }
  return values;
}

struct OutputSegmentationPointType
{
  float x;
  float y;
  float z;
  std::uint8_t class_id;
  float probability;
  float entropy;
} __attribute__((packed));

PTv3Config make_test_config()
{
  return PTv3Config(
    "", 128, {16, 32, 64}, {-10.0f, -10.0f, -3.0f, 10.0f, 10.0f, 3.0f}, {0.2f, 0.2f, 0.2f},
    {"car", "truck", "drivable_flat"}, {"z", "z-trans"}, {2, 2, 2, 2},
    {
      255,
      0,
      0,  // car
      0,
      255,
      0,  // truck
      0,
      0,
      255,  // drivable_flat
    },
    0.8f, {"truck"}, "xyzi", "none", true);
}

TEST(PostprocessKernelTest, SegmentationPointcloudFiltersConfiguredClassIndices)
{
  CudaStreamGuard stream;
  const auto config = make_test_config();
  PostprocessCuda postprocess(config, stream.get());

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

  DeviceBuffer<float> input_features_d(num_points * 4);
  DeviceBuffer<std::int64_t> pred_labels_d(num_points);
  DeviceBuffer<float> pred_probs_d(num_points * num_classes);
  DeviceBuffer<OutputSegmentationPointType> output_points_d(num_points);

  copy_to_device(input_features_d.get(), input_features);
  copy_to_device(pred_labels_d.get(), pred_labels);
  copy_to_device(pred_probs_d.get(), pred_probs);

  const auto num_segmented_points = postprocess.createSegmentationPointcloud(
    input_features_d.get(), pred_labels_d.get(), pred_probs_d.get(),
    reinterpret_cast<std::uint8_t *>(output_points_d.get()), num_classes, num_points);

  EXPECT_EQ(num_segmented_points, 3U);

  const auto output_points = copy_to_host(output_points_d.get(), num_segmented_points);

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
  const auto ground_label =
    static_cast<std::uint8_t>(autoware::ptv3::experimental::SemanticLabel::GROUND);

  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), car_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), ground_label), class_ids.end());
  EXPECT_NE(std::find(class_ids.begin(), class_ids.end(), 255U), class_ids.end());

  for (const auto & output_point : output_points) {
    if (output_point.x == 4.0f) {
      EXPECT_EQ(output_point.class_id, 255U);
      EXPECT_EQ(output_point.probability, 0.0f);
    }
  }
}

}  // namespace
