// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__PROCESSING_STATE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__PROCESSING_STATE_HPP_

#include <autoware/cuda_utils/cuda_check_error.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <utility>
#include <vector>

namespace autoware::cuda_pointcloud_preprocessor::dag
{

/**
 * @brief Processing state that flows through the DAG pipeline
 *
 * Instead of passing CudaPointCloud2 objects (which manage memory),
 * we pass this lightweight state that contains:
 * - Raw GPU pointer (may or may not own memory)
 * - Metadata (header, dimensions, fields)
 *
 * Filters work directly on the GPU pointer, modifying data in-place.
 * Only at entry (organize) and exit (finalize) do we create actual CudaPointCloud2 objects.
 */
struct PointcloudProcessingState
{
  std::uint8_t * device_data{nullptr};
  bool owns_memory{false};

  std_msgs::msg::Header header;
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::uint32_t point_step{0};
  std::uint32_t row_step{0};
  std::vector<sensor_msgs::msg::PointField> fields;
  bool is_bigendian{false};
  bool is_dense{false};
  std::uint32_t * device_crop_mask{nullptr};

  inline std::uint32_t numPoints() const { return width * height; }
  inline std::size_t dataSize() const { return row_step * height; }

  PointcloudProcessingState() = default;

  ~PointcloudProcessingState()
  {
    if (owns_memory && device_data != nullptr) {
      cudaFree(device_data);
    }
    // device_crop_mask is always owned by the state
    if (device_crop_mask != nullptr) {
      cudaFree(device_crop_mask);
    }
  }

  PointcloudProcessingState(const PointcloudProcessingState & other)
  : device_data(other.device_data),
    owns_memory(false),
    header(other.header),
    height(other.height),
    width(other.width),
    point_step(other.point_step),
    row_step(other.row_step),
    fields(other.fields),
    is_bigendian(other.is_bigendian),
    is_dense(other.is_dense),
    device_crop_mask(nullptr)
  {
    // Copy crop mask data if it exists
    if (other.device_crop_mask != nullptr) {
      const std::size_t num_points = other.numPoints();
      if (num_points > 0) {
        CHECK_CUDA_ERROR(cudaMalloc(&device_crop_mask, num_points * sizeof(std::uint32_t)));
        CHECK_CUDA_ERROR(cudaMemcpy(
          device_crop_mask, other.device_crop_mask, num_points * sizeof(std::uint32_t),
          cudaMemcpyDeviceToDevice));
      }
    }
  }

  PointcloudProcessingState(PointcloudProcessingState && other) noexcept
  : device_data(other.device_data),
    owns_memory(other.owns_memory),
    header(std::move(other.header)),
    height(other.height),
    width(other.width),
    point_step(other.point_step),
    row_step(other.row_step),
    fields(std::move(other.fields)),
    is_bigendian(other.is_bigendian),
    is_dense(other.is_dense),
    device_crop_mask(other.device_crop_mask)
  {
    other.device_data = nullptr;
    other.device_crop_mask = nullptr;
    other.owns_memory = false;
  }

  PointcloudProcessingState & operator=(const PointcloudProcessingState & other)
  {
    if (this != &other) {
      if (owns_memory && device_data != nullptr) {
        cudaFree(device_data);
      }
      // device_crop_mask is always owned
      if (device_crop_mask != nullptr) {
        cudaFree(device_crop_mask);
      }

      device_data = other.device_data;
      owns_memory = false;
      header = other.header;
      height = other.height;
      width = other.width;
      point_step = other.point_step;
      row_step = other.row_step;
      fields = other.fields;
      is_bigendian = other.is_bigendian;
      is_dense = other.is_dense;
      // Copy crop mask data if it exists
      device_crop_mask = nullptr;
      if (other.device_crop_mask != nullptr) {
        const std::size_t num_points = other.numPoints();
        if (num_points > 0) {
          CHECK_CUDA_ERROR(cudaMalloc(&device_crop_mask, num_points * sizeof(std::uint32_t)));
          CHECK_CUDA_ERROR(cudaMemcpy(
            device_crop_mask, other.device_crop_mask, num_points * sizeof(std::uint32_t),
            cudaMemcpyDeviceToDevice));
        }
      }
    }
    return *this;
  }

  PointcloudProcessingState & operator=(PointcloudProcessingState && other) noexcept
  {
    if (this != &other) {
      if (owns_memory && device_data != nullptr) {
        cudaFree(device_data);
      }
      // device_crop_mask is always owned
      if (device_crop_mask != nullptr) {
        cudaFree(device_crop_mask);
      }

      device_data = other.device_data;
      owns_memory = other.owns_memory;
      header = std::move(other.header);
      height = other.height;
      width = other.width;
      point_step = other.point_step;
      row_step = other.row_step;
      fields = std::move(other.fields);
      is_bigendian = other.is_bigendian;
      is_dense = other.is_dense;
      device_crop_mask = other.device_crop_mask;

      other.device_data = nullptr;
      other.device_crop_mask = nullptr;
      other.owns_memory = false;
    }
    return *this;
  }
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__PROCESSING_STATE_HPP_
