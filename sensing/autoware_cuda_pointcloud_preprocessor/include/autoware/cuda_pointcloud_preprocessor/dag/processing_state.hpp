#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__PROCESSING_STATE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__PROCESSING_STATE_HPP_

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
  bool is_finalized{
    false};  // True if already compacted (e.g., downsample output), false if needs mask application

  std_msgs::msg::Header header;
  std::uint32_t height{0};
  std::uint32_t width{0};
  std::uint32_t point_step{0};
  std::uint32_t row_step{0};
  std::vector<sensor_msgs::msg::PointField> fields;
  bool is_bigendian{false};
  bool is_dense{false};

  inline std::uint32_t numPoints() const { return width * height; }
  inline std::size_t dataSize() const { return row_step * height; }

  PointcloudProcessingState() = default;

  ~PointcloudProcessingState()
  {
    if (owns_memory && device_data != nullptr) {
      cudaFree(device_data);
    }
  }

  PointcloudProcessingState(const PointcloudProcessingState & other)
  : device_data(other.device_data),
    owns_memory(false),
    is_finalized(other.is_finalized),
    header(other.header),
    height(other.height),
    width(other.width),
    point_step(other.point_step),
    row_step(other.row_step),
    fields(other.fields),
    is_bigendian(other.is_bigendian),
    is_dense(other.is_dense)
  {
  }

  PointcloudProcessingState(PointcloudProcessingState && other) noexcept
  : device_data(other.device_data),
    owns_memory(other.owns_memory),
    is_finalized(other.is_finalized),
    header(std::move(other.header)),
    height(other.height),
    width(other.width),
    point_step(other.point_step),
    row_step(other.row_step),
    fields(std::move(other.fields)),
    is_bigendian(other.is_bigendian),
    is_dense(other.is_dense)
  {
    other.device_data = nullptr;
    other.owns_memory = false;
    other.is_finalized = false;
  }

  PointcloudProcessingState & operator=(const PointcloudProcessingState & other)
  {
    if (this != &other) {
      if (owns_memory && device_data != nullptr) {
        cudaFree(device_data);
      }

      device_data = other.device_data;
      owns_memory = false;
      is_finalized = other.is_finalized;
      header = other.header;
      height = other.height;
      width = other.width;
      point_step = other.point_step;
      row_step = other.row_step;
      fields = other.fields;
      is_bigendian = other.is_bigendian;
      is_dense = other.is_dense;
    }
    return *this;
  }

  PointcloudProcessingState & operator=(PointcloudProcessingState && other) noexcept
  {
    if (this != &other) {
      if (owns_memory && device_data != nullptr) {
        cudaFree(device_data);
      }

      device_data = other.device_data;
      owns_memory = other.owns_memory;
      is_finalized = other.is_finalized;
      header = std::move(other.header);
      height = other.height;
      width = other.width;
      point_step = other.point_step;
      row_step = other.row_step;
      fields = std::move(other.fields);
      is_bigendian = other.is_bigendian;
      is_dense = other.is_dense;

      other.device_data = nullptr;
      other.owns_memory = false;
      other.is_finalized = false;
    }
    return *this;
  }
};

}  // namespace autoware::cuda_pointcloud_preprocessor::dag

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__DAG__PROCESSING_STATE_HPP_
