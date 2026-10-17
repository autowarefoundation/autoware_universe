// Copyright 2026 NEWSLab, National Taiwan University
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

#include "autoware/cuda_pointcloud_preprocessor/cuda_crop_box_filter/cuda_crop_box_filter.hpp"

#include <sensor_msgs/msg/point_field.hpp>

#include <thrust/execution_policy.h>
#include <thrust/scan.h>

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace autoware::cuda_pointcloud_preprocessor
{
namespace
{

constexpr int kBlockSize = 256;

/// Mark each point 1 when it is kept, 0 when it is dropped.
///
/// NaN is dropped in both polarities. Every comparison against a NaN coordinate
/// is false, so `inside` is false and a naive `negative` would keep it — which
/// would hand NDT a NaN and is not what the CPU filter does.
__global__ void markKernel(
  const std::uint8_t * __restrict__ data, std::uint32_t * __restrict__ mask, std::size_t num_points,
  std::size_t point_step, std::size_t off_x, std::size_t off_y, std::size_t off_z, float min_x,
  float max_x, float min_y, float max_y, float min_z, float max_z, bool negative)
{
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= num_points) {
    return;
  }
  const std::uint8_t * p = data + i * point_step;
  float x;
  float y;
  float z;
  memcpy(&x, p + off_x, sizeof(float));
  memcpy(&y, p + off_y, sizeof(float));
  memcpy(&z, p + off_z, sizeof(float));

  const bool finite = isfinite(x) && isfinite(y) && isfinite(z);
  const bool inside =
    x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z;
  const bool keep = finite && (negative ? !inside : inside);
  mask[i] = keep ? 1u : 0u;
}

/// Copy the kept points to the slots the exclusive scan assigned them.
__global__ void scatterKernel(
  const std::uint8_t * __restrict__ in, std::uint8_t * __restrict__ out,
  const std::uint32_t * __restrict__ mask, const std::uint32_t * __restrict__ indices,
  std::size_t num_points, std::size_t point_step)
{
  const std::size_t i = blockIdx.x * static_cast<std::size_t>(blockDim.x) + threadIdx.x;
  if (i >= num_points || mask[i] == 0u) {
    return;
  }
  const std::uint8_t * src = in + i * point_step;
  std::uint8_t * dst = out + static_cast<std::size_t>(indices[i]) * point_step;
  for (std::size_t b = 0; b < point_step; ++b) {
    dst[b] = src[b];
  }
}

void check(cudaError_t err, const char * what)
{
  if (err != cudaSuccess) {
    throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(err));
  }
}

}  // namespace

CudaCropBoxFilter::CudaCropBoxFilter(const BoxParams & params) : params_(params)
{
  check(cudaStreamCreate(&stream_), "cudaStreamCreate");
}

CudaCropBoxFilter::~CudaCropBoxFilter()
{
  if (d_mask_ != nullptr) {
    cudaFree(d_mask_);
  }
  if (d_indices_ != nullptr) {
    cudaFree(d_indices_);
  }
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
  }
}

namespace
{
std::string describeDatatype(std::uint8_t datatype)
{
  using sensor_msgs::msg::PointField;
  switch (datatype) {
    case PointField::INT8:
      return "INT8";
    case PointField::UINT8:
      return "UINT8";
    case PointField::INT16:
      return "INT16";
    case PointField::UINT16:
      return "UINT16";
    case PointField::INT32:
      return "INT32";
    case PointField::UINT32:
      return "UINT32";
    case PointField::FLOAT32:
      return "FLOAT32";
    case PointField::FLOAT64:
      return "FLOAT64";
    default:
      return "datatype " + std::to_string(static_cast<int>(datatype));
  }
}
}  // namespace

std::string CudaCropBoxFilter::findXyzOffsets(
  const cuda_blackboard::CudaPointCloud2 & cloud, std::size_t offsets[3])
{
  // Look a coordinate up by name first, and only then check what the kernel
  // needs of it. Skipping fields that fail the checks and reporting "not found"
  // at the end would blame a missing field for a present one of the wrong type,
  // which is the harder of the two to diagnose from a log line.
  const char * names[3] = {"x", "y", "z"};
  for (int axis = 0; axis < 3; ++axis) {
    const sensor_msgs::msg::PointField * field = nullptr;
    for (const auto & f : cloud.fields) {
      if (f.name == names[axis]) {
        field = &f;
        break;
      }
    }
    if (field == nullptr) {
      return std::string("no field named '") + names[axis] + "'";
    }
    // The kernel reads four bytes as a float. Anything else would need a
    // conversion this filter deliberately does not do, and reading a FLOAT64
    // field as a float yields plausible-looking garbage rather than an error.
    if (field->datatype != sensor_msgs::msg::PointField::FLOAT32) {
      return std::string("field '") + names[axis] + "' is " + describeDatatype(field->datatype) +
             ", but this filter reads it as FLOAT32";
    }
    if (field->count != 1) {
      return std::string("field '") + names[axis] + "' has count " +
             std::to_string(field->count) + ", but this filter reads a single element";
    }
    // A field must lie wholly inside the point. Without this a malformed cloud
    // declaring x at offset 60 with point_step 20 would have the kernel read
    // past the end of every point -- out of bounds on the device, where it
    // fails as corrupt output rather than a fault.
    if (field->offset + sizeof(float) > cloud.point_step) {
      return std::string("field '") + names[axis] + "' ends at byte " +
             std::to_string(field->offset + sizeof(float)) + ", past point_step " +
             std::to_string(cloud.point_step);
    }
    offsets[axis] = field->offset;
  }
  return {};
}

void CudaCropBoxFilter::ensureCapacity(std::size_t num_points)
{
  if (num_points <= capacity_) {
    return;
  }
  if (d_mask_ != nullptr) {
    cudaFree(d_mask_);
    d_mask_ = nullptr;
  }
  if (d_indices_ != nullptr) {
    cudaFree(d_indices_);
    d_indices_ = nullptr;
  }
  check(cudaMalloc(&d_mask_, num_points * sizeof(std::uint32_t)), "cudaMalloc mask");
  check(cudaMalloc(&d_indices_, num_points * sizeof(std::uint32_t)), "cudaMalloc indices");
  capacity_ = num_points;
}

std::unique_ptr<cuda_blackboard::CudaPointCloud2> CudaCropBoxFilter::filter(
  const cuda_blackboard::CudaPointCloud2 & input)
{
  std::size_t off[3];
  layout_error_ = findXyzOffsets(input, off);
  if (!layout_error_.empty()) {
    return nullptr;
  }

  const std::size_t point_step = input.point_step;
  const std::size_t num_points =
    static_cast<std::size_t>(input.width) * static_cast<std::size_t>(input.height);

  auto output = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  output->header = input.header;
  output->fields = input.fields;
  output->point_step = input.point_step;
  output->is_bigendian = input.is_bigendian;
  output->is_dense = true;
  output->height = 1;

  if (num_points == 0 || point_step == 0) {
    output->width = 0;
    output->row_step = 0;
    output->data = cuda_blackboard::CudaUniquePtr<std::uint8_t[]>();
    return output;
  }

  ensureCapacity(num_points);

  const int blocks = static_cast<int>((num_points + kBlockSize - 1) / kBlockSize);
  markKernel<<<blocks, kBlockSize, 0, stream_>>>(
    input.data.get(), d_mask_, num_points, point_step, off[0], off[1], off[2], params_.min_x,
    params_.max_x, params_.min_y, params_.max_y, params_.min_z, params_.max_z, params_.negative);
  check(cudaGetLastError(), "markKernel");

  thrust::exclusive_scan(thrust::cuda::par.on(stream_), d_mask_, d_mask_ + num_points, d_indices_);

  // The scan is exclusive, so the kept count is the last index plus the last
  // mask entry. Two small copies rather than a separate reduction kernel.
  std::uint32_t last_index = 0;
  std::uint32_t last_mask = 0;
  check(
    cudaMemcpyAsync(
      &last_index, d_indices_ + num_points - 1, sizeof(std::uint32_t), cudaMemcpyDeviceToHost,
      stream_),
    "copy last index");
  check(
    cudaMemcpyAsync(
      &last_mask, d_mask_ + num_points - 1, sizeof(std::uint32_t), cudaMemcpyDeviceToHost, stream_),
    "copy last mask");
  check(cudaStreamSynchronize(stream_), "sync after scan");

  const std::size_t kept = last_index + last_mask;

  output->width = static_cast<std::uint32_t>(kept);
  output->row_step = static_cast<std::uint32_t>(kept * point_step);

  if (kept == 0) {
    output->data = cuda_blackboard::CudaUniquePtr<std::uint8_t[]>();
    return output;
  }

  // Allocated through the blackboard's own helper so the buffer carries the
  // deleter the library expects; a raw cudaMalloc here would be freed by a
  // deleter that did not allocate it.
  output->data = cuda_blackboard::make_unique<std::uint8_t[]>(kept * point_step);
  std::uint8_t * out_raw = output->data.get();

  scatterKernel<<<blocks, kBlockSize, 0, stream_>>>(
    input.data.get(), out_raw, d_mask_, d_indices_, num_points, point_step);
  check(cudaGetLastError(), "scatterKernel");
  check(cudaStreamSynchronize(stream_), "sync after scatter");

  return output;
}

}  // namespace autoware::cuda_pointcloud_preprocessor
