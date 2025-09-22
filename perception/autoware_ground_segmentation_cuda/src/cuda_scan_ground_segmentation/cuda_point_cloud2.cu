// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <autoware/cuda_scan_ground_segmentation/cuda_common.hpp>
#include <autoware/cuda_scan_ground_segmentation/cuda_point_cloud2.hpp>
#include <autoware/cuda_scan_ground_segmentation/cuda_utility.cuh>

namespace autoware::cuda
{

PointCloud2::PointCloud2(std::shared_ptr<CudaStream> stream, std::shared_ptr<CudaMempool> mempool)
: data_(stream, mempool), stream_(stream), mempool_(mempool)
{
}

PointCloud2::PointCloud2(
  const PointCloud2 & other, std::shared_ptr<CudaStream> stream,
  std::shared_ptr<CudaMempool> mempool)
: data_(other.data_, stream, mempool), stream_(stream), mempool_(mempool)
{
}

PointCloud2::PointCloud2(PointCloud2 && other)
: data_(std::move(other.data_)), stream_(other.stream_), mempool_(other.mempool_)
{
  other.stream_.reset();
  other.mempool_.reset();
}

PointCloud2 & PointCloud2::operator=(const PointCloud2 & other)
{
  data_ = other.data_;

  return *this;
}

PointCloud2 & PointCloud2::operator=(PointCloud2 && other)
{
  data_ = std::move(other.data_);

  return *this;
}

__global__ void convert_from_point_cloud2(
  std::uint8_t * input, PointCloud2::Ptr output, int point_num, int offset_x, int offset_y,
  int offset_z, int point_step)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;
  PointXYZ p;

  for (int i = index; i < point_num; i += stride) {
    auto point_ptr = input + i * point_step;
    p.x = *reinterpret_cast<float *>(point_ptr + offset_x);
    p.y = *reinterpret_cast<float *>(point_ptr + offset_y);
    p.z = *reinterpret_cast<float *>(point_ptr + offset_z);

    output.emplace(p, i);
  }
}

void PointCloud2::from_point_cloud2(const sensor_msgs::msg::PointCloud2 & msg)
{
  int point_num = msg.width * msg.height;
  int offset_x, offset_y, offset_z;

  for (const auto & f : msg.fields) {
    if (f.name == "x") {
      offset_x = f.offset;
    } else if (f.name == "y") {
      offset_y = f.offset;
    } else if (f.name == "z") {
      offset_z = f.offset;
    }
  }

  data_.resize(point_num);

  device_vector<uint8_t> tmp_buffer(msg.data, stream_, mempool_);

  CHECK_CUDA_ERROR(
    launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), convert_from_point_cloud2, tmp_buffer.data(), this->data(),
      point_num, offset_x, offset_y, offset_z, (int)(msg.point_step)));
}

__global__ void convert_to_point_cloud2(
  PointCloud2::Ptr input, uint8_t * output, int point_num, int offset_x, int offset_y, int offset_z,
  int point_step)
{
  int index = threadIdx.x + blockIdx.x * blockDim.x;
  int stride = blockDim.x * gridDim.x;
  PointXYZ p;

  for (int i = index; i < point_num; i += stride) {
    p = input[i];
    auto point_ptr = output + i * point_step;

    *reinterpret_cast<float *>(point_ptr + offset_x) = p.x;
    *reinterpret_cast<float *>(point_ptr + offset_y) = p.y;
    *reinterpret_cast<float *>(point_ptr + offset_z) = p.z;
  }
}

void PointCloud2::to_point_cloud2(sensor_msgs::msg::PointCloud2 & msg)
{
  msg.height = 1;
  msg.width = data_.size();
  msg.is_dense = true;
  msg.is_bigendian = false;
  msg.point_step = 3 * sizeof(float);  // x, y, z only
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step);

  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = sizeof(float);
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 2 * sizeof(float);
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;

  int point_num = data_.size();

  if (point_num <= 0) {
    return;
  }

  device_vector<uint8_t> tmp_data(point_num * msg.point_step, stream_, mempool_);

  CHECK_CUDA_ERROR(
    launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), convert_to_point_cloud2, this->data(), tmp_data.data(),
      point_num, (int)msg.fields[0].offset, (int)msg.fields[1].offset, (int)msg.fields[2].offset,
      (int)msg.point_step));

  tmp_data.to_vector(msg.data);
}

void PointCloud2::from_cuda_point_cloud2(const cuda_blackboard::CudaPointCloud2 & msg)
{
  int point_num = msg.width * msg.height;
  int offset_x, offset_y, offset_z;

  for (const auto & f : msg.fields) {
    if (f.name == "x") {
      offset_x = f.offset;
    } else if (f.name == "y") {
      offset_y = f.offset;
    } else if (f.name == "z") {
      offset_z = f.offset;
    }
  }

  data_.resize(point_num);

  CHECK_CUDA_ERROR(
    launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), convert_from_point_cloud2, msg.data.get(), this->data(),
      point_num, offset_x, offset_y, offset_z, (int)(msg.point_step)));
}

void PointCloud2::to_cuda_point_cloud2(cuda_blackboard::CudaPointCloud2 & msg)
{
  msg.height = 1;
  msg.width = data_.size();
  msg.is_dense = true;
  msg.is_bigendian = false;
  msg.point_step = 3 * sizeof(float);  // x, y, z only
  msg.row_step = msg.point_step * msg.width;

  msg.fields.resize(3);
  msg.fields[0].name = "x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[0].count = 1;

  msg.fields[1].name = "y";
  msg.fields[1].offset = sizeof(float);
  msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[1].count = 1;

  msg.fields[2].name = "z";
  msg.fields[2].offset = 2 * sizeof(float);
  msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  msg.fields[2].count = 1;

  int point_num = data_.size();

  if (point_num <= 0) {
    return;
  }

  msg.data = cuda_blackboard::make_unique<uint8_t[]>(
    msg.height * msg.width * msg.point_step * sizeof(uint8_t));

  CHECK_CUDA_ERROR(
    launchAsync<BLOCK_SIZE_X>(
      point_num, 0, stream_->get(), convert_to_point_cloud2, this->data(), msg.data.get(),
      point_num, (int)msg.fields[0].offset, (int)msg.fields[1].offset, (int)msg.fields[2].offset,
      (int)msg.point_step));
}

PointCloud2::Ptr PointCloud2::data()
{
  return Ptr(data_.data(), data_.size());
}
}  // namespace autoware::cuda
