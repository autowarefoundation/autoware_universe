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

#ifndef AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_POINT_CLOUD2_HPP_
#define AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_POINT_CLOUD2_HPP_

#include "cuda_common.hpp"
#include "cuda_point_types.hpp"
#include "device_vector.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <cuda_runtime.h>
#include <pcl/point_types.h>

namespace autoware::cuda
{

class PointCloud2
{
public:
  class Ptr;  // A pointer used by CUDA kernels

  PointCloud2(
    std::shared_ptr<CudaStream> stream = std::make_shared<CudaStream>(true),
    std::shared_ptr<CudaMempool> mempool = nullptr);
  PointCloud2(
    const PointCloud2 & other,
    std::shared_ptr<CudaStream> stream = std::make_shared<CudaStream>(true),
    std::shared_ptr<CudaMempool> mempool = nullptr);
  PointCloud2(PointCloud2 && other);

  PointCloud2 & operator=(const PointCloud2 &);
  PointCloud2 & operator=(PointCloud2 &&);

  // Convert a sensor_msgs::msg::PointCloud2 message to this GPU PointCloud2
  void from_point_cloud2(const sensor_msgs::msg::PointCloud2 & msg);
  // Convert to a sensor_msgs::msg::PointCloud2 message
  void to_point_cloud2(sensor_msgs::msg::PointCloud2 & msg);

  // Convert from a cuda_blackboard GPU PointCloud2
  void from_cuda_point_cloud2(const cuda_blackboard::CudaPointCloud2 & msg);
  void to_cuda_point_cloud2(cuda_blackboard::CudaPointCloud2 & msg);

  // Return a pointer so it can be used by CUDA kernels
  Ptr data();

  size_t size() { return data_.size(); }
  void clear() { data_.clear(); }
  void resize(int new_size) { data_.resize(new_size); }
  void reserve(int new_size) { data_.reserve(new_size); }

  bool empty() { return data_.empty(); }

private:
  // For now, I just need x, y, z
  device_vector<float4> data_;

  std::shared_ptr<CudaStream> stream_;
  std::shared_ptr<CudaMempool> mempool_;
};

struct PointCloud2::Ptr
{
public:
  CUDAH Ptr() : data_(nullptr), point_num_(0) {}
  CUDAH Ptr(float4 * data, int point_num) : data_(data), point_num_(point_num) {}
  CUDAH Ptr(const Ptr & other) : data_(other.data_), point_num_(other.point_num_) {}
  CUDAH Ptr(Ptr && other) : data_(other.data_), point_num_(other.point_num_)
  {
    other.data_ = nullptr;
    other.point_num_ = 0;
  }

  CUDAH Ptr & operator=(const Ptr & other)
  {
    data_ = other.data_;
    point_num_ = other.point_num_;

    return *this;
  }

  CUDAH Ptr & operator=(Ptr && other)
  {
    data_ = other.data_;
    point_num_ = other.point_num_;

    other.data_ = nullptr;
    other.point_num_ = 0;

    return *this;
  }

  CUDAH PointXYZ & operator[](int idx) { return *reinterpret_cast<PointXYZ *>(&data_[idx]); }

  CUDAH const PointXYZ & operator[](int idx) const
  {
    // Warning: no out-of-bound check
    return *reinterpret_cast<const PointXYZ *>(&data_[idx]);
  }

  // Save a point to the memory
  CUDAH void emplace(const PointXYZ & p, int idx)
  {
    data_[idx] = *reinterpret_cast<const float4 *>(&p);
  }

  CUDAH int size() const { return point_num_; }

private:
  float4 * data_;  // On the GPU global memory
  int point_num_;
};

}  // namespace autoware::cuda

#endif  // AUTOWARE__CUDA_SCAN_GROUND_SEGMENTATION__CUDA_POINT_CLOUD2_HPP_
