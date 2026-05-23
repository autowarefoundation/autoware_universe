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

#include "position_validator.hpp"
#include "preprocess/voxelization.hpp"

#include <Eigen/Core>
#include <NvInfer.h>
#include <torch/nn/functional/padding.h>
#include <torch/torch.h>

#include <chrono>
#include <memory>
#include <vector>

namespace autoware::localized_position_validator
{
namespace F = torch::nn::functional;

at::Tensor vectorToTensor(const std::vector<Eigen::Vector4f> & points) {
  const int n = points.size();

  // allocate a flat vector with 4 * n elements
  std::vector<float> flat;
  flat.reserve(n * 4);
  for (const auto& pt : points) {
    flat.push_back(pt[0]);
    flat.push_back(pt[1]);
    flat.push_back(pt[2]);
    flat.push_back(pt[3]);
  }

  // create a tensor from the raw data
  at::Tensor tensor = torch::from_blob(
    flat.data(), {n, 4}, at::TensorOptions().dtype(at::kFloat).device(at::kCPU)).clone();

  return tensor;
}

PositionValidatorTRT::PositionValidatorTRT(
  const TrtCommonConfig & model_param,
  const std::vector<float> & voxel_size, const std::vector<float> & coors_range,
  const int & max_voxels = 40000, const int & max_points_per_voxel = 50,
  const int & profile_points_min = 500, const int & profile_points_opt = 5000,
  const int & profile_points_max = 10000)
{
  voxel_size_.insert(voxel_size_.end(), voxel_size.begin(), voxel_size.end());
  coors_range_.insert(coors_range_.end(), coors_range.begin(), coors_range.end());
  max_voxels_ = max_voxels;
  max_points_per_voxel_ = max_points_per_voxel;
  profile_points_min_ = profile_points_min;
  profile_points_max_ = profile_points_max;

  // initialize trt wrappers
  model_trt_ptr_ = std::make_unique<tensorrt_common::TrtCommon>(model_param);

  cudaStreamCreate(&stream_);

  const auto pillars_prof_min = nvinfer1::Dims3{profile_points_min_, 50, 4};
  const auto pillars_prof_opt = nvinfer1::Dims3{profile_points_opt, 50, 4};
  const auto pillars_prof_max = nvinfer1::Dims3{profile_points_max_, 50, 4};

  const auto coors_batch_prof_min = nvinfer1::Dims2{profile_points_min_, 4};
  const auto coors_batch_prof_opt = nvinfer1::Dims2{profile_points_opt, 4};
  const auto coors_batch_prof_max = nvinfer1::Dims2{profile_points_max_, 4};

  const auto npoints_per_pillar_prof_min = nvinfer1::Dims{1, {profile_points_min_}};
  const auto npoints_per_pillar_prof_opt = nvinfer1::Dims{1, {profile_points_opt}};
  const auto npoints_per_pillar_prof_max = nvinfer1::Dims{1, {profile_points_max_}};

  std::vector<tensorrt_common::ProfileDims> model_profile_dims{
    tensorrt_common::ProfileDims("pillars", pillars_prof_min, pillars_prof_opt, pillars_prof_max),
    tensorrt_common::ProfileDims(
      "coors_batch", coors_batch_prof_min, coors_batch_prof_opt, coors_batch_prof_max),
    tensorrt_common::ProfileDims("npoints_per_pillar",
      npoints_per_pillar_prof_min, npoints_per_pillar_prof_opt, npoints_per_pillar_prof_max),
  };
  auto model_profile_dims_ptr =
    std::make_unique<std::vector<tensorrt_common::ProfileDims>>(model_profile_dims);

  // setup trt engines
  if (!model_trt_ptr_->setup(std::move(model_profile_dims_ptr))) {
    throw std::runtime_error("Failed to setup TRT engine.");
  }

  if (!model_trt_ptr_->setTensorAddress("output", output_tensor_.data_ptr())) {
    throw std::runtime_error("Failed to set output.");
  }
}

PositionValidatorTRT::~PositionValidatorTRT()
{
  if (stream_) {
    cudaStreamSynchronize(stream_);
    cudaStreamDestroy(stream_);
  }
}

bool PositionValidatorTRT::inference(
  const std::vector<Eigen::Vector4f> & points, int & predicted_class, float & confidence)
{
  // create pillars_, coors_batch_ and npoints_per_pillar_
  preprocess(points);


  // check whether the number of input points is within the profile range
  //
  // TODO(a-maumau): implement a preprocess that will fill or reduce the
  //                 input points within the profile range
  const size_t input_points_num = pillars_.size(0);
  if (
    input_points_num < static_cast<size_t>(profile_points_min_) ||
    input_points_num > static_cast<size_t>(profile_points_max_)) {
    return false;
  }

  const auto pillars_shape = nvinfer1::Dims3{
    static_cast<int>(pillars_.size(0)), static_cast<int>(pillars_.size(1)),
    static_cast<int>(pillars_.size(2))};
  const auto coors_batch_shape = nvinfer1::Dims2{
    static_cast<int>(coors_batch_.size(0)), static_cast<int>(coors_batch_.size(1))};
  const auto npoints_per_pillar_shape = nvinfer1::Dims{1,
    {static_cast<int>(npoints_per_pillar_.size(0))}
  };

  // for dynamic input shapes
  if (
    !model_trt_ptr_->setInputShape("pillars", pillars_shape) ||
    !model_trt_ptr_->setInputShape("coors_batch", coors_batch_shape) ||
    !model_trt_ptr_->setInputShape("npoints_per_pillar", npoints_per_pillar_shape)) {
    throw std::runtime_error("Failed to set input shape.");
  }

  std::vector<void *> input_tensors = {
    pillars_.data_ptr(), coors_batch_.data_ptr(), npoints_per_pillar_.data_ptr()
  };
  model_trt_ptr_->setTensorsAddresses(input_tensors);
  model_trt_ptr_->enqueueV3(stream_);

  // get max values and indices per row
  at::Tensor max_probs, argmax_tensor;
  std::tie(max_probs, argmax_tensor) = output_tensor_.to(at::kCPU).max(1);

  predicted_class = argmax_tensor[0].item<int>();
  confidence = max_probs[0].item<float>();

  return true;
}

void PositionValidatorTRT::preprocess(const std::vector<Eigen::Vector4f> & points)
{
  at::Tensor tensor_cpu = vectorToTensor(points);

  // inherit dtype/device from points
  auto options_float = tensor_cpu.options();
  // same device, int dtype
  auto options_int = tensor_cpu.options().dtype(torch::kInt32);

  // create tensors
  at::Tensor voxels = torch::zeros(
    {max_voxels_, max_points_per_voxel_, tensor_cpu.size(1)}, options_float);
  at::Tensor coors = torch::zeros({max_voxels_, 3}, options_int);
  at::Tensor num_points_per_voxel = torch::zeros({max_voxels_}, options_int);

  at::Tensor tensor_gpu = tensor_cpu.to(at::kCUDA, 0);
  at::Tensor voxels_gpu = voxels.to(at::kCUDA, 0);
  at::Tensor coors_gpu = coors.to(at::kCUDA, 0);
  at::Tensor num_points_per_voxel_gpu = num_points_per_voxel.to(at::kCUDA, 0);

  const int voxel_num = voxelization::hard_voxelize(
    tensor_gpu, voxels_gpu, coors_gpu, num_points_per_voxel_gpu,
    voxel_size_, coors_range_, max_points_per_voxel_, max_voxels_, 3, true);

  pillars_ = voxels_gpu.index({at::indexing::Slice(0, voxel_num)});

  // it might depend on the libtorch version
  //at::Tensor coors_out = coors_gpu.index({at::indexing::Slice(0, voxel_num)}).flip({-1});
  // (z, y, x) -> (x, y, z)
  at::Tensor coors_out = torch::flip(coors_gpu.index({at::indexing::Slice(0, voxel_num)}), {-1});
  // pad 1 column at front with value 0
  coors_batch_ = F::pad(coors_out, F::PadFuncOptions({1, 0}).value(0));

  npoints_per_pillar_ = num_points_per_voxel_gpu.index(
    {at::indexing::Slice(0, voxel_num)});

  TORCH_CHECK(pillars_.is_cuda() && pillars_.is_contiguous());
  TORCH_CHECK(coors_batch_.is_cuda() && coors_batch_.is_contiguous());
  TORCH_CHECK(npoints_per_pillar_.is_cuda() && npoints_per_pillar_.is_contiguous());
}

}  // namespace autoware::localized_position_validator
