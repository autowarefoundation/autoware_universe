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

#include "autoware/lidar_frnet/lidar_frnet.hpp"

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <autoware/cuda_utils/cuda_utils.hpp>
#include <autoware/point_types/memory.hpp>
#include <autoware/point_types/types.hpp>
#include <autoware/tensorrt_common/tensorrt_common.hpp>
#include <autoware/tensorrt_common/utils.hpp>
#include <rclcpp/logging.hpp>

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::lidar_frnet
{

LidarFRNet::LidarFRNet(
  const TrtCommonConfig & trt_config, const utils::NetworkParams & network_params,
  const utils::PreprocessingParams & preprocessing_params,
  const utils::PostprocessingParams & postprocessing_params, const rclcpp::Logger & logger)
: logger_(logger),
  network_params_(network_params),
  preprocessing_params_(preprocessing_params),
  postprocessing_params_(postprocessing_params)
{
  auto profile_dims = std::make_unique<std::vector<ProfileDims>>(std::initializer_list<ProfileDims>{
    ProfileDims(
      "points", {2, {network_params.num_points_profile.min, 4}},
      {2, {network_params.num_points_profile.opt, 4}},
      {2, {network_params.num_points_profile.max, 4}}),
    ProfileDims(
      "coors", {2, {network_params.num_points_profile.min, 3}},
      {2, {network_params.num_points_profile.opt, 3}},
      {2, {network_params.num_points_profile.max, 3}}),
    ProfileDims(
      "voxel_coors", {2, {network_params.num_unique_coors_profile.min, 3}},
      {2, {network_params.num_unique_coors_profile.opt, 3}},
      {2, {network_params.num_unique_coors_profile.max, 3}}),
    ProfileDims(
      "inverse_map", {1, {network_params.num_points_profile.min}},
      {1, {network_params.num_points_profile.opt}}, {1, {network_params.num_points_profile.max}})});

  auto network_io = std::make_unique<std::vector<NetworkIO>>(std::initializer_list<NetworkIO>{
    NetworkIO("points", {2, {-1, 4}}), NetworkIO("coors", {2, {-1, 3}}),
    NetworkIO("voxel_coors", {2, {-1, 3}}), NetworkIO("inverse_map", {1, {-1}}),
    NetworkIO("seg_logit", {2, {-1, network_params_.num_classes}})});

  network_ptr_ = std::make_unique<TrtCommon>(trt_config);
  if (!network_ptr_->setup(std::move(profile_dims), std::move(network_io))) {
    throw std::runtime_error("Failed to setup network");
  }

  CHECK_CUDA_ERROR(cudaStreamCreate(&stream_));
  preprocess_ptr_ = std::make_unique<PreprocessCuda>(preprocessing_params, stream_);
  postprocess_ptr_ = std::make_unique<PostprocessCuda>(postprocessing_params, stream_);

  initTensors();

  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("processing/inner");
}

bool LidarFRNet::process(
  const sensor_msgs::msg::PointCloud2 & cloud_in, sensor_msgs::msg::PointCloud2 & cloud_seg_out,
  sensor_msgs::msg::PointCloud2 & cloud_viz_out, sensor_msgs::msg::PointCloud2 & cloud_filtered,
  const utils::ActiveComm & active_comm, std::unordered_map<std::string, double> & proc_timing)
{
  stop_watch_ptr_->toc("processing/inner", true);
  std::call_once(init_cloud_, [&cloud_in]() {
    if (!point_types::is_data_layout_compatible_with_point_xyzirc(cloud_in.fields)) {
      throw std::runtime_error("Unsupported point cloud type. Expected XYZIRC type.");
    }
  });

  const auto input_num_points = cloud_in.width * cloud_in.height;
  if (
    input_num_points < network_params_.num_points_profile.min ||
    input_num_points > network_params_.num_points_profile.max) {
    RCLCPP_ERROR(
      logger_,
      "Number of points (%d) in input pointcloud does not match the profile. Consider to update "
      "num_points in config file [min: %ld, opt: %ld, max: %ld].",
      input_num_points, network_params_.num_points_profile.min,
      network_params_.num_points_profile.opt, network_params_.num_points_profile.max);
    return false;
  }

  cuda_utils::clear_async(cloud_in_d_.get(), network_params_.num_points_profile.max, stream_);
  CHECK_CUDA_ERROR(cudaMemcpyAsync(
    cloud_in_d_.get(), cloud_in.data.data(), sizeof(InputPointType) * input_num_points,
    cudaMemcpyHostToDevice));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (!preprocess(input_num_points)) {
    RCLCPP_ERROR(logger_, "Preprocess failed.");
    return false;
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  proc_timing.emplace(
    "debug/processing_time/preprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!inference()) {
    RCLCPP_ERROR(logger_, "Inference failed.");
    return false;
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  proc_timing.emplace(
    "debug/processing_time/inference_ms", stop_watch_ptr_->toc("processing/inner", true));

  if (!postprocess(input_num_points, active_comm, cloud_seg_out, cloud_viz_out, cloud_filtered)) {
    RCLCPP_ERROR(logger_, "Postprocess failed.");
    return false;
  }
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  proc_timing.emplace(
    "debug/processing_time/postprocess_ms", stop_watch_ptr_->toc("processing/inner", true));

  return true;
}

bool LidarFRNet::preprocess(const uint32_t input_num_points)
{
  cuda_utils::clear_async(coors_keys_d_.get(), network_params_.num_points_profile.max, stream_);
  cuda_utils::clear_async(num_points_d_.get(), 1, stream_);
  cuda_utils::clear_async(
    proj_idxs_d_.get(),
    preprocessing_params_.interpolation.w * preprocessing_params_.interpolation.h, stream_);
  cuda_utils::clear_async(
    proj_2d_d_.get(), preprocessing_params_.interpolation.w * preprocessing_params_.interpolation.h,
    stream_);
  cuda_utils::clear_async(points_d_.get(), network_params_.num_points_profile.max * 4, stream_);
  cuda_utils::clear_async(coors_d_.get(), network_params_.num_points_profile.max * 3, stream_);
  cuda_utils::clear_async(
    voxel_coors_d_.get(), network_params_.num_unique_coors_profile.max * 3, stream_);
  cuda_utils::clear_async(inverse_map_d_.get(), network_params_.num_points_profile.max, stream_);
  uint32_t num_unique_coors{0};

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  CHECK_CUDA_ERROR(preprocess_ptr_->projectPoints_launch(
    cloud_in_d_.get(), input_num_points, num_points_d_.get(), points_d_.get(), coors_d_.get(),
    coors_keys_d_.get(), proj_idxs_d_.get(), proj_2d_d_.get()));
  CHECK_CUDA_ERROR(preprocess_ptr_->interpolatePoints_launch(
    proj_idxs_d_.get(), proj_2d_d_.get(), num_points_d_.get(), points_d_.get(), coors_d_.get(),
    coors_keys_d_.get()));

  uint32_t num_points{};
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  CHECK_CUDA_ERROR(
    cudaMemcpy(&num_points, num_points_d_.get(), sizeof(uint32_t), cudaMemcpyDeviceToHost));

  if (
    num_points < network_params_.num_points_profile.min ||
    num_points > network_params_.num_points_profile.max) {
    RCLCPP_ERROR(
      logger_,
      "Number of points (%d) after interpolation does not match the profile. Consider to update "
      "num_points in config file [min: %ld, opt: %ld, max: %ld].",
      num_points, network_params_.num_points_profile.min, network_params_.num_points_profile.opt,
      network_params_.num_points_profile.max);
    return false;
  }

  preprocess_ptr_->generateUniqueCoors(
    num_points, coors_d_.get(), coors_keys_d_.get(), num_unique_coors, voxel_coors_d_.get(),
    inverse_map_d_.get());

  if (
    num_unique_coors < network_params_.num_unique_coors_profile.min ||
    num_unique_coors > network_params_.num_unique_coors_profile.max) {
    RCLCPP_ERROR(
      logger_,
      "Number of unique coors (%d) does not match the profile. Consider to update num_unique_coors "
      "in "
      "config file [min: %ld, opt: %ld, max: %ld].",
      num_unique_coors, network_params_.num_unique_coors_profile.min,
      network_params_.num_unique_coors_profile.opt, network_params_.num_unique_coors_profile.max);
    return false;
  }

  network_ptr_->setInputShape("points", {2, {num_points, 4}});
  network_ptr_->setInputShape("coors", {2, {num_points, 3}});
  network_ptr_->setInputShape("voxel_coors", {2, {num_unique_coors, 3}});
  network_ptr_->setInputShape("inverse_map", {1, {num_points}});

  return true;
}

bool LidarFRNet::inference()
{
  network_ptr_->enqueueV3(stream_);
  return true;
}

bool LidarFRNet::postprocess(
  const uint32_t input_num_points, const utils::ActiveComm & active_comm,
  sensor_msgs::msg::PointCloud2 & cloud_seg_out, sensor_msgs::msg::PointCloud2 & cloud_viz_out,
  sensor_msgs::msg::PointCloud2 & cloud_filtered)
{
  cuda_utils::clear_async(seg_data_d_.get(), network_params_.num_points_profile.max, stream_);
  cuda_utils::clear_async(viz_data_d_.get(), network_params_.num_points_profile.max, stream_);
  cuda_utils::clear_async(cloud_filtered_d_.get(), network_params_.num_points_profile.max, stream_);
  cuda_utils::clear_async(num_points_filtered_d_.get(), 1, stream_);
  uint32_t num_points_filtered{0};

  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
  CHECK_CUDA_ERROR(postprocess_ptr_->fillCloud_launch(
    cloud_in_d_.get(), seg_logit_d_.get(), input_num_points, active_comm,
    num_points_filtered_d_.get(), seg_data_d_.get(), viz_data_d_.get(), cloud_filtered_d_.get()));
  CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

  if (active_comm.seg) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      cloud_seg_out.data.data(), seg_data_d_.get(),
      sizeof(OutputSegmentationPointType) * input_num_points, cudaMemcpyDeviceToHost, stream_));
  }

  if (active_comm.viz) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      cloud_viz_out.data.data(), viz_data_d_.get(),
      sizeof(OutputVisualizationPointType) * input_num_points, cudaMemcpyDeviceToHost, stream_));
  }

  if (active_comm.filtered) {
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      &num_points_filtered, num_points_filtered_d_.get(), sizeof(uint32_t), cudaMemcpyDeviceToHost,
      stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      cloud_filtered.data.data(), cloud_filtered_d_.get(),
      sizeof(InputPointType) * num_points_filtered, cudaMemcpyDeviceToHost, stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));
    cloud_filtered.data.resize(num_points_filtered * cloud_filtered.point_step);
    cloud_filtered.width = num_points_filtered;
    cloud_filtered.row_step = num_points_filtered * cloud_filtered.point_step;
  }
  return true;
}

void LidarFRNet::initTensors()
{
  cloud_in_d_ = cuda_utils::make_unique<InputPointType[]>(network_params_.num_points_profile.max);
  coors_keys_d_ = cuda_utils::make_unique<int64_t[]>(network_params_.num_points_profile.max);
  num_points_d_ = cuda_utils::make_unique<uint32_t[]>(1);
  proj_idxs_d_ = cuda_utils::make_unique<uint32_t[]>(
    preprocessing_params_.interpolation.w * preprocessing_params_.interpolation.h);
  proj_2d_d_ = cuda_utils::make_unique<uint64_t[]>(
    preprocessing_params_.interpolation.w * preprocessing_params_.interpolation.h);
  seg_data_d_ =
    cuda_utils::make_unique<OutputSegmentationPointType[]>(network_params_.num_points_profile.max);
  viz_data_d_ =
    cuda_utils::make_unique<OutputVisualizationPointType[]>(network_params_.num_points_profile.max);
  cloud_filtered_d_ =
    cuda_utils::make_unique<InputPointType[]>(network_params_.num_points_profile.max);
  num_points_filtered_d_ = cuda_utils::make_unique<uint32_t[]>(1);

  points_d_ = cuda_utils::make_unique<float[]>(network_params_.num_points_profile.max * 4);
  coors_d_ = cuda_utils::make_unique<int64_t[]>(network_params_.num_points_profile.max * 3);
  voxel_coors_d_ =
    cuda_utils::make_unique<int64_t[]>(network_params_.num_unique_coors_profile.max * 3);
  inverse_map_d_ = cuda_utils::make_unique<int64_t[]>(network_params_.num_points_profile.max);
  seg_logit_d_ = cuda_utils::make_unique<float[]>(
    network_params_.num_points_profile.max * network_params_.num_classes);

  network_ptr_->setTensorAddress("points", points_d_.get());
  network_ptr_->setTensorAddress("coors", coors_d_.get());
  network_ptr_->setTensorAddress("voxel_coors", voxel_coors_d_.get());
  network_ptr_->setTensorAddress("inverse_map", inverse_map_d_.get());
  network_ptr_->setTensorAddress("seg_logit", seg_logit_d_.get());
}

}  // namespace autoware::lidar_frnet
