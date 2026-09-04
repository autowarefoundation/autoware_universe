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

#include "autoware/ptv3/preprocess/sweep_aggregator.hpp"

#include "autoware/ptv3/preprocess/sweep_kernel.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/point_types/memory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace autoware::ptv3
{
namespace
{

CloudFormat detectCloudFormat(const cuda_blackboard::CudaPointCloud2 & cloud)
{
  const auto & fields = cloud.fields;
  const auto num_fields = fields.size();

  if (num_fields == 10 && point_types::is_data_layout_compatible_with_point_xyzircaedt(fields)) {
    return CloudFormat::XYZIRCAEDT;
  }
  if (num_fields == 9 && point_types::is_data_layout_compatible_with_point_xyziradrt(fields)) {
    return CloudFormat::XYZIRADRT;
  }
  if (num_fields == 6 && point_types::is_data_layout_compatible_with_point_xyzirc(fields)) {
    return CloudFormat::XYZIRC;
  }
  if (num_fields == 4 && point_types::is_data_layout_compatible_with_point_xyzi(fields)) {
    return CloudFormat::XYZI;
  }

  return CloudFormat::UNKNOWN;
}

}  // namespace

SweepAggregator::SweepAggregator(const PTv3Config & config, cudaStream_t stream)
: config_(config), stream_(stream)
{
  densification_ptr_ = std::make_unique<PointCloudDensification>(DensificationParam(
    config_.densification_world_frame_id_,
    static_cast<unsigned int>(config_.densification_num_past_frames_)));

  points_d_ = autoware::cuda_utils::make_unique<float[]>(
    config_.densified_cloud_capacity_ * config_.num_point_feature_size_);
  affine_past2current_d_ =
    autoware::cuda_utils::make_unique<float[]>(Eigen::Affine3f::MatrixType::SizeAtCompileTime);
}

void SweepAggregator::enqueuePointCloud(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr,
  const Eigen::Affine3f & affine_world2current)
{
  densification_ptr_->enqueuePointCloud(msg_ptr, affine_world2current);
}

DensifiedCloud SweepAggregator::aggregate()
{
  DensifiedCloud densified;
  densified.points = points_d_.get();

  std::size_t point_counter{0};

  for (auto cache_iter = densification_ptr_->getPointCloudCacheIter();
       !densification_ptr_->isCacheEnd(cache_iter); cache_iter++) {
    const auto & msg_ptr = cache_iter->input_pointcloud_msg_ptr;
    const auto frame_num_points = static_cast<std::size_t>(msg_ptr->height * msg_ptr->width);
    const bool is_current_frame = densification_ptr_->getIdx(cache_iter) == 0;

    // The segmentation reconstruction buffers and output messages are sized per frame, so the
    // current frame must fit the single-frame capacity, not just the densified one.
    if (is_current_frame && frame_num_points > static_cast<std::size_t>(config_.cloud_capacity_)) {
      throw std::runtime_error(
        "The current frame (" + std::to_string(frame_num_points) +
        " points) exceeds the cloud capacity (" + std::to_string(config_.cloud_capacity_) +
        "). Increase cloud_capacity.");
    }
    if (
      point_counter + frame_num_points >
      static_cast<std::size_t>(config_.densified_cloud_capacity_)) {
      if (is_current_frame) {
        throw std::runtime_error(
          "The current frame (" + std::to_string(frame_num_points) +
          " points) exceeds the densified cloud capacity (" +
          std::to_string(config_.densified_cloud_capacity_) + "). Increase cloud_capacity.");
      }
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("ptv3"), "Exceeding densified cloud capacity. Used "
                                      << densification_ptr_->getIdx(cache_iter) << " out of "
                                      << densification_ptr_->getCacheSize() << " frame(s)");
      break;
    }

    const auto format = detectCloudFormat(*msg_ptr);
    if (format == CloudFormat::UNKNOWN) {
      throw std::runtime_error(
        "Unsupported point cloud type. Expected one of: XYZIRCAEDT (10 fields), "
        "XYZIRADRT (9 fields), XYZIRC (6 fields), or XYZI (4 fields).");
    }

    if (is_current_frame) {
      densified.num_current_points = frame_num_points;
      densified.current_msg = msg_ptr;
      densified.current_format = format;
    }

    Eigen::Affine3f affine_past2current =
      densification_ptr_->getAffineWorldToCurrent() * cache_iter->affine_past2world;
    static_assert(!Eigen::Matrix4f::IsRowMajor, "matrices should be col-major.");

    const auto time_lag = static_cast<float>(
      densification_ptr_->getCurrentTimestamp() - rclcpp::Time(msg_ptr->header.stamp).seconds());

    CHECK_CUDA_ERROR(cudaMemcpyAsync(
      affine_past2current_d_.get(), affine_past2current.data(),
      Eigen::Affine3f::MatrixType::SizeAtCompileTime * sizeof(float), cudaMemcpyHostToDevice,
      stream_));
    CHECK_CUDA_ERROR(cudaStreamSynchronize(stream_));

    generateSweepFeaturesLaunch(
      msg_ptr->data.get(), format, frame_num_points, is_current_frame ? 0.f : time_lag,
      config_.sweep_close_radius_, affine_past2current_d_.get(), config_.num_point_feature_size_,
      points_d_.get() + point_counter * config_.num_point_feature_size_, config_.threads_per_block_,
      stream_);
    CHECK_CUDA_ERROR(cudaPeekAtLastError());

    point_counter += frame_num_points;
  }

  densified.num_points = point_counter;
  return densified;
}

}  // namespace autoware::ptv3
