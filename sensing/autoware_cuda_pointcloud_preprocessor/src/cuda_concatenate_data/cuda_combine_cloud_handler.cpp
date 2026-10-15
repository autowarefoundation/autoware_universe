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

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_combine_cloud_handler.hpp"

#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_combine_cloud_handler_kernel.hpp"
#include "autoware/cuda_pointcloud_preprocessor/cuda_concatenate_data/cuda_traits.hpp"

#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/pointcloud_preprocessor/concatenate_data/concatenation_info_manager.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>

#include <cuda_runtime.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

CombineCloudHandler<CudaPointCloud2Traits>::CombineCloudHandler(
  rclcpp::Node & node, const std::vector<std::string> & input_topics, std::string output_frame,
  bool is_motion_compensated, bool publish_synchronized_pointcloud,
  bool keep_input_frame_in_synchronized_pointcloud, OutputPointType output_point_type)
: CombineCloudHandlerBase(
    node, input_topics, output_frame, is_motion_compensated, publish_synchronized_pointcloud,
    keep_input_frame_in_synchronized_pointcloud, output_point_type)
{
  for (const auto & topic : input_topics_) {
    CudaConcatStruct cuda_concat_struct;
    CHECK_CUDA_ERROR(cudaStreamCreate(&cuda_concat_struct.stream));
    cuda_concat_struct_map_[topic] = std::move(cuda_concat_struct);
  }
}

void CombineCloudHandler<CudaPointCloud2Traits>::allocate_pointclouds()
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & topic : input_topics_) {
    auto & concat_struct = cuda_concat_struct_map_[topic];
    concat_struct.cloud_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    concat_struct.cloud_ptr->data =
      cuda_blackboard::make_unique<std::uint8_t[]>(concat_struct.max_pointcloud_size);
  }

  concatenated_cloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
  concatenated_cloud_ptr_->data =
    cuda_blackboard::make_unique<std::uint8_t[]>(max_concat_pointcloud_size_);
}

bool CombineCloudHandler<CudaPointCloud2Traits>::is_input_layout_supported(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  // The kernels stride the buffer by the output point size, so the input must have exactly the
  // output layout.
  if (cloud.point_step != output_point_step_) {
    return false;
  }
  return output_point_type_ == OutputPointType::XYZIRCT
           ? autoware::point_types::is_data_layout_compatible_with_point_xyzirct(cloud.fields)
           : autoware::point_types::is_data_layout_compatible_with_point_xyzirc(cloud.fields);
}

void CombineCloudHandler<CudaPointCloud2Traits>::launch_transform(
  const std::uint8_t * input, int num_points, const TransformStruct & transform,
  std::uint32_t time_offset_ns, std::uint8_t * output, cudaStream_t & stream) const
{
  if (output_point_type_ == OutputPointType::XYZIRCT) {
    transform_launch(
      reinterpret_cast<const PointXYZIRCT *>(input), num_points, transform, time_offset_ns,
      reinterpret_cast<PointXYZIRCT *>(output), stream);
  } else {
    transform_launch(
      reinterpret_cast<const PointXYZIRC *>(input), num_points, transform, time_offset_ns,
      reinterpret_cast<PointXYZIRC *>(output), stream);
  }
}

ConcatenatedCloudResult<CudaPointCloud2Traits>
CombineCloudHandler<CudaPointCloud2Traits>::combine_pointclouds(
  std::unordered_map<
    std::string, typename CudaPointCloud2Traits::PointCloudMessage::ConstSharedPtr> &
    topic_to_cloud_map,
  const std::shared_ptr<CollectorInfoBase> & collector_info)
{
  ConcatenatedCloudResult<CudaPointCloud2Traits> concatenate_cloud_result;
  std::lock_guard<std::mutex> lock(mutex_);

  if (topic_to_cloud_map.empty()) return concatenate_cloud_result;

  std::vector<rclcpp::Time> pc_stamps;
  pc_stamps.reserve(topic_to_cloud_map.size());

  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    pc_stamps.emplace_back(cloud->header.stamp);
    concatenate_cloud_result.topic_to_original_stamp_map[topic] =
      rclcpp::Time(cloud->header.stamp).seconds();
  }

  std::sort(pc_stamps.begin(), pc_stamps.end(), std::greater<rclcpp::Time>());
  auto oldest_stamp = pc_stamps.back();

  // Before combining the pointclouds, initialize and reserve space for the concatenated pointcloud
  concatenate_cloud_result.concatenate_cloud_ptr =
    std::make_unique<cuda_blackboard::CudaPointCloud2>();
  concatenate_cloud_result.concatenation_info_ptr =
    std::make_unique<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>(
      concatenation_info_manager_.reset_and_get_base_info());

  // Reserve space based on the total size of the pointcloud data to speed up the concatenation
  // process
  size_t total_data_size = 0;
  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    total_data_size += (cloud->height * cloud->row_step);
  }

  constexpr int CHUNK_SIZE = 1024;

  // Resize concatenated cloud if needed to the next multiple of CHUNK_SIZE to reduce the number of
  // reallocations
  if (total_data_size > max_concat_pointcloud_size_ || !concatenated_cloud_ptr_) {
    max_concat_pointcloud_size_ = CHUNK_SIZE * (1 + total_data_size / CHUNK_SIZE);
    concatenated_cloud_ptr_ = std::make_unique<cuda_blackboard::CudaPointCloud2>();
    concatenated_cloud_ptr_->data =
      cuda_blackboard::make_unique<std::uint8_t[]>(max_concat_pointcloud_size_);
  }

  concatenate_cloud_result.concatenate_cloud_ptr = std::move(concatenated_cloud_ptr_);

  auto * output_points = concatenate_cloud_result.concatenate_cloud_ptr->data.get();
  std::size_t concatenated_start_index = 0;

  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    const std::size_t num_points = cloud->height * cloud->width;

    // Compute motion compensation transform
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // Transform if needed
    auto transform_opt = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
      output_frame_, cloud->header.frame_id, node_.now(), rclcpp::Duration::from_seconds(1.0),
      node_.get_logger());

    if (transform_opt) {
      transform = *transform_opt;
    }

    rclcpp::Time current_cloud_stamp = rclcpp::Time(cloud->header.stamp);

    if (is_motion_compensated_) {
      transform = compute_transform_to_adjust_for_old_timestamp(oldest_stamp, current_cloud_stamp) *
                  transform;
    }

    TransformStruct transform_struct;
    transform_struct.translation_x = transform(0, 3);
    transform_struct.translation_y = transform(1, 3);
    transform_struct.translation_z = transform(2, 3);
    transform_struct.m11 = transform(0, 0);
    transform_struct.m12 = transform(0, 1);
    transform_struct.m13 = transform(0, 2);
    transform_struct.m21 = transform(1, 0);
    transform_struct.m22 = transform(1, 1);
    transform_struct.m23 = transform(1, 2);
    transform_struct.m31 = transform(2, 0);
    transform_struct.m32 = transform(2, 1);
    transform_struct.m33 = transform(2, 2);

    // Point times, if kept, are rebased from this cloud's header stamp onto oldest_stamp.
    const auto time_offset_ns =
      static_cast<std::uint32_t>((current_cloud_stamp - oldest_stamp).nanoseconds());
    auto & stream = cuda_concat_struct_map_[topic].stream;
    launch_transform(
      cloud->data.get(), num_points, transform_struct, time_offset_ns,
      output_points + concatenated_start_index * output_point_step_, stream);
    concatenated_start_index += num_points;
    concatenation_info_manager_.update_source_from_point_cloud(
      *cloud, topic, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
      *concatenate_cloud_result.concatenation_info_ptr);
  }

  concatenate_cloud_result.concatenate_cloud_ptr->header.frame_id = output_frame_;
  concatenate_cloud_result.concatenate_cloud_ptr->width = concatenated_start_index;
  concatenate_cloud_result.concatenate_cloud_ptr->height = 1;
  concatenate_cloud_result.concatenate_cloud_ptr->point_step = output_point_step_;
  concatenate_cloud_result.concatenate_cloud_ptr->row_step =
    concatenated_start_index * output_point_step_;
  concatenate_cloud_result.concatenate_cloud_ptr->fields = output_fields_;
  concatenate_cloud_result.concatenate_cloud_ptr->is_bigendian = false;
  concatenate_cloud_result.concatenate_cloud_ptr->is_dense = true;

  // Second round is for when we need to publish sync pointclouds
  if (publish_synchronized_pointcloud_) {
    if (!concatenate_cloud_result.topic_to_transformed_cloud_map) {
      // Initialize the map if it is not present
      concatenate_cloud_result.topic_to_transformed_cloud_map =
        std::unordered_map<std::string, cuda_blackboard::CudaPointCloud2::UniquePtr>();
    }

    concatenated_start_index = 0;

    for (const auto & [topic, cloud] : topic_to_cloud_map) {
      const std::size_t num_points = cloud->height * cloud->width;
      const std::size_t data_size = cloud->height * cloud->row_step;

      auto & concat_struct = cuda_concat_struct_map_[topic];

      if (data_size > concat_struct.max_pointcloud_size || !concat_struct.cloud_ptr) {
        concat_struct.max_pointcloud_size = (data_size + 1024) / 1024 * 1024;
        concat_struct.cloud_ptr = std::make_unique<cuda_blackboard::CudaPointCloud2>();
        concat_struct.cloud_ptr->data = cuda_blackboard::make_unique<std::uint8_t[]>(data_size);
      }
      // convert to original sensor frame if necessary

      auto & output_cloud = (*concatenate_cloud_result.topic_to_transformed_cloud_map)[topic];
      bool need_transform_to_sensor_frame = (cloud->header.frame_id != output_frame_);

      output_cloud = std::move(concat_struct.cloud_ptr);

      auto & stream = cuda_concat_struct_map_[topic].stream;

      if (keep_input_frame_in_synchronized_pointcloud_ && need_transform_to_sensor_frame) {
        Eigen::Matrix4f transform;
        auto transform_opt = managed_tf_buffer_->getTransform<Eigen::Matrix4f>(
          cloud->header.frame_id, output_frame_, node_.now(), rclcpp::Duration::from_seconds(1.0),
          node_.get_logger());

        if (transform_opt.has_value()) {
          transform = *transform_opt;
        }

        TransformStruct transform_struct;
        transform_struct.translation_x = transform(0, 3);
        transform_struct.translation_y = transform(1, 3);
        transform_struct.translation_z = transform(2, 3);
        transform_struct.m11 = transform(0, 0);
        transform_struct.m12 = transform(0, 1);
        transform_struct.m13 = transform(0, 2);
        transform_struct.m21 = transform(1, 0);
        transform_struct.m22 = transform(1, 1);
        transform_struct.m23 = transform(1, 2);
        transform_struct.m31 = transform(2, 0);
        transform_struct.m32 = transform(2, 1);
        transform_struct.m33 = transform(2, 2);

        // Point times were already rebased in the first round.
        launch_transform(
          output_points + concatenated_start_index * output_point_step_, num_points,
          transform_struct, 0U, output_cloud->data.get(), stream);
        output_cloud->header.frame_id = cloud->header.frame_id;
      } else {
        CHECK_CUDA_ERROR(cudaMemcpyAsync(
          output_cloud->data.get(), output_points + concatenated_start_index * output_point_step_,
          data_size, cudaMemcpyDeviceToDevice, stream));
        output_cloud->header.frame_id = output_frame_;
      }

      output_cloud->header.stamp = oldest_stamp;
      output_cloud->width = cloud->width;
      output_cloud->height = cloud->height;
      output_cloud->point_step = output_point_step_;
      output_cloud->row_step = cloud->width * output_point_step_;
      output_cloud->fields = output_fields_;
      output_cloud->is_bigendian = false;
      output_cloud->is_dense = true;

      concatenated_start_index += cloud->height * cloud->width;
    }
  }

  // Sync all streams
  for (const auto & [topic, cuda_concat_struct] : cuda_concat_struct_map_) {
    CHECK_CUDA_ERROR(cudaStreamSynchronize(cuda_concat_struct.stream));
  }

  concatenate_cloud_result.concatenate_cloud_ptr->header.stamp = oldest_stamp;

  if (const auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(collector_info)) {
    const auto reference_timestamp_min = advanced_info->timestamp - advanced_info->noise_window;
    const auto reference_timestamp_max = advanced_info->timestamp + advanced_info->noise_window;

    builtin_interfaces::msg::Time reference_timestamp_min_msg;
    reference_timestamp_min_msg.sec = static_cast<int32_t>(reference_timestamp_min);
    reference_timestamp_min_msg.nanosec =
      static_cast<uint32_t>((reference_timestamp_min - reference_timestamp_min_msg.sec) * 1e9);

    builtin_interfaces::msg::Time reference_timestamp_max_msg;
    reference_timestamp_max_msg.sec = static_cast<int32_t>(reference_timestamp_max);
    reference_timestamp_max_msg.nanosec =
      static_cast<uint32_t>((reference_timestamp_max - reference_timestamp_max_msg.sec) * 1e9);

    StrategyAdvancedConfig strategy_config(
      reference_timestamp_min_msg, reference_timestamp_max_msg);
    auto serialized_config = strategy_config.serialize();
    ConcatenationInfoManager::set_config(
      serialized_config, *concatenate_cloud_result.concatenation_info_ptr);
  }

  concatenation_info_manager_.set_result(
    *concatenate_cloud_result.concatenate_cloud_ptr,
    *concatenate_cloud_result.concatenation_info_ptr);

  return concatenate_cloud_result;
}

}  // namespace autoware::pointcloud_preprocessor

template class autoware::pointcloud_preprocessor::CombineCloudHandler<
  autoware::pointcloud_preprocessor::CudaPointCloud2Traits>;
