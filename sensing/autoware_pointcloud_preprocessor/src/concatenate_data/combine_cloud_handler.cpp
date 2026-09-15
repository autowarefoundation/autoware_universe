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

#include "autoware/pointcloud_preprocessor/concatenate_data/combine_cloud_handler.hpp"

#include "autoware/pointcloud_preprocessor/concatenate_data/concatenation_info_manager.hpp"

#include <pcl_ros/transforms.hpp>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

bool CombineCloudHandler<PointCloud2Traits>::is_input_layout_supported(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  // Fields are read by name, so any cloud that carries the output fields can be rewritten.
  return autoware::point_types::is_data_convertible_to(cloud.fields, output_fields_);
}

template <typename PointT, typename GeneratorT>
void CombineCloudHandler<PointCloud2Traits>::convert_cloud(
  const typename PointCloud2Traits::PointCloudMessage::ConstSharedPtr & input_cloud,
  typename PointCloud2Traits::PointCloudMessage::UniquePtr & output_cloud,
  std::uint32_t time_offset_ns)
{
  constexpr bool has_time_stamp = std::is_same_v<PointT, PointXYZIRCT>;

  output_cloud->header = input_cloud->header;

  PointCloud2Modifier<PointT, GeneratorT> output_modifier{
    *output_cloud, input_cloud->header.frame_id};
  output_modifier.reserve(input_cloud->width);

  sensor_msgs::PointCloud2ConstIterator<float> it_x(*input_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*input_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*input_cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_i(*input_cloud, "intensity");
  sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_r(*input_cloud, "return_type");
  sensor_msgs::PointCloud2ConstIterator<std::uint16_t> it_c(*input_cloud, "channel");
  std::optional<sensor_msgs::PointCloud2ConstIterator<std::uint32_t>> it_t;
  if constexpr (has_time_stamp) {
    it_t.emplace(*input_cloud, "time_stamp");
  }

  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
    PointT point;
    point.x = *it_x;
    point.y = *it_y;
    point.z = *it_z;
    point.intensity = *it_i;
    point.return_type = *it_r;
    point.channel = *it_c;
    if constexpr (has_time_stamp) {
      point.time_stamp = time_offset_ns + **it_t;
      ++*it_t;
    }
    output_modifier.push_back(std::move(point));
  }
}

void CombineCloudHandler<PointCloud2Traits>::convert_to_output_cloud(
  const typename PointCloud2Traits::PointCloudMessage::ConstSharedPtr & input_cloud,
  typename PointCloud2Traits::PointCloudMessage::UniquePtr & output_cloud,
  std::uint32_t time_offset_ns) const
{
  if (output_point_type_ == OutputPointType::XYZIRCT) {
    convert_cloud<PointXYZIRCT, autoware::point_types::PointXYZIRCTGenerator>(
      input_cloud, output_cloud, time_offset_ns);
  } else {
    convert_cloud<PointXYZIRC, autoware::point_types::PointXYZIRCGenerator>(
      input_cloud, output_cloud, 0U);
  }
}

void CombineCloudHandler<PointCloud2Traits>::correct_pointcloud_motion(
  const std::unique_ptr<PointCloud2Traits::PointCloudMessage> & transformed_cloud_ptr,
  const std::vector<rclcpp::Time> & pc_stamps,
  std::unordered_map<rclcpp::Time, Eigen::Matrix4f, RclcppTimeHash> & transform_memo,
  std::unique_ptr<PointCloud2Traits::PointCloudMessage> & transformed_delay_compensated_cloud_ptr)
{
  Eigen::Matrix4f adjust_to_old_data_transform = Eigen::Matrix4f::Identity();
  rclcpp::Time current_cloud_stamp = rclcpp::Time(transformed_cloud_ptr->header.stamp);
  for (const auto & stamp : pc_stamps) {
    if (stamp >= current_cloud_stamp) continue;

    Eigen::Matrix4f new_to_old_transform;
    if (transform_memo.find(stamp) != transform_memo.end()) {
      new_to_old_transform = transform_memo[stamp];
    } else {
      new_to_old_transform =
        compute_transform_to_adjust_for_old_timestamp(stamp, current_cloud_stamp);
      transform_memo[stamp] = new_to_old_transform;
    }
    adjust_to_old_data_transform = new_to_old_transform * adjust_to_old_data_transform;
    current_cloud_stamp = stamp;
  }
  pcl_ros::transformPointCloud(
    adjust_to_old_data_transform, *transformed_cloud_ptr, *transformed_delay_compensated_cloud_ptr);
}

// TODO(vividf): refactor this function for readability
ConcatenatedCloudResult<PointCloud2Traits>
CombineCloudHandler<PointCloud2Traits>::combine_pointclouds(
  std::unordered_map<std::string, PointCloud2Traits::PointCloudMessage::ConstSharedPtr> &
    topic_to_cloud_map,
  const std::shared_ptr<CollectorInfoBase> & collector_info)
{
  ConcatenatedCloudResult<PointCloud2Traits> concatenate_cloud_result;

  if (topic_to_cloud_map.empty()) return concatenate_cloud_result;

  std::vector<rclcpp::Time> pc_stamps;
  pc_stamps.reserve(topic_to_cloud_map.size());

  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    pc_stamps.emplace_back(cloud->header.stamp);
    concatenate_cloud_result.topic_to_original_stamp_map[topic] =
      rclcpp::Time(cloud->header.stamp).seconds();
  }
  std::sort(pc_stamps.begin(), pc_stamps.end(), std::greater<rclcpp::Time>());
  const auto oldest_stamp = pc_stamps.back();

  std::unordered_map<rclcpp::Time, Eigen::Matrix4f, RclcppTimeHash> transform_memo;

  // Before combining the pointclouds, initialize and reserve space for the concatenated pointcloud
  concatenate_cloud_result.concatenate_cloud_ptr =
    std::make_unique<sensor_msgs::msg::PointCloud2>();
  concatenate_cloud_result.concatenation_info_ptr =
    std::make_unique<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>(
      concatenation_info_manager_.reset_and_get_base_info());
  {
    // Normally, pcl::concatenatePointCloud() copies the field layout (e.g., XYZIRC)
    // from the non-empty point cloud when given one empty and one non-empty input.
    //
    // However, if all input clouds in topic_to_cloud_map are empty,
    // the function receives two empty point clouds and does nothing,
    // resulting in concatenate_cloud_ptr not carrying the output layout.
    //
    // To avoid this, we explicitly set the fields of concatenate_cloud_ptr here.
    auto & concatenate_cloud = *concatenate_cloud_result.concatenate_cloud_ptr;
    if (output_point_type_ == OutputPointType::XYZIRCT) {
      PointCloud2Modifier<PointXYZIRCT, autoware::point_types::PointXYZIRCTGenerator>
        concatenate_cloud_modifier{concatenate_cloud, output_frame_};
    } else {
      PointCloud2Modifier<PointXYZIRC, autoware::point_types::PointXYZIRCGenerator>
        concatenate_cloud_modifier{concatenate_cloud, output_frame_};
    }
  }
  bool is_concatenated_cloud_dense = true;

  // Reserve space based on the total size of the pointcloud data to speed up the concatenation
  // process
  size_t total_data_size = 0;
  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    total_data_size += cloud->data.size();
  }
  concatenate_cloud_result.concatenate_cloud_ptr->data.reserve(total_data_size);

  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    // Rewrite into the output point type. Point times, if kept, are rebased from this cloud's
    // header stamp onto the concatenated cloud's (oldest_stamp).
    const auto time_offset_ns =
      static_cast<std::uint32_t>((rclcpp::Time(cloud->header.stamp) - oldest_stamp).nanoseconds());
    auto converted_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    convert_to_output_cloud(cloud, converted_cloud, time_offset_ns);

    auto transformed_cloud_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    managed_tf_buffer_->transformPointcloud(
      output_frame_, *converted_cloud, *transformed_cloud_ptr, converted_cloud->header.stamp,
      rclcpp::Duration::from_seconds(1.0), node_.get_logger());

    // compensate pointcloud
    std::unique_ptr<sensor_msgs::msg::PointCloud2> transformed_delay_compensated_cloud_ptr;
    if (is_motion_compensated_) {
      transformed_delay_compensated_cloud_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
      correct_pointcloud_motion(
        transformed_cloud_ptr, pc_stamps, transform_memo, transformed_delay_compensated_cloud_ptr);
    } else {
      transformed_delay_compensated_cloud_ptr = std::move(transformed_cloud_ptr);
    }

    if (
      transformed_delay_compensated_cloud_ptr->width *
        transformed_delay_compensated_cloud_ptr->height >
      0) {
      pcl::concatenatePointCloud(
        *concatenate_cloud_result.concatenate_cloud_ptr, *transformed_delay_compensated_cloud_ptr,
        *concatenate_cloud_result.concatenate_cloud_ptr);
      is_concatenated_cloud_dense = is_concatenated_cloud_dense && cloud->is_dense;
    }

    // update concatenation info
    concatenation_info_manager_.update_source_from_point_cloud(
      *transformed_delay_compensated_cloud_ptr, topic,
      autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK,
      *concatenate_cloud_result.concatenation_info_ptr);

    if (publish_synchronized_pointcloud_) {
      if (!concatenate_cloud_result.topic_to_transformed_cloud_map) {
        // Initialize the map if it is not present
        concatenate_cloud_result.topic_to_transformed_cloud_map =
          std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::UniquePtr>();
      }
      // convert to original sensor frame if necessary
      bool need_transform_to_sensor_frame = (cloud->header.frame_id != output_frame_);
      if (keep_input_frame_in_synchronized_pointcloud_ && need_transform_to_sensor_frame) {
        auto transformed_cloud_ptr_in_sensor_frame =
          std::make_unique<sensor_msgs::msg::PointCloud2>();
        managed_tf_buffer_->transformPointcloud(
          cloud->header.frame_id, *transformed_delay_compensated_cloud_ptr,
          *transformed_cloud_ptr_in_sensor_frame,
          transformed_cloud_ptr_in_sensor_frame->header.stamp, rclcpp::Duration::from_seconds(1.0),
          node_.get_logger());
        transformed_cloud_ptr_in_sensor_frame->header.stamp = oldest_stamp;
        transformed_cloud_ptr_in_sensor_frame->header.frame_id = cloud->header.frame_id;

        (*concatenate_cloud_result.topic_to_transformed_cloud_map)[topic] =
          std::move(transformed_cloud_ptr_in_sensor_frame);
      } else {
        transformed_delay_compensated_cloud_ptr->header.stamp = oldest_stamp;
        transformed_delay_compensated_cloud_ptr->header.frame_id = output_frame_;
        (*concatenate_cloud_result.topic_to_transformed_cloud_map)[topic] =
          std::move(transformed_delay_compensated_cloud_ptr);
      }
    }
  }
  concatenate_cloud_result.concatenate_cloud_ptr->header.stamp = oldest_stamp;
  concatenate_cloud_result.concatenate_cloud_ptr->is_dense = is_concatenated_cloud_dense;

  // concatenated cloud is no longer structured so recalculate the height, width, and row_step
  concatenate_cloud_result.concatenate_cloud_ptr->height = 1;
  {
    const auto & data_size = concatenate_cloud_result.concatenate_cloud_ptr->data.size();
    const auto & point_step = concatenate_cloud_result.concatenate_cloud_ptr->point_step;
    if (data_size % point_step != 0) {
      throw std::runtime_error("PointCloud2 data size is not divisible by point_step");
    }
    concatenate_cloud_result.concatenate_cloud_ptr->row_step = data_size;
    concatenate_cloud_result.concatenate_cloud_ptr->width = data_size / point_step;
  }

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
