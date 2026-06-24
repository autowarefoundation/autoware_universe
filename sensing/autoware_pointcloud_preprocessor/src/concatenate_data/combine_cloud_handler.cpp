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
#include "autoware/pointcloud_preprocessor/utility/conversion.hpp"

#include <Eigen/Dense>  // for Matrix4f::inverse() (the declaration in Core needs the LU definition)

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::convert_to_xyzirc_cloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input_cloud,
  sensor_msgs::msg::PointCloud2::UniquePtr & xyzirc_cloud)
{
  xyzirc_cloud->header = input_cloud->header;

  PointCloud2Modifier<PointXYZIRC, autoware::point_types::PointXYZIRCGenerator> output_modifier{
    *xyzirc_cloud, input_cloud->header.frame_id};
  output_modifier.reserve(input_cloud->width);

  bool has_valid_intensity =
    std::any_of(input_cloud->fields.begin(), input_cloud->fields.end(), [](const auto & field) {
      return field.name == "intensity" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_return_type =
    std::any_of(input_cloud->fields.begin(), input_cloud->fields.end(), [](const auto & field) {
      return field.name == "return_type" && field.datatype == sensor_msgs::msg::PointField::UINT8;
    });

  bool has_valid_channel =
    std::any_of(input_cloud->fields.begin(), input_cloud->fields.end(), [](const auto & field) {
      return field.name == "channel" && field.datatype == sensor_msgs::msg::PointField::UINT16;
    });

  sensor_msgs::PointCloud2ConstIterator<float> it_x(*input_cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(*input_cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(*input_cloud, "z");

  if (has_valid_intensity && has_valid_return_type && has_valid_channel) {
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_i(*input_cloud, "intensity");
    sensor_msgs::PointCloud2ConstIterator<std::uint8_t> it_r(*input_cloud, "return_type");
    sensor_msgs::PointCloud2ConstIterator<std::uint16_t> it_c(*input_cloud, "channel");

    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_r, ++it_c) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      point.intensity = *it_i;
      point.return_type = *it_r;
      point.channel = *it_c;
      output_modifier.push_back(std::move(point));
    }
  } else {
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
      PointXYZIRC point;
      point.x = *it_x;
      point.y = *it_y;
      point.z = *it_z;
      output_modifier.push_back(std::move(point));
    }
  }
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::transform_pointcloud(
  const Eigen::Matrix4f & transform, const sensor_msgs::msg::PointCloud2 & in,
  sensor_msgs::msg::PointCloud2 & out)
{
  // Same implementation as pcl::transformPointCloud
  out = in;

  if (out.data.empty()) return;

  sensor_msgs::PointCloud2Iterator<float> it_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(out, "z");
  for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z) {
    const Eigen::Vector4f point(*it_x, *it_y, *it_z, 1.0f);
    const Eigen::Vector4f transformed = transform * point;
    *it_x = transformed.x();
    *it_y = transformed.y();
    *it_z = transformed.z();
  }
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::append_pointcloud(
  const sensor_msgs::msg::PointCloud2 & src, sensor_msgs::msg::PointCloud2 & dst)
{
  // Both clouds share the PointXYZIRC layout (identical fields and point_step), so concatenating is
  // a byte append of the point data. width/height/row_step are recomputed by the caller afterwards.
  if (src.data.empty()) return;
  dst.data.insert(dst.data.end(), src.data.begin(), src.data.end());
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::correct_pointcloud_motion(
  const std::unique_ptr<sensor_msgs::msg::PointCloud2> & transformed_cloud_ptr,
  const std::vector<int64_t> & pc_nanoseconds,
  std::unordered_map<int64_t, Eigen::Matrix4f> & transform_memo,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & transformed_delay_compensated_cloud_ptr,
  MotionCompensationStatus * status)
{
  Eigen::Matrix4f adjust_to_old_data_transform = Eigen::Matrix4f::Identity();
  int64_t current_cloud_nanoseconds = utils::to_nanoseconds(transformed_cloud_ptr->header.stamp);
  for (const auto & stamp_nanoseconds : pc_nanoseconds) {
    if (stamp_nanoseconds >= current_cloud_nanoseconds) continue;

    Eigen::Matrix4f new_to_old_transform;
    if (transform_memo.find(stamp_nanoseconds) != transform_memo.end()) {
      new_to_old_transform = transform_memo[stamp_nanoseconds];
    } else {
      new_to_old_transform = compute_transform_to_adjust_for_old_timestamp(
        utils::to_ros_time(stamp_nanoseconds), utils::to_ros_time(current_cloud_nanoseconds),
        status);
      transform_memo[stamp_nanoseconds] = new_to_old_transform;
    }
    adjust_to_old_data_transform = new_to_old_transform * adjust_to_old_data_transform;
    current_cloud_nanoseconds = stamp_nanoseconds;
  }
  transform_pointcloud(
    adjust_to_old_data_transform, *transformed_cloud_ptr, *transformed_delay_compensated_cloud_ptr);
}

std::vector<int64_t> CombineCloudHandler<sensor_msgs::msg::PointCloud2>::collect_input_timestamps(
  const std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
    topic_to_cloud_map,
  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result)
{
  std::vector<int64_t> pc_nanoseconds;
  pc_nanoseconds.reserve(topic_to_cloud_map.size());

  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    pc_nanoseconds.emplace_back(utils::to_nanoseconds(cloud->header.stamp));
    result.topic_to_original_stamp_map[topic] = utils::to_seconds(cloud->header.stamp);
  }

  // Descending order (newest first) is required by correct_pointcloud_motion().
  std::sort(pc_nanoseconds.begin(), pc_nanoseconds.end(), std::greater<int64_t>());
  return pc_nanoseconds;
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::initialize_concatenated_cloud(
  const std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
    topic_to_cloud_map,
  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result)
{
  result.concatenate_cloud_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  result.concatenation_info_ptr =
    std::make_unique<autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo>(
      concatenation_info_manager_.reset_and_get_base_info());

  {
    // Normally, pcl::concatenatePointCloud() copies the field layout (e.g., XYZIRC)
    // from the non-empty point cloud when given one empty and one non-empty input.
    //
    // However, if all input clouds in topic_to_cloud_map are empty,
    // the function receives two empty point clouds and does nothing,
    // resulting in concatenate_cloud_ptr not being compatible with the XYZIRC format.
    //
    // To avoid this, we explicitly set the fields of concatenate_cloud_ptr to XYZIRC here.
    PointCloud2Modifier<PointXYZIRC, autoware::point_types::PointXYZIRCGenerator>
      concatenate_cloud_modifier{*result.concatenate_cloud_ptr, output_frame_};
  }

  // An empty cloud is trivially dense; process_input_cloud() folds in each appended cloud's density.
  result.concatenate_cloud_ptr->is_dense = true;

  // Reserve space based on the total size of the pointcloud data to speed up the concatenation
  // process
  size_t total_data_size = 0;
  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    total_data_size += cloud->data.size();
  }
  result.concatenate_cloud_ptr->data.reserve(total_data_size);
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::store_synchronized_cloud(
  const std::string & topic, const std::string & input_frame_id,
  const Eigen::Matrix4f & sensor_to_output, const builtin_interfaces::msg::Time & oldest_stamp,
  std::unique_ptr<sensor_msgs::msg::PointCloud2> compensated_cloud,
  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result)
{
  if (!result.topic_to_transformed_cloud_map) {
    // Initialize the map if it is not present
    result.topic_to_transformed_cloud_map =
      std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::UniquePtr>();
  }

  // convert to original sensor frame if necessary
  const bool need_transform_to_sensor_frame = (input_frame_id != output_frame_);
  if (keep_input_frame_in_synchronized_pointcloud_ && need_transform_to_sensor_frame) {
    auto cloud_in_sensor_frame = std::make_unique<sensor_msgs::msg::PointCloud2>();
    transform_pointcloud(sensor_to_output.inverse(), *compensated_cloud, *cloud_in_sensor_frame);
    cloud_in_sensor_frame->header.stamp = oldest_stamp;
    cloud_in_sensor_frame->header.frame_id = input_frame_id;
    (*result.topic_to_transformed_cloud_map)[topic] = std::move(cloud_in_sensor_frame);
  } else {
    compensated_cloud->header.stamp = oldest_stamp;
    compensated_cloud->header.frame_id = output_frame_;
    (*result.topic_to_transformed_cloud_map)[topic] = std::move(compensated_cloud);
  }
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::process_input_cloud(
  const std::string & topic, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
  const std::vector<int64_t> & pc_nanoseconds,
  const builtin_interfaces::msg::Time & oldest_stamp,
  std::unordered_map<int64_t, Eigen::Matrix4f> & transform_memo,
  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result)
{
  // convert to XYZIRC pointcloud if pointcloud is not empty
  auto xyzirc_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
  convert_to_xyzirc_cloud(cloud, xyzirc_cloud);

  // Transform the cloud into the output frame
  const auto sensor_to_output = get_transform_to_output_frame(xyzirc_cloud->header.frame_id);
  if (!sensor_to_output.has_value()) {
    result.dropped_frames_missing_transform.push_back(xyzirc_cloud->header.frame_id);
    concatenation_info_manager_.update_source_from_point_cloud(
      *xyzirc_cloud, topic, autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_INVALID,
      *result.concatenation_info_ptr);
    return;
  }

  auto transformed_cloud_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
  transform_pointcloud(*sensor_to_output, *xyzirc_cloud, *transformed_cloud_ptr);
  transformed_cloud_ptr->header.frame_id = output_frame_;

  // compensate pointcloud
  std::unique_ptr<sensor_msgs::msg::PointCloud2> transformed_delay_compensated_cloud_ptr;
  if (is_motion_compensated_) {
    transformed_delay_compensated_cloud_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    correct_pointcloud_motion(
      transformed_cloud_ptr, pc_nanoseconds, transform_memo,
      transformed_delay_compensated_cloud_ptr, &result.motion_compensation_status);
  } else {
    transformed_delay_compensated_cloud_ptr = std::move(transformed_cloud_ptr);
  }

  if (
    transformed_delay_compensated_cloud_ptr->width *
      transformed_delay_compensated_cloud_ptr->height >
    0) {
    append_pointcloud(*transformed_delay_compensated_cloud_ptr, *result.concatenate_cloud_ptr);
    // Fold this source cloud's density into the running output density (uses the original cloud's
    // is_dense, which the XYZIRC conversion does not carry over).
    result.concatenate_cloud_ptr->is_dense =
      result.concatenate_cloud_ptr->is_dense && cloud->is_dense;
  }

  // update concatenation info
  concatenation_info_manager_.update_source_from_point_cloud(
    *transformed_delay_compensated_cloud_ptr, topic,
    autoware_sensing_msgs::msg::SourcePointCloudInfo::STATUS_OK, *result.concatenation_info_ptr);

  if (publish_synchronized_pointcloud_) {
    store_synchronized_cloud(
      topic, cloud->header.frame_id, *sensor_to_output, oldest_stamp,
      std::move(transformed_delay_compensated_cloud_ptr), result);
  }
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::finalize_concatenated_cloud(
  const builtin_interfaces::msg::Time & oldest_stamp,
  sensor_msgs::msg::PointCloud2 & concatenate_cloud)
{
  concatenate_cloud.header.stamp = oldest_stamp;

  // concatenated cloud is no longer structured so recalculate the height, width, and row_step
  concatenate_cloud.height = 1;
  const auto data_size = concatenate_cloud.data.size();
  const auto point_step = concatenate_cloud.point_step;
  if (data_size % point_step != 0) {
    throw std::runtime_error("PointCloud2 data size is not divisible by point_step");
  }
  concatenate_cloud.row_step = data_size;
  concatenate_cloud.width = data_size / point_step;
}

void CombineCloudHandler<sensor_msgs::msg::PointCloud2>::apply_advanced_collector_config(
  const std::shared_ptr<CollectorInfoBase> & collector_info,
  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> & result)
{
  const auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(collector_info);
  if (!advanced_info) return;

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

  StrategyAdvancedConfig strategy_config(reference_timestamp_min_msg, reference_timestamp_max_msg);
  const auto serialized_config = strategy_config.serialize();
  ConcatenationInfoManager::set_config(serialized_config, *result.concatenation_info_ptr);
}

ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2>
CombineCloudHandler<sensor_msgs::msg::PointCloud2>::combine_pointclouds(
  std::unordered_map<std::string, sensor_msgs::msg::PointCloud2::ConstSharedPtr> &
    topic_to_cloud_map,
  const std::shared_ptr<CollectorInfoBase> & collector_info)
{
  ConcatenatedCloudResult<sensor_msgs::msg::PointCloud2> concatenate_cloud_result;
  if (topic_to_cloud_map.empty()) return concatenate_cloud_result;

  const std::vector<int64_t> pc_nanoseconds =
    collect_input_timestamps(topic_to_cloud_map, concatenate_cloud_result);
  const builtin_interfaces::msg::Time oldest_stamp = utils::to_ros_time(pc_nanoseconds.back());

  initialize_concatenated_cloud(topic_to_cloud_map, concatenate_cloud_result);

  std::unordered_map<int64_t, Eigen::Matrix4f> transform_memo;
  for (const auto & [topic, cloud] : topic_to_cloud_map) {
    process_input_cloud(
      topic, cloud, pc_nanoseconds, oldest_stamp, transform_memo, concatenate_cloud_result);
  }

  finalize_concatenated_cloud(oldest_stamp, *concatenate_cloud_result.concatenate_cloud_ptr);

  apply_advanced_collector_config(collector_info, concatenate_cloud_result);

  concatenation_info_manager_.set_result(
    *concatenate_cloud_result.concatenate_cloud_ptr,
    *concatenate_cloud_result.concatenation_info_ptr);

  return concatenate_cloud_result;
}

}  // namespace autoware::pointcloud_preprocessor
