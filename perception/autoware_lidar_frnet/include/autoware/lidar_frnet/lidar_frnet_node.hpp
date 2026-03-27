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

#ifndef AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_NODE_HPP_
#define AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_NODE_HPP_

#include "autoware/lidar_frnet/lidar_frnet.hpp"
#include "autoware/lidar_frnet/ros_utils.hpp"
#include "autoware/lidar_frnet/utils.hpp"
#include "autoware/lidar_frnet/visibility_control.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <array>
#include <memory>
#include <optional>
#include <string>

namespace autoware::lidar_frnet
{

/**
 * @brief ROS 2 node for Lidar FRNet: subscribes to point cloud, runs pipeline, publishes seg/viz
 *        and filtered clouds; optional ego crop box with static TF and debug visualization.
 */
class LIDAR_FRNET_PUBLIC LidarFRNetNode : public rclcpp::Node
{
public:
  /** @brief Construct node from options; initializes params, pipeline, I/O, and diagnostics. */
  explicit LidarFRNetNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Callback for input point cloud: run pipeline and publish outputs when subscribers exist.
   * @param msg Input point cloud (CudaPointCloud2)
   */
  void cloudCallback(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg);

  /**
   * @brief Diagnostic callback: report processing time and consecutive delay status.
   * @param stat Diagnostic status wrapper to fill
   */
  void diagnoseProcessingTime(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Publish ego crop box as marker when subscribed; no-op if crop box
   *        disabled or no subscribers. Uses given stamp for header.
   * @param stamp Timestamp for published messages
   */
  void publishEgoCropBoxDebug(rclcpp::Time stamp);

  /**
   * @brief Look up static transform from sensor frame to crop reference frame, cache it, then
   *        release TF buffer and listener. Call once when crop box is enabled.
   * @param sensor_frame_id Frame ID of the sensor (e.g. from point cloud header)
   * @return true if transform was found and cached, false on lookup failure
   */
  bool setStaticCropBoxTransform(const std::string & sensor_frame_id);

private:
  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    cloud_in_sub_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    cloud_seg_pub_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    cloud_viz_pub_{nullptr};
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>
    cloud_filtered_pub_{nullptr};

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_pub_{nullptr};

  std::unique_ptr<LidarFRNet> frnet_{nullptr};
  std::unique_ptr<diagnostic_updater::Updater> diag_updater_{nullptr};

  const ros_utils::PointCloudLayout cloud_seg_layout_{
    ros_utils::generateSegmentationPointCloudLayout()};
  const ros_utils::PointCloudLayout cloud_viz_layout_{
    ros_utils::generateVisualizationPointCloudLayout()};

  // Filtered layout is initialized dynamically from first input message
  std::optional<ros_utils::PointCloudLayout> cloud_filtered_layout_;
  std::optional<CloudFormat> filtered_output_format_;
  std::string filtered_output_format_param_;
  std::once_flag init_filtered_layout_;

  /** Max point count (profile max) for seg/viz/filtered output allocation. num_points can exceed
   * input. */
  size_t max_output_points_{0};

  utils::DiagnosticParams diag_params_{};
  std::optional<double> last_processing_time_ms_;
  std::optional<rclcpp::Time> last_in_time_processing_timestamp_;

  // TF for static sensor
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::optional<std::array<float, 12>> crop_sensor_to_ref_;  // cached after first successful lookup
  std::string crop_reference_frame_;
  std::array<float, 6> crop_box_bounds_;
  bool crop_box_enabled_;

  // Debug: ego crop box preview
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ego_crop_box_marker_pub_;
  std::optional<visualization_msgs::msg::Marker> ego_crop_box_marker_msg_;

  /**
   * @brief Return cached ego crop box marker message with given stamp.
   * @param stamp Timestamp to set in header
   * @return Marker (call only when ego_crop_box_marker_msg_ has value)
   */
  visualization_msgs::msg::Marker getMarkerMsg(rclcpp::Time stamp) const;
};

}  // namespace autoware::lidar_frnet

#endif  // AUTOWARE__LIDAR_FRNET__LIDAR_FRNET_NODE_HPP_
