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

#ifndef AUTOWARE__BEVFUSION__BEVFUSION_NODE_HPP_
#define AUTOWARE__BEVFUSION__BEVFUSION_NODE_HPP_

#include "autoware/bevfusion/bevfusion_trt.hpp"
#include "autoware/bevfusion/detection_class_remapper.hpp"
#include "autoware/bevfusion/postprocess/non_maximum_suppression.hpp"
#include "autoware/bevfusion/preprocess/pointcloud_densification.hpp"
#include "autoware/bevfusion/visibility_control.hpp"

#include <Eigen/Core>
#include <autoware/universe_utils/system/stop_watch.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/published_time_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <cuda_blackboard/negotiated_types.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_object_kinematics.hpp>
#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::bevfusion
{

class BEVFUSION_PUBLIC BEVFusionNode : public rclcpp::Node
{
public:
  using Matrix4f = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>;

  explicit BEVFusionNode(const rclcpp::NodeOptions & options);

private:
  void cloudCallback(const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg_ptr);
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg, std::size_t camera_id);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo & msg, std::size_t camera_id);
  void diagnoseProcessingTime(diagnostic_updater::DiagnosticStatusWrapper & stat);

  std::unique_ptr<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>
    cloud_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr> image_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::ConstSharedPtr> camera_info_subs_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr objects_pub_{
    nullptr};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};

  DetectionClassRemapper detection_class_remapper_;
  std::vector<std::string> class_names_;
  std::optional<std::string> lidar_frame_;
  float max_camera_lidar_delay_;

  std::vector<sensor_msgs::msg::Image::ConstSharedPtr> image_msgs_;
  std::vector<float> camera_masks_;
  std::vector<std::optional<sensor_msgs::msg::CameraInfo>> camera_info_msgs_;
  std::vector<std::optional<Matrix4f>> lidar2camera_extrinsics_;

  bool sensor_fusion_{false};
  bool images_available_{false};
  bool intrinsics_available_{false};
  bool extrinsics_available_{false};
  bool intrinsics_extrinsics_precomputed_{false};

  // for diagnostics
  double max_allowed_processing_time_ms_;
  double max_acceptable_consecutive_delay_ms_;
  // set as optional to avoid sending error diagnostics before the node starts processing
  std::optional<double> last_processing_time_ms_;
  std::optional<rclcpp::Time> last_in_time_processing_timestamp_;
  diagnostic_updater::Updater diagnostic_processing_time_updater_{this};

  NonMaximumSuppression iou_bev_nms_;

  std::unique_ptr<BEVFusionTRT> detector_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_detector_trt_;

  // debugger
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{nullptr};
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_ptr_{nullptr};
  std::unique_ptr<autoware_utils::PublishedTimePublisher> published_time_pub_{nullptr};
};
}  // namespace autoware::bevfusion

#endif  // AUTOWARE__BEVFUSION__BEVFUSION_NODE_HPP_
