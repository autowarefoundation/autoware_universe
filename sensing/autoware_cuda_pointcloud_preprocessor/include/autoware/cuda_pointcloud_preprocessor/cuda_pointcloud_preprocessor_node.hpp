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

#ifndef AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_
#define AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_

#include "autoware/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.hpp"
#include "autoware/cuda_pointcloud_preprocessor/point_types.hpp"

#include <autoware/point_types/types.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <cuda_blackboard/cuda_adaptation.hpp>
#include <cuda_blackboard/cuda_blackboard_publisher.hpp>
#include <cuda_blackboard/cuda_blackboard_subscriber.hpp>
#include <cuda_blackboard/cuda_pointcloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>

#define CHECK_OFFSET(structure1, structure2, field)             \
  static_assert(                                                \
    offsetof(structure1, field) == offsetof(structure2, field), \
    "Offset of " #field " in " #structure1 " does not match expected offset.")

namespace autoware::cuda_pointcloud_preprocessor
{

static_assert(sizeof(InputPointType) == sizeof(autoware::point_types::PointXYZIRCAEDT));
static_assert(sizeof(OutputPointType) == sizeof(autoware::point_types::PointXYZIRC));

CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, x);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, y);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, z);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, intensity);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, return_type);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, channel);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, azimuth);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, elevation);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, distance);
CHECK_OFFSET(InputPointType, autoware::point_types::PointXYZIRCAEDT, time_stamp);

CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, x);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, y);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, z);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, intensity);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, return_type);
CHECK_OFFSET(OutputPointType, autoware::point_types::PointXYZIRCAEDT, channel);

class CudaPointcloudPreprocessorNode : public rclcpp::Node
{
public:
  explicit CudaPointcloudPreprocessorNode(const rclcpp::NodeOptions & node_options);

private:
  bool getTransform(
    const std::string & target_frame, const std::string & source_frame,
    tf2::Transform * tf2_transform_ptr);

  // Callback
  void pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_pointcloud_msg_ptr);
  void cudaPointcloudCallback(
    const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> cuda_msg);
  void twistCallback(
    const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr pointcloud_msg);
  void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg);

  // Helper Functions
  void validatePointcloudLayout(const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg);
  std::pair<double, std::uint32_t> getFirstPointTimeInfo(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg);

  void updateTwistQueue(double first_point_stamp);
  void updateImuQueue(double first_point_stamp);
  std::optional<geometry_msgs::msg::TransformStamped> lookupTransformToBase(
    const std::string & source_frame);
  std::unique_ptr<cuda_blackboard::CudaPointCloud2> processPointcloud(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const geometry_msgs::msg::TransformStamped & transform_msg,
    const std::uint32_t first_point_rel_stamp);

  void publishDiagnostics(
    const sensor_msgs::msg::PointCloud2 & input_pointcloud_msg,
    const std::unique_ptr<cuda_blackboard::CudaPointCloud2> & output_pointcloud_ptr);

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  // Diagnostic
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_;

  std::string base_frame_;
  bool use_3d_undistortion_;
  bool use_imu_;
  double processing_time_threshold_sec_;
  double timestamp_mismatch_fraction_threshold_;

  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> twist_queue_;
  std::deque<geometry_msgs::msg::Vector3Stamped> angular_velocity_queue_;

  // Subscribers
  autoware_utils::InterProcessPollingSubscriber<
    sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::SharedPtr imu_sub_;
  autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::SharedPtr
    twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_{};

  // CUDA pub
  std::unique_ptr<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>> pub_;

  std::unique_ptr<CudaPointcloudPreprocessor> cuda_pointcloud_preprocessor_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;
};

}  // namespace autoware::cuda_pointcloud_preprocessor

#endif  // AUTOWARE__CUDA_POINTCLOUD_PREPROCESSOR__CUDA_POINTCLOUD_PREPROCESSOR_NODE_HPP_
