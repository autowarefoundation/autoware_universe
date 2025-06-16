// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_
#define AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_

#include "autoware/pointcloud_preprocessor/diagnostics/diagnostics_base.hpp"
#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/ros/diagnostics_interface.hpp>
#include <autoware_utils/ros/polling_subscriber.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
using rcl_interfaces::msg::SetParametersResult;
using sensor_msgs::msg::PointCloud2;

class DistortionCorrectorComponent : public rclcpp::Node
{
public:
  explicit DistortionCorrectorComponent(const rclcpp::NodeOptions & options);

private:
  autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::SharedPtr
    twist_sub_;
  autoware_utils::InterProcessPollingSubscriber<
    sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::SharedPtr imu_sub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<PointCloud2>::SharedPtr undistorted_pointcloud_pub_;

  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils::DebugPublisher> debug_publisher_;

  // Parameters
  std::string base_frame_;
  bool use_imu_;
  bool use_3d_distortion_correction_;
  bool update_azimuth_and_distance_;
  double processing_time_threshold_sec_;
  double timestamp_mismatch_fraction_threshold_;

  std::optional<AngleConversion> angle_conversion_opt_;

  std::unique_ptr<DistortionCorrectorBase> distortion_corrector_;

  // Diagnostic
  std::unique_ptr<autoware_utils::DiagnosticsInterface> diagnostics_interface_;

  void pointcloud_callback(PointCloud2::UniquePtr pointcloud_msg);
  void publish_diagnostics(const std::vector<std::shared_ptr<const DiagnosticsBase>> & diagnostics);
};

}  // namespace autoware::pointcloud_preprocessor

#endif  // AUTOWARE__POINTCLOUD_PREPROCESSOR__DISTORTION_CORRECTOR__DISTORTION_CORRECTOR_NODE_HPP_
