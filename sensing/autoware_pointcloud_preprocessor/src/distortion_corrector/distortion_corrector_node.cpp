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

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector_node.hpp"

#include "autoware/pointcloud_preprocessor/distortion_corrector/distortion_corrector.hpp"
#include "autoware/pointcloud_preprocessor/utility/diagnostic_utils.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::pointcloud_preprocessor
{
/** @brief Constructor. */
DistortionCorrectorComponent::DistortionCorrectorComponent(const rclcpp::NodeOptions & options)
: Node("distortion_corrector_node", options)
{
  // initialize debug tool

  using autoware_utils::DebugPublisher;
  using autoware_utils::StopWatch;
  stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
  debug_publisher_ = std::make_unique<DebugPublisher>(this, "distortion_corrector");
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");

  // Parameter
  base_frame_ = declare_parameter<std::string>("base_frame");
  use_imu_ = declare_parameter<bool>("use_imu");
  use_3d_distortion_correction_ = declare_parameter<bool>("use_3d_distortion_correction");
  update_azimuth_and_distance_ = declare_parameter<bool>("update_azimuth_and_distance");
  processing_time_threshold_ = declare_parameter<float>("processing_time_threshold");
  mismatch_fraction_threshold_ = declare_parameter<float>("mismatch_fraction_threshold");

  // Publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    // Publisher
    undistorted_pointcloud_pub_ = this->create_publisher<PointCloud2>(
      "~/output/pointcloud", rclcpp::SensorDataQoS(), pub_options);
  }

  // Twist queue size needs to be larger than 'twist frequency' / 'pointcloud frequency'.
  // To avoid individual tuning, a sufficiently large value is hard-coded.
  // With 100, it can handle twist updates up to 1000Hz if the pointcloud is 10Hz.
  const uint16_t TWIST_QUEUE_SIZE = 100;

  // Subscriber
  twist_sub_ = autoware_utils::InterProcessPollingSubscriber<
    geometry_msgs::msg::TwistWithCovarianceStamped, autoware_utils::polling_policy::All>::
    create_subscription(this, "~/input/twist", rclcpp::QoS(TWIST_QUEUE_SIZE));
  imu_sub_ = autoware_utils::InterProcessPollingSubscriber<
    sensor_msgs::msg::Imu, autoware_utils::polling_policy::All>::
    create_subscription(this, "~/input/imu", rclcpp::QoS(TWIST_QUEUE_SIZE));
  pointcloud_sub_ = this->create_subscription<PointCloud2>(
    "~/input/pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&DistortionCorrectorComponent::pointcloud_callback, this, std::placeholders::_1));

  // Setup the distortion corrector

  if (use_3d_distortion_correction_) {
    distortion_corrector_ = std::make_unique<DistortionCorrector3D>(*this);
  } else {
    distortion_corrector_ = std::make_unique<DistortionCorrector2D>(*this);
  }

  // Diagnostic Updater
  setup_diagnostics(
    this, diagnostic_updater_, this, &DistortionCorrectorComponent::check_diagnostics);
}

void DistortionCorrectorComponent::pointcloud_callback(PointCloud2::UniquePtr pointcloud_msg)
{
  stop_watch_ptr_->toc("processing_time", true);
  const auto points_sub_count = undistorted_pointcloud_pub_->get_subscription_count() +
                                undistorted_pointcloud_pub_->get_intra_process_subscription_count();

  if (points_sub_count < 1) {
    return;
  }

  std::vector<geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr> twist_msgs =
    twist_sub_->take_data();
  for (const auto & msg : twist_msgs) {
    distortion_corrector_->process_twist_message(msg);
  }

  if (use_imu_) {
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_msgs = imu_sub_->take_data();
    for (const auto & msg : imu_msgs) {
      distortion_corrector_->process_imu_message(base_frame_, msg);
    }
  }

  distortion_corrector_->set_pointcloud_transform(base_frame_, pointcloud_msg->header.frame_id);
  distortion_corrector_->initialize();

  if (update_azimuth_and_distance_ && !angle_conversion_opt_.has_value()) {
    angle_conversion_opt_ = distortion_corrector_->try_compute_angle_conversion(*pointcloud_msg);
    if (angle_conversion_opt_.has_value()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Success to get the conversion formula between Cartesian coordinates and LiDAR azimuth "
        "coordinates");
    } else {
      RCLCPP_ERROR_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000 /* ms */,
        "Failed to get the angle conversion between Cartesian coordinates and LiDAR azimuth "
        "coordinates. This pointcloud will not update azimuth and distance");
    }
  }

  distortion_corrector_->undistort_pointcloud(use_imu_, angle_conversion_opt_, *pointcloud_msg);

  const rclcpp::Time stamp(pointcloud_msg->header.stamp);
  pointcloud_timestamp_ = stamp.seconds();

  const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
  const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
  const double pipeline_latency_ms =
    std::chrono::duration<double, std::milli>(
      std::chrono::nanoseconds((this->get_clock()->now() - stamp).nanoseconds()))
      .count();

  undistorted_pointcloud_pub_->publish(std::move(pointcloud_msg));
  if (debug_publisher_) {
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  // diagnostic
  last_processing_time_ms_ = processing_time_ms;
  last_pipeline_latency_ms_ = pipeline_latency_ms;
  mismatch_count_ = distortion_corrector_->get_mismatch_count();
  mismatch_fraction_ = distortion_corrector_->get_mismatch_fraction();
  diagnostic_updater_.force_update();
}

void DistortionCorrectorComponent::check_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (mismatch_fraction_ > mismatch_fraction_threshold_) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Mismatch_fraction exceed the threshold");
  } else if (last_processing_time_ms_ > processing_time_threshold_ * 1000.0) {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, "Processing time exceeded threshold");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Distortion correction successful");
  }

  stat.add("timestamp", format_timestamp(pointcloud_timestamp_));
  stat.add("mismatch_count", mismatch_count_);
  stat.add("mismatch_fraction", mismatch_fraction_);
  stat.add("processing_time_ms", last_processing_time_ms_);
  stat.add("pipeline_latency_ms", last_pipeline_latency_ms_);
  stat.add("use_3d_distortion_correction", use_3d_distortion_correction_);
  stat.add("update_azimuth_and_distance", update_azimuth_and_distance_);
}

}  // namespace autoware::pointcloud_preprocessor

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::pointcloud_preprocessor::DistortionCorrectorComponent)
