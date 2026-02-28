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

#include "autoware/lidar_frnet/lidar_frnet_node.hpp"

#include "autoware/lidar_frnet/point_type.hpp"
#include "autoware/lidar_frnet/ros_utils.hpp"
#include "autoware/lidar_frnet/utils.hpp"

#include <Eigen/Geometry>
#include <autoware/cuda_utils/cuda_check_error.hpp>
#include <autoware/tensorrt_common/utils.hpp>
#include <cuda_blackboard/cuda_unique_ptr.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace autoware::lidar_frnet
{
LidarFRNetNode::LidarFRNetNode(const rclcpp::NodeOptions & options) : Node("lidar_frnet", options)
{
  auto class_names = declare_parameter<std::vector<std::string>>("class_names");
  auto trt_config = TrtCommonConfig(
    declare_parameter<std::string>("onnx_path"), declare_parameter<std::string>("trt_precision"));
  // Parse crop box
  crop_reference_frame_ =
    this->declare_parameter<std::string>("filter.ego_crop_box.reference_frame");
  float min_x = static_cast<float>(this->declare_parameter<double>("filter.ego_crop_box.min_x"));
  float min_y = static_cast<float>(this->declare_parameter<double>("filter.ego_crop_box.min_y"));
  float min_z = static_cast<float>(this->declare_parameter<double>("filter.ego_crop_box.min_z"));
  float max_x = static_cast<float>(this->declare_parameter<double>("filter.ego_crop_box.max_x"));
  float max_y = static_cast<float>(this->declare_parameter<double>("filter.ego_crop_box.max_y"));
  float max_z = static_cast<float>(this->declare_parameter<double>("filter.ego_crop_box.max_z"));
  crop_box_bounds_ = {min_x, min_y, min_z, max_x, max_y, max_z};
  crop_box_enabled_ = false;
  for (float v : crop_box_bounds_) {
    if (v != 0.0f) {
      crop_box_enabled_ = true;
      break;
    }
  }

  // Ego crop box debug messages: build once (polygon + marker), only stamp updated when publishing
  if (crop_box_enabled_) {
    geometry_msgs::msg::PolygonStamped polygon_msg;
    ros_utils::setPolygonMsg(crop_box_bounds_, crop_reference_frame_, polygon_msg);
    ego_crop_box_polygon_msg_.emplace(std::move(polygon_msg));
    visualization_msgs::msg::Marker marker_msg;
    ros_utils::setMarkerMsg(crop_box_bounds_, crop_reference_frame_, marker_msg);
    ego_crop_box_marker_msg_.emplace(std::move(marker_msg));
  }

  // TF for static sensor to reference when crop box is enabled (lookup once, then buffer/listener
  // freed)
  if (crop_box_enabled_) {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  auto postprocessing_params = utils::PostprocessingParams(
    declare_parameter<double>("filter.class_confidence_threshold"),
    declare_parameter<std::vector<std::string>>("filter.classes"), crop_box_bounds_, class_names,
    declare_parameter<std::vector<int64_t>>("palette"));

  const auto num_points_profile = declare_parameter<std::vector<int64_t>>("num_points");
  max_output_points_ = static_cast<size_t>(num_points_profile.at(2));
  const auto network_params = utils::NetworkParams(
    class_names, num_points_profile, declare_parameter<std::vector<int64_t>>("num_unique_coors"),
    declare_parameter<double>("fov_up_deg"), declare_parameter<double>("fov_down_deg"),
    declare_parameter<int64_t>("frustum_width"), declare_parameter<uint16_t>("frustum_height"),
    declare_parameter<int64_t>("interpolation_width"),
    declare_parameter<int64_t>("interpolation_height"), crop_box_enabled_, crop_box_bounds_);

  diag_params_ = utils::DiagnosticParams(
    declare_parameter<double>("max_allowed_processing_time_ms"),
    declare_parameter<double>("max_acceptable_consecutive_delay_ms"),
    declare_parameter<double>("validation_callback_interval_ms"));

  frnet_ =
    std::make_unique<LidarFRNet>(trt_config, network_params, postprocessing_params, get_logger());

  cloud_in_sub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardSubscriber<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/input/pointcloud",
      std::bind(&LidarFRNetNode::cloudCallback, this, std::placeholders::_1));

  cloud_seg_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud/segmentation");
  cloud_viz_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud/visualization");
  cloud_filtered_pub_ =
    std::make_unique<cuda_blackboard::CudaBlackboardPublisher<cuda_blackboard::CudaPointCloud2>>(
      *this, "~/output/pointcloud/filtered");

  // Debug: ego crop box preview (only published when subscribed)
  ego_crop_box_polygon_pub_ =
    this->create_publisher<geometry_msgs::msg::PolygonStamped>("~/debug/ego_crop_box_polygon", 10);
  ego_crop_box_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("~/debug/ego_crop_box_marker", 10);

  published_time_pub_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);

  // Initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ = std::make_unique<DebugPublisher>(this, this->get_name());
    stop_watch_ptr_->tic("cyclic");
    stop_watch_ptr_->tic("processing/total");
  }

  // Setup diagnostics
  {
    diag_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diag_updater_->setHardwareID(this->get_name());
    diag_updater_->add("processing_time_status", this, &LidarFRNetNode::diagnoseProcessingTime);
    diag_updater_->setPeriod(diag_params_.validation_callback_interval_ms * 1e-3);  // to seconds
  }

  if (this->declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

bool LidarFRNetNode::setStaticCropBoxTransform(const std::string & sensor_frame_id)
{
  if (!tf_buffer_) {
    return false;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(crop_reference_frame_, sensor_frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Ego crop box: TF lookup failed: %s", ex.what());
    return false;
  }
  const Eigen::Affine3d transform = tf2::transformToEigen(tf);
  std::array<float, 12> transform_out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      transform_out[r * 3 + c] = static_cast<float>(transform.linear()(r, c));
    }
    transform_out[9 + r] = static_cast<float>(transform.translation()(r));
  }
  crop_sensor_to_ref_.emplace(transform_out);
  tf_listener_.reset();
  tf_buffer_.reset();
  RCLCPP_INFO(
    get_logger(), "Ego crop box: cached static transform %s -> %s, TF listener shut down.",
    sensor_frame_id.c_str(), crop_reference_frame_.c_str());
  return true;
}

geometry_msgs::msg::PolygonStamped LidarFRNetNode::getPolygonMsg(rclcpp::Time stamp) const
{
  geometry_msgs::msg::PolygonStamped msg = *ego_crop_box_polygon_msg_;
  msg.header.stamp = stamp;
  return msg;
}

visualization_msgs::msg::Marker LidarFRNetNode::getMarkerMsg(rclcpp::Time stamp) const
{
  visualization_msgs::msg::Marker msg = *ego_crop_box_marker_msg_;
  msg.header.stamp = stamp;
  return msg;
}

void LidarFRNetNode::cloudCallback(
  const std::shared_ptr<const cuda_blackboard::CudaPointCloud2> & msg)
{
  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing/total", true);
  }

  // Initialize filtered layout from first message (preserves input format)
  std::call_once(init_filtered_layout_, [this, &msg]() {
    cloud_filtered_layout_.emplace(ros_utils::generateFilteredPointCloudLayoutFromInput(*msg));
    RCLCPP_INFO(
      this->get_logger(), "Initialized filtered cloud layout with %zu fields, point_step=%zu",
      cloud_filtered_layout_->fields.size(), cloud_filtered_layout_->point_step);
  });

  const auto active_comm = utils::ActiveComm(
    cloud_seg_pub_->get_subscription_count() +
        cloud_seg_pub_->get_intra_process_subscription_count() >
      0,
    cloud_viz_pub_->get_subscription_count() +
        cloud_viz_pub_->get_intra_process_subscription_count() >
      0,
    cloud_filtered_pub_->get_subscription_count() +
        cloud_filtered_pub_->get_intra_process_subscription_count() >
      0);

  if (!active_comm) {
    return;
  }

  // Static transform sensor to reference for crop box: lookup once, then use cached value
  if (crop_box_enabled_) {
    if (!crop_sensor_to_ref_.has_value()) {
      if (!setStaticCropBoxTransform(msg->header.frame_id)) {
        return;
      }
    }
  }

  const std::array<float, 12> * crop_sensor_to_ref_ptr =
    crop_sensor_to_ref_.has_value() ? &*crop_sensor_to_ref_ : nullptr;

  // Allocate output messages (seg/viz use max capacity; pipeline sets actual width after process)
  auto cloud_seg_msg_ptr =
    ros_utils::generatePointCloudMessageFromInput(*msg, cloud_seg_layout_, max_output_points_);
  auto cloud_viz_msg_ptr =
    ros_utils::generatePointCloudMessageFromInput(*msg, cloud_viz_layout_, max_output_points_);
  auto cloud_filtered_msg_ptr =
    ros_utils::generatePointCloudMessageFromInput(*msg, *cloud_filtered_layout_);

  std::unordered_map<std::string, double> proc_timing;
  if (!frnet_->process(
        msg, *cloud_seg_msg_ptr, *cloud_viz_msg_ptr, *cloud_filtered_msg_ptr, active_comm,
        proc_timing, crop_sensor_to_ref_ptr))
    return;

  // Publish output messages
  if (active_comm.seg) {
    cloud_seg_pub_->publish(std::move(cloud_seg_msg_ptr));
  }

  if (active_comm.viz) {
    cloud_viz_pub_->publish(std::move(cloud_viz_msg_ptr));
  }

  if (active_comm.filtered) {
    cloud_filtered_pub_->publish(std::move(cloud_filtered_msg_ptr));
  }

  publishEgoCropBoxDebug(rclcpp::Time(msg->header.stamp));

  // Note: published_time_pub_ cannot be used with CudaBlackboardPublisher
  // because it doesn't inherit from rclcpp::PublisherBase
  // published_time_pub_->publish_if_subscribed(cloud_seg_pub_, msg->header.stamp);

  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    last_processing_time_ms_.emplace(stop_watch_ptr_->toc("processing/total", true));
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds((this->get_clock()->now() - msg->header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time/total_ms", *last_processing_time_ms_);
    for (const auto & [topic, time_ms] : proc_timing) {
      debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        topic, time_ms);
    }
  }
}

void LidarFRNetNode::diagnoseProcessingTime(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const rclcpp::Time timestamp_now = this->get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus::_level_type diag_level =
    diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string summary_msg = "OK";

  if (last_processing_time_ms_) {
    if (*last_processing_time_ms_ > diag_params_.max_allowed_processing_time_ms) {
      stat.add("is_processing_time_ms_in_expected_range", false);
      summary_msg =
        "Processing time exceeds the acceptable limit of " +
        std::to_string(diag_params_.max_allowed_processing_time_ms) + " ms by " +
        std::to_string(*last_processing_time_ms_ - diag_params_.max_allowed_processing_time_ms) +
        " ms.";
      if (!last_in_time_processing_timestamp_) {
        last_in_time_processing_timestamp_.emplace(timestamp_now);
      }
      diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    } else {
      stat.add("is_processing_time_ms_in_expected_range", true);
      last_in_time_processing_timestamp_.emplace(timestamp_now);
    }
    stat.add("processing_time_ms", *last_processing_time_ms_);

    const double delayed_state_duration =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (timestamp_now - *last_in_time_processing_timestamp_).nanoseconds()))
        .count();

    if (delayed_state_duration > diag_params_.max_acceptable_consecutive_delay_ms) {
      stat.add("is_consecutive_processing_delay_in_range", false);
      summary_msg +=
        " Processing delay has consecutively exceeded the acceptable limit continuously.";
      diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      stat.add("is_consecutive_processing_delay_in_range", true);
    }
    stat.add("consecutive_processing_delay_ms", delayed_state_duration);
  } else {
    summary_msg = "Waiting for the node to perform inference.";
  }

  stat.summary(diag_level, summary_msg);
}

void LidarFRNetNode::publishEgoCropBoxDebug(rclcpp::Time stamp)
{
  if (!crop_box_enabled_) {
    return;
  }
  const bool has_polygon_sub = ego_crop_box_polygon_pub_->get_subscription_count() +
                                 ego_crop_box_polygon_pub_->get_intra_process_subscription_count() >
                               0;
  const bool has_marker_sub = ego_crop_box_marker_pub_->get_subscription_count() +
                                ego_crop_box_marker_pub_->get_intra_process_subscription_count() >
                              0;
  if (!has_polygon_sub && !has_marker_sub) {
    return;
  }

  if (has_polygon_sub) {
    ego_crop_box_polygon_pub_->publish(getPolygonMsg(stamp));
  }

  if (has_marker_sub) {
    ego_crop_box_marker_pub_->publish(getMarkerMsg(stamp));
  }
}

}  // namespace autoware::lidar_frnet

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::lidar_frnet::LidarFRNetNode)
