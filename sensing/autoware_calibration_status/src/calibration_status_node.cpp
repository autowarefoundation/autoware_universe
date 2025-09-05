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

#include "autoware/calibration_status/calibration_status_node.hpp"

#include "autoware/calibration_status/utils.hpp"

#include <rclcpp/qos.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <rmw/qos_profiles.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::calibration_status
{

CalibrationStatusNode::CalibrationStatusNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("calibration_status", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  current_velocity_(0.0),
  last_velocity_update_(this->get_clock()->now())
{
  const CalibrationStatusConfig calibration_status_config(
    this->declare_parameter<double>("lidar_range"),
    this->declare_parameter<int64_t>("dilation_size"),
    this->declare_parameter<std::vector<int64_t>>("height"),
    this->declare_parameter<std::vector<int64_t>>("width"));

  calibration_status_ = std::make_unique<CalibrationStatus>(
    this->declare_parameter<std::string>("onnx_path"),
    this->declare_parameter<std::string>("trt_precision"),
    this->declare_parameter<std::int64_t>("cloud_capacity"), calibration_status_config);

  // Runtime mode configuration
  runtime_mode_ = string_to_runtime_mode(this->declare_parameter<std::string>("runtime_mode"));
  period_ = this->declare_parameter<double>("period");
  queue_size_ = this->declare_parameter<int64_t>("queue_size");
  miscalibration_confidence_threshold_ =
    this->declare_parameter<double>("miscalibration_confidence_threshold");

  // Prerequisite configuration
  check_velocity_ = this->declare_parameter<bool>("prerequisite.check_velocity");
  velocity_source_ =
    string_to_velocity_source(this->declare_parameter<std::string>("prerequisite.velocity_source"));
  velocity_threshold_ = this->declare_parameter<double>("prerequisite.velocity_threshold");
  if (check_velocity_) {
    setup_velocity_source_interface();
  }

  // Initialize diagnostic publisher
  diagnostic_pub_ =
    this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1));

  // Input configuration
  cloud_topics_ = this->declare_parameter<std::vector<std::string>>("input.cloud_topics");
  image_topics_ = this->declare_parameter<std::vector<std::string>>("input.image_topics");
  approx_deltas_ = this->declare_parameter<std::vector<double>>("input.approx_deltas");
  size_t max_topics = std::max(cloud_topics_.size(), image_topics_.size());

  camera_info_msgs_.resize(image_topics_.size());
  for (size_t i = 0; i < image_topics_.size(); ++i) {
    // Remove topic suffix and add "camera_info" suffix
    auto camera_info_topic = image_topics_.at(i);
    if (auto pos = camera_info_topic.rfind('/'); pos != std::string::npos) {
      camera_info_topic.replace(pos + 1, std::string::npos, "camera_info");
    } else {
      camera_info_topic = "camera_info";
    }

    camera_info_subs_.emplace_back(this->create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic, rclcpp::SensorDataQoS(),
      [this, i](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        this->camera_info_callback(msg, i);
      }));
  }

  if (approx_deltas_.size() != max_topics) {
    RCLCPP_ERROR(
      this->get_logger(),
      "approx_deltas size (%zu) must match max(cloud_topics, image_topics) size (%zu)",
      approx_deltas_.size(), max_topics);
    throw std::invalid_argument("Invalid approx_deltas parameter size");
  }
  setup_runtime_mode_interface();

  if (this->declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void CalibrationStatusNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg, size_t image_idx)
{
  if (image_idx < camera_info_msgs_.size()) {
    if (camera_info_msgs_.at(image_idx)) return;
    camera_info_msgs_.at(image_idx) = msg;
  } else {
    throw std::out_of_range("Camera info index out of range: " + std::to_string(image_idx));
  }
}

void CalibrationStatusNode::setup_runtime_mode_interface()
{
  if (runtime_mode_ == RuntimeMode::MANUAL) {
    calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/input/validate_calibration_srv", std::bind(
                                            &CalibrationStatusNode::handle_calibration_request,
                                            this, std::placeholders::_1, std::placeholders::_2));
  } else if (runtime_mode_ == RuntimeMode::PERIODIC) {
    periodic_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&CalibrationStatusNode::periodic_calibration_check, this));
  } else if (runtime_mode_ == RuntimeMode::ACTIVE) {
    setup_sensor_synchronization();
  }
}

void CalibrationStatusNode::setup_velocity_source_interface()
{
  switch (velocity_source_) {
    case VelocitySource::TWIST:
      twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "~/input/velocity", rclcpp::SensorDataQoS(),
        std::bind(&CalibrationStatusNode::twist_callback, this, std::placeholders::_1));
      break;
    case VelocitySource::TWIST_STAMPED:
      twist_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "~/input/velocity", rclcpp::SensorDataQoS(),
        std::bind(&CalibrationStatusNode::twist_stamped_callback, this, std::placeholders::_1));
      break;
    case VelocitySource::TWIST_WITH_COV:
      twist_with_cov_sub_ = this->create_subscription<geometry_msgs::msg::TwistWithCovariance>(
        "~/input/velocity", rclcpp::SensorDataQoS(),
        std::bind(&CalibrationStatusNode::twist_with_cov_callback, this, std::placeholders::_1));
      break;
    case VelocitySource::TWIST_WITH_COV_STAMPED:
      twist_with_cov_stamped_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "~/input/velocity", rclcpp::SensorDataQoS(),
          std::bind(
            &CalibrationStatusNode::twist_with_cov_stamped_callback, this, std::placeholders::_1));
      break;
    case VelocitySource::ODOMETRY:
      odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "~/input/velocity", rclcpp::SensorDataQoS(),
        std::bind(&CalibrationStatusNode::odometry_callback, this, std::placeholders::_1));
      break;
    default:
      throw std::invalid_argument("Unsupported velocity source");
  }
}

void CalibrationStatusNode::setup_sensor_synchronization()
{
  // Determine the number of topic pairs to synchronize
  size_t num_pairs{};
  bool use_cloud_ns{false};

  if (cloud_topics_.size() == 1 && image_topics_.size() > 1) {
    // Single LiDAR with multiple cameras
    num_pairs = image_topics_.size();
  } else if (cloud_topics_.size() > 1 && image_topics_.size() == 1) {
    // Multiple LiDARs with single camera
    num_pairs = cloud_topics_.size();
    use_cloud_ns = true;  // Associate outputs to cloud topics as there's only one image topic
  } else if (cloud_topics_.size() == image_topics_.size()) {
    // One-to-one pairing
    num_pairs = cloud_topics_.size();
  } else {
    throw std::invalid_argument(
      "Invalid topic configuration: only 1:N, N:1, and 1:1 pairing supported");
  }

  cloud_subs_.resize(num_pairs);
  image_subs_.resize(num_pairs);
  preview_image_pubs_.resize(num_pairs);
  synchronizers_.resize(num_pairs);

  for (size_t i = 0; i < num_pairs; ++i) {
    // Determine which topics to use for this pair
    size_t cloud_idx = (cloud_topics_.size() == 1) ? 0 : i;
    size_t image_idx = (image_topics_.size() == 1) ? 0 : i;
    auto preview_image_topic = use_cloud_ns ? cloud_topics_.at(i) : image_topics_.at(i);
    if (auto pos = preview_image_topic.rfind('/'); pos != std::string::npos) {
      preview_image_topic.replace(pos + 1, std::string::npos, "points_projected");
    } else {
      preview_image_topic = "points_projected";
    }

    cloud_subs_[i] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
      this, cloud_topics_[cloud_idx], rmw_qos_profile_sensor_data);
    image_subs_[i] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, image_topics_[image_idx], rmw_qos_profile_sensor_data);
    preview_image_pubs_[i] =
      this->create_publisher<sensor_msgs::msg::Image>(preview_image_topic, rclcpp::SensorDataQoS());

    // Create synchronizer
    synchronizers_[i] = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(queue_size_), *cloud_subs_[i], *image_subs_[i]);

    // Set approximate time delta
    synchronizers_[i]->setMaxIntervalDuration(rclcpp::Duration::from_seconds(approx_deltas_[i]));

    // Register callback
    synchronizers_[i]->registerCallback(
      std::bind(
        &CalibrationStatusNode::synchronized_callback, this, std::placeholders::_1,
        std::placeholders::_2, i));

    RCLCPP_INFO(
      this->get_logger(), "Synchronization pair %zu: %s <-> %s (delta: %.3f s)", i,
      cloud_topics_[cloud_idx].c_str(), image_topics_[image_idx].c_str(), approx_deltas_[i]);
  }
}

void CalibrationStatusNode::handle_calibration_request(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  (void)response;
  // TODO(amadeuszsz): Implement actual calibration validation logic for MANUAL runtime mode
  throw std::logic_error(
    "Manual calibration request handling is not implemented yet. This is a placeholder for future "
    "logic.");
}

void CalibrationStatusNode::periodic_calibration_check()
{
  // TODO(amadeuszsz): Implement periodic calibration check logic for PERIODIC runtime mode
  throw std::logic_error(
    "Periodic calibration check is not implemented yet. This is a placeholder for future logic.");
}

void CalibrationStatusNode::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double velocity = std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y);
  update_vehicle_velocity(velocity);
}

void CalibrationStatusNode::twist_stamped_callback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  double velocity = std::sqrt(
    msg->twist.linear.x * msg->twist.linear.x + msg->twist.linear.y * msg->twist.linear.y);
  update_vehicle_velocity(velocity);
}

void CalibrationStatusNode::twist_with_cov_callback(
  const geometry_msgs::msg::TwistWithCovariance::SharedPtr msg)
{
  double velocity = std::sqrt(
    msg->twist.linear.x * msg->twist.linear.x + msg->twist.linear.y * msg->twist.linear.y);
  update_vehicle_velocity(velocity);
}

void CalibrationStatusNode::twist_with_cov_stamped_callback(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  double velocity = std::sqrt(
    msg->twist.twist.linear.x * msg->twist.twist.linear.x +
    msg->twist.twist.linear.y * msg->twist.twist.linear.y);
  update_vehicle_velocity(velocity);
}

void CalibrationStatusNode::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  double velocity = std::sqrt(
    msg->twist.twist.linear.x * msg->twist.twist.linear.x +
    msg->twist.twist.linear.y * msg->twist.twist.linear.y);
  update_vehicle_velocity(velocity);
}

void CalibrationStatusNode::synchronized_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, size_t pair_idx)
{
  if (camera_info_msgs_.at(pair_idx) == nullptr) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Camera info message for pair %zu is not available. Skipping calibration check.", pair_idx);
    return;
  }
  const auto velocity_check_status = get_velocity_check_status();
  if (velocity_check_status.is_vehicle_moving) {
    publish_diagnostic_status(velocity_check_status, pair_idx);
    return;
  }

  // Get the transform
  Eigen::Affine3d eigen_transform;
  try {
    auto transform = tf_buffer_.lookupTransform(
      image_msg->header.frame_id, cloud_msg->header.frame_id, cloud_msg->header.stamp);
    eigen_transform = tf2::transformToEigen(transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
    return;
  }

  // Prepare preview image message if subscribed
  auto preview_img_msg = std::make_shared<sensor_msgs::msg::Image>();
  uint8_t * preview_img_data = nullptr;
  const auto is_preview_subscribed =
    preview_image_pubs_.at(pair_idx)->get_subscription_count() > 0 ||
    preview_image_pubs_.at(pair_idx)->get_intra_process_subscription_count() > 0;
  if (is_preview_subscribed) {
    preview_img_msg->header = image_msg->header;
    preview_img_msg->height = image_msg->height;
    preview_img_msg->width = image_msg->width;
    preview_img_msg->encoding = image_msg->encoding;
    preview_img_msg->step = image_msg->step;
    preview_img_msg->is_bigendian = image_msg->is_bigendian;
    preview_img_msg->data.resize(image_msg->data.size());
    preview_img_data = preview_img_msg->data.data();
  }

  auto result = calibration_status_->process(
    cloud_msg, image_msg, camera_info_msgs_[pair_idx], eigen_transform, preview_img_data);

  publish_diagnostic_status(velocity_check_status, pair_idx, result);

  if (is_preview_subscribed) {
    preview_image_pubs_.at(pair_idx)->publish(*preview_img_msg);
  }
}

void CalibrationStatusNode::update_vehicle_velocity(double velocity)
{
  current_velocity_ = velocity;
  last_velocity_update_ = this->get_clock()->now();
}

VelocityCheckStatus CalibrationStatusNode::get_velocity_check_status()
{
  if (!check_velocity_) {
    return {false, 0.0, false, 0.0};
  }

  auto now = this->get_clock()->now();
  auto velocity_age = (now - last_velocity_update_).seconds();

  return {true, current_velocity_, current_velocity_ > velocity_threshold_, velocity_age};
}

void CalibrationStatusNode::publish_diagnostic_status(
  const VelocityCheckStatus & velocity_check_status, const size_t pair_idx,
  const CalibrationStatusResult & result)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name =
    std::string(this->get_name()) + std::string(": ") + this->get_fully_qualified_name();
  status.hardware_id = this->get_name();

  auto is_calibrated =
    (result.calibration_confidence >
     result.miscalibration_confidence + miscalibration_confidence_threshold_);

  if (velocity_check_status.is_vehicle_moving) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Vehicle is moving. Calibration check has been skipped.";
  } else if (is_calibrated) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Calibration is valid";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Calibration is invalid.";
  }

  diagnostic_msgs::msg::KeyValue key_value;
  key_value.key = "Source image topic";
  key_value.value = image_topics_.at((image_topics_.size() == 1) ? 0 : pair_idx);
  status.values.push_back(key_value);
  key_value.key = "Source point cloud topic";
  key_value.value = cloud_topics_.at((cloud_topics_.size() == 1) ? 0 : pair_idx);
  status.values.push_back(key_value);
  key_value.key = "Is velocity check activated";
  key_value.value = velocity_check_status.is_activated ? "True" : "False";
  status.values.push_back(key_value);
  key_value.key = "Current velocity";
  key_value.value = std::to_string(velocity_check_status.current_velocity);
  status.values.push_back(key_value);
  key_value.key = "Velocity threshold";
  key_value.value = std::to_string(velocity_threshold_);
  status.values.push_back(key_value);
  key_value.key = "Miscalibration confidence threshold";
  key_value.value = std::to_string(miscalibration_confidence_threshold_);
  status.values.push_back(key_value);
  key_value.key = "Is vehicle moving";
  key_value.value = velocity_check_status.is_vehicle_moving ? "True" : "False";
  status.values.push_back(key_value);
  key_value.key = "Velocity age (s)";
  key_value.value = std::to_string(velocity_check_status.velocity_age);
  status.values.push_back(key_value);
  key_value.key = "Is calibrated";
  key_value.value = is_calibrated ? "True" : "False";
  status.values.push_back(key_value);
  key_value.key = "Calibration confidence";
  key_value.value = std::to_string(result.calibration_confidence);
  status.values.push_back(key_value);
  key_value.key = "Miscalibration confidence";
  key_value.value = std::to_string(result.miscalibration_confidence);
  status.values.push_back(key_value);
  key_value.key = "Preprocessing time (ms)";
  key_value.value = std::to_string(result.preprocessing_time_ms);
  status.values.push_back(key_value);
  key_value.key = "Inference time (ms)";
  key_value.value = std::to_string(result.inference_time_ms);
  status.values.push_back(key_value);
  key_value.key = "Number of points projected";
  key_value.value = std::to_string(result.num_points_projected);
  status.values.push_back(key_value);

  diagnostic_msgs::msg::DiagnosticArray diagnostic_array;
  diagnostic_array.header.stamp = this->get_clock()->now();
  diagnostic_array.header.frame_id = this->get_name();
  diagnostic_array.status.push_back(status);
  diagnostic_pub_->publish(diagnostic_array);
}

}  // namespace autoware::calibration_status

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::calibration_status::CalibrationStatusNode)
