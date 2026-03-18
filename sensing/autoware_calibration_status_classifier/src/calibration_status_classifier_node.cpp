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

#include "autoware/calibration_status_classifier/calibration_status_classifier_node.hpp"

#include "autoware/calibration_status_classifier/calibration_status_classifier_filters.hpp"
#include "autoware/calibration_status_classifier/camera_lidar_info_collector.hpp"
#include "autoware/calibration_status_classifier/filter.hpp"
#include "autoware/calibration_status_classifier/ros_utils.hpp"

#include <rclcpp/qos.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <rmw/qos_profiles.h>

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::calibration_status_classifier
{

CalibrationStatusClassifierNode::CalibrationStatusClassifierNode(
  const rclcpp::NodeOptions & options)
: rclcpp::Node("calibration_status_classifier", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // Core library for calibration status
  const CalibrationStatusClassifierConfig calibration_status_classifier_config(
    this->declare_parameter<double>("max_depth"), this->declare_parameter<int64_t>("dilation_size"),
    this->declare_parameter<std::vector<int64_t>>("height"),
    this->declare_parameter<std::vector<int64_t>>("width"));

  calibration_status_classifier_ = std::make_unique<CalibrationStatusClassifier>(
    this->declare_parameter<std::string>("onnx_path"),
    this->declare_parameter<std::string>("trt_precision"),
    this->declare_parameter<std::int64_t>("cloud_capacity"),
    this->declare_parameter<std::vector<double>>("ego_box"), calibration_status_classifier_config);

  // Runtime mode configuration
  runtime_mode_ = string_to_runtime_mode(this->declare_parameter<std::string>("runtime_mode"));
  period_ = this->declare_parameter<double>("period");
  queue_size_ = this->declare_parameter<int64_t>("queue_size");

  // Prerequisite filter configuration
  const auto linear_velocity_enabled =
    this->declare_parameter<bool>("prerequisite.linear_velocity_check.enabled");
  const auto linear_velocity_source_str =
    this->declare_parameter<std::string>("prerequisite.linear_velocity_check.source");
  const auto linear_velocity_threshold =
    this->declare_parameter<double>("prerequisite.linear_velocity_check.threshold");
  const auto linear_velocity_timeout =
    this->declare_parameter<double>("prerequisite.linear_velocity_check.timeout");

  const auto angular_velocity_enabled =
    this->declare_parameter<bool>("prerequisite.angular_velocity_check.enabled");
  const auto angular_velocity_source_str =
    this->declare_parameter<std::string>("prerequisite.angular_velocity_check.source");
  const auto angular_velocity_threshold =
    this->declare_parameter<double>("prerequisite.angular_velocity_check.threshold");
  const auto angular_velocity_timeout =
    this->declare_parameter<double>("prerequisite.angular_velocity_check.timeout");

  const auto objects_enabled = this->declare_parameter<bool>("prerequisite.objects_check.enabled");
  const auto objects_source_str =
    this->declare_parameter<std::string>("prerequisite.objects_check.source");
  const auto objects_threshold =
    static_cast<size_t>(this->declare_parameter<int64_t>("prerequisite.objects_check.threshold"));
  const auto objects_timeout =
    this->declare_parameter<double>("prerequisite.objects_check.timeout");

  std::vector<std::unique_ptr<Filter>> filters;
  if (linear_velocity_enabled) {
    filters.push_back(
      std::make_unique<LinearVelocityFilter>(linear_velocity_threshold, linear_velocity_timeout));
  }
  if (angular_velocity_enabled) {
    filters.push_back(
      std::make_unique<AngularVelocityFilter>(
        angular_velocity_threshold, angular_velocity_timeout));
  }
  if (objects_enabled) {
    filters.push_back(std::make_unique<ObjectsFilter>(objects_threshold, objects_timeout));
  }
  filters_ = CalibrationStatusClassifierFilters(std::move(filters));

  // Setup subscribers
  if (linear_velocity_enabled) {
    setup_linear_velocity_source_interface(
      string_to_linear_velocity_source(linear_velocity_source_str));
  }
  if (angular_velocity_enabled) {
    setup_angular_velocity_source_interface(
      string_to_angular_velocity_source(angular_velocity_source_str));
  }
  if (objects_enabled) {
    setup_object_detection_interface(string_to_objects_source(objects_source_str));
  }

  // Input configuration
  camera_lidar_in_out_info_ = compose_in_out_topics(
    this->declare_parameter<std::vector<std::string>>("input.cloud_topics"),
    this->declare_parameter<std::vector<std::string>>("input.image_topics"),
    this->declare_parameter<std::vector<double>>("input.approx_deltas"),
    this->declare_parameter<std::vector<double>>("input.miscalibration_confidence_thresholds"),
    this->declare_parameter<std::vector<bool>>("input.already_rectified"));

  auto camera_lidar_info_collector =
    std::make_shared<CameraLidarInfoCollector>(this, camera_lidar_in_out_info_);
  camera_lidar_info_ = camera_lidar_info_collector->get_cameras_lidars_info();

  setup_runtime_mode_interface();

  if (this->declare_parameter<bool>("build_only", false)) {
    RCLCPP_INFO(this->get_logger(), "TensorRT engine is built. Shutting down the node.");
    rclcpp::shutdown();
  }
}

void CalibrationStatusClassifierNode::setup_runtime_mode_interface()
{
  setup_input_synchronization();
  if (runtime_mode_ == RuntimeMode::MANUAL) {
    calibration_service_ = this->create_service<std_srvs::srv::Trigger>(
      "~/input/validate_calibration_srv",
      std::bind(
        &CalibrationStatusClassifierNode::handle_calibration_request, this, std::placeholders::_1,
        std::placeholders::_2));
  } else if (runtime_mode_ == RuntimeMode::PERIODIC) {
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period_),
      std::bind(&CalibrationStatusClassifierNode::periodic_callback, this));

  } else if (runtime_mode_ == RuntimeMode::ACTIVE) {
  }
}

void CalibrationStatusClassifierNode::setup_linear_velocity_source_interface(
  LinearVelocitySource source)
{
  const std::string topic = "~/input/linear_velocity";
  switch (source) {
    case LinearVelocitySource::TWIST_STAMPED:
      linear_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          filters_.update<LinearVelocityFilter>(
            LinearVelocityInfo{msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z},
            rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    case LinearVelocitySource::TWIST_WITH_COV_STAMPED:
      linear_velocity_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          topic, rclcpp::SensorDataQoS(),
          [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
            filters_.update<LinearVelocityFilter>(
              LinearVelocityInfo{
                msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z},
              rclcpp::Time(msg->header.stamp).seconds());
          });
      break;
    case LinearVelocitySource::ODOMETRY:
      linear_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          filters_.update<LinearVelocityFilter>(
            LinearVelocityInfo{
              msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z},
            rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    default:
      throw std::invalid_argument("Unsupported linear velocity source");
  }
}

void CalibrationStatusClassifierNode::setup_angular_velocity_source_interface(
  AngularVelocitySource source)
{
  const std::string topic = "~/input/angular_velocity";
  switch (source) {
    case AngularVelocitySource::TWIST_STAMPED:
      angular_velocity_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, rclcpp::SensorDataQoS(),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          filters_.update<AngularVelocityFilter>(
            AngularVelocityInfo{msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z},
            rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    case AngularVelocitySource::TWIST_WITH_COV_STAMPED:
      angular_velocity_sub_ =
        this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
          topic, rclcpp::SensorDataQoS(),
          [this](const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg) {
            filters_.update<AngularVelocityFilter>(
              AngularVelocityInfo{
                msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z},
              rclcpp::Time(msg->header.stamp).seconds());
          });
      break;
    case AngularVelocitySource::ODOMETRY:
      angular_velocity_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          filters_.update<AngularVelocityFilter>(
            AngularVelocityInfo{
              msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z},
            rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    default:
      throw std::invalid_argument("Unsupported angular velocity source");
  }
}

void CalibrationStatusClassifierNode::setup_object_detection_interface(ObjectsSource source)
{
  const std::string topic = "~/input/objects";
  switch (source) {
    case ObjectsSource::PREDICTED_OBJECTS:
      objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
        topic, rclcpp::SensorDataQoS(),
        [this](const autoware_perception_msgs::msg::PredictedObjects::SharedPtr msg) {
          std::vector<ObjectInfo> objects;
          objects.reserve(msg->objects.size());
          for (const auto & obj : msg->objects) {
            const auto & p = obj.kinematics.initial_pose_with_covariance.pose.position;
            objects.push_back({p.x, p.y, p.z});
          }
          filters_.update<ObjectsFilter>(objects, rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    case ObjectsSource::TRACKED_OBJECTS:
      objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::TrackedObjects>(
        topic, rclcpp::SensorDataQoS(),
        [this](const autoware_perception_msgs::msg::TrackedObjects::SharedPtr msg) {
          std::vector<ObjectInfo> objects;
          objects.reserve(msg->objects.size());
          for (const auto & obj : msg->objects) {
            const auto & p = obj.kinematics.pose_with_covariance.pose.position;
            objects.push_back({p.x, p.y, p.z});
          }
          filters_.update<ObjectsFilter>(objects, rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    case ObjectsSource::DETECTED_OBJECTS:
      objects_sub_ = this->create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
        topic, rclcpp::SensorDataQoS(),
        [this](const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg) {
          std::vector<ObjectInfo> objects;
          objects.reserve(msg->objects.size());
          for (const auto & obj : msg->objects) {
            const auto & p = obj.kinematics.pose_with_covariance.pose.position;
            objects.push_back({p.x, p.y, p.z});
          }
          filters_.update<ObjectsFilter>(objects, rclcpp::Time(msg->header.stamp).seconds());
        });
      break;
    default:
      throw std::invalid_argument("Unsupported objects source");
  }
}

void CalibrationStatusClassifierNode::setup_input_synchronization()
{
  const auto num_pairs = camera_lidar_in_out_info_.size();
  cloud_subs_.resize(num_pairs);
  image_subs_.resize(num_pairs);
  preview_image_pubs_.resize(num_pairs);
  synchronizers_.resize(num_pairs);
  synchronized_data_.resize(num_pairs);
  diagnostics_interfaces_.resize(num_pairs);

  for (size_t i = 0; i < num_pairs; ++i) {
    cloud_subs_.at(i) =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
        this, camera_lidar_in_out_info_.at(i).lidar_topic, rmw_qos_profile_sensor_data);
    image_subs_.at(i) = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, camera_lidar_in_out_info_.at(i).camera_topic, rmw_qos_profile_sensor_data);
    preview_image_pubs_.at(i) = this->create_publisher<sensor_msgs::msg::Image>(
      camera_lidar_in_out_info_.at(i).projected_points_topic, rclcpp::SensorDataQoS());

    // Create synchronizer
    synchronizers_.at(i) = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(queue_size_), *cloud_subs_.at(i), *image_subs_.at(i));

    // Set approximate time delta
    synchronizers_.at(i)->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(camera_lidar_in_out_info_.at(i).approx_delta));

    // Register callback
    synchronizers_.at(i)->registerCallback(
      std::bind(
        &CalibrationStatusClassifierNode::synchronized_callback, this, std::placeholders::_1,
        std::placeholders::_2, i));

    diagnostics_interfaces_.at(i) = std::make_unique<autoware_utils::DiagnosticsInterface>(
      this, "calibration_status_classifier_" + std::to_string(i));

    RCLCPP_INFO(
      this->get_logger(), "Miscalibration detection pair %zu: %s <-> %s (sync delta: %.3f s)", i,
      camera_lidar_in_out_info_.at(i).lidar_topic.c_str(),
      camera_lidar_in_out_info_.at(i).camera_topic.c_str(),
      camera_lidar_in_out_info_.at(i).approx_delta);
  }
}

void CalibrationStatusClassifierNode::handle_calibration_request(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  (void)request;
  std::size_t available_pairs{0};
  for (size_t pair_idx = 0; pair_idx < synchronized_data_.size(); ++pair_idx) {
    if (run(pair_idx)) {
      ++available_pairs;
    }
  }
  std::string str_msg_suffix = " (" + std::to_string(available_pairs) + " out of " +
                               std::to_string(synchronized_data_.size()) + " pair(s)).";
  if (available_pairs == synchronized_data_.size()) {
    response->success = true;
    response->message = "Calibration validation completed for all topic pairs" + str_msg_suffix;
  } else {
    response->success = false;
    response->message =
      "Calibration validation could not be completed for all topic pairs" + str_msg_suffix;
  }
}

void CalibrationStatusClassifierNode::periodic_callback()
{
  for (size_t pair_idx = 0; pair_idx < synchronized_data_.size(); ++pair_idx) {
    run(pair_idx);
  }
}

void CalibrationStatusClassifierNode::synchronized_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & image_msg, size_t pair_idx)
{
  std::unique_lock<std::mutex> lock(synchronized_data_mutex_);
  synchronized_data_.at(pair_idx) = {cloud_msg, image_msg};
  lock.unlock();

  if (runtime_mode_ == RuntimeMode::ACTIVE) {
    run(pair_idx);
  }
}

bool CalibrationStatusClassifierNode::run(std::size_t pair_idx)
{
  std::unique_lock<std::mutex> lock(synchronized_data_mutex_);
  if (
    synchronized_data_.at(pair_idx).first == nullptr ||
    synchronized_data_.at(pair_idx).second == nullptr) {
    return false;
  }
  auto [cloud_msg, image_msg] = std::move(synchronized_data_.at(pair_idx));
  lock.unlock();

  const auto cloud_stamp = rclcpp::Time(cloud_msg->header.stamp);
  const auto image_stamp = rclcpp::Time(image_msg->header.stamp);
  const auto common_stamp = std::max(cloud_stamp, image_stamp);

  InputMetadata input_metadata;
  input_metadata.cloud_stamp = cloud_stamp;
  input_metadata.image_stamp = image_stamp;
  input_metadata.common_stamp = common_stamp;
  input_metadata.filters_result = filters_.evaluate(common_stamp.seconds());

  if (!input_metadata.filters_result.all_passed) {
    publish_diagnostic_status(input_metadata, pair_idx);
    return true;
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

  auto result = calibration_status_classifier_->process(
    cloud_msg, image_msg, camera_lidar_info_.at(pair_idx), preview_img_data);

  publish_diagnostic_status(input_metadata, pair_idx, result);

  if (is_preview_subscribed) {
    preview_image_pubs_.at(pair_idx)->publish(*preview_img_msg);
  }

  return true;
}

void CalibrationStatusClassifierNode::publish_diagnostic_status(
  const InputMetadata & input_metadata, const size_t pair_idx,
  const CalibrationStatusClassifierResult & result)
{
  diagnostics_interfaces_.at(pair_idx)->clear();
  auto now = this->get_clock()->now();
  const auto miscalibration_confidence_threshold =
    camera_lidar_in_out_info_.at(pair_idx).miscalibration_confidence_threshold;
  auto is_calibrated =
    (result.calibration_confidence >
     result.miscalibration_confidence + miscalibration_confidence_threshold);

  if (!input_metadata.filters_result.all_passed) {
    diagnostics_interfaces_.at(pair_idx)->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Calibration check skipped due to filter not passed.");
  } else if (is_calibrated) {
    diagnostics_interfaces_.at(pair_idx)->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::OK, "Calibration is valid");
  } else {
    diagnostics_interfaces_.at(pair_idx)->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Calibration is invalid.");
  }

  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Source image topic", camera_lidar_in_out_info_.at(pair_idx).camera_topic);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Source image frame", camera_lidar_info_.at(pair_idx).camera_frame_id);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Source point cloud topic", camera_lidar_in_out_info_.at(pair_idx).lidar_topic);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Source point cloud frame", camera_lidar_info_.at(pair_idx).lidar_frame_id);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Point cloud and image time difference (ms)",
    std::abs((input_metadata.cloud_stamp - input_metadata.image_stamp).seconds()) * 1e3);

  // Standardized filter diagnostics
  for (const auto & fr : input_metadata.filters_result.filter_results) {
    diagnostics_interfaces_.at(pair_idx)->add_key_value("Is " + fr.name + " check activated", true);
    diagnostics_interfaces_.at(pair_idx)->add_key_value("Current " + fr.name, fr.current_value);
    diagnostics_interfaces_.at(pair_idx)->add_key_value(fr.name + " unit", fr.unit);
    diagnostics_interfaces_.at(pair_idx)->add_key_value(fr.name + " threshold", fr.threshold);
    diagnostics_interfaces_.at(pair_idx)->add_key_value(
      "Is " + fr.name + " threshold exceeded", fr.is_threshold_exceeded);
    diagnostics_interfaces_.at(pair_idx)->add_key_value("Is " + fr.name + " passed", fr.is_passed);
    diagnostics_interfaces_.at(pair_idx)->add_key_value(fr.name + " timeout (s)", fr.timeout_sec);
    diagnostics_interfaces_.at(pair_idx)->add_key_value(fr.name + " age (ms)", fr.state_age * 1e3);
  }

  // Calibration result diagnostics
  diagnostics_interfaces_.at(pair_idx)->add_key_value("Is calibrated", is_calibrated);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Miscalibration confidence threshold", miscalibration_confidence_threshold);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Calibration confidence", result.calibration_confidence);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Miscalibration confidence", result.miscalibration_confidence);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Preprocessing time (ms)", result.preprocessing_time_ms);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Inference time (ms)", result.inference_time_ms);
  diagnostics_interfaces_.at(pair_idx)->add_key_value(
    "Number of points projected", result.num_points_projected);

  diagnostics_interfaces_.at(pair_idx)->publish(now);
}

}  // namespace autoware::calibration_status_classifier

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::calibration_status_classifier::CalibrationStatusClassifierNode)
