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

#include <iomanip>
#define EIGEN_MPL2_ONLY

#include "autoware/image_projection_based_fusion/fusion_collector.hpp"
#include "autoware/image_projection_based_fusion/fusion_node.hpp"
#include "autoware/image_projection_based_fusion/fusion_types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware/image_projection_based_fusion/utils/utils.hpp>

#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>

#include <boost/optional.hpp>

#include <cmath>
#include <limits>
#include <list>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::image_projection_based_fusion
{
using autoware_utils::ScopedTimeTrack;

template <class Msg3D, class Msg2D, class ExportObj>
FusionNode<Msg3D, Msg2D, ExportObj>::FusionNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // set rois_number
  rois_number_ = static_cast<std::size_t>(declare_parameter<int32_t>("rois_number"));
  if (rois_number_ < 1) {
    throw std::runtime_error(
      "Minimum rois_number_ is 1. Current rois_number_ is " + std::to_string(rois_number_));
  }
  if (rois_number_ > 8) {
    RCLCPP_WARN(
      this->get_logger(),
      "Current rois_number_ is %zu. Large rois number may cause performance issue.", rois_number_);
  }

  // Set parameters
  msg3d_timeout_sec_ = declare_parameter<double>("msg3d_timeout_sec");
  rois_timeout_sec_ = declare_parameter<double>("rois_timeout_sec");

  auto rois_timestamp_offsets = declare_parameter<std::vector<double>>("rois_timestamp_offsets");
  if (rois_timestamp_offsets.size() < rois_number_) {
    throw std::runtime_error(
      "Mismatch: rois_number (" + std::to_string(rois_number_) +
      ") does not match rois_timestamp_offsets size (" +
      std::to_string(rois_timestamp_offsets.size()) + ").");
  }

  for (std::size_t i = 0; i < rois_number_; i++) {
    id_to_offset_map_[i] = rois_timestamp_offsets[i];
  }

  std::vector<std::string> input_rois_topics;
  std::vector<std::string> input_camera_info_topics;

  input_rois_topics.resize(rois_number_);
  input_camera_info_topics.resize(rois_number_);

  for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
    input_rois_topics.at(roi_i) = declare_parameter<std::string>(
      "input/rois" + std::to_string(roi_i),
      "/perception/object_recognition/detection/rois" + std::to_string(roi_i));

    input_camera_info_topics.at(roi_i) = declare_parameter<std::string>(
      "input/camera_info" + std::to_string(roi_i),
      "/sensing/camera/camera" + std::to_string(roi_i) + "/camera_info");
  }

  // Subscribe to Camera Info
  camera_info_subs_.resize(rois_number_);
  for (auto rois_id = 0u; rois_id < rois_number_; ++rois_id) {
    auto topic = input_camera_info_topics.at(rois_id);
    auto qos = rclcpp::QoS{1}.best_effort();

    camera_info_subs_[rois_id] = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      topic, qos, [this, rois_id](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        this->camera_info_callback(msg, rois_id);
      });
  }

  // Subscribe to ROIs
  rois_subs_.resize(rois_number_);

  for (auto rois_id = 0u; rois_id < rois_number_; ++rois_id) {
    auto topic = input_rois_topics.at(rois_id);
    auto qos = rclcpp::QoS{1}.best_effort();

    rois_subs_[rois_id] = this->create_subscription<Msg2D>(
      topic, qos, [this, rois_id](const typename Msg2D::ConstSharedPtr msg) {
        this->rois_callback(msg, rois_id);
      });
  }

  // Subscribe 3D input msg
  msg3d_sub_ = this->create_subscription<Msg3D>(
    "input", rclcpp::QoS(1).best_effort(),
    [this](const typename Msg3D::ConstSharedPtr msg) { this->sub_callback(msg); });

  // initialization on each 2d detections
  initialize_det2d_status(rois_number_);

  // parameters for approximation grid
  approx_grid_cell_w_size_ = declare_parameter<float>("approximation_grid_cell_width");
  approx_grid_cell_h_size_ = declare_parameter<float>("approximation_grid_cell_height");

  // parameters for out_of_scope filter
  filter_scope_min_x_ = declare_parameter<double>("filter_scope_min_x");
  filter_scope_max_x_ = declare_parameter<double>("filter_scope_max_x");
  filter_scope_min_y_ = declare_parameter<double>("filter_scope_min_y");
  filter_scope_max_y_ = declare_parameter<double>("filter_scope_max_y");
  filter_scope_min_z_ = declare_parameter<double>("filter_scope_min_z");
  filter_scope_max_z_ = declare_parameter<double>("filter_scope_max_z");

  rosbag_length_ = declare_parameter<double>("rosbag_length");
  publish_previous_but_late_output_msg_ =
    declare_parameter<bool>("publish_previous_but_late_output_msg");
  matching_strategy_ = declare_parameter<std::string>("matching_strategy.type");

  // debugger
  debug_mode_ = declare_parameter<bool>("debug_mode");
  if (debug_mode_) {
    std::vector<std::string> input_camera_topics;
    input_camera_topics.resize(rois_number_);
    for (std::size_t roi_i = 0; roi_i < rois_number_; ++roi_i) {
      input_camera_topics.at(roi_i) = declare_parameter<std::string>(
        "input/image" + std::to_string(roi_i),
        "/sensing/camera/camera" + std::to_string(roi_i) + "/image_rect_color");
    }
    auto image_buffer_size =
      static_cast<std::size_t>(declare_parameter<int32_t>("image_buffer_size"));
    debugger_ =
      std::make_shared<Debugger>(this, rois_number_, image_buffer_size, input_camera_topics);

    // input topic timing publisher
    debug_internal_pub_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());
  }
  collector_debug_mode_ = declare_parameter<bool>("collector_debug_mode");

  // time keeper
  bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
  if (use_time_keeper) {
    detailed_processing_time_publisher_ =
      this->create_publisher<autoware_utils::ProcessingTimeDetail>(
        "~/debug/processing_time_detail_ms", 1);
    auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
    time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
  }

  // initialize debug tool
  {
    stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ = std::make_unique<autoware_utils::DebugPublisher>(this, get_name());
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");
  }

  // Diagnostic Updater
  diagnostic_updater_.setHardwareID(node_name + "_checker");
  diagnostic_updater_.add(node_name + "_status", this, &FusionNode::check_fusion_status);
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::initialize_strategy()
{
  if (matching_strategy_ == "naive") {
    fusion_matching_strategy_ = std::make_unique<NaiveMatchingStrategy<Msg3D, Msg2D, ExportObj>>(
      std::dynamic_pointer_cast<FusionNode>(shared_from_this()), id_to_offset_map_);
  } else if (matching_strategy_ == "advanced") {
    fusion_matching_strategy_ = std::make_unique<AdvancedMatchingStrategy<Msg3D, Msg2D, ExportObj>>(
      std::dynamic_pointer_cast<FusionNode>(shared_from_this()), id_to_offset_map_);
    // subscribe diagnostics
    sub_diag_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", 10, std::bind(&FusionNode::diagnostic_callback, this, std::placeholders::_1));
  } else {
    throw std::runtime_error("Matching strategy must be 'advanced' or 'naive'");
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::initialize_collector_list()
{
  // Initialize collector list
  for (size_t i = 0; i < num_of_collectors; ++i) {
    fusion_collectors_.emplace_back(std::make_shared<FusionCollector<Msg3D, Msg2D, ExportObj>>(
      std::dynamic_pointer_cast<FusionNode>(shared_from_this()), rois_number_, det2d_status_list_,
      collector_debug_mode_));
  }
  init_collector_list_ = true;
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::initialize_det2d_status(std::size_t rois_number)
{
  // Camera projection settings
  std::vector<bool> point_project_to_unrectified_image =
    declare_parameter<std::vector<bool>>("point_project_to_unrectified_image");
  if (rois_number > point_project_to_unrectified_image.size()) {
    throw std::runtime_error(
      "The number of point_project_to_unrectified_image does not match the number of rois "
      "topics.");
  }
  std::vector<bool> approximate_camera_projection =
    declare_parameter<std::vector<bool>>("approximate_camera_projection");
  if (rois_number > approximate_camera_projection.size()) {
    const std::size_t current_size = approximate_camera_projection.size();
    throw std::runtime_error(
      "The number of elements in approximate_camera_projection should be the same as rois_number_. "
      "It has " +
      std::to_string(current_size) + " elements, but rois_number is " +
      std::to_string(rois_number) + ".");
  }

  // 2D detection status initialization
  det2d_status_list_.resize(rois_number_);
  for (std::size_t rois_id = 0; rois_id < rois_number_; ++rois_id) {
    det2d_status_list_.at(rois_id).id = rois_id;
    det2d_status_list_.at(rois_id).project_to_unrectified_image =
      point_project_to_unrectified_image.at(rois_id);
    det2d_status_list_.at(rois_id).approximate_camera_projection =
      approximate_camera_projection.at(rois_id);
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_camera_info_msg,
  const std::size_t rois_id)
{
  if (rois_id >= det2d_status_list_.size()) {
    throw std::out_of_range("rois_id " + std::to_string(rois_id) + " is out of range.");
  }

  // Create the CameraProjection only when the camera info arrives for the first time.
  // This assume the camera info does not change while the node is running
  auto & det2d_status = det2d_status_list_.at(rois_id);
  if (det2d_status.camera_projector_ptr == nullptr && check_camera_info(*input_camera_info_msg)) {
    det2d_status.camera_projector_ptr = std::make_unique<CameraProjection>(
      *input_camera_info_msg, approx_grid_cell_w_size_, approx_grid_cell_h_size_,
      det2d_status.project_to_unrectified_image, det2d_status.approximate_camera_projection);
    det2d_status.camera_projector_ptr->initialize();

    std::unique_lock<std::mutex> fusion_collectors_lock(fusion_collectors_mutex_);
    for (auto & collector : fusion_collectors_) {
      collector->add_camera_projection(rois_id, det2d_status.camera_projector_ptr);
    }
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::preprocess([[maybe_unused]] Msg3D & output_msg)
{
  // Default implementation: No preprocessing.
  // This function can be overridden by derived classes if needed.
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::export_process(
  typename Msg3D::SharedPtr & output_det3d_msg,
  std::unordered_map<std::size_t, double> id_to_stamp_map,
  std::shared_ptr<FusionCollectorInfoBase> collector_info)
{
  ExportObj output_msg;
  postprocess(*(output_det3d_msg), output_msg);

  // Update timestamp
  current_output_msg_timestamp_ = rclcpp::Time(output_msg.header.stamp).seconds();

  // Handle late messages during rosbag replay
  if (
    current_output_msg_timestamp_ < latest_output_msg_timestamp_ &&
    !publish_previous_but_late_output_msg_) {
    double timestamp_diff = latest_output_msg_timestamp_ - current_output_msg_timestamp_;

    publish_output_msg_ = (timestamp_diff > rosbag_length_);    // Force publish if rosbag looped
    drop_previous_but_late_output_msg_ = !publish_output_msg_;  // Drop otherwise
  } else {
    publish_output_msg_ = true;  // Publish normally
  }

  if (publish_output_msg_) {
    publish(output_msg);
  }

  // Move collected diagnostics info
  diagnostic_collector_info_ = std::move(collector_info);
  diagnostic_id_to_stamp_map_ = std::move(id_to_stamp_map);
  diagnostic_updater_.force_update();

  // Add processing time for debugging
  if (debug_publisher_) {
    auto cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    auto pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - output_det3d_msg->header.stamp).nanoseconds()))
        .count();

    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
    debug_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);
  }

  // debug
  if (debug_internal_pub_) {
    for (std::size_t rois_id = 0; rois_id < rois_number_; ++rois_id) {
      auto rois_timestamp = diagnostic_id_to_stamp_map_[rois_id];
      auto timestamp_interval_ms = (rois_timestamp - current_output_msg_timestamp_) * 1000;
      debug_internal_pub_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/roi" + std::to_string(rois_id) + "/timestamp_interval_ms", timestamp_interval_ms);
      debug_internal_pub_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
        "debug/roi" + std::to_string(rois_id) + "/timestamp_interval_offset_ms",
        timestamp_interval_ms - id_to_offset_map_[rois_id] * 1000);
    }
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::sub_callback(const typename Msg3D::ConstSharedPtr msg3d)
{
  if (!fusion_matching_strategy_) {
    initialize_strategy();
  }

  if (!init_collector_list_) {
    initialize_collector_list();
  }

  // Debug logging for message latency
  if (collector_debug_mode_) {
    auto arrive_time = this->get_clock()->now().seconds();
    auto msg3d_stamp = rclcpp::Time(msg3d->header.stamp).seconds();
    RCLCPP_DEBUG(
      this->get_logger(), "msg3d timestamp: %lf, arrive time: %lf, latency: %lf", msg3d_stamp,
      arrive_time, arrive_time - msg3d_stamp);
  }

  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  stop_watch_ptr_->toc("processing_time", true);
  manage_collector_list();

  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> selected_collector = nullptr;
  // Lock fusion collectors list
  std::unique_lock<std::mutex> fusion_collectors_lock(fusion_collectors_mutex_);

  auto msg3d_timestamp = rclcpp::Time(msg3d->header.stamp).seconds();

  // Create matching parameters
  auto matching_context = std::make_shared<Msg3dMatchingContext>();
  matching_context->msg3d_timestamp = msg3d_timestamp;

  // Try to find an existing FusionCollector that matches this message
  auto fusion_collector =
    !fusion_collectors_.empty()
      ? fusion_matching_strategy_->match_msg3d_to_collector(fusion_collectors_, matching_context)
      : std::nullopt;

  if (fusion_collector && fusion_collector.value()) {
    selected_collector = fusion_collector.value();
  }

  // If no suitable collector was found, reuse the collector if the status is Idle
  if (!selected_collector || selected_collector->get_status() == CollectorStatus::Finished) {
    auto it = std::find_if(
      fusion_collectors_.begin(), fusion_collectors_.end(),
      [](const auto & collector) { return collector->get_status() == CollectorStatus::Idle; });

    if (it != fusion_collectors_.end()) {
      selected_collector = *it;
    }
  }

  fusion_collectors_lock.unlock();
  if (selected_collector) {
    fusion_matching_strategy_->set_collector_info(selected_collector, matching_context);
    selected_collector->process_msg3d(msg3d, msg3d_timeout_sec_);
  } else {
    // Handle case where no suitable collector is found
    RCLCPP_WARN(get_logger(), "No available FusionCollector in IDLE state.");
  }

  if (matching_strategy_ == "advanced") {
    // remove outdated messages in the concatenated map
    manage_concatenated_status_map(msg3d_timestamp);
  }

  if (debugger_) {
    debugger_->clear();
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::rois_callback(
  const typename Msg2D::ConstSharedPtr rois_msg, const std::size_t rois_id)
{
  if (!fusion_matching_strategy_) {
    initialize_strategy();
  }

  if (!init_collector_list_) {
    initialize_collector_list();
  }

  if (collector_debug_mode_) {
    auto arrive_time = this->get_clock()->now().seconds();
    RCLCPP_DEBUG(
      this->get_logger(), " rois %zu timestamp: %lf arrive time: %lf seconds, latency: %lf",
      rois_id, rclcpp::Time(rois_msg->header.stamp).seconds(), arrive_time,
      arrive_time - rclcpp::Time(rois_msg->header.stamp).seconds());
  }

  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  manage_collector_list();

  std::shared_ptr<FusionCollector<Msg3D, Msg2D, ExportObj>> selected_collector = nullptr;
  // Lock fusion collectors list
  std::unique_lock<std::mutex> fusion_collectors_lock(fusion_collectors_mutex_);

  auto rois_timestamp = rclcpp::Time(rois_msg->header.stamp).seconds();

  // Create matching parameters
  auto matching_context = std::make_shared<RoisMatchingContext>();
  matching_context->rois_id = rois_id;
  matching_context->rois_timestamp = rois_timestamp;

  // Try to find an existing FusionCollector that matches this message
  auto fusion_collector =
    !fusion_collectors_.empty()
      ? fusion_matching_strategy_->match_rois_to_collector(fusion_collectors_, matching_context)
      : std::nullopt;

  if (fusion_collector && fusion_collector.value()) {
    selected_collector = fusion_collector.value();
  }

  // If no suitable collector was found, reuse the collector if the status is Idle
  if (!selected_collector || selected_collector->get_status() == CollectorStatus::Finished) {
    auto it = std::find_if(
      fusion_collectors_.begin(), fusion_collectors_.end(),
      [](const auto & collector) { return collector->get_status() == CollectorStatus::Idle; });

    if (it != fusion_collectors_.end()) {
      selected_collector = *it;
    }
  }

  fusion_collectors_lock.unlock();
  if (selected_collector) {
    fusion_matching_strategy_->set_collector_info(selected_collector, matching_context);
    selected_collector->process_rois(rois_id, rois_msg, rois_timeout_sec_);
  } else {
    // Handle case where no suitable collector is found
    RCLCPP_WARN(get_logger(), "No available FusionCollector in IDLE state.");
  }

  if (debugger_) {
    debugger_->clear();
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::diagnostic_callback(
  const diagnostic_msgs::msg::DiagnosticArray::SharedPtr diagnostic_msg)
{
  for (const auto & status : diagnostic_msg->status) {
    // Filter for the concatenate_and_time_sync_node diagnostic message
    if (status.name == std::string_view("concatenate_data: /sensing/lidar/concatenate_data")) {
      std::optional<double> concatenate_timestamp_opt;

      // First pass: Locate concatenated_cloud_timestamp
      for (const auto & value : status.values) {
        if (value.key == std::string_view("Concatenated pointcloud timestamp")) {
          try {
            concatenate_timestamp_opt = std::stod(value.value);
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Error parsing concatenated cloud timestamp: %s", e.what());
          }
        }
      }

      // Second pass: Fill key-value map only if timestamp was valid
      if (concatenate_timestamp_opt.has_value()) {
        std::unordered_map<std::string, std::string> key_value_map;
        for (const auto & value : status.values) {
          key_value_map.emplace(value.key, value.value);
        }

        concatenated_status_map_.emplace(
          concatenate_timestamp_opt.value(), std::move(key_value_map));
      }
    }
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::manage_concatenated_status_map(double current_timestamp)
{
  constexpr double threshold_seconds = 1.0;  // Define threshold as a constant

  // Remove old entries from concatenated_status_map_
  auto it = concatenated_status_map_.begin();
  while (it != concatenated_status_map_.end()) {
    if (current_timestamp - it->first > threshold_seconds) {
      RCLCPP_DEBUG(
        get_logger(), "Removing old concatenation status for timestamp: %.9f", it->first);
      it = concatenated_status_map_.erase(it);
    } else {
      ++it;
    }
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::postprocess(
  [[maybe_unused]] const Msg3D & processing_msg, [[maybe_unused]] ExportObj & output_msg)
{
  // Default implementation: No postprocessing.
  // This function can be overridden by derived classes if needed.
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::publish(const ExportObj & output_msg)
{
  if (!pub_ptr_) {
    RCLCPP_WARN(get_logger(), "Publish failed: pub_ptr_ is null.");
    return;
  }

  if (pub_ptr_->get_subscription_count() < 1) {
    RCLCPP_DEBUG(get_logger(), "No subscribers, skipping publish.");
    return;
  }

  pub_ptr_->publish(output_msg);
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::manage_collector_list()
{
  std::lock_guard<std::mutex> collectors_lock(fusion_collectors_mutex_);

  int num_processing_collectors = 0;

  for (auto & collector : fusion_collectors_) {
    if (collector->get_status() == CollectorStatus::Finished) {
      collector->reset();
    }

    if (collector->get_status() == CollectorStatus::Processing) {
      num_processing_collectors++;
    }
  }

  if (num_processing_collectors == num_of_collectors) {
    auto min_it = fusion_collectors_.end();
    constexpr double k_max_timestamp = std::numeric_limits<double>::max();
    double min_timestamp = k_max_timestamp;

    for (auto it = fusion_collectors_.begin(); it != fusion_collectors_.end(); ++it) {
      if ((*it)->get_status() == CollectorStatus::Processing) {
        auto info = (*it)->get_info();
        double timestamp = k_max_timestamp;

        if (auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(info)) {
          timestamp = naive_info->timestamp;
        } else if (auto advanced_info = std::dynamic_pointer_cast<AdvancedCollectorInfo>(info)) {
          timestamp = advanced_info->timestamp;
        } else {
          continue;
        }

        if (timestamp < min_timestamp) {
          min_timestamp = timestamp;
          min_it = it;
        }
      }
    }

    // Reset the collector with the oldest timestamp if found
    if (min_it != fusion_collectors_.end()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Reset the oldest collector because the number of processing collectors ("
          << num_processing_collectors << ") equal to the limit (" << num_of_collectors << ").");
      (*min_it)->reset();
    }
  }
}

template <class Msg3D, class Msg2D, class ExportObj>
std::optional<std::unordered_map<std::string, std::string>>
FusionNode<Msg3D, Msg2D, ExportObj>::find_concatenation_status(double timestamp)
{
  auto it = concatenated_status_map_.find(timestamp);
  if (it != concatenated_status_map_.end()) {
    return it->second;
  }
  return std::nullopt;
}

template <class Msg3D, class Msg2D, class ExportObj>
std::string FusionNode<Msg3D, Msg2D, ExportObj>::format_timestamp(double timestamp)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << timestamp;
  return oss.str();
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::show_diagnostic_message(
  std::unordered_map<std::size_t, double> id_to_stamp_map,
  std::shared_ptr<FusionCollectorInfoBase> collector_info)
{
  msg3d_fused_ = false;
  diagnostic_collector_info_ = std::move(collector_info);
  diagnostic_id_to_stamp_map_ = std::move(id_to_stamp_map);
  diagnostic_updater_.force_update();
}

template <class Msg3D, class Msg2D, class ExportObj>
void FusionNode<Msg3D, Msg2D, ExportObj>::check_fusion_status(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if (publish_output_msg_ || drop_previous_but_late_output_msg_ || !msg3d_fused_) {
    stat.add("msg3d/is_fused", msg3d_fused_);

    if (msg3d_fused_) {
      stat.add("fused_timestamp", format_timestamp(current_output_msg_timestamp_));
    }

    if (
      auto naive_info = std::dynamic_pointer_cast<NaiveCollectorInfo>(diagnostic_collector_info_)) {
      stat.add("first_input_arrival_timestamp", format_timestamp(naive_info->timestamp));
    } else if (
      auto advanced_info =
        std::dynamic_pointer_cast<AdvancedCollectorInfo>(diagnostic_collector_info_)) {
      stat.add(
        "reference_timestamp_min",
        format_timestamp(advanced_info->timestamp - advanced_info->noise_window));
      stat.add(
        "reference_timestamp_max",
        format_timestamp(advanced_info->timestamp + advanced_info->noise_window));
    }

    bool rois_miss = false;
    bool fusion_success = msg3d_fused_;

    for (std::size_t id = 0; id < rois_number_; ++id) {
      std::string rois_prefix = "rois" + std::to_string(id);
      bool input_rois_fused = true;

      auto it = diagnostic_id_to_stamp_map_.find(id);
      if (it != diagnostic_id_to_stamp_map_.end()) {
        stat.add(rois_prefix + "/timestamp", format_timestamp(it->second));
      } else {
        rois_miss = true;
        fusion_success = false;
        input_rois_fused = false;
      }
      stat.add(rois_prefix + "/is_fused", input_rois_fused);
    }

    stat.add("fusion_success", fusion_success);

    int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string message = "Fused output is published and include all rois and msg3d";

    if (drop_previous_but_late_output_msg_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message =
        rois_miss
          ? "Fused output msg misses some ROIs and is not published because it arrived too late"
          : "Fused output msg is not published as it is too late";
    } else if (rois_miss) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Fused output msg is published but misses some ROIs";
    } else if (!msg3d_fused_) {
      level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      message = "Fused output msg is not published as msg3d is missed";
    }

    stat.summary(level, message);

    // Reset status flags
    publish_output_msg_ = false;
    drop_previous_but_late_output_msg_ = false;
    msg3d_fused_ = true;
  } else {
    stat.summary(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "Fusion node launched successfully, but waiting for input pointcloud");
  }
}

// Explicit instantiation for the supported types

// pointpainting fusion
template class FusionNode<PointCloudMsgType, RoiMsgType, DetectedObjects>;

// roi cluster fusion
template class FusionNode<ClusterMsgType, RoiMsgType, ClusterMsgType>;

// roi detected-object fusion
template class FusionNode<DetectedObjects, RoiMsgType, DetectedObjects>;

// roi pointcloud fusion
template class FusionNode<PointCloudMsgType, RoiMsgType, ClusterMsgType>;

// segment pointcloud fusion
template class FusionNode<PointCloudMsgType, Image, PointCloudMsgType>;

}  // namespace autoware::image_projection_based_fusion
