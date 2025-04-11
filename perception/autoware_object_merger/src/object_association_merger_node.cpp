// Copyright 2020 Tier IV, Inc.
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

#include <type_traits>
#define EIGEN_MPL2_ONLY

#include "autoware/object_merger/object_association_merger_node.hpp"
#include "autoware/object_recognition_utils/object_recognition_utils.hpp"
#include "autoware_utils/geometry/geometry.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/optional.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using Label = autoware_perception_msgs::msg::ObjectClassification;

namespace
{
bool isUnknownObjectOverlapped(
  const autoware_perception_msgs::msg::DetectedObject & unknown_object,
  const autoware_perception_msgs::msg::DetectedObject & known_object,
  const double precision_threshold, const double recall_threshold,
  const std::map<int, double> & distance_threshold_map,
  const std::map<int, double> & generalized_iou_threshold_map)
{
  const double generalized_iou_threshold = generalized_iou_threshold_map.at(
    autoware::object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double distance_threshold = distance_threshold_map.at(
    autoware::object_recognition_utils::getHighestProbLabel(known_object.classification));
  const double sq_distance_threshold = std::pow(distance_threshold, 2.0);
  const double sq_distance = autoware_utils::calc_squared_distance2d(
    unknown_object.kinematics.pose_with_covariance.pose,
    known_object.kinematics.pose_with_covariance.pose);
  if (sq_distance_threshold < sq_distance) return false;
  const auto precision =
    autoware::object_recognition_utils::get2dPrecision(unknown_object, known_object);
  const auto recall = autoware::object_recognition_utils::get2dRecall(unknown_object, known_object);
  const auto generalized_iou =
    autoware::object_recognition_utils::get2dGeneralizedIoU(unknown_object, known_object);
  return precision > precision_threshold || recall > recall_threshold ||
         generalized_iou > generalized_iou_threshold;
}
}  // namespace

namespace
{
std::map<int, double> convertListToClassMap(const std::vector<double> & distance_threshold_list)
{
  std::map<int /*class label*/, double /*distance_threshold*/> distance_threshold_map;
  int class_label = 0;
  for (const auto & distance_threshold : distance_threshold_list) {
    distance_threshold_map.insert(std::make_pair(class_label, distance_threshold));
    class_label++;
  }
  return distance_threshold_map;
}
}  // namespace

namespace autoware::object_merger
{
ObjectAssociationMergerNode::ObjectAssociationMergerNode(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("object_association_merger_node", node_options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  object0_sub_(this, "input/object0", rclcpp::QoS{1}.get_rmw_qos_profile()),
  object1_sub_(this, "input/object1", rclcpp::QoS{1}.get_rmw_qos_profile())
{
  // Parameters
  base_link_frame_id_ = declare_parameter<std::string>("base_link_frame_id");
  priority_mode_ = static_cast<PriorityMode>(declare_parameter<int>("priority_mode"));
  sync_queue_size_ = declare_parameter<int>("sync_queue_size");
  remove_overlapped_unknown_objects_ = declare_parameter<bool>("remove_overlapped_unknown_objects");
  overlapped_judge_param_.precision_threshold =
    declare_parameter<double>("precision_threshold_to_judge_overlapped");
  overlapped_judge_param_.recall_threshold =
    declare_parameter<double>("recall_threshold_to_judge_overlapped");
  overlapped_judge_param_.generalized_iou_threshold =
    convertListToClassMap(declare_parameter<std::vector<double>>("generalized_iou_threshold"));

  // get distance_threshold_map from distance_threshold_list
  /** TODO(Shin-kyoto):
   *  this implementation assumes index of vector shows class_label.
   *  if param supports map, refactor this code.
   */
  overlapped_judge_param_.distance_threshold_map =
    convertListToClassMap(declare_parameter<std::vector<double>>("distance_threshold_list"));

  const auto tmp = this->declare_parameter<std::vector<int64_t>>("can_assign_matrix");
  const std::vector<int> can_assign_matrix(tmp.begin(), tmp.end());
  const auto max_dist_matrix = this->declare_parameter<std::vector<double>>("max_dist_matrix");
  const auto max_rad_matrix = this->declare_parameter<std::vector<double>>("max_rad_matrix");
  const auto min_iou_matrix = this->declare_parameter<std::vector<double>>("min_iou_matrix");
  data_association_ = std::make_unique<autoware::object_merger::DataAssociation>(
    can_assign_matrix, max_dist_matrix, max_rad_matrix, min_iou_matrix);

  // Create publishers and subscribers
  using std::placeholders::_1;
  using std::placeholders::_2;
  sync_ptr_ = std::make_shared<Sync>(SyncPolicy(sync_queue_size_), object0_sub_, object1_sub_);
  sync_ptr_->registerCallback(
    std::bind(&ObjectAssociationMergerNode::objectsCallback, this, _1, _2));

  merged_object_pub_ = create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "output/object", rclcpp::QoS{1});

  // Debug publisher
  processing_time_publisher_ =
    std::make_unique<autoware_utils::DebugPublisher>(this, "object_association_merger");
  stop_watch_ptr_ = std::make_unique<autoware_utils::StopWatch<std::chrono::milliseconds>>();
  stop_watch_ptr_->tic("cyclic_time");
  stop_watch_ptr_->tic("processing_time");
  published_time_publisher_ = std::make_unique<autoware_utils::PublishedTimePublisher>(this);
  // Timeout process initialization
  message_timeout_sec_ = this->declare_parameter<double>("message_timeout_sec");
  initialization_timeout_sec_ = this->declare_parameter<double>("initialization_timeout_sec");
  last_sync_time_ = this->now();
  node_start_time_ = this->now();
  received_first_message_ = false;
  message_interval_ = 0.0;
  time_source_initialized_ = false;
  timeout_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(message_timeout_sec_ / 2),
    std::bind(&ObjectAssociationMergerNode::timeoutCallback, this));
  diagnostics_interface_ptr_ =
    std::make_unique<autoware_utils::DiagnosticsInterface>(this, "object_association_merger");
}

void ObjectAssociationMergerNode::objectsCallback(
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects0_msg,
  const autoware_perception_msgs::msg::DetectedObjects::ConstSharedPtr & input_objects1_msg)
{
  // Guard
  if (merged_object_pub_->get_subscription_count() < 1) {
    return;
  }
  stop_watch_ptr_->toc("processing_time", true);
  rclcpp::Time now = this->now();
  // If this is the first sync (timestamp is zero), set interval to 0.0
  // Otherwise, compute the time difference from the last sync
  message_interval_ =
    (last_sync_time_.nanoseconds() == 0) ? 0.0 : (now - last_sync_time_).seconds();
  // Update the last sync time to now
  last_sync_time_ = now;
  checkStatus(
    message_interval_, message_timeout_sec_, "No recent messages received or synchronized",
    input_objects0_msg->header.stamp);
  received_first_message_ = true;

  /* transform to base_link coordinate */
  autoware_perception_msgs::msg::DetectedObjects transformed_objects0, transformed_objects1;
  if (
    !autoware::object_recognition_utils::transformObjects(
      *input_objects0_msg, base_link_frame_id_, tf_buffer_, transformed_objects0) ||
    !autoware::object_recognition_utils::transformObjects(
      *input_objects1_msg, base_link_frame_id_, tf_buffer_, transformed_objects1)) {
    return;
  }

  // build output msg
  autoware_perception_msgs::msg::DetectedObjects output_msg;
  output_msg.header = input_objects0_msg->header;
  output_msg.header.frame_id = base_link_frame_id_;

  /* global nearest neighbor */
  std::unordered_map<int, int> direct_assignment, reverse_assignment;
  const auto & objects0 = transformed_objects0.objects;
  const auto & objects1 = transformed_objects1.objects;
  Eigen::MatrixXd score_matrix =
    data_association_->calcScoreMatrix(transformed_objects1, transformed_objects0);
  data_association_->assign(score_matrix, direct_assignment, reverse_assignment);

  for (size_t object0_idx = 0; object0_idx < objects0.size(); ++object0_idx) {
    const auto & object0 = objects0.at(object0_idx);
    if (direct_assignment.find(object0_idx) != direct_assignment.end()) {  // found and merge
      const auto & object1 = objects1.at(direct_assignment.at(object0_idx));
      switch (priority_mode_) {
        case PriorityMode::Object0:
          output_msg.objects.push_back(object0);
          break;
        case PriorityMode::Object1:
          output_msg.objects.push_back(object1);
          break;
        case PriorityMode::Confidence:
          if (object1.existence_probability <= object0.existence_probability)
            output_msg.objects.push_back(object0);
          else
            output_msg.objects.push_back(object1);
          break;
      }
    } else {  // not found
      output_msg.objects.push_back(object0);
    }
  }
  for (size_t object1_idx = 0; object1_idx < objects1.size(); ++object1_idx) {
    const auto & object1 = objects1.at(object1_idx);
    if (reverse_assignment.find(object1_idx) != reverse_assignment.end()) {  // found
    } else {                                                                 // not found
      output_msg.objects.push_back(object1);
    }
  }

  // Remove overlapped unknown object
  if (remove_overlapped_unknown_objects_) {
    std::vector<autoware_perception_msgs::msg::DetectedObject> unknown_objects, known_objects;
    unknown_objects.reserve(output_msg.objects.size());
    known_objects.reserve(output_msg.objects.size());
    for (const auto & object : output_msg.objects) {
      if (
        autoware::object_recognition_utils::getHighestProbLabel(object.classification) ==
        Label::UNKNOWN) {
        unknown_objects.push_back(object);
      } else {
        known_objects.push_back(object);
      }
    }
    output_msg.objects.clear();
    output_msg.objects = known_objects;
    for (const auto & unknown_object : unknown_objects) {
      bool is_overlapped = false;
      for (const auto & known_object : known_objects) {
        if (isUnknownObjectOverlapped(
              unknown_object, known_object, overlapped_judge_param_.precision_threshold,
              overlapped_judge_param_.recall_threshold,
              overlapped_judge_param_.distance_threshold_map,
              overlapped_judge_param_.generalized_iou_threshold)) {
          is_overlapped = true;
          break;
        }
      }
      if (!is_overlapped) {
        output_msg.objects.push_back(unknown_object);
      }
    }
  }

  // publish output msg
  merged_object_pub_->publish(output_msg);
  published_time_publisher_->publish_if_subscribed(merged_object_pub_, output_msg.header.stamp);
  // publish processing time
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/cyclic_time_ms", stop_watch_ptr_->toc("cyclic_time", true));
  processing_time_publisher_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "debug/processing_time_ms", stop_watch_ptr_->toc("processing_time", true));
}

void ObjectAssociationMergerNode::checkStatus(
  double elapsed_time, double timeout, const std::string & message_prefix,
  const rclcpp::Time & publish_time_stamp)
{
  const bool elapsed_timeout = (elapsed_time >= timeout);
  const bool interval_timeout = (message_interval_ >= message_timeout_sec_);
  const bool timeout_occurred = elapsed_timeout || interval_timeout;
  diagnostics_interface_ptr_->clear();
  diagnostics_interface_ptr_->add_key_value("timeout_occurred", timeout_occurred);
  diagnostics_interface_ptr_->add_key_value("elapsed_time_since_sync_or_startup", elapsed_time);
  diagnostics_interface_ptr_->add_key_value("messages_interval", message_interval_);
  std::string message;
  if (elapsed_timeout) {
    message = "[WARN] " + message_prefix + " - Elapsed time " + std::to_string(elapsed_time) +
              "s exceeded timeout threshold of " + std::to_string(timeout) + "s.";
  } else if (interval_timeout) {
    message = "[WARN] " + message_prefix + " - Message interval " +
              std::to_string(message_interval_) + "s exceeded allowed interval of " +
              std::to_string(message_timeout_sec_) + "s.";
  } else {
    message = "[OK] Status is normal.";
  }
  diagnostics_interface_ptr_->update_level_and_message(
    timeout_occurred ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                     : diagnostic_msgs::msg::DiagnosticStatus::OK,
    message);
  diagnostics_interface_ptr_->publish(publish_time_stamp);
}

void ObjectAssociationMergerNode::timeoutCallback()
{
  rclcpp::Time now = this->now();
  // If the time source is not initialized, return early
  if (now.nanoseconds() == 0) {
    return;
  }
  // Initialize the time source if it hasn't been initialized yet
  if (!time_source_initialized_) {
    node_start_time_ = now;
    time_source_initialized_ = true;
    return;
  }

  // Handle case: no messages received since startup
  if (!received_first_message_) {
    // If no message has been received within the initialization timeout, publish a warning
    const double time_since_startup = (now - node_start_time_).seconds();
    checkStatus(
      time_since_startup, initialization_timeout_sec_,
      "No synchronized messages received since startup", now);
    return;
  }

  // Handle case: timeout due to lack of recent sync
  const double time_since_last_sync = (now - last_sync_time_).seconds();
  checkStatus(
    time_since_last_sync, message_timeout_sec_, "No recent messages received or synchronized", now);
}

}  // namespace autoware::object_merger

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::object_merger::ObjectAssociationMergerNode)
