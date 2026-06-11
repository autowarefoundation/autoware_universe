// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__OBJECT_MERGER__OBJECT_FUSION_MERGER_NODE_HPP_
#define AUTOWARE__OBJECT_MERGER__OBJECT_FUSION_MERGER_NODE_HPP_

#include "autoware_utils/system/stop_watch.hpp"

#include <autoware/agnocast_wrapper/message_filters.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_object.hpp"
#include "autoware_perception_msgs/msg/detected_objects.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::object_merger
{
/**
 * @brief Merge two detected-object streams by expanding each main object with uniquely overlapped
 * sub objects.
 */
class ObjectFusionMergerNode : public autoware::agnocast_wrapper::Node
{
public:
  /**
   * @brief Construct the object fusion merger node.
   *
   * @param node_options ROS node options used for component construction.
   */
  explicit ObjectFusionMergerNode(const rclcpp::NodeOptions & node_options);

private:
  using DetectedObject = autoware_perception_msgs::msg::DetectedObject;
  using DetectedObjects = autoware_perception_msgs::msg::DetectedObjects;
  using SyncPolicy = autoware::agnocast_wrapper::message_filters::sync_policies::ApproximateTime<
    DetectedObjects, DetectedObjects>;
  using Sync = autoware::agnocast_wrapper::message_filters::Synchronizer<SyncPolicy>;

  /**
   * @brief Transform synchronized inputs, perform fusion, and publish both outputs.
   *
   * @param main_objects_msg Main detected objects input.
   * @param sub_objects_msg Sub detected objects input.
   */
  void callback(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DetectedObjects) & main_objects_msg,
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(DetectedObjects) & sub_objects_msg);

  /**
   * @brief Group sub objects by unique overlap and produce fused and unmatched outputs.
   *
   * @param main_objects_msg Main detected objects transformed into the base frame.
   * @param sub_objects_msg Sub detected objects transformed into the base frame.
   * @param fused_objects Output message populated with the main-based fused objects.
   * @param other_objects Output message populated with the unmatched sub objects.
   */
  void fuse_objects(
    const DetectedObjects & main_objects_msg, const DetectedObjects & sub_objects_msg,
    DetectedObjects & fused_objects, DetectedObjects & other_objects);

  autoware::agnocast_wrapper::Buffer tf_buffer_;
  autoware::agnocast_wrapper::TransformListener tf_listener_;
  AUTOWARE_PUBLISHER_PTR(DetectedObjects) fused_objects_pub_;
  AUTOWARE_PUBLISHER_PTR(DetectedObjects) other_objects_pub_;
  autoware::agnocast_wrapper::message_filters::Subscriber<DetectedObjects> main_object_sub_{};
  autoware::agnocast_wrapper::message_filters::Subscriber<DetectedObjects> sub_object_sub_{};
  std::shared_ptr<Sync> sync_ptr_;

  std::string base_link_frame_id_;
  bool keep_input_dimensions_;

  std::unique_ptr<autoware_utils_debug::BasicDebugPublisher<autoware::agnocast_wrapper::Node>>
    processing_time_publisher_;
  std::unique_ptr<autoware_utils::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<
    autoware_utils_debug::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>
    published_time_publisher_;
};
}  // namespace autoware::object_merger

#endif  // AUTOWARE__OBJECT_MERGER__OBJECT_FUSION_MERGER_NODE_HPP_
