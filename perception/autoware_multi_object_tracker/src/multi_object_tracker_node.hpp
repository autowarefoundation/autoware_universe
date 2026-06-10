// Copyright 2020 TIER IV, Inc.
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

#ifndef MULTI_OBJECT_TRACKER_NODE_HPP_
#define MULTI_OBJECT_TRACKER_NODE_HPP_

#include "debugger/debugger.hpp"
#include "multi_object_tracker_core.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"

#include <memory>
#include <vector>

namespace autoware::multi_object_tracker
{

class MultiObjectTracker : public autoware::agnocast_wrapper::Node
{
public:
  explicit MultiObjectTracker(const rclcpp::NodeOptions & node_options);

private:
  // ROS interface
  std::vector<AUTOWARE_SUBSCRIPTION_PTR(autoware_perception_msgs::msg::DetectedObjects)>
    sub_objects_array_{};

  AUTOWARE_PUBLISHER_PTR(autoware_perception_msgs::msg::TrackedObjects) tracked_objects_pub_;
  AUTOWARE_PUBLISHER_PTR(autoware_perception_msgs::msg::DetectedObjects) merged_objects_pub_;

  AUTOWARE_PUBLISHER_PTR(autoware_utils_debug::ProcessingTimeDetail)
  detailed_processing_time_publisher_;

  // publish timer
  AUTOWARE_TIMER_PTR publish_timer_;

  // parameters and internal state
  MultiObjectTrackerParameters params_;
  MultiObjectTrackerInternalState state_;

  // debugger
  std::unique_ptr<TrackerDebugger> debugger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<
    autoware_utils_debug::BasicPublishedTimePublisher<autoware::agnocast_wrapper::Node>>
    published_time_publisher_;

  // callback functions
  void onTimer();
  void processObjects();
  void onMeasurement(
    const size_t channel_index,
    AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_perception_msgs::msg::DetectedObjects) msg);

  // publish processes
  void publish();
  void publishOptional(const rclcpp::Time & object_time, const size_t tracked_objects_size);
};

}  // namespace autoware::multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_NODE_HPP_
