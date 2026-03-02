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
//
//
// Author: v1.0 Yukihiro Saito
///

#ifndef MULTI_OBJECT_TRACKER_NODE_HPP_
#define MULTI_OBJECT_TRACKER_NODE_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/odometry.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "debugger/debugger.hpp"
#include "multi_object_tracker_core.hpp"
#include "processor/input_manager.hpp"
#include "processor/processor.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/convert.hpp>
#include <tf2/transform_datatypes.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include "autoware_perception_msgs/msg/tracked_objects.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

class MultiObjectTracker : public rclcpp::Node
{
public:
  explicit MultiObjectTracker(const rclcpp::NodeOptions & node_options);

private:
  // ROS interface
  std::vector<rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr>
    sub_objects_array_{};

  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr tracked_objects_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr merged_objects_pub_;

  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;

  // publish timer
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // parameters and internal state
  MultiObjectTrackerParameters params_;
  MultiObjectTrackerInternalState state_;

  // debugger
  std::unique_ptr<TrackerDebugger> debugger_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  // callback functions
  void onTimer();
  void onTrigger();

  // publish processes
  void publish(const rclcpp::Time & time);
  void publishOptional(
    const rclcpp::Time & publish_time, const rclcpp::Time & current_time,
    const size_t tracked_objects_size);
};

}  // namespace autoware::multi_object_tracker

#endif  // MULTI_OBJECT_TRACKER_NODE_HPP_
