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

#ifndef DEBUGGER__DEBUG_OBJECT_HPP_
#define DEBUGGER__DEBUG_OBJECT_HPP_

#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"

#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/functional/hash.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace autoware::multi_object_tracker
{

struct ObjectData
{
  rclcpp::Time time;

  // object uuid
  boost::uuids::uuid uuid;
  std::string uuid_str;

  // association link, pair of coordinates
  // tracker to detection
  geometry_msgs::msg::Point tracker_point;
  geometry_msgs::msg::Point detection_point;
  bool is_associated{false};

  // existence probabilities
  std::vector<float> existence_vector;
  float total_existence_probability;

  // detection channel id
  uint channel_id;
};

class TrackerObjectDebugger
{
public:
  TrackerObjectDebugger(
    const std::string & frame_id, const std::vector<types::InputChannel> & channels_config);

private:
  bool is_initialized_{false};
  std::string frame_id_;
  const std::vector<types::InputChannel> channels_config_;

  visualization_msgs::msg::MarkerArray markers_;
  rclcpp::Time message_time_;

  std::vector<ObjectData> object_data_list_;
  std::list<int32_t> unused_marker_ids_;
  std::vector<std::vector<ObjectData>> object_data_groups_;

public:
  void collect(
    const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
    const types::DynamicObjectList & detected_objects,
    const std::unordered_map<int, int> & direct_assignment,
    const std::unordered_map<int, int> & reverse_assignment);

  void reset();
  void draw(
    const std::vector<std::vector<ObjectData>> & object_data_groups,
    visualization_msgs::msg::MarkerArray & marker_array) const;
  void process();
  void getMessage(visualization_msgs::msg::MarkerArray & marker_array) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // DEBUGGER__DEBUG_OBJECT_HPP_
