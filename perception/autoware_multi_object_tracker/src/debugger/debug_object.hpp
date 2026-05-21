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

#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_hash.hpp>

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
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
  std::vector<types::ExistenceProbability> existence_vector;
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

  rclcpp::Time message_time_;

  std::unordered_map<boost::uuids::uuid, std::vector<ObjectData>, boost::hash<boost::uuids::uuid>>
    object_data_map_;
  std::vector<std::vector<ObjectData>> object_data_groups_;

public:
  void collect(
    const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
    const types::AssociatedObjects & associated_objects);

  void reset();
  void draw(
    const std::vector<std::vector<ObjectData>> & object_data_groups,
    visualization_msgs::msg::MarkerArray & marker_array) const;
  void process();
  void getMessage(visualization_msgs::msg::MarkerArray & marker_array) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // DEBUGGER__DEBUG_OBJECT_HPP_
