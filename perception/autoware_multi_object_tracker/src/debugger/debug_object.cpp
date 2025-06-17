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

#include "debug_object.hpp"

#include "autoware_perception_msgs/msg/tracked_object.hpp"

#include <boost/uuid/uuid.hpp>

#include <functional>
#include <iomanip>
#include <list>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace
{
std::string uuidToString(const unique_identifier_msgs::msg::UUID & uuid_msg)
{
  std::stringstream ss;
  for (auto i = 0; i < 16; ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << +uuid_msg.uuid[i];
  }
  return ss.str();
}

boost::uuids::uuid uuidToBoostUuid(const unique_identifier_msgs::msg::UUID & uuid_msg)
{
  const std::string uuid_str = uuidToString(uuid_msg);
  boost::uuids::string_generator gen;
  boost::uuids::uuid uuid = gen(uuid_str);
  return uuid;
}

int32_t uuidToInt(const boost::uuids::uuid & uuid)
{
  return boost::uuids::hash_value(uuid);
}
}  // namespace

namespace autoware::multi_object_tracker
{

TrackerObjectDebugger::TrackerObjectDebugger(
  const std::string & frame_id, const std::vector<types::InputChannel> & channels_config)
: frame_id_(frame_id), channels_config_(channels_config)
{
  // initialize markers
  markers_.markers.clear();
  message_time_ = rclcpp::Time(0, 0);
}

void TrackerObjectDebugger::reset()
{
  // clear markers, object data list
  object_data_list_.clear();
  markers_.markers.clear();
}

void TrackerObjectDebugger::collect(
  const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
  const types::DynamicObjectList & detected_objects,
  const std::unordered_map<int, int> & direct_assignment,
  const std::unordered_map<int, int> & /*reverse_assignment*/)
{
  is_initialized_ = true;

  message_time_ = message_time;

  int tracker_idx = 0;
  for (auto tracker_itr = list_tracker.begin(); tracker_itr != list_tracker.end();
       ++tracker_itr, ++tracker_idx) {
    ObjectData object_data;
    object_data.time = message_time;

    types::DynamicObject tracked_object;
    (*(tracker_itr))->getTrackedObject(message_time, tracked_object);
    object_data.uuid = uuidToBoostUuid(tracked_object.uuid);
    object_data.uuid_str = (*(tracker_itr))->getUuidString();

    // tracker
    bool is_associated = false;
    geometry_msgs::msg::Point tracker_point, detection_point;
    tracker_point.x = tracked_object.pose.position.x;
    tracker_point.y = tracked_object.pose.position.y;
    tracker_point.z = tracked_object.pose.position.z;

    // associated detection
    if (direct_assignment.find(tracker_idx) != direct_assignment.end()) {
      const auto & associated_object =
        detected_objects.objects.at(direct_assignment.find(tracker_idx)->second);
      detection_point.x = associated_object.pose.position.x;
      detection_point.y = associated_object.pose.position.y;
      detection_point.z = associated_object.pose.position.z;
      is_associated = true;
    } else {
      // no detection
      detection_point.x = tracker_point.x;
      detection_point.y = tracker_point.y;
      detection_point.z = tracker_point.z;
    }
    object_data.channel_id = detected_objects.channel_index;
    object_data.tracker_point = tracker_point;
    object_data.detection_point = detection_point;
    object_data.is_associated = is_associated;

    // existence probabilities
    object_data.existence_vector = (*tracker_itr)->getExistenceProbabilityVector();
    object_data.total_existence_probability = (*tracker_itr)->getTotalExistenceProbability();

    object_data_list_.push_back(object_data);
  }
}

void TrackerObjectDebugger::process()
{
  if (!is_initialized_) return;

  // Check if object_data_list_ is empty
  if (object_data_list_.empty()) return;

  // sort by uuid, collect the same uuid object_data as a group, and loop for the groups
  object_data_groups_.clear();
  {
    // sort by uuid
    std::sort(
      object_data_list_.begin(), object_data_list_.end(),
      [](const ObjectData & a, const ObjectData & b) { return a.uuid < b.uuid; });

    // collect the same uuid object_data as a group
    std::vector<ObjectData> object_data_group{};
    boost::uuids::uuid previous_uuid = object_data_list_.front().uuid;
    for (const auto & object_data : object_data_list_) {
      // if the uuid is different, push the group and clear the group
      if (object_data.uuid != previous_uuid) {
        // push the group
        object_data_groups_.push_back(object_data_group);
        object_data_group.clear();
        previous_uuid = object_data.uuid;
      }
      // push the object_data to the group
      object_data_group.push_back(object_data);
    }
    // push the last group
    object_data_groups_.push_back(object_data_group);
  }
}

void TrackerObjectDebugger::draw(
  const std::vector<std::vector<ObjectData>> & object_data_groups,
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  // initialize markers
  marker_array.markers.clear();

  constexpr int PALETTE_SIZE = 16;
  constexpr std::array<std::array<double, 3>, PALETTE_SIZE> color_array = {{
    {{0.0, 0.0, 1.0}},     // Blue
    {{0.0, 1.0, 0.0}},     // Green
    {{1.0, 1.0, 0.0}},     // Yellow
    {{1.0, 0.0, 0.0}},     // Red
    {{0.0, 1.0, 1.0}},     // Cyan
    {{1.0, 0.0, 1.0}},     // Magenta
    {{1.0, 0.64, 0.0}},    // Orange
    {{0.75, 1.0, 0.0}},    // Lime
    {{0.0, 0.5, 0.5}},     // Teal
    {{0.5, 0.0, 0.5}},     // Purple
    {{1.0, 0.75, 0.8}},    // Pink
    {{0.65, 0.17, 0.17}},  // Brown
    {{0.5, 0.0, 0.0}},     // Maroon
    {{0.5, 0.5, 0.0}},     // Olive
    {{0.0, 0.0, 0.5}},     // Navy
    {{0.5, 0.5, 0.5}}      // Grey
  }};

  for (const auto & object_data_group : object_data_groups) {
    if (object_data_group.empty()) continue;
    const auto object_data_front = object_data_group.front();
    const auto object_data_back = object_data_group.back();

    // set a reference marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = object_data_front.time;
    marker.id = uuidToInt(object_data_front.uuid);
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;  // white
    marker.lifetime = rclcpp::Duration::from_seconds(0.15);

    // get marker - existence_probability
    visualization_msgs::msg::Marker text_marker;
    text_marker = marker;
    text_marker.ns = "existence_probability";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.z += 1.8;
    text_marker.scale.z = 0.5;
    text_marker.pose.position.x = object_data_front.tracker_point.x;
    text_marker.pose.position.y = object_data_front.tracker_point.y;
    text_marker.pose.position.z = object_data_front.tracker_point.z + 2.5;

    // show the last existence probability
    // print existence probability with channel name
    // probability to text, two digits of percentage
    std::string existence_probability_text = "";

    // total probability
    existence_probability_text += "total:";
    existence_probability_text +=
      std::to_string(static_cast<int>(object_data_front.total_existence_probability * 100)) + "\n";

    // probability per channel
    const size_t channel_size = channels_config_.size();
    for (size_t i = 0; i < channel_size; ++i) {
      if (object_data_front.existence_vector[i] < 0.00101) continue;
      existence_probability_text +=
        channels_config_[i].short_name +
        std::to_string(static_cast<int>(object_data_front.existence_vector[i] * 100)) + ":";
    }
    if (!existence_probability_text.empty()) {
      existence_probability_text.pop_back();
    }
    existence_probability_text += "\n" + object_data_front.uuid_str.substr(0, 6);

    text_marker.text = existence_probability_text;

    // loop for each object_data in the group
    // boxed to tracker positions
    // and link lines to the detected positions
    const double marker_height_offset = 1.0;
    const double assign_height_offset = 0.6;

    visualization_msgs::msg::Marker marker_track_boxes;
    marker_track_boxes = marker;
    marker_track_boxes.ns = "track_boxes";
    marker_track_boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_track_boxes.action = visualization_msgs::msg::Marker::ADD;
    marker_track_boxes.scale.x = 0.4;
    marker_track_boxes.scale.y = 0.4;
    marker_track_boxes.scale.z = 0.4;
    marker_track_boxes.color.a = 0.9;
    marker_track_boxes.color.r = 1.0;
    marker_track_boxes.color.g = 1.0;
    marker_track_boxes.color.b = 1.0;

    // make detected object markers per channel
    std::vector<visualization_msgs::msg::Marker> marker_detect_boxes_per_channel;
    std::vector<visualization_msgs::msg::Marker> marker_detect_lines_per_channel;

    for (size_t idx = 0; idx < channels_config_.size(); idx++) {
      // get color - by channel index
      std_msgs::msg::ColorRGBA color;
      color.a = 0.9;
      color.r = color_array[idx % PALETTE_SIZE][0];
      color.g = color_array[idx % PALETTE_SIZE][1];
      color.b = color_array[idx % PALETTE_SIZE][2];

      visualization_msgs::msg::Marker marker_detect_boxes;
      marker_detect_boxes = marker;
      marker_detect_boxes.ns = "detect_boxes_" + channels_config_[idx].short_name;
      marker_detect_boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker_detect_boxes.action = visualization_msgs::msg::Marker::ADD;
      marker_detect_boxes.scale.x = 0.2;
      marker_detect_boxes.scale.y = 0.2;
      marker_detect_boxes.scale.z = 0.2;
      marker_detect_boxes.color = color;
      marker_detect_boxes_per_channel.push_back(marker_detect_boxes);

      visualization_msgs::msg::Marker marker_lines;
      marker_lines = marker;
      marker_lines.ns = "association_lines_" + channels_config_[idx].short_name;
      marker_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker_lines.action = visualization_msgs::msg::Marker::ADD;
      marker_lines.scale.x = 0.15;
      marker_lines.points.clear();
      marker_lines.color = color;
      marker_detect_lines_per_channel.push_back(marker_lines);
    }

    bool is_associated = false;
    for (const auto & object_data : object_data_group) {
      int channel_id = object_data.channel_id;

      // set box
      geometry_msgs::msg::Point box_point;
      box_point.x = object_data.tracker_point.x;
      box_point.y = object_data.tracker_point.y;
      box_point.z = object_data.tracker_point.z + marker_height_offset;
      marker_track_boxes.points.push_back(box_point);

      // set association marker, if exists
      if (!object_data.is_associated) continue;
      is_associated = true;

      // associated object box
      visualization_msgs::msg::Marker & marker_detect_boxes =
        marker_detect_boxes_per_channel.at(channel_id);
      box_point.x = object_data.detection_point.x;
      box_point.y = object_data.detection_point.y;
      box_point.z = object_data.detection_point.z + marker_height_offset + assign_height_offset;
      marker_detect_boxes.points.push_back(box_point);

      // association line
      visualization_msgs::msg::Marker & marker_lines =
        marker_detect_lines_per_channel.at(channel_id);
      geometry_msgs::msg::Point line_point;
      line_point.x = object_data.tracker_point.x;
      line_point.y = object_data.tracker_point.y;
      line_point.z = object_data.tracker_point.z + marker_height_offset;
      marker_lines.points.push_back(line_point);
      line_point.x = object_data.detection_point.x;
      line_point.y = object_data.detection_point.y;
      line_point.z = object_data.detection_point.z + marker_height_offset + assign_height_offset;
      marker_lines.points.push_back(line_point);
    }

    // add markers
    for (size_t i = 0; i < channels_config_.size(); i++) {
      if (marker_detect_boxes_per_channel.at(i).points.empty()) {
        marker_detect_boxes_per_channel.at(i).action = visualization_msgs::msg::Marker::DELETE;
      }
      marker_array.markers.push_back(marker_detect_boxes_per_channel.at(i));
    }
    for (size_t i = 0; i < channels_config_.size(); i++) {
      if (marker_detect_lines_per_channel.at(i).points.empty()) {
        marker_detect_lines_per_channel.at(i).action = visualization_msgs::msg::Marker::DELETE;
      }
      marker_array.markers.push_back(marker_detect_lines_per_channel.at(i));
    }

    // if not associated, gray out the track box and text
    if (!is_associated) {
      marker_track_boxes.color.r = 0.5;
      marker_track_boxes.color.g = 0.5;
      marker_track_boxes.color.b = 0.5;
      marker_track_boxes.color.a = 0.8;
      text_marker.color.r = 0.5;
      text_marker.color.g = 0.5;
      text_marker.color.b = 0.5;
      text_marker.color.a = 0.9;
    }
    marker_array.markers.push_back(text_marker);
    marker_array.markers.push_back(marker_track_boxes);
  }

  return;
}

void TrackerObjectDebugger::getMessage(visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (!is_initialized_) return;

  // draw markers
  draw(object_data_groups_, marker_array);

  return;
}

}  // namespace autoware::multi_object_tracker
