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

#include <boost/uuid/uuid.hpp>

#include <algorithm>
#include <list>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{

boost::uuids::uuid uuidToBoostUuid(const unique_identifier_msgs::msg::UUID & uuid_msg)
{
  boost::uuids::uuid uuid;
  std::copy(uuid_msg.uuid.begin(), uuid_msg.uuid.end(), uuid.begin());
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
  message_time_ = rclcpp::Time(0, 0);
}

void TrackerObjectDebugger::reset()
{
  object_data_map_.clear();
}

void TrackerObjectDebugger::collect(
  const rclcpp::Time & message_time, const std::list<std::shared_ptr<Tracker>> & list_tracker,
  const types::AssociatedObjects & associated_objects)
{
  is_initialized_ = true;

  const auto & detected_objects = associated_objects.objects;
  const auto & association_result = associated_objects.association;

  message_time_ = message_time;

  for (auto tracker_itr = list_tracker.begin(); tracker_itr != list_tracker.end(); ++tracker_itr) {
    ObjectData object_data;
    object_data.time = message_time;

    types::DynamicObject tracked_object;
    (*tracker_itr)->getTrackedObject(message_time, tracked_object);
    object_data.uuid = uuidToBoostUuid(tracked_object.uuid);
    object_data.uuid_str = (*tracker_itr)->getUuidString();

    geometry_msgs::msg::Point tracker_point, detection_point;
    tracker_point.x = tracked_object.pose.position.x;
    tracker_point.y = tracked_object.pose.position.y;
    tracker_point.z = tracked_object.pose.position.z;

    bool is_associated = false;
    const unique_identifier_msgs::msg::UUID tracker_uuid = (*tracker_itr)->getUUID();
    const auto it = association_result.tracker_to_measurement.find(tracker_uuid);
    if (it != association_result.tracker_to_measurement.end()) {
      const auto & measurement_uuid = it->second;
      const auto measurement_idx_opt = detected_objects.getObjectIndexByUuid(measurement_uuid);
      if (measurement_idx_opt) {
        const auto & associated_object = detected_objects.objects.at(*measurement_idx_opt);
        detection_point.x = associated_object.pose.position.x;
        detection_point.y = associated_object.pose.position.y;
        detection_point.z = associated_object.pose.position.z;
        is_associated = true;
      }
    }

    if (!is_associated) {
      detection_point = tracker_point;
    }
    object_data.channel_id = detected_objects.channel_index;
    object_data.tracker_point = tracker_point;
    object_data.detection_point = detection_point;
    object_data.is_associated = is_associated;

    object_data.existence_vector = (*tracker_itr)->getExistenceProbabilityVector();
    object_data.total_existence_probability = (*tracker_itr)->getTotalExistenceProbability();
    object_data.tracker_type_str = types::toShortString((*tracker_itr)->getTrackerType());

    auto & group = object_data_map_[object_data.uuid];
    group.push_back(std::move(object_data));
  }
}

void TrackerObjectDebugger::process()
{
  if (!is_initialized_) return;

  object_data_groups_.clear();
  object_data_groups_.reserve(object_data_map_.size());
  for (auto & [uuid, group] : object_data_map_) {
    if (!group.empty()) {
      object_data_groups_.push_back(std::move(group));
    }
  }
}

void TrackerObjectDebugger::draw(
  const std::vector<std::vector<ObjectData>> & object_data_groups,
  visualization_msgs::msg::MarkerArray & marker_array) const
{
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

  const size_t num_channels = channels_config_.size();

  // Allocate per-channel marker vectors once for all groups
  std::vector<visualization_msgs::msg::Marker> marker_detect_boxes_per_channel(num_channels);
  std::vector<visualization_msgs::msg::Marker> marker_detect_lines_per_channel(num_channels);

  // Initialize channel-invariant fields once
  for (size_t idx = 0; idx < num_channels; idx++) {
    std_msgs::msg::ColorRGBA color;
    color.a = 0.9;
    color.r = color_array[idx % PALETTE_SIZE][0];
    color.g = color_array[idx % PALETTE_SIZE][1];
    color.b = color_array[idx % PALETTE_SIZE][2];

    auto & mdb = marker_detect_boxes_per_channel[idx];
    mdb.header.frame_id = frame_id_;
    mdb.ns = "detect_boxes_" + channels_config_[idx].short_name;
    mdb.type = visualization_msgs::msg::Marker::CUBE_LIST;
    mdb.scale.x = 0.2;
    mdb.scale.y = 0.2;
    mdb.scale.z = 0.2;
    mdb.color = color;
    mdb.lifetime = rclcpp::Duration::from_seconds(0.15);

    auto & ml = marker_detect_lines_per_channel[idx];
    ml.header.frame_id = frame_id_;
    ml.ns = "association_lines_" + channels_config_[idx].short_name;
    ml.type = visualization_msgs::msg::Marker::LINE_LIST;
    ml.scale.x = 0.15;
    ml.color = color;
    ml.lifetime = rclcpp::Duration::from_seconds(0.15);
  }

  for (const auto & object_data_group : object_data_groups) {
    if (object_data_group.empty()) continue;
    const auto & object_data_front = object_data_group.front();

    const int32_t group_id = uuidToInt(object_data_front.uuid);

    // Reset per-group fields; reuse allocations from previous group
    for (size_t idx = 0; idx < num_channels; idx++) {
      auto & mdb = marker_detect_boxes_per_channel[idx];
      mdb.id = group_id;
      mdb.header.stamp = object_data_front.time;
      mdb.action = visualization_msgs::msg::Marker::ADD;
      mdb.points.clear();

      auto & ml = marker_detect_lines_per_channel[idx];
      ml.id = group_id;
      ml.header.stamp = object_data_front.time;
      ml.action = visualization_msgs::msg::Marker::ADD;
      ml.points.clear();
    }

    // Reference marker with common per-group fields
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = object_data_front.time;
    marker.id = group_id;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.15);

    // Text marker - existence probability
    visualization_msgs::msg::Marker text_marker = marker;
    text_marker.ns = "existence_probability";
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.scale.z = 0.5;
    text_marker.pose.position.x = object_data_front.tracker_point.x;
    text_marker.pose.position.y = object_data_front.tracker_point.y;
    text_marker.pose.position.z = object_data_front.tracker_point.z + 2.5;

    {
      std::ostringstream oss;
      oss << "total:" << static_cast<int>(object_data_front.total_existence_probability * 100)
          << "\n";
      for (const auto & prob : object_data_front.existence_vector) {
        if (prob.existence_probability < 0.00101f) continue;
        oss << channels_config_[prob.channel_index].short_name
            << static_cast<int>(prob.existence_probability * 100) << ":";
      }
      std::string text = oss.str();
      if (!text.empty()) text.pop_back();
      text += "\n" + object_data_front.uuid_str.substr(0, 6);
      text += "\n" + object_data_front.tracker_type_str;
      text_marker.text = std::move(text);
    }

    constexpr double marker_height_offset = 1.0;
    constexpr double assign_height_offset = 0.6;

    visualization_msgs::msg::Marker marker_track_boxes = marker;
    marker_track_boxes.ns = "track_boxes";
    marker_track_boxes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_track_boxes.action = visualization_msgs::msg::Marker::ADD;
    marker_track_boxes.scale.x = 0.4;
    marker_track_boxes.scale.y = 0.4;
    marker_track_boxes.scale.z = 0.4;
    marker_track_boxes.color.a = 0.9;

    bool is_associated = false;
    for (const auto & object_data : object_data_group) {
      const int channel_id = static_cast<int>(object_data.channel_id);

      geometry_msgs::msg::Point box_point;
      box_point.x = object_data.tracker_point.x;
      box_point.y = object_data.tracker_point.y;
      box_point.z = object_data.tracker_point.z + marker_height_offset;
      marker_track_boxes.points.push_back(box_point);

      if (!object_data.is_associated) continue;
      is_associated = true;

      auto & mdb = marker_detect_boxes_per_channel[channel_id];
      box_point.x = object_data.detection_point.x;
      box_point.y = object_data.detection_point.y;
      box_point.z = object_data.detection_point.z + marker_height_offset + assign_height_offset;
      mdb.points.push_back(box_point);

      auto & ml = marker_detect_lines_per_channel[channel_id];
      geometry_msgs::msg::Point line_point;
      line_point.x = object_data.tracker_point.x;
      line_point.y = object_data.tracker_point.y;
      line_point.z = object_data.tracker_point.z + marker_height_offset;
      ml.points.push_back(line_point);
      line_point.x = object_data.detection_point.x;
      line_point.y = object_data.detection_point.y;
      line_point.z = object_data.detection_point.z + marker_height_offset + assign_height_offset;
      ml.points.push_back(line_point);
    }

    for (size_t i = 0; i < num_channels; i++) {
      if (marker_detect_boxes_per_channel[i].points.empty()) {
        marker_detect_boxes_per_channel[i].action = visualization_msgs::msg::Marker::DELETE;
      }
      marker_array.markers.push_back(marker_detect_boxes_per_channel[i]);
    }
    for (size_t i = 0; i < num_channels; i++) {
      if (marker_detect_lines_per_channel[i].points.empty()) {
        marker_detect_lines_per_channel[i].action = visualization_msgs::msg::Marker::DELETE;
      }
      marker_array.markers.push_back(marker_detect_lines_per_channel[i]);
    }

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
}

void TrackerObjectDebugger::getMessage(visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (!is_initialized_) return;
  draw(object_data_groups_, marker_array);
}

}  // namespace autoware::multi_object_tracker
