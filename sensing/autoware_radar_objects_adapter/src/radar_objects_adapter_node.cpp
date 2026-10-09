// Copyright 2025 The Autoware Contributors
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

#include "radar_objects_adapter_node.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>

namespace autoware::radar_objects_adapter
{

RadarObjectsAdapterNode::RadarObjectsAdapterNode(const rclcpp::NodeOptions & options)
: Node("radar_objects_adapter", options)
{
  radar_objects_sub_ = this->create_subscription<autoware_sensing_msgs::msg::RadarObjects>(
    "~/input/objects", rclcpp::SensorDataQoS(),
    std::bind(&RadarObjectsAdapterNode::objects_callback, this, std::placeholders::_1));

  radar_info_sub_ = this->create_subscription<autoware_sensing_msgs::msg::RadarInfo>(
    "~/input/radar_info", rclcpp::SensorDataQoS(),
    std::bind(&RadarObjectsAdapterNode::radar_info_callback, this, std::placeholders::_1));

  detections_pub_ = this->create_publisher<autoware_perception_msgs::msg::DetectedObjects>(
    "~/output/detections", rclcpp::QoS(10).reliable().transient_local());

  tracks_pub_ = this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    "~/output/tracks", rclcpp::QoS(10).reliable().transient_local());

  RadarObjectsAdapterParams params;

  params.default_position_z = this->declare_parameter<float>("default_position_z");
  params.default_velocity_z = this->declare_parameter<float>("default_velocity_z");
  params.default_acceleration_z = this->declare_parameter<float>("default_acceleration_z");

  params.default_size_x = this->declare_parameter<float>("default_size_x");
  params.default_size_y = this->declare_parameter<float>("default_size_y");
  params.default_size_z = this->declare_parameter<float>("default_size_z");

  // The tracks of this radar are told from those of another by the hash of the input topic name.
  const std::string topic_name = radar_objects_sub_->get_topic_name();

  // Load the classification remap policy: radar label -> perception label, as names in the
  // parameters and as label ids in the remapper.
  std::map<std::string, std::string> classification_remap_str;
  classification_remap_str["UNKNOWN"] =
    declare_parameter<std::string>("classification_remap.UNKNOWN", "UNKNOWN");
  classification_remap_str["CAR"] =
    declare_parameter<std::string>("classification_remap.CAR", "CAR");
  classification_remap_str["TRUCK"] =
    declare_parameter<std::string>("classification_remap.TRUCK", "TRUCK");
  classification_remap_str["MOTORCYCLE"] =
    declare_parameter<std::string>("classification_remap.MOTORCYCLE", "MOTORCYCLE");
  classification_remap_str["BICYCLE"] =
    declare_parameter<std::string>("classification_remap.BICYCLE", "BICYCLE");
  classification_remap_str["PEDESTRIAN"] =
    declare_parameter<std::string>("classification_remap.PEDESTRIAN", "PEDESTRIAN");
  classification_remap_str["ANIMAL"] =
    declare_parameter<std::string>("classification_remap.ANIMAL", "ANIMAL");
  classification_remap_str["HAZARD"] =
    declare_parameter<std::string>("classification_remap.HAZARD", "UNKNOWN");

  // The names are turned into label ids outside the node; what the node keeps is the warning
  // for each entry whose perception label is not a known name.
  const auto [classification_remap, unknown_perception_labels] =
    make_classification_remap_from_string_pair(classification_remap_str);
  for (const auto & [radar_label, perception_label] : unknown_perception_labels) {
    RCLCPP_WARN(
      this->get_logger(),
      "classification_remap: invalid Perception label '%s' for radar '%s'. Using UNKNOWN.",
      perception_label.c_str(), radar_label.c_str());
  }

  adapter_ = std::make_shared<RadarObjectsAdapter>(params, classification_remap, topic_name);
}

void RadarObjectsAdapterNode::objects_callback(
  const autoware_sensing_msgs::msg::RadarObjects & objects_msg)
{
  const auto result = adapter_->convert(objects_msg);
  if (!result) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s", result.error().what());
    return;
  }

  publish_detections(result->detections);
  publish_tracks(result->tracks);
}

void RadarObjectsAdapterNode::publish_detections(
  const autoware_perception_msgs::msg::DetectedObjects & output_object)
{
  auto output_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(detections_pub_);
  *output_msg_ptr = output_object;

  detections_pub_->publish(std::move(output_msg_ptr));
}

void RadarObjectsAdapterNode::publish_tracks(
  const autoware_perception_msgs::msg::TrackedObjects & output_object)
{
  auto output_msg_ptr = ALLOCATE_OUTPUT_MESSAGE_UNIQUE(tracks_pub_);
  *output_msg_ptr = output_object;

  tracks_pub_->publish(std::move(output_msg_ptr));
}

void RadarObjectsAdapterNode::radar_info_callback(
  const autoware_sensing_msgs::msg::RadarInfo & radar_info_msg)
{
  const auto configuration = adapter_->update_radar_info(radar_info_msg);
  if (!configuration) {
    RCLCPP_ERROR_ONCE(get_logger(), "%s", configuration.error().what());
    return;
  }

  // Once per field, as the one warning per field it replaces: RCLCPP_WARN_ONCE in a loop would
  // fire for the first field only.
  for (const auto & [name, value] : configuration->defaulted_fields()) {
    if (warned_defaulted_fields_.insert(name).second) {
      RCLCPP_WARN(
        get_logger(), "The field %s is not available in the radar info message. Defaulting to %f.",
        name.c_str(), value);
    }
  }
}

}  // namespace autoware::radar_objects_adapter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::radar_objects_adapter::RadarObjectsAdapterNode)
