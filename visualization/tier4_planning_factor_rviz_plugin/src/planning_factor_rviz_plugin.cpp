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

#include "planning_factor_rviz_plugin.hpp"

#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware_utils/math/constants.hpp>
#include <autoware_utils/math/trigonometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>

#include <string>

namespace autoware::rviz_plugins
{

using autoware::motion_utils::createDeadLineVirtualWallMarker;
using autoware::motion_utils::createSlowDownVirtualWallMarker;
using autoware::motion_utils::createStopVirtualWallMarker;
using autoware_utils::create_default_marker;
using autoware_utils::create_marker_scale;

namespace
{

std_msgs::msg::ColorRGBA convertFromColorCode(const uint64_t code, const float alpha)
{
  const float r = static_cast<int>(code >> 16) / 255.0;
  const float g = static_cast<int>((code << 48) >> 56) / 255.0;
  const float b = static_cast<int>((code << 56) >> 56) / 255.0;

  return autoware_utils::create_marker_color(r, g, b, alpha);
}

std_msgs::msg::ColorRGBA getGreen(const float alpha)
{
  constexpr uint64_t code = 0x00e676;
  return convertFromColorCode(code, alpha);
}

std_msgs::msg::ColorRGBA getRed(const float alpha)
{
  constexpr uint64_t code = 0xff3d00;
  return convertFromColorCode(code, alpha);
}

std_msgs::msg::ColorRGBA getGray(const float alpha)
{
  constexpr uint64_t code = 0xbdbdbd;
  return convertFromColorCode(code, alpha);
}

visualization_msgs::msg::Marker createArrowMarker(
  const size_t id, const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color, const std::string & name, const double height_offset,
  const double arrow_length = 1.0)
{
  const double line_width = 0.25 * arrow_length;
  const double head_width = 0.5 * arrow_length;
  const double head_height = 0.5 * arrow_length;

  auto marker = create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name + "_arrow", id,
    visualization_msgs::msg::Marker::ARROW,
    create_marker_scale(line_width, head_width, head_height), color);

  geometry_msgs::msg::Point src, dst;
  src = position;
  src.z += height_offset + arrow_length;
  dst = position;
  dst.z += height_offset;

  marker.points.push_back(src);
  marker.points.push_back(dst);

  return marker;
}

visualization_msgs::msg::Marker createCircleMarker(
  const size_t id, const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color, const std::string & name, const double radius,
  const double height_offset, const double line_width = 0.1)
{
  auto marker = create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name, id, visualization_msgs::msg::Marker::LINE_STRIP,
    create_marker_scale(line_width, 0.0, 0.0), color);

  constexpr size_t num_points = 20;
  for (size_t i = 0; i < num_points; ++i) {
    geometry_msgs::msg::Point point;
    const double ratio = static_cast<double>(i) / static_cast<double>(num_points);
    const double theta = 2 * autoware_utils::pi * ratio;
    point.x = position.x + radius * autoware_utils::cos(theta);
    point.y = position.y + radius * autoware_utils::sin(theta);
    point.z = position.z + height_offset;
    marker.points.push_back(point);
  }
  marker.points.push_back(marker.points.front());

  return marker;
}

visualization_msgs::msg::Marker createNameTextMarker(
  const size_t id, const geometry_msgs::msg::Point & position, const std::string & name,
  const double height_offset, const double text_size = 0.5)
{
  auto marker = create_default_marker(
    "map", rclcpp::Clock{RCL_ROS_TIME}.now(), name + "_name_text", id,
    visualization_msgs::msg::Marker::TEXT_VIEW_FACING, create_marker_scale(0.0, 0.0, text_size),
    getGray(0.999));

  marker.text = name;

  marker.pose.position = position;
  marker.pose.position.z += height_offset;

  return marker;
}

visualization_msgs::msg::MarkerArray createTargetMarker(
  const size_t id, const geometry_msgs::msg::Point & position,
  const std_msgs::msg::ColorRGBA & color, const std::string & name,
  const double height_offset = 2.0, const double arrow_length = 1.0, const double line_width = 0.1)
{
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(
    createArrowMarker(id, position, color, name, height_offset, arrow_length));
  marker_array.markers.push_back(createCircleMarker(
    id, position, color, name + "_circle1", 0.5 * arrow_length, height_offset + 0.75 * arrow_length,
    line_width));
  marker_array.markers.push_back(createCircleMarker(
    id, position, color, name + "_circle2", 0.75 * arrow_length,
    height_offset + 0.75 * arrow_length, line_width));
  marker_array.markers.push_back(createNameTextMarker(
    id, position, name, height_offset + 1.5 * arrow_length, 0.5 * arrow_length));

  return marker_array;
}
}  // namespace

void PlanningFactorRvizPlugin::processMessage(
  const autoware_internal_planning_msgs::msg::PlanningFactorArray::ConstSharedPtr msg)
{
  size_t i = 0L;
  for (const auto & factor : msg->factors) {
    const auto text = factor.module + (factor.detail.empty() ? "" : " (" + factor.detail + ")");

    switch (factor.behavior) {
      case autoware_internal_planning_msgs::msg::PlanningFactor::STOP:
        for (const auto & control_point : factor.control_points) {
          const auto virtual_wall = createStopVirtualWallMarker(
            control_point.pose, text, msg->header.stamp, i++, baselink2front_.getFloat());
          add_marker(std::make_shared<visualization_msgs::msg::MarkerArray>(virtual_wall));
        }
        break;

      case autoware_internal_planning_msgs::msg::PlanningFactor::SLOW_DOWN:
        for (const auto & control_point : factor.control_points) {
          const auto virtual_wall = createSlowDownVirtualWallMarker(
            control_point.pose, text, msg->header.stamp, i++, baselink2front_.getFloat());
          add_marker(std::make_shared<visualization_msgs::msg::MarkerArray>(virtual_wall));
        }
        break;
    }

    for (const auto & safety_factor : factor.safety_factors.factors) {
      const auto color = safety_factor.is_safe ? getGreen(0.999) : getRed(0.999);
      for (const auto & point : safety_factor.points) {
        const auto safety_factor_marker = createTargetMarker(i++, point, color, factor.module);
        add_marker(std::make_shared<visualization_msgs::msg::MarkerArray>(safety_factor_marker));
      }
    }
  }
}
}  // namespace autoware::rviz_plugins

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(autoware::rviz_plugins::PlanningFactorRvizPlugin, rviz_common::Display)
