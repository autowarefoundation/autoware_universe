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

#include "node.hpp"

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/math/normalization.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_projection/UTM.h>
#include <tf2/LinearMath/Quaternion.h>

#include <map>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
namespace utils
{
using autoware_perception_msgs::msg::TrafficLightElement;

std::map<std::string, uint8_t> map_color2msg{
  {"unknown", TrafficLightElement::UNKNOWN}, {"red", TrafficLightElement::RED},
  {"yellow", TrafficLightElement::AMBER},    {"green", TrafficLightElement::GREEN},
  {"white", TrafficLightElement::WHITE},
};

std::map<std::string, uint8_t> map_arrow2msg{
  {"unknown", TrafficLightElement::UNKNOWN},
  {"circle", TrafficLightElement::CIRCLE},
  {"up", TrafficLightElement::UP_ARROW},
  {"left", TrafficLightElement::LEFT_ARROW},
  {"right", TrafficLightElement::RIGHT_ARROW},
  {"up_right", TrafficLightElement::UP_RIGHT_ARROW},
  {"up_left", TrafficLightElement::UP_LEFT_ARROW},
  {"down", TrafficLightElement::DOWN_ARROW},
  {"down_left", TrafficLightElement::DOWN_LEFT_ARROW},
  {"down_right", TrafficLightElement::DOWN_RIGHT_ARROW},
  {"cross", TrafficLightElement::CROSS},
};

uint8_t convertMapcolor2Msg(const lanelet::ConstPoint3d & p)
{
  auto msg = map_color2msg.find(p.attribute("color").value());
  if (msg == map_color2msg.end()) {
    return TrafficLightElement::UNKNOWN;
  }
  return msg->second;
}

uint8_t convertMaparrow2Msg(const lanelet::ConstPoint3d & p)
{
  if (!p.hasAttribute("arrow")) {
    return TrafficLightElement::CIRCLE;
  }
  auto msg = map_arrow2msg.find(p.attribute("arrow").value());
  if (msg == map_arrow2msg.end()) {
    return TrafficLightElement::UNKNOWN;
  }
  return msg->second;
}

bool isCompareColorAndShape(const lanelet::ConstPoint3d & p, const TrafficLightElement & elem)
{
  uint8_t p_color = convertMapcolor2Msg(p);
  uint8_t p_shape = convertMaparrow2Msg(p);

  if (elem.color == p_color && elem.shape == p_shape) {
    return true;
  }
  return false;
}

double getArrayDirection(const lanelet::ConstPoint3d & p)
{
  std::string arrow = p.attribute("arrow").value();
  if (arrow == "right") {
    return 0.0;
  } else if (arrow == "down_right") {
    return M_PI_4;
  } else if (arrow == "down") {
    return M_PI_2;
  } else if (arrow == "down_left") {
    return 3 * M_PI_4;
  } else if (arrow == "left") {
    return M_PI;
  } else if (arrow == "up_left") {
    return -3 * M_PI_4;
  } else if (arrow == "up") {
    return -M_PI_2;
  } else if (arrow == "up_right") {
    return -M_PI_4;
  } else {
    return 0.0;
  }
}

void addColor(visualization_msgs::msg::Marker & marker, const uint8_t & p_color)
{
  if (p_color == TrafficLightElement::RED) {
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
  } else if (p_color == TrafficLightElement::GREEN) {
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else if (p_color == TrafficLightElement::AMBER) {
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
  } else {
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
  }
}

void lightAsMarker(
  lanelet::ConstPoint3d p, visualization_msgs::msg::Marker & marker, const std::string & ns,
  const rclcpp::Time & current_time, const double & yaw)
{
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.frame_locked = true;
  marker.ns = ns;
  marker.id = p.id();
  marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  if (!p.hasAttribute("arrow")) {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.pose.position.x = p.x();
    marker.pose.position.y = p.y();
    marker.pose.position.z = p.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    float s = 0.3;

    marker.scale.x = s;
    marker.scale.y = s;
    marker.scale.z = s;
  } else {
    marker.type = visualization_msgs::msg::Marker::ARROW;

    float length = 0.3;

    const double pitch = getArrayDirection(p);
    tf2::Quaternion q;
    q.setRPY(0.0, pitch, yaw);

    marker.pose.position.x = p.x() - (length / 2.0) * std::cos(pitch) * std::cos(yaw);
    marker.pose.position.y = p.y() - (length / 2.0) * std::cos(pitch) * std::sin(yaw);
    marker.pose.position.z = p.z() + (length / 2.0) * std::sin(pitch);

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = length;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
  }

  marker.color.a = 0.999f;

  uint8_t p_color = convertMapcolor2Msg(p);
  addColor(marker, p_color);
}
}  // namespace utils

TrafficLightMapVisualizerNode::TrafficLightMapVisualizerNode(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("traffic_light_map_visualizer_node", node_options)
{
  using std::placeholders::_1;

  light_marker_pub_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/output/traffic_light", 1);
  tl_state_sub_ = create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
    "~/input/tl_state", 1,
    std::bind(&TrafficLightMapVisualizerNode::trafficSignalsCallback, this, _1));
  vector_map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&TrafficLightMapVisualizerNode::binMapCallback, this, _1));
}
void TrafficLightMapVisualizerNode::trafficSignalsCallback(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr input_traffic_signals)
{
  visualization_msgs::msg::MarkerArray output_msg;
  const auto current_time = now();

  for (auto aw_tl_reg_elem = aw_tl_reg_elems_.begin(); aw_tl_reg_elem != aw_tl_reg_elems_.end();
       aw_tl_reg_elem++) {
    // each traffic light, which is not regulatory element
    for (auto light_bulb : (*aw_tl_reg_elem)->lightBulbs()) {
      if (!light_bulb.hasAttribute("traffic_light_id")) {
        RCLCPP_WARN(get_logger(), "'traffic_light_id' is not exist in 'light_bulbs'.");
        continue;
      }

      // get yaw of traffic light
      double yaw{-1.0};
      for (auto traffic_light : (*aw_tl_reg_elem)->trafficLights()) {
        if (traffic_light.id() == light_bulb.attribute("traffic_light_id")) {
          const auto ls_traffic_light = static_cast<lanelet::ConstLineString3d>(traffic_light);
          const auto & traffic_light_bl = ls_traffic_light.front();  // bottom left
          const auto & traffic_light_br = ls_traffic_light.back();   // bottom right
          yaw = autoware_utils::normalize_radian(std::atan2(
            traffic_light_br.y() - traffic_light_bl.y(),
            traffic_light_br.x() - traffic_light_bl.x()));
          break;
        }
      }
      if (yaw == -1.0) {
        RCLCPP_WARN(
          get_logger(),
          "same 'traffic_light_id' is not exist between 'refers' and 'light_bulbs' in regulatory "
          "element.");
        continue;
      }

      // reflection of traffic light recognition results
      for (const auto & input_traffic_signal : input_traffic_signals->traffic_light_groups) {
        if ((*aw_tl_reg_elem)->id() == input_traffic_signal.traffic_light_group_id) {
          // each point, which is one light bulb
          for (auto pt : light_bulb) {
            if (!pt.hasAttribute("color")) {
              RCLCPP_WARN(get_logger(), "'color' is not exist in 'point'.");
              continue;
            }
            for (const auto & elem : input_traffic_signal.elements) {
              visualization_msgs::msg::Marker marker;
              if (utils::isCompareColorAndShape(pt, elem)) {
                utils::lightAsMarker(pt, marker, "traffic_light", current_time, yaw);
              }
              output_msg.markers.push_back(marker);
            }
          }
        }
      }
    }
  }

  light_marker_pub_->publish(output_msg);
}

void TrafficLightMapVisualizerNode::binMapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_map_msg)
{
  lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

  lanelet::utils::conversion::fromBinMsg(*input_map_msg, viz_lanelet_map);
  RCLCPP_DEBUG(get_logger(), "Map is loaded\n");

  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  aw_tl_reg_elems_ = lanelet::utils::query::autowareTrafficLights(all_lanelets);
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::TrafficLightMapVisualizerNode)
