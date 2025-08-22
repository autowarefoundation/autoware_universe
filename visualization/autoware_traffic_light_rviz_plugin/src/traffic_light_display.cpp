//
//  Copyright 2025 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware_traffic_light_rviz_plugin/traffic_light_display.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/message_conversion.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <autoware_perception_msgs/msg/traffic_light_element.hpp>

#include <Ogre.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware_traffic_light_rviz_plugin
{

std::string elementToString(const autoware_perception_msgs::msg::TrafficLightElement & element)
{
  auto color = element.color;
  auto shape = element.shape;
  if (shape == autoware_perception_msgs::msg::TrafficLightElement::CIRCLE) {
    switch (color) {
      case autoware_perception_msgs::msg::TrafficLightElement::RED:
        return "RED";
      case autoware_perception_msgs::msg::TrafficLightElement::AMBER:
        return "AMBER";
      case autoware_perception_msgs::msg::TrafficLightElement::GREEN:
        return "GREEN";
      default:
        return "UNKNOWN";
    }
  } else {
    switch (shape) {
      case autoware_perception_msgs::msg::TrafficLightElement::LEFT_ARROW:
        return "LEFT";
      case autoware_perception_msgs::msg::TrafficLightElement::RIGHT_ARROW:
        return "RIGHT";
      case autoware_perception_msgs::msg::TrafficLightElement::UP_ARROW:
        return "UP";
      case autoware_perception_msgs::msg::TrafficLightElement::DOWN_ARROW:
        return "DOWN";
      case autoware_perception_msgs::msg::TrafficLightElement::DOWN_LEFT_ARROW:
        return "DOWN_LEFT";
      case autoware_perception_msgs::msg::TrafficLightElement::DOWN_RIGHT_ARROW:
        return "DOWN_RIGHT";
      case autoware_perception_msgs::msg::TrafficLightElement::UP_LEFT_ARROW:
        return "UP_LEFT";
      case autoware_perception_msgs::msg::TrafficLightElement::UP_RIGHT_ARROW:
        return "UP_RIGHT";
      default:
        return "UNKNOWN";
    }
  }
}

std::vector<TrafficLightInfo> getTrafficLightInfo(
  const lanelet::LaneletMap & map, const lanelet::Id & traffic_light_id)
{
  std::vector<TrafficLightInfo> traffic_lights;
  auto traffic_light_reg_elems = map.regulatoryElementLayer.find(traffic_light_id);
  if (traffic_light_reg_elems == map.regulatoryElementLayer.end()) {
    return {};
  }
  auto traffic_light =
    dynamic_cast<const lanelet::autoware::AutowareTrafficLight *>(traffic_light_reg_elems->get());
  if (!traffic_light) {
    return {};
  }
  for (const auto & traffic_light_linestring : traffic_light->trafficLights()) {
    auto id = traffic_light_linestring.id();
    auto p1 = traffic_light_linestring.lineString()->front();
    auto p2 = traffic_light_linestring.lineString()->back();
    Point3d center{(p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2, (p1.z() + p2.z()) / 2};
    TrafficLightInfo traffic_light_info;
    traffic_light_info.id = id;
    traffic_light_info.linestring_center = center;
    for (const auto & bulb : traffic_light->lightBulbs()) {
      for (const auto & point : bulb) {
        TrafficLightBulbInfo bulb_info;
        bulb_info.id = point.id();
        bulb_info.color = point.attributeOr("color", "none");
        bulb_info.shape = point.attributeOr("arrow", "none");
        bulb_info.position = Point3d{point.x(), point.y(), point.z()};
        traffic_light_info.bulbs.emplace_back(bulb_info);
      }
    }
    traffic_lights.emplace_back(traffic_light_info);
  }
  return traffic_lights;
}

std::vector<TrafficLightBulbInfo> getBlightBulbs(
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & current_elements,
  const std::vector<TrafficLightBulbInfo> & bulbs)
{
  using autoware_perception_msgs::msg::TrafficLightElement;

  std::vector<TrafficLightBulbInfo> current_bulbs;

  auto append_circle_color_bulb = [&current_bulbs, &bulbs](const std::string & color) {
    for (const auto & bulb : bulbs) {
      if (bulb.color == color && (bulb.shape == "none" || bulb.shape == "circle")) {
        current_bulbs.emplace_back(bulb);
      }
    }
  };

  auto append_arrow_bulb = [&current_bulbs, &bulbs](const std::string & arrow) {
    for (const auto & bulb : bulbs) {
      if (bulb.shape == arrow) {
        current_bulbs.emplace_back(bulb);
      }
    }
  };

  for (const auto & current_element : current_elements) {
    if (current_element.shape == TrafficLightElement::CIRCLE) {
      switch (current_element.color) {
        case TrafficLightElement::RED:
          append_circle_color_bulb("red");
          break;
        case TrafficLightElement::AMBER:
          append_circle_color_bulb("yellow");
          break;
        case TrafficLightElement::GREEN:
          append_circle_color_bulb("green");
        default:
          break;
      }
    }
    if (current_element.shape == TrafficLightElement::LEFT_ARROW) {
      append_arrow_bulb("left");
    } else if (current_element.shape == TrafficLightElement::RIGHT_ARROW) {
      append_arrow_bulb("right");
    } else if (current_element.shape == TrafficLightElement::UP_ARROW) {
      append_arrow_bulb("up");
    } else if (current_element.shape == TrafficLightElement::DOWN_ARROW) {
      append_arrow_bulb("down");
    } else if (current_element.shape == TrafficLightElement::DOWN_LEFT_ARROW) {
      append_arrow_bulb("down_left");
    } else if (current_element.shape == TrafficLightElement::DOWN_RIGHT_ARROW) {
      append_arrow_bulb("down_right");
    } else if (current_element.shape == TrafficLightElement::UP_LEFT_ARROW) {
      append_arrow_bulb("up_left");
    } else if (current_element.shape == TrafficLightElement::UP_RIGHT_ARROW) {
      append_arrow_bulb("up_right");
    }
  }
  return current_bulbs;
}

TrafficLightDisplay::TrafficLightDisplay() = default;

TrafficLightDisplay::~TrafficLightDisplay() = default;

void TrafficLightDisplay::onInitialize()
{
  auto rviz_ros_node = context_->getRosNodeAbstraction();

  // Create scene node for text displays
  root_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  lanelet_map_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Lanelet Map Topic", QString("/map/vector_map"), QString("autoware_map_msgs/msg/LaneletMapBin"),
    QString("Topic for lanelet map data"), this, SLOT(topic_updated_lanelet_map()));
  lanelet_map_topic_property_->initialize(rviz_ros_node);

  traffic_light_topic_property_ = std::make_unique<rviz_common::properties::RosTopicProperty>(
    "Traffic Light Topic", QString("/perception/traffic_light_recognition/traffic_signals"),
    QString("autoware_perception_msgs/msg/TrafficLightGroupArray"),
    QString("Topic for traffic light data"), this, SLOT(topic_updated_traffic_light()));
  traffic_light_topic_property_->initialize(rviz_ros_node);

  timeout_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Timeout", 1.0, "Timeout in seconds for traffic light data. Set 0 to disable timeout.", this);
  timeout_property_->setMin(0.0);

  text_z_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Text Z Offset", 1.3, "Z offset in meters for traffic light text display", this);

  text_x_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Text X Offset", 0.0, "X offset in meters for traffic light text display", this);

  text_y_offset_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Text Y Offset", 0.0, "Y offset in meters for traffic light text display", this);

  show_text_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Show Text", true, "Show/hide traffic light state text", this);

  show_bulb_property_ = std::make_unique<rviz_common::properties::BoolProperty>(
    "Show Bulb", true, "Show/hide traffic light bulb visualization", this);

  text_prefix_property_ = std::make_unique<rviz_common::properties::StringProperty>(
    "Text Prefix", "", "Prefix string to add before traffic light state text", this);

  font_size_property_ = std::make_unique<rviz_common::properties::FloatProperty>(
    "Font Size", 0.5, "Font size for traffic light state text", this);
  font_size_property_->setMin(0.1);

  text_color_property_ = std::make_unique<rviz_common::properties::ColorProperty>(
    "Text Color", QColor(255, 255, 255), "Color for traffic light state text", this);
}

void TrafficLightDisplay::setupRosSubscriptions()
{
  topic_updated_lanelet_map();
  topic_updated_traffic_light();
}

void TrafficLightDisplay::onEnable()
{
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  setupRosSubscriptions();
  root_node_->setVisible(true);
  for (const auto & [id, text_node] : traffic_light_text_nodes_) {
    text_node->setVisible(show_text_property_->getBool());
  }
  for (const auto & [id, bulb_display] : traffic_light_bulb_displays_) {
    bulb_display->getEntity()->setVisible(show_bulb_property_->getBool());
  }
}

void TrafficLightDisplay::onDisable()
{
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  lanelet_map_sub_.reset();
  traffic_light_group_array_sub_.reset();
  root_node_->setVisible(false);
  for (const auto & [id, text_node] : traffic_light_text_nodes_) {
    text_node->setVisible(false);
  }
  for (const auto & [id, bulb_display] : traffic_light_bulb_displays_) {
    bulb_display->getEntity()->setVisible(false);
  }
}

void TrafficLightDisplay::reset()
{
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  std::lock_guard<std::mutex> lock_lanelet(lanelet_map_mutex_);
  std::lock_guard<std::mutex> lock_traffic(traffic_light_mutex_);

  rviz_common::Display::reset();

  // Clear all display objects
  traffic_light_text_displays_.clear();
  traffic_light_text_nodes_.clear();
  traffic_light_bulb_displays_.clear();

  // Reset data
  lanelet_map_.reset();
  traffic_light_groups_.reset();
  last_traffic_light_received_time_ = rclcpp::Time();

  // Reset subscriptions
  lanelet_map_sub_.reset();
  traffic_light_group_array_sub_.reset();

  if (root_node_) {
    root_node_->removeAndDestroyAllChildren();
  }
}

void TrafficLightDisplay::hideAllDisplays()
{
  std::lock_guard<std::mutex> lock_property(property_mutex_);
  for (const auto & [id, text_node] : traffic_light_text_nodes_) {
    text_node->setVisible(false);
  }
  for (const auto & [id, bulb_display] : traffic_light_bulb_displays_) {
    bulb_display->getEntity()->setVisible(false);
  }
}

bool TrafficLightDisplay::checkTimeout() const
{
  const float timeout = timeout_property_->getFloat();
  if (timeout > 0.0) {
    auto current_time = context_->getRosNodeAbstraction().lock()->get_raw_node()->now();
    if ((current_time - last_traffic_light_received_time_).seconds() > timeout) {
      return true;
    }
  }
  return false;
}

void TrafficLightDisplay::updateTrafficLightText(
  const TrafficLightInfo & info, const std::string & state_text)
{
  // Note: This method is called from update() which already holds property_mutex_
  if (traffic_light_text_displays_.find(info.id) == traffic_light_text_displays_.end()) {
    auto text_display = std::make_unique<rviz_rendering::MovableText>(
      state_text, "Liberation Sans", font_size_property_->getFloat());
    text_display->setTextAlignment(
      rviz_rendering::MovableText::H_CENTER, rviz_rendering::MovableText::V_CENTER);
    const QColor & color = text_color_property_->getColor();
    text_display->setColor(
      Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), color.alphaF()));
    traffic_light_text_displays_[info.id] = std::move(text_display);
  }

  if (traffic_light_text_nodes_.find(info.id) == traffic_light_text_nodes_.end()) {
    traffic_light_text_nodes_[info.id] = root_node_->createChildSceneNode();
    traffic_light_text_nodes_[info.id]->attachObject(traffic_light_text_displays_[info.id].get());
  }

  std::string display_text = text_prefix_property_->getStdString() + state_text;
  Ogre::Vector3 position(
    static_cast<float>(info.linestring_center.x) + text_x_offset_property_->getFloat(),
    static_cast<float>(info.linestring_center.y) + text_y_offset_property_->getFloat(),
    static_cast<float>(info.linestring_center.z) + text_z_offset_property_->getFloat());
  traffic_light_text_nodes_[info.id]->setPosition(position);
  traffic_light_text_displays_[info.id]->setCaption(display_text);
  traffic_light_text_displays_[info.id]->setCharacterHeight(font_size_property_->getFloat());
  const QColor & color = text_color_property_->getColor();
  traffic_light_text_displays_[info.id]->setColor(
    Ogre::ColourValue(color.redF(), color.greenF(), color.blueF(), color.alphaF()));
  traffic_light_text_nodes_[info.id]->setVisible(show_text_property_->getBool());
}

void TrafficLightDisplay::updateTrafficLightBulbs(
  const TrafficLightInfo & info,
  const std::vector<autoware_perception_msgs::msg::TrafficLightElement> & elements)
{
  // Note: This method is called from update() which already holds property_mutex_
  auto current_bulbs = getBlightBulbs(elements, info.bulbs);
  for (const auto & bulb : current_bulbs) {
    if (traffic_light_bulb_displays_.find(bulb.id) == traffic_light_bulb_displays_.end()) {
      auto bulb_display = std::make_unique<rviz_rendering::Shape>(
        rviz_rendering::Shape::Type::Sphere, scene_manager_, root_node_);

      Ogre::ColourValue color;
      if (bulb.color == "red") {
        color = Ogre::ColourValue(1.0, 0.0, 0.0, 1.0);
      } else if (bulb.color == "yellow") {
        color = Ogre::ColourValue(1.0, 0.5, 0.0, 1.0);
      } else if (bulb.color == "green") {
        color = Ogre::ColourValue(0.0, 1.0, 0.0, 1.0);
      }
      bulb_display->setColor(color);

      bulb_display->setPosition(
        Ogre::Vector3(
          static_cast<float>(bulb.position.x), static_cast<float>(bulb.position.y),
          static_cast<float>(bulb.position.z)));
      const float radius = 0.4;
      bulb_display->setScale(Ogre::Vector3(radius, radius, radius));
      traffic_light_bulb_displays_[bulb.id] = std::move(bulb_display);
    }
    traffic_light_bulb_displays_[bulb.id]->getEntity()->setVisible(show_bulb_property_->getBool());
  }
}

void TrafficLightDisplay::update(float wall_dt, float ros_dt)
{
  (void)wall_dt;
  (void)ros_dt;

  std::lock_guard<std::mutex> lock_lanelet(lanelet_map_mutex_);
  std::lock_guard<std::mutex> lock_traffic(traffic_light_mutex_);

  if (!lanelet_map_ || !traffic_light_groups_) {
    return;
  }

  if (checkTimeout()) {
    hideAllDisplays();
    return;
  }

  hideAllDisplays();

  for (const auto & traffic_light_group : traffic_light_groups_->traffic_light_groups) {
    auto tl_infos = getTrafficLightInfo(*lanelet_map_, traffic_light_group.traffic_light_group_id);

    std::stringstream ss;
    for (const auto & elem : traffic_light_group.elements) {
      ss << elementToString(elem) << " ";
    }

    std::lock_guard<std::mutex> lock_property(property_mutex_);
    for (const auto & info : tl_infos) {
      updateTrafficLightText(info, ss.str());
      updateTrafficLightBulbs(info, traffic_light_group.elements);
    }
  }
}

void TrafficLightDisplay::topic_updated_lanelet_map()
{
  lanelet_map_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  lanelet_map_sub_ =
    rviz_ros_node->get_raw_node()->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      lanelet_map_topic_property_->getTopicStd(), rclcpp::QoS(1).transient_local(),
      std::bind(&TrafficLightDisplay::onLaneletMapReceived, this, std::placeholders::_1));
}

void TrafficLightDisplay::topic_updated_traffic_light()
{
  traffic_light_group_array_sub_.reset();
  auto rviz_ros_node = context_->getRosNodeAbstraction().lock();
  traffic_light_group_array_sub_ =
    rviz_ros_node->get_raw_node()
      ->create_subscription<autoware_perception_msgs::msg::TrafficLightGroupArray>(
        traffic_light_topic_property_->getTopicStd(), rclcpp::QoS(10),
        std::bind(
          &TrafficLightDisplay::onTrafficLightGroupArrayReceived, this, std::placeholders::_1));
}

void TrafficLightDisplay::onLaneletMapReceived(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(lanelet_map_mutex_);
  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_);
}

void TrafficLightDisplay::onTrafficLightGroupArrayReceived(
  const autoware_perception_msgs::msg::TrafficLightGroupArray::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(traffic_light_mutex_);
  traffic_light_groups_ = msg;
  last_traffic_light_received_time_ =
    context_->getRosNodeAbstraction().lock()->get_raw_node()->now();
}

}  // namespace autoware_traffic_light_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware_traffic_light_rviz_plugin::TrafficLightDisplay, rviz_common::Display)
