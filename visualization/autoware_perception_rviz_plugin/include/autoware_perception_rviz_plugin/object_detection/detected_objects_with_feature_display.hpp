// Copyright 2021 Apex.AI, Inc.
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
#ifndef AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_
#define AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_

#include "autoware_perception_rviz_plugin/object_detection/object_polygon_display_base.hpp"

#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/ros_topic_display.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tier4_perception_msgs/msg/detail/detected_objects_with_feature__struct.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
/// \brief Class defining rviz plugin to visualize DetectedObjects
class AUTOWARE_PERCEPTION_RVIZ_PLUGIN_PUBLIC DetectedObjectsWithFeatureDisplay
: public rviz_common::RosTopicDisplay<tier4_perception_msgs::msg::DetectedObjectsWithFeature>
{
  Q_OBJECT

public:
  using Color = std::array<float, 3U>;
  using Marker = visualization_msgs::msg::Marker;
  using MarkerCommon = rviz_default_plugins::displays::MarkerCommon;
  using ObjectClassificationMsg = autoware_perception_msgs::msg::ObjectClassification;
  using DetectedObjectsWithFeature = tier4_perception_msgs::msg::DetectedObjectsWithFeature;
  using RosTopicDisplay = rviz_common::RosTopicDisplay<DetectedObjectsWithFeature>;


  DetectedObjectsWithFeatureDisplay(const std::string & default_topic = "detected_objects_with_feature");
protected:
  void onInitialize() override;
  void reset() override;  
  void processMessage(DetectedObjectsWithFeature::ConstSharedPtr msg) override;

  void load(const rviz_common::Config & config) override
  {
    RosTopicDisplay::Display::load(config);
    m_marker_common.load(config);
  }

  void update(float wall_dt, float ros_dt) override { m_marker_common.update(wall_dt, ros_dt); }
  double get_line_width() { return m_point_size_property.getFloat(); }
  double get_point_size() { return m_point_size_property.getFloat(); }
  QColor get_point_color() { return m_point_color_property.getColor(); }
  double get_point_alpha() { return m_point_alpha_property.getFloat(); }
private:
  // All rviz plugins should have this. Should be initialized with pointer to this class
  MarkerCommon m_marker_common;  
  // Property to set line width of object shape
  rviz_common::properties::FloatProperty m_line_width_property;
  // Property to set point size of cluster point cloud
  rviz_common::properties::FloatProperty m_point_size_property;
  // Property to set point color of cluster point cloud
  rviz_common::properties::ColorProperty m_point_color_property;
  // Property to show intensity of cluster point cloud
  rviz_common::properties::BoolProperty m_display_intensity_property;
  // Property to set point alpha of cluster point cloud
  rviz_common::properties::FloatProperty m_point_alpha_property;
  // Property to set intensity color threshold of cluster point cloud
  rviz_common::properties::FloatProperty m_intensity_color_scale_max;
  // Property to set color mode of cluster point cloud
  rviz_common::properties::EnumProperty m_color_mode_property;
  // Property to set colormap of cluster point cloud
  rviz_common::properties::EnumProperty m_colormap_property;

  // Property to set the default topic name
  std::string m_default_topic;

  void clear_markers() { m_marker_common.clearMarkers(); }

  void add_marker(visualization_msgs::msg::Marker::ConstSharedPtr marker_ptr)
  {
    m_marker_common.addMessage(marker_ptr);
  }

};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_
