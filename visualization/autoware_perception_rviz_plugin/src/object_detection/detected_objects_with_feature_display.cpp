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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "autoware_perception_rviz_plugin/object_detection/detected_objects_with_feature_display.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <qcolor.h>

#include <rclcpp/duration.hpp>

#include <memory>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
DetectedObjectsWithFeatureDisplay::DetectedObjectsWithFeatureDisplay(
  const std::string & default_topic)
: m_marker_common(this),
  m_line_width_property{"Line Width", 0.03, "Line width of object-shape", this},
  m_point_size_property{"Point Size", 0.1, "Point size of cluster", this},
  m_point_color_property{"Point Color", QColor(255, 0, 0), "Point color of object-shape", this},
  m_display_intensity_property{"Display Intensity", true, "Display intensity of point cloud", this},
  m_intensity_color_scale_max{
    "Intensity Threshold", 50.0, "Intensity threshold of point cloud", this},
  m_color_mode_property{"Color Mode", 0, "Color mode of point cloud", this},
  m_colormap_property{"Colormap", "Jet", "Colormap to use for intensity coloring", this},
  m_default_topic{default_topic}
{
  m_intensity_color_scale_max.setMin(1.0);
  m_intensity_color_scale_max.setMax(255.0);
  m_color_mode_property.addOption("Flat", 0);
  m_color_mode_property.addOption("Intensity", 1);
  m_color_mode_property.addOption("Cluster", 2);
  m_colormap_property.addOption("Jet", 0);
  m_colormap_property.addOption("HSV", 1);
  m_colormap_property.addOption("Viridis", 2);
  m_colormap_property.addOption("Red", 3);  
  m_colormap_property.addOption("Gray", 4);
  m_colormap_property.addOption("Turbo", 5);
  m_colormap_property.addOption("Rainbow", 6);
  m_colormap_property.addOption("Parula", 7); 
}

void DetectedObjectsWithFeatureDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  m_marker_common.initialize(this->context_, this->scene_node_);
  QString message_type = QString::fromStdString(rosidl_generator_traits::name<DetectedObjectsWithFeature>());
  this->topic_property_->setMessageType(message_type);
  this->topic_property_->setDescription("Topic to subscribe to.");  
}

void DetectedObjectsWithFeatureDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common.clearMarkers();
}
std_msgs::msg::ColorRGBA generateDistinctColor(size_t idx)
{
  // Use HSV hue variation for distinct colors
  float hue = static_cast<float>((idx * 47) % 360);  // 47 is a prime for spacing
  float c = 1.0f;
  float x = c * (1 - std::fabs(std::fmod(hue / 60.0f, 2) - 1));
  float r = 0, g = 0, b = 0;
  if (hue < 60)      { r = c; g = x; b = 0; }
  else if (hue < 120){ r = x; g = c; b = 0; }
  else if (hue < 180){ r = 0; g = c; b = x; }
  else if (hue < 240){ r = 0; g = x; b = c; }
  else if (hue < 300){ r = x; g = 0; b = c; }
  else               { r = c; g = 0; b = x; }

  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  return color;
}
std_msgs::msg::ColorRGBA colormapJet(float value_normalized)
{
  value_normalized = std::clamp(value_normalized, 0.0f, 1.0f);
  float r = std::clamp(1.5f - std::abs(4.0f * value_normalized - 3.0f), 0.0f, 1.0f);
  float g = std::clamp(1.5f - std::abs(4.0f * value_normalized - 2.0f), 0.0f, 1.0f);
  float b = std::clamp(1.5f - std::abs(4.0f * value_normalized - 1.0f), 0.0f, 1.0f);

  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  return color;
}

std_msgs::msg::ColorRGBA colormapViridis(float v)
{
  v = std::clamp(v, 0.0f, 1.0f);
  float r = std::clamp(0.267f + v * (0.993f - 0.267f), 0.0f, 1.0f);
  float g = std::clamp(0.004f + v * (0.906f - 0.004f), 0.0f, 1.0f);
  float b = std::clamp(0.329f + v * (0.933f - 0.329f), 0.0f, 1.0f);
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  return color;
}

std_msgs::msg::ColorRGBA colormapHSV(float v)
{
  v = std::clamp(v, 0.0f, 1.0f);
  float h = v * 360.0f;  // hue in degrees
  float s = 1.0f, l = 0.5f;

  float c = (1.0f - std::fabs(2.0f * l - 1.0f)) * s;
  float x = c * (1.0f - std::fabs(std::fmod(h / 60.0f, 2) - 1.0f));
  float m = l - c / 2.0f;

  float r = 0, g = 0, b = 0;
  if (h < 60)      { r = c; g = x; b = 0; }
  else if (h < 120){ r = x; g = c; b = 0; }
  else if (h < 180){ r = 0; g = c; b = x; }
  else if (h < 240){ r = 0; g = x; b = c; }
  else if (h < 300){ r = x; g = 0; b = c; }
  else             { r = c; g = 0; b = x; }

  std_msgs::msg::ColorRGBA color;
  color.r = r + m;
  color.g = g + m;
  color.b = b + m;
  color.a = 1.0f;
  return color;
}
std_msgs::msg::ColorRGBA colormapRed(float v)
{
  std_msgs::msg::ColorRGBA color;
  v = std::clamp(v, 0.0f, 1.0f);
  color.r = v;
  color.g = 0.0f;
  color.b = 0.0f;
  color.a = 1.0f;
  return color;
}
std_msgs::msg::ColorRGBA colormapGray(float value)
{
  value = std::clamp(value, 0.0f, 1.0f);
  std_msgs::msg::ColorRGBA color;
  color.r = value;
  color.g = value;
  color.b = value;
  color.a = 1.0f;
  return color;
}
std_msgs::msg::ColorRGBA colormapTurbo(float v)
{
  v = std::clamp(v, 0.0f, 1.0f);

  // The coefficients for Turbo are derived from Google’s approximation
  float r = std::clamp(0.996f * v - 0.1046f, 0.0f, 1.0f);
  float g = std::clamp(0.996f * v - 0.0625f, 0.0f, 1.0f);
  float b = std::clamp(0.996f * v - 0.0878f, 0.0f, 1.0f);

  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  
  return color;
}
std_msgs::msg::ColorRGBA colormapRainbow(float v)
{
  v = std::clamp(v, 0.0f, 1.0f);
  float r = std::sin(2.0f * M_PI * v + 0.0f) * 0.5f + 0.5f;
  float g = std::sin(2.0f * M_PI * v + 2.0f * M_PI / 3.0f) * 0.5f + 0.5f;
  float b = std::sin(2.0f * M_PI * v + 4.0f * M_PI / 3.0f) * 0.5f + 0.5f;

  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = 1.0f;
  return color;
}
std_msgs::msg::ColorRGBA colormapParula(float v)
{
  v = std::clamp(v, 0.0f, 1.0f);

  // Parula approximation (6 key points from MATLAB)
  const std::vector<std::array<float, 3>> parula_data = {
    {0.2081f, 0.1663f, 0.5292f},  // dark blue
    {0.2291f, 0.3220f, 0.5451f},  // blue
    {0.2669f, 0.4887f, 0.5561f},  // teal
    {0.3052f, 0.6502f, 0.5653f},  // greenish
    {0.5849f, 0.7823f, 0.4863f},  // yellow-green
    {0.9763f, 0.9831f, 0.0538f}   // yellow
  };

  float scaled = v * (parula_data.size() - 1);
  int idx = static_cast<int>(scaled);
  float frac = scaled - idx;

  if (idx >= static_cast<int>(parula_data.size()) - 1) {
    idx = static_cast<int>(parula_data.size()) - 2;
    frac = 1.0f;
  }

  std_msgs::msg::ColorRGBA color;
  color.r = (1 - frac) * parula_data[idx][0] + frac * parula_data[idx + 1][0];
  color.g = (1 - frac) * parula_data[idx][1] + frac * parula_data[idx + 1][1];
  color.b = (1 - frac) * parula_data[idx][2] + frac * parula_data[idx + 1][2];
  color.a = 1.0f;

  return color;
}

void DetectedObjectsWithFeatureDisplay::processMessage(DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  clear_markers();
  int id = 0;
  for (const auto & feature_object : msg->feature_objects) {
    const auto & cluster = feature_object.feature.cluster;
    if (cluster.width <= 0 || cluster.height <= 0) {      
      continue;  // Skip this cluster
    }
    // Create a marker to display the cluster point cloud
    double point_size = get_point_size();
    auto pointcloud_marker_ptr = std::make_shared<Marker>();
    pointcloud_marker_ptr->header = msg->header;
    pointcloud_marker_ptr->ns = "cluster_point_cloud";
    pointcloud_marker_ptr->id = static_cast<int>(id);
    pointcloud_marker_ptr->type = visualization_msgs::msg::Marker::POINTS;
    pointcloud_marker_ptr->action = visualization_msgs::msg::Marker::ADD;
    pointcloud_marker_ptr->pose.position.x = 0.0;
    pointcloud_marker_ptr->pose.position.y = 0.0;
    pointcloud_marker_ptr->pose.position.z = 0.0;
    pointcloud_marker_ptr->pose.orientation.w = 1.0;
    pointcloud_marker_ptr->scale.x = point_size; 
    pointcloud_marker_ptr->scale.y = point_size;
    pointcloud_marker_ptr->lifetime = rclcpp::Duration::from_seconds(0.15);
    pointcloud_marker_ptr->points.clear();
    pointcloud_marker_ptr->colors.clear();
    pointcloud_marker_ptr->points.reserve(cluster.width * cluster.height);
    pointcloud_marker_ptr->colors.reserve(cluster.width * cluster.height);
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cluster, "intensity");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      geometry_msgs::msg::Point point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      pointcloud_marker_ptr->points.push_back(point);
    }

    int mode = m_color_mode_property.getOptionInt();
    if (mode == 1) {
      using ColorMapFn = std_msgs::msg::ColorRGBA(*)(float);
      ColorMapFn color_fn = colormapJet;
      switch (m_colormap_property.getOptionInt()) {
        case 0: color_fn = colormapJet; break;
        case 1: color_fn = colormapHSV; break;
        case 2: color_fn = colormapViridis; break;
        case 3: color_fn = colormapRed; break;
        case 4: color_fn = colormapGray; break;
        case 5: color_fn = colormapTurbo; break;
        case 6: color_fn = colormapRainbow; break;
        case 7: color_fn = colormapParula; break;
        default: color_fn = colormapJet; break;
      }
      // Use intensity to color the points
      float intensity_threshold = m_intensity_color_scale_max.getFloat();
      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        float intensity_norm = static_cast<float>(*iter_intensity) / intensity_threshold;
        pointcloud_marker_ptr->colors.push_back(color_fn(intensity_norm));
        //pointcloud_marker_ptr->colors.push_back(color);
      }
    } else if (mode == 2 /* Cluster */) {
      std_msgs::msg::ColorRGBA cluster_color = generateDistinctColor(id);
      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        pointcloud_marker_ptr->colors.push_back(cluster_color);
      }
    } else {
      // Use the default color
      std_msgs::msg::ColorRGBA color;
      color.r = get_point_color().redF();
      color.g = get_point_color().greenF();
      color.b = get_point_color().blueF();
      color.a = 1.0f;
      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        pointcloud_marker_ptr->colors.push_back(color);
      }
    }
    add_marker(pointcloud_marker_ptr);
    id++;
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsWithFeatureDisplay, rviz_common::Display)
