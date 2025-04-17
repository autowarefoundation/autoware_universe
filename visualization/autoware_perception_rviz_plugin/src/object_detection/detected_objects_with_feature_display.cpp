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
  m_default_topic{default_topic}
{
  m_intensity_color_scale_max.setMin(1.0);
  m_intensity_color_scale_max.setMax(255.0);
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

    if (m_display_intensity_property.getBool()) {
      // Use intensity to color the points
      float intensity_threshold = m_intensity_color_scale_max.getFloat();
      for(; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        std_msgs::msg::ColorRGBA color;
        color.r = std::clamp(static_cast<float>(*iter_intensity) / intensity_threshold, 0.0f, 1.0f);
        color.g = 0.0f;
        color.b = 0.0f;
        color.a = 1.0f;
        pointcloud_marker_ptr->colors.push_back(color);
      }
    } else {
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
