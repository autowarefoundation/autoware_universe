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

#include "autoware_perception_rviz_plugin/object_detection/detected_objects_with_feature_helper.hpp"

#include <QObject>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
  m_point_size_property{"Point Size", 0.1, "Point size of cluster", this},
  m_color_mode_property("Color Mode", 0, "Color mode of point cloud", this),
  m_colormap_property{"Colormap", "Jet", "Colormap to use for intensity coloring", this},
  m_point_color_property{"Point Color", QColor(255, 0, 0), "Point color of object-shape", this},
  m_intensity_color_scale_max{
    "Intensity Threshold", 50.0, "Intensity threshold of point cloud", this},
  m_show_colorbar_property{
    "Show Colorbar", false, "Display the colormap colorbar in a separate window", this},
  m_default_topic{default_topic}
{
  m_intensity_color_scale_max.setMin(1.0);
  m_intensity_color_scale_max.setMax(255.0);

  m_color_mode_property.addOption("Flat", 0);
  m_color_mode_property.addOption("Intensity", 1);
  m_color_mode_property.addOption("Cluster", 2);

  const auto & names = getColormapInfo().names;
  for (int i = 0; i < NUM_COLORMAPS; ++i) {
    m_colormap_property.addOption(names[i], i);
  }
}

void DetectedObjectsWithFeatureDisplay::onInitialize()
{
  RosTopicDisplay::onInitialize();
  m_marker_common.initialize(this->context_, this->scene_node_);

  // Set topic message type and description
  QString message_type =
    QString::fromStdString(rosidl_generator_traits::name<DetectedObjectsWithFeature>());
  this->topic_property_->setMessageType(message_type);
  this->topic_property_->setDescription("Topic to subscribe to.");

  // Generate the colorbar for the colormap
  generateColorbar();
  // Create the colorbar widget
  m_colorbar_widget = new ColorbarWidget();

  // m_colorbar_widget->setWindowTitle("Colorbar");
  m_colorbar_widget->setColorbarImage(m_colorbar_image);  // Set the generated colorbar image
  m_colorbar_widget->setMinMax(0.0f, m_intensity_color_scale_max.getFloat());
  m_colorbar_widget->setWindowFlags(Qt::Tool);
  m_colorbar_widget->show();

  QObject::connect(
    &m_color_mode_property, &rviz_common::properties::Property::changed, this,
    [this]() { updateColormapPropertiesVisibility(); });
  QObject::connect(
    &m_show_colorbar_property, &rviz_common::properties::Property::changed, this,
    [this]() { updateColorbarVisibility(); });
  QObject::connect(
    &m_colormap_property, &rviz_common::properties::Property::changed, this,
    [this]() { updateColormapAndColorbar(); });
  QObject::connect(
    &m_intensity_color_scale_max, &rviz_common::properties::Property::changed, this,
    [this]() { updateColormapAndColorbar(); });
  updateColormapPropertiesVisibility();
  
}

void DetectedObjectsWithFeatureDisplay::updateColormapAndColorbar()
{
  generateColorbar();  // Regenerate the image with the current colormap
  if (m_colorbar_widget) {
    m_colorbar_widget->setMinMax(0.0f, m_intensity_color_scale_max.getFloat());
    m_colorbar_widget->setColorbarImage(m_colorbar_image);
    if (m_colorbar_widget->isVisible()) {
      m_colorbar_widget->update();  // Request a repaint of the colorbar widget
    }
  }
}

void DetectedObjectsWithFeatureDisplay::updateColormapPropertiesVisibility()
{
  int mode = m_color_mode_property.getOptionInt();
  bool is_intensity_mode_active = (mode == INTENSITY_COLOR);
  m_colormap_property.setHidden(!is_intensity_mode_active);
  m_show_colorbar_property.setHidden(!is_intensity_mode_active);
  m_intensity_color_scale_max.setHidden(!is_intensity_mode_active);
  m_point_color_property.setHidden(mode != FLAT_COLOR);
  updateColorbarVisibility();
}

void DetectedObjectsWithFeatureDisplay::updateColorbarVisibility()
{
  if (!m_colorbar_widget) {
    return;
  }
  bool should_show =
    m_color_mode_property.getOptionInt() == INTENSITY_COLOR && m_show_colorbar_property.getBool();
  if (should_show && !m_colorbar_widget->isVisible()) {
    // Ensure the colorbar image and labels are up-to-date before showing
    updateColormapAndColorbar();  // This generates the image and sets it
    // updateIntensityMax();      // This sets the min/max labels
    m_colorbar_widget->show();
  } else if (!should_show && m_colorbar_widget->isVisible()) {
    m_colorbar_widget->hide();
  }
}

void DetectedObjectsWithFeatureDisplay::generateColorbar()
{
  constexpr int width = 20;
  constexpr int height = 200;
  const auto & colormap_info = getColormapInfo();
  ColormapFuncType colormap_fn = colormap_info.getFunctionSafe(m_colormap_property.getOptionInt());
  // Create a colorbar image
  m_colorbar_image = QImage(width, height, QImage::Format_RGB32);

  for (int y = 0; y < height; ++y) {
    float norm = static_cast<float>(y) / (height - 1);
    std_msgs::msg::ColorRGBA color = colormap_fn(norm);
    QRgb rgb = qRgb(color.r * 255, color.g * 255, color.b * 255);
    for (int x = 0; x < width; ++x) {
      m_colorbar_image.setPixel(x, y, rgb);
    }
  }
}

DetectedObjectsWithFeatureDisplay::~DetectedObjectsWithFeatureDisplay()
{
  delete m_colorbar_widget;
}

void DetectedObjectsWithFeatureDisplay::reset()
{
  RosTopicDisplay::reset();
  m_marker_common.clearMarkers();
}

void DetectedObjectsWithFeatureDisplay::processMessage(
  DetectedObjectsWithFeature::ConstSharedPtr msg)
{
  clear_markers();
  
  const int mode = m_color_mode_property.getOptionInt();
  const float max_intensity = m_intensity_color_scale_max.getFloat();
  const ColormapFuncType color_fn =
    getColormapInfo().getFunctionSafe(m_colormap_property.getOptionInt());
  const auto default_color = [&]() {
    std_msgs::msg::ColorRGBA c;
    c.r = get_point_color().redF();
    c.g = get_point_color().greenF();
    c.b = get_point_color().blueF();
    c.a = 1.0f;
    return c;
  }();
    
  int id = 0;
  for (const auto & feature_object : msg->feature_objects) {
    const auto & cluster = feature_object.feature.cluster;
    if (cluster.width == 0 || cluster.height == 0) {
      continue;  // Skip this cluster
    }
    // Create a marker to display the cluster point cloud
    auto marker = std::make_shared<Marker>();
    marker->header = msg->header;
    marker->ns = "cluster_point_cloud";
    marker->id = id++;
    marker->type = visualization_msgs::msg::Marker::POINTS;
    marker->action = visualization_msgs::msg::Marker::ADD;
    marker->pose.orientation.w = 1.0;
    marker->scale.x = marker->scale.y = get_point_size();
    marker->lifetime = rclcpp::Duration::from_seconds(0.15);

    const size_t n_points = cluster.width * cluster.height;
    marker->points.clear();
    marker->colors.clear();
    marker->points.reserve(n_points);
    marker->colors.reserve(n_points);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
    sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity(cluster, "intensity");
    // Set the position of each point in the marker
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      geometry_msgs::msg::Point pt;
      pt.x = *iter_x;
      pt.y = *iter_y;
      pt.z = *iter_z;
      marker->points.push_back(pt);
    }
    // Set the color of each point based on the selected mode
    if (mode == INTENSITY_COLOR) {
      for (; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        float intensity_norm = static_cast<float>(*iter_intensity) / max_intensity;
        marker->colors.push_back(color_fn(intensity_norm));
      }
    } else if (mode == CLUSTER_COLOR) {
      const auto cluster_color = generateDistinctColor(marker->id);
      for (; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        marker->colors.push_back(cluster_color);
      }
    } else {
      for (; iter_intensity != iter_intensity.end(); ++iter_intensity) {
        marker->colors.push_back(default_color);
      }
    }
    add_marker(marker);
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsWithFeatureDisplay, rviz_common::Display)
