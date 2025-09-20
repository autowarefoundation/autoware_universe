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
// cspell:ignore COLORMAPS colormap
#include "tier4_perception_rviz_plugin/object_detection/detected_objects_wf_display.hpp"

#include "tier4_perception_rviz_plugin/object_detection/detected_objects_wf_helper.hpp"

#include <QObject>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
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
  m_show_text_marker_property{
    "Show Cluster Info", true, "Display the text marker of cluster", this},
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
  m_colorbar_widget = std::make_unique<ColorbarWidget>();

  m_colorbar_widget->setColorbarImage(m_colorbar_image);  // Set the generated colorbar image
  m_colorbar_widget->setMinMax(0.0f, m_intensity_color_scale_max.getFloat());
  m_colorbar_widget->setWindowFlags(Qt::Tool);

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

  if (m_show_text_marker_property.getBool()) {
    for (const auto & feature_object : msg->feature_objects) {
      const auto & cluster = feature_object.feature.cluster;

      // Convert cluster to pcl::PointCloud
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(cluster, cloud);

      // 2. Voxel grid the cluster for analysis
      pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setLeafSize(0.3f, 0.3f, 10000.f);  // Use the same size as in clustering node
      voxel_grid.setInputCloud(cloud.makeShared());
      voxel_grid.setMinimumPointsNumberPerVoxel(1);
      voxel_grid.setSaveLeafLayout(true);
      voxel_grid.filter(*voxel_map_ptr);

      int total_points = static_cast<int>(cloud.size());
      int voxel_count = static_cast<int>(voxel_map_ptr->points.size());

      float density = voxel_count > 0 ? static_cast<float>(total_points) / voxel_count : 0.0f;

      // Step 1: Compute the centroid
      geometry_msgs::msg::Point centroid{};
      centroid.x = 0.0;
      centroid.y = 0.0;
      centroid.z = 0.0;
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(cluster, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(cluster, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(cluster, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        centroid.x += *iter_x;
        centroid.y += *iter_y;
        centroid.z += *iter_z;
      }
      // Normalize by the number of points
      centroid.x /= static_cast<float>(total_points);
      centroid.y /= static_cast<float>(total_points);
      centroid.z /= static_cast<float>(total_points);

      // Step 2: Create a text marker
      auto text_marker = std::make_shared<visualization_msgs::msg::Marker>();
      text_marker->header = msg->header;
      text_marker->ns = "cluster_info";
      text_marker->id = id++;  // Use a unique ID
      text_marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker->action = visualization_msgs::msg::Marker::ADD;
      text_marker->pose.orientation.w = 1.0;
      text_marker->pose.position = centroid;
      text_marker->pose.position.z += 2.0;  // Slightly above the cluster
      text_marker->pose.orientation.w = 1.0;
      text_marker->scale.z = 0.5;  // Font size
      text_marker->color.r = 1.0;
      text_marker->color.g = 1.0;
      text_marker->color.b = 1.0;
      text_marker->color.a = 1.0;
      text_marker->lifetime = rclcpp::Duration::from_seconds(0.15);

      // Step 3: Set the displayed text
      // Compute bounding box
      float min_x = std::numeric_limits<float>::max();
      float max_x = std::numeric_limits<float>::lowest();
      float min_y = min_x, max_y = max_x;
      float min_z = min_x, max_z = max_x;

      float intensity_sum = 0.0f;

      sensor_msgs::PointCloud2ConstIterator<float> iter_x_m(cluster, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y_m(cluster, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z_m(cluster, "z");
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_intensity_m(cluster, "intensity");

      for (; iter_x_m != iter_x_m.end(); ++iter_x_m, ++iter_y_m, ++iter_z_m, ++iter_intensity_m) {
        float x = *iter_x_m, y = *iter_y_m, z = *iter_z_m;
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
        intensity_sum += static_cast<float>(*iter_intensity_m);
      }

      float avg_intensity = intensity_sum / total_points;
      float dx = max_x - min_x, dy = max_y - min_y, dz = max_z - min_z;
      float volume = dx * dy * dz;

      std::ostringstream oss;
      // 3. Render text in RViz
      oss << "Pts: " << total_points << "\nVoxels: " << voxel_count << "\nDensity: " << std::fixed
          << std::setprecision(2) << density << "\nVolume: " << std::fixed << std::setprecision(1)
          << volume << "\nDims: " << dx << " x " << dy << " x " << dz
          << "\nAvg Intensity: " << std::fixed << std::setprecision(2) << avg_intensity;

      text_marker->text = oss.str();
      // Step 4: Add the marker
      add_marker(text_marker);
    }
  }
}

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

// Export the plugin
#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  autoware::rviz_plugins::object_detection::DetectedObjectsWithFeatureDisplay, rviz_common::Display)
