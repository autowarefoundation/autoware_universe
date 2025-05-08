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

#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{


  class ColorbarWidget : public QWidget
  {
  public:
  explicit ColorbarWidget(QWidget *parent = nullptr) : QWidget(parent) {
        setFixedSize(100, 300);
        setWindowTitle("Colorbar");
        
        m_font = QFont("Arial", 9);
        m_font.setBold(true);

        // Set default colors
        m_textColor = Qt::white;
        m_tickColor = Qt::white;
        m_backgroundColor = QColor(0, 0, 0, 180);  // Semi-transparent dark background
      }
  
      void setColorbarImage(const QImage &image) {
          m_colorbar_image = image;
          update();  // Trigger a repaint
      }
      void setMinMax(float min_value, float max_value)
      {
        m_min_value = min_value;
        m_max_value = max_value;
        update();
      }
      
  protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // Draw colorbar with margin
    int colorbarWidth = 30;
    int colorbarX = 10;
    int colorbarY = 40;
    int colorbarHeight = height() - 80;
    
    painter.drawImage(QRect(colorbarX, colorbarY, colorbarWidth, colorbarHeight), 
                     m_colorbar_image.scaled(colorbarWidth, colorbarHeight));

    // Draw frame around colorbar
    painter.setPen(QPen(m_tickColor, 1.5));
    painter.drawRect(colorbarX, colorbarY, colorbarWidth, colorbarHeight);

    // Set font
    painter.setFont(m_font);
    painter.setPen(m_textColor);

    // Draw title
    painter.drawText(QRect(0, 10, width(), 20), Qt::AlignCenter, m_title);

    // Draw ticks and labels
    int numTicks = 5;
    for (int i = 1; i < numTicks; ++i) {
      float value = m_min_value + (m_max_value - m_min_value) * (i / float(numTicks));
      int yPos = colorbarY + colorbarHeight - (i * colorbarHeight / numTicks);

      // Draw tick
      painter.setPen(QPen(m_tickColor, 2.0));
      painter.drawLine(colorbarX + colorbarWidth, yPos, colorbarX + colorbarWidth + 5, yPos);
      
      // Draw value label
      QString label = QString::number(value, 'f', 2);
      QRect labelRect(
        colorbarX + colorbarWidth + 10, yPos - 10, width() - colorbarX - colorbarWidth - 15, 20);

      painter.setPen(m_textColor);
      painter.drawText(labelRect, Qt::AlignLeft | Qt::AlignVCenter, label);
    }
    painter.setPen(QPen(m_tickColor, 2.5)); // Thicker for min/max
    
    // Min tick (bottom)
    int minY = colorbarY + colorbarHeight;
    painter.drawLine(colorbarX + colorbarWidth, minY, colorbarX + colorbarWidth + 10, minY);
    
    // Max tick (top)
    int maxY = colorbarY;
    painter.drawLine(colorbarX + colorbarWidth, maxY, colorbarX + colorbarWidth + 10, maxY);
    // Draw min/max labels (more prominent)
    painter.setPen(QPen(m_textColor, 1.5));
    QFont boldFont = m_font;
    boldFont.setPointSize(11);
    painter.setFont(boldFont);
    
    // Min value
    QRect minRect(colorbarX + colorbarWidth + 12, colorbarY + colorbarHeight - 15, 
      width() - colorbarX - colorbarWidth - 20, 25);
painter.drawText(minRect, Qt::AlignLeft | Qt::AlignVCenter, 
         QString::number(m_min_value, 'f', 2));

// Max value
QRect maxRect(colorbarX + colorbarWidth + 12, colorbarY - 15, 
      width() - colorbarX - colorbarWidth - 20, 25);
painter.drawText(maxRect, Qt::AlignLeft | Qt::AlignVCenter, 
         QString::number(m_max_value, 'f', 2));
  }
  
  private:
    QImage m_colorbar_image;
    float m_min_value{0.0f};
    float m_max_value{1.0f};
    QString m_title;
    QFont m_font;    
    QColor m_textColor;
    QColor m_tickColor;
    QColor m_backgroundColor;
  };


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


  DetectedObjectsWithFeatureDisplay(
    const std::string & default_topic = "detected_objects_with_feature");
  ~DetectedObjectsWithFeatureDisplay() override;
protected:
  void onInitialize() override;
  void reset() override;  
  void processMessage(DetectedObjectsWithFeature::ConstSharedPtr msg) override;

  void load(const rviz_common::Config & config) override
  {
    RosTopicDisplay::Display::load(config);
    m_marker_common.load(config);
  }
  void onPropertyChanged(const rviz_common::properties::Property * property);
  void update(float wall_dt, float ros_dt) override { m_marker_common.update(wall_dt, ros_dt); }
  double get_line_width() { return m_point_size_property.getFloat(); }
  double get_point_size() { return m_point_size_property.getFloat(); }
  QColor get_point_color() { return m_point_color_property.getColor(); }
  // Member variable to store the colorbar image
  QImage m_colorbar_image;
  // Method to generate the colorbar image
  void generateColorbar();
  void updateColorbarVisibility();
  void updateColormapAndColorbar();
  void updateColormapPropertiesVisibility();
private:
  // All rviz plugins should have this. Should be initialized with pointer to this class
  MarkerCommon m_marker_common;  
  // Property to set line width of object shape
  rviz_common::properties::FloatProperty m_line_width_property;
  // Property to set point size of cluster point cloud
  rviz_common::properties::FloatProperty m_point_size_property;
  // Property to set color mode of cluster point cloud
  rviz_common::properties::EnumProperty m_color_mode_property;
  // Property to set colormap of cluster point cloud
  rviz_common::properties::EnumProperty m_colormap_property;
  // Property to set point color of cluster point cloud
  rviz_common::properties::ColorProperty m_point_color_property;
  // Property to set intensity color threshold of cluster point cloud
  rviz_common::properties::FloatProperty m_intensity_color_scale_max;
  // Property to show colorbar
  rviz_common::properties::BoolProperty m_show_colorbar_property;

  // Property to set the default topic name
  std::string m_default_topic;

  void clear_markers() { m_marker_common.clearMarkers(); }

  void add_marker(visualization_msgs::msg::Marker::ConstSharedPtr marker_ptr)
  {
    m_marker_common.addMessage(marker_ptr);
  }
    // Colorbar widget
    ColorbarWidget* m_colorbar_widget;
};

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // AUTOWARE_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WITH_FEATURE_DISPLAY_HPP_
