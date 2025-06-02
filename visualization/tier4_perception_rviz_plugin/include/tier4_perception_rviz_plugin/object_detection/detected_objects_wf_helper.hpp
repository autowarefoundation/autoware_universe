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
#ifndef TIER4_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WF_HELPER_HPP_
#define TIER4_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WF_HELPER_HPP_
// cspell:ignore Viridis Parula COLORMAPS VIRIDIS PARULA colormap parula
#include "tier4_perception_rviz_plugin/visibility_control.hpp"

#include <QColor>
#include <QPainter>
#include <QString>
#include <QWidget>

#include <std_msgs/msg/detail/color_rgba__struct.hpp>

#include <vector>

namespace autoware
{
namespace rviz_plugins
{
namespace object_detection
{
std_msgs::msg::ColorRGBA generateDistinctColor(size_t idx);

std_msgs::msg::ColorRGBA colormapJet(float value_normalized);
std_msgs::msg::ColorRGBA colormapViridis(float v);
std_msgs::msg::ColorRGBA colormapHSV(float v);
std_msgs::msg::ColorRGBA colormapRed(float v);
std_msgs::msg::ColorRGBA colormapGray(float value);
std_msgs::msg::ColorRGBA colormapTurbo(float x);
std_msgs::msg::ColorRGBA colormapRainbow(float v);
std_msgs::msg::ColorRGBA colormapParula(float v);

class ColorbarWidget : public QWidget
{
public:
  explicit ColorbarWidget(QWidget * parent = nullptr);
  void setColorbarImage(const QImage & image);
  void setMinMax(float min_value, float max_value);

protected:
  void paintEvent(QPaintEvent * event) override;

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

// Type alias for colormap functions
using ColormapFuncType = std::function<std_msgs::msg::ColorRGBA(float)>;

// Static const array of colormap functions (Initialized once)
// Ensure these enum values match the order and options in the constructor
enum ColormapEnum {
  JET_CM = 0,
  HSV_CM,
  VIRIDIS_CM,
  RED_CM,
  GRAY_CM,
  TURBO_CM,
  RAINBOW_CM,
  PARULA_CM,
  NUM_COLORMAPS
};
enum ColorModeEnum { FLAT_COLOR = 0, INTENSITY_COLOR, CLUSTER_COLOR };
struct ColormapInfo
{
  const std::array<const char *, NUM_COLORMAPS> & names;
  const std::array<ColormapFuncType, NUM_COLORMAPS> & functions;

  ColormapFuncType getFunctionSafe(int index) const
  {
    if (index < 0 || index >= static_cast<int>(functions.size())) {
      return functions[0];  // Fallback to default (JET)
    }
    return functions[index];
  }
};

const ColormapInfo & getColormapInfo();

}  // namespace object_detection
}  // namespace rviz_plugins
}  // namespace autoware

#endif  // TIER4_PERCEPTION_RVIZ_PLUGIN__OBJECT_DETECTION__DETECTED_OBJECTS_WF_HELPER_HPP_
