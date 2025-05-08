// Copyright 2025 Tier IV, Inc.
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

#ifndef COLORED_POSE_WITH_COVARIANCE_HISTORY__COLORED_POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_
#define COLORED_POSE_WITH_COVARIANCE_HISTORY__COLORED_POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/message_filter_display.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <deque>
#include <memory>
#include <string>

namespace rviz_rendering
{
class BillboardLine;
}  // namespace rviz_rendering
namespace rviz_common::properties
{
class ColorProperty;
class FloatProperty;
class IntProperty;
class BoolProperty;
class EnumProperty;
class RosTopicProperty;
}  // namespace rviz_common::properties

namespace rviz_plugins
{
class ColoredPoseWithCovarianceHistory
: public rviz_common::MessageFilterDisplay<geometry_msgs::msg::PoseWithCovarianceStamped>
{
  Q_OBJECT

public:
  ColoredPoseWithCovarianceHistory();
  ~ColoredPoseWithCovarianceHistory() override;

  enum class ValueType { Int32, Float32 };

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void update(float wall_dt, float ros_dt) override;

private Q_SLOTS:
  void update_value_type();
  void update_value_topic();
  void update_visualization();

private:  // NOLINT : suppress redundancy warnings
          //          followings cannot be declared with the Q_SLOTS macro
  void subscribe() override;
  void unsubscribe() override;
  void processMessage(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message) override;
  void process_int32_message(
    const autoware_internal_debug_msgs::msg::Int32Stamped::ConstSharedPtr message);
  void process_float32_message(
    const autoware_internal_debug_msgs::msg::Float32Stamped::ConstSharedPtr message);

  void update_history();

  Ogre::ColourValue get_color_from_value(double value);

  struct pose_with_value
  {
    geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose;
    double value;
    pose_with_value(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr & pose, double value)
    : pose(pose), value(value)
    {
    }
  };

  std::deque<pose_with_value> history_;
  std::string target_frame_;
  ValueType current_value_type_;
  std::unique_ptr<rviz_rendering::BillboardLine> lines_;
  rclcpp::Time last_stamp_;
  double last_value_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Int32Stamped>::SharedPtr int32_sub_;
  rclcpp::Subscription<autoware_internal_debug_msgs::msg::Float32Stamped>::SharedPtr float32_sub_;

  rviz_common::properties::EnumProperty * property_value_type_;
  rviz_common::properties::RosTopicProperty * property_value_topic_;
  rviz_common::properties::IntProperty * property_buffer_size_;
  rviz_common::properties::BoolProperty * property_line_view_;
  rviz_common::properties::FloatProperty * property_line_width_;
  rviz_common::properties::FloatProperty * property_line_alpha_;
  rviz_common::properties::ColorProperty * property_line_min_color_;
  rviz_common::properties::ColorProperty * property_line_max_color_;
  rviz_common::properties::ColorProperty * property_line_less_color_;
  rviz_common::properties::ColorProperty * property_line_greater_color_;
  rviz_common::properties::BoolProperty * property_auto_min_max_;
  rviz_common::properties::FloatProperty * property_min_value_;
  rviz_common::properties::FloatProperty * property_max_value_;
};

}  // namespace rviz_plugins

#endif  // COLORED_POSE_WITH_COVARIANCE_HISTORY__COLORED_POSE_WITH_COVARIANCE_HISTORY_DISPLAY_HPP_
