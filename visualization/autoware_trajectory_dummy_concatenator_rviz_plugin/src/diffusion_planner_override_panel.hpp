// Copyright 2026 TIER IV, Inc.
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

#ifndef DIFFUSION_PLANNER_OVERRIDE_PANEL_HPP_
#define DIFFUSION_PLANNER_OVERRIDE_PANEL_HPP_

#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <std_msgs/msg/bool.hpp>

namespace autoware::trajectory_dummy_concatenator_rviz_plugin
{

class DiffusionPlannerOverridePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DiffusionPlannerOverridePanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void onToggleOverride();
  void onTopicEdited();

private:
  void updateButtonStyle();
  void publishOverride(bool value);

  QPushButton * toggle_button_;
  QLineEdit * topic_edit_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_override_;

  bool override_enabled_{false};

  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr rviz_ros_node_;
};

}  // namespace autoware::trajectory_dummy_concatenator_rviz_plugin

#endif  // DIFFUSION_PLANNER_OVERRIDE_PANEL_HPP_
