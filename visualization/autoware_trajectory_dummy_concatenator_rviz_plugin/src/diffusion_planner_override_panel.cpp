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

#include "diffusion_planner_override_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>
#include <rviz_common/config.hpp>
#include <rviz_common/display_context.hpp>

namespace autoware::trajectory_dummy_concatenator_rviz_plugin
{

DiffusionPlannerOverridePanel::DiffusionPlannerOverridePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout(this);

  // Topic input row
  {
    auto * row = new QHBoxLayout();
    row->addWidget(new QLabel("Topic:"));
    topic_edit_ = new QLineEdit("/planning/trajectory_selector/diffusion_planner_override");
    connect(
      topic_edit_, &QLineEdit::editingFinished, this,
      &DiffusionPlannerOverridePanel::onTopicEdited);
    row->addWidget(topic_edit_);
    layout->addLayout(row);
  }

  // Toggle button
  toggle_button_ = new QPushButton("OVERRIDE OFF\n(Diffusion Planner: ENABLED)");
  toggle_button_->setMinimumHeight(60);
  toggle_button_->setFont(QFont("Sans", 10, QFont::Bold));
  connect(
    toggle_button_, &QPushButton::clicked, this, &DiffusionPlannerOverridePanel::onToggleOverride);
  layout->addWidget(toggle_button_);

  setLayout(layout);
  updateButtonStyle();
}

void DiffusionPlannerOverridePanel::onInitialize()
{
  rviz_ros_node_ = getDisplayContext()->getRosNodeAbstraction();

  auto node = rviz_ros_node_.lock()->get_raw_node();
  pub_override_ = node->create_publisher<std_msgs::msg::Bool>(
    topic_edit_->text().toStdString(), rclcpp::QoS(1).reliable());
}

void DiffusionPlannerOverridePanel::onToggleOverride()
{
  override_enabled_ = !override_enabled_;
  publishOverride(override_enabled_);
  updateButtonStyle();
}

void DiffusionPlannerOverridePanel::onTopicEdited()
{
  if (rviz_ros_node_.expired()) return;

  auto node = rviz_ros_node_.lock()->get_raw_node();
  pub_override_ = node->create_publisher<std_msgs::msg::Bool>(
    topic_edit_->text().toStdString(), rclcpp::QoS(1).reliable());
}

void DiffusionPlannerOverridePanel::updateButtonStyle()
{
  if (override_enabled_) {
    toggle_button_->setText("OVERRIDE ON\n(Diffusion Planner: BLOCKED)");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #CC3333; color: white; border-radius: 6px; }");
  } else {
    toggle_button_->setText("OVERRIDE OFF\n(Diffusion Planner: ENABLED)");
    toggle_button_->setStyleSheet(
      "QPushButton { background-color: #336633; color: white; border-radius: 6px; }");
  }
}

void DiffusionPlannerOverridePanel::publishOverride(bool value)
{
  if (!pub_override_) return;

  std_msgs::msg::Bool msg;
  msg.data = value;
  pub_override_->publish(msg);
}

void DiffusionPlannerOverridePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("topic", topic_edit_->text());
}

void DiffusionPlannerOverridePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString topic;
  if (config.mapGetString("topic", &topic)) {
    topic_edit_->setText(topic);
  }
}

}  // namespace autoware::trajectory_dummy_concatenator_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::trajectory_dummy_concatenator_rviz_plugin::DiffusionPlannerOverridePanel,
  rviz_common::Panel)
