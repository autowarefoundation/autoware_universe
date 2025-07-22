#include "autoware_lane_change_panel/lane_change_panel.hpp"

LaneChangePanel::LaneChangePanel(QWidget *parent)
  : rviz_common::Panel(parent)
{
  auto *layout = new QHBoxLayout;

  left_button_ = new QPushButton("← Left");
  right_button_ = new QPushButton("Right →");

  layout->addWidget(left_button_);
  layout->addWidget(right_button_);
  setLayout(layout);

  node_ = std::make_shared<rclcpp::Node>("lane_change_panel");
  publisher_ = node_->create_publisher<std_msgs::msg::String>("/planning/lane_change_override", 10);

  connect(left_button_, &QPushButton::clicked, this, &LaneChangePanel::onLeftClicked);
  connect(right_button_, &QPushButton::clicked, this, &LaneChangePanel::onRightClicked);
}

void LaneChangePanel::onLeftClicked()
{
  std_msgs::msg::String msg;
  msg.data = "left";
  publisher_->publish(msg);
}

void LaneChangePanel::onRightClicked()
{
  std_msgs::msg::String msg;
  msg.data = "right";
  publisher_->publish(msg);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LaneChangePanel, rviz_common::Panel)
