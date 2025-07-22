#ifndef LANE_CHANGE_PANEL_HPP
#define LANE_CHANGE_PANEL_HPP

#include <QPushButton>
#include <QHBoxLayout>
#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class LaneChangePanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit LaneChangePanel(QWidget *parent = nullptr);

public Q_SLOTS:
  void onLeftClicked();
  void onRightClicked();

private:
  QPushButton *left_button_;
  QPushButton *right_button_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif // LANE_CHANGE_PANEL_HPP
