#ifndef LANE_CHANGE_PANEL_HPP
#define LANE_CHANGE_PANEL_HPP

#include <QPushButton>
#include <QHBoxLayout>
#include <QTimer>

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <autoware_planning_msgs/srv/set_lane_change_override.hpp>

class ManualLaneSelection : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ManualLaneSelection(QWidget *parent = nullptr);

private:
  void send_lane_change_request(uint8_t direction);

  QPushButton *left_button_;
  QPushButton *auto_button_;
  QPushButton *right_button_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<autoware_planning_msgs::srv::SetLaneChangeOverride>::SharedPtr client_;
  QTimer *timer_;
};

#endif // LANE_CHANGE_PANEL_HPP
