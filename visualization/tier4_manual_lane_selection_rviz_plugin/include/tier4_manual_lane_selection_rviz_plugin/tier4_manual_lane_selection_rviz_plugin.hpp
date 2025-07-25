#ifndef TIER4_MANUAL_LANE_SELECTION_RVIZ_PLUGIN__TIER4_MANUAL_LANE_SELECTION_RVIZ_PLUGIN_HPP_
#define TIER4_MANUAL_LANE_SELECTION_RVIZ_PLUGIN__TIER4_MANUAL_LANE_SELECTION_RVIZ_PLUGIN_HPP_

#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_planning_msgs/srv/set_lane_change_override.hpp>
#include <std_msgs/msg/string.hpp>

class ManualLaneSelection : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit ManualLaneSelection(QWidget * parent = nullptr);

private:
  void send_lane_change_request(uint8_t direction);

  QPushButton * left_button_;
  QPushButton * auto_button_;
  QPushButton * right_button_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<autoware_planning_msgs::srv::SetLaneChangeOverride>::SharedPtr client_;
  QTimer * timer_;
};

#endif  // TIER4_MANUAL_LANE_SELECTION_RVIZ_PLUGIN__TIER4_MANUAL_LANE_SELECTION_RVIZ_PLUGIN_HPP_
