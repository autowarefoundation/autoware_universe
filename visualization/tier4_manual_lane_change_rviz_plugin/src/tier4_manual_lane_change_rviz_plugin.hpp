// Copyright 2025 Autoware Foundation
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

#ifndef TIER4_MANUAL_LANE_CHANGE_RVIZ_PLUGIN_HPP_
#define TIER4_MANUAL_LANE_CHANGE_RVIZ_PLUGIN_HPP_

#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <std_msgs/msg/string.hpp>
#include <tier4_external_api_msgs/srv/set_preferred_lane.hpp>

#include <memory>

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
  rclcpp::Client<tier4_external_api_msgs::srv::SetPreferredLane>::SharedPtr client_;
  QTimer * timer_;
};

#endif  // TIER4_MANUAL_LANE_CHANGE_RVIZ_PLUGIN_HPP_
