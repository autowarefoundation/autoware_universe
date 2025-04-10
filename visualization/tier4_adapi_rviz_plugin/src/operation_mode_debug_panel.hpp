//
//  Copyright 2020 TIER IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#ifndef OPERATION_MODE_DEBUG_PANEL_HPP_
#define OPERATION_MODE_DEBUG_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>

namespace tier4_adapi_rviz_plugins
{

class OperationModeDebugPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit OperationModeDebugPanel(QWidget * parent = nullptr);
  void onInitialize() override;

public Q_SLOTS:  // NOLINT for Qt
  void onClickAutonomous();
  void onClickStop();
  void onClickLocal();
  void onClickRemote();
  void onClickAutowareControl();
  void onClickDirectControl();

protected:
  using OperationModeState = autoware_adapi_v1_msgs::msg::OperationModeState;
  using ChangeOperationMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;

  QLabel * operation_mode_label_ptr_{nullptr};
  QPushButton * stop_button_ptr_{nullptr};
  QPushButton * auto_button_ptr_{nullptr};
  QPushButton * local_button_ptr_{nullptr};
  QPushButton * remote_button_ptr_{nullptr};

  QLabel * control_mode_label_ptr_{nullptr};
  QPushButton * enable_button_ptr_{nullptr};
  QPushButton * disable_button_ptr_{nullptr};

  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Subscription<OperationModeState>::SharedPtr sub_operation_mode_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_autonomous_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_stop_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_local_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_change_to_remote_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_enable_autoware_control_;
  rclcpp::Client<ChangeOperationMode>::SharedPtr client_enable_direct_control_;

  void onOperationMode(const OperationModeState::ConstSharedPtr msg);
  void changeOperationMode(const rclcpp::Client<ChangeOperationMode>::SharedPtr client);
};

}  // namespace tier4_adapi_rviz_plugins

#endif  // OPERATION_MODE_DEBUG_PANEL_HPP_
