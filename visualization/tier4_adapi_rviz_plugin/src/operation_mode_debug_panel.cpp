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

#include "operation_mode_debug_panel.hpp"

#include <QGridLayout>
#include <QString>
#include <rviz_common/display_context.hpp>

#include <memory>
#include <string>

namespace tier4_adapi_rviz_plugins
{

OperationModeDebugPanel::OperationModeDebugPanel(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QGridLayout;
  setLayout(layout);

  operation_mode_label_ptr_ = new QLabel("INIT");
  operation_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  operation_mode_label_ptr_->setStyleSheet("border:1px solid black;");
  layout->addWidget(operation_mode_label_ptr_, 0, 0, 2, 1);

  auto_button_ptr_ = new QPushButton("AUTO");
  connect(auto_button_ptr_, SIGNAL(clicked()), SLOT(onClickAutonomous()));
  layout->addWidget(auto_button_ptr_, 0, 1);

  stop_button_ptr_ = new QPushButton("STOP");
  connect(stop_button_ptr_, SIGNAL(clicked()), SLOT(onClickStop()));
  layout->addWidget(stop_button_ptr_, 0, 2);

  local_button_ptr_ = new QPushButton("LOCAL");
  connect(local_button_ptr_, SIGNAL(clicked()), SLOT(onClickLocal()));
  layout->addWidget(local_button_ptr_, 1, 1);

  remote_button_ptr_ = new QPushButton("REMOTE");
  connect(remote_button_ptr_, SIGNAL(clicked()), SLOT(onClickRemote()));
  layout->addWidget(remote_button_ptr_, 1, 2);

  control_mode_label_ptr_ = new QLabel("INIT");
  control_mode_label_ptr_->setAlignment(Qt::AlignCenter);
  control_mode_label_ptr_->setStyleSheet("border:1px solid black;");
  layout->addWidget(control_mode_label_ptr_, 2, 0);

  enable_button_ptr_ = new QPushButton("Enable");
  connect(enable_button_ptr_, SIGNAL(clicked()), SLOT(onClickAutowareControl()));
  layout->addWidget(enable_button_ptr_, 2, 1);

  disable_button_ptr_ = new QPushButton("Disable");
  connect(disable_button_ptr_, SIGNAL(clicked()), SLOT(onClickDirectControl()));
  layout->addWidget(disable_button_ptr_, 2, 2);
}

void OperationModeDebugPanel::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  sub_operation_mode_ = raw_node_->create_subscription<OperationModeState>(
    "/api/operation_mode/state", rclcpp::QoS{1}.transient_local(),
    std::bind(&OperationModeDebugPanel::onOperationMode, this, std::placeholders::_1));

  client_change_to_autonomous_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_autonomous");

  client_change_to_stop_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_stop");

  client_change_to_local_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_local");

  client_change_to_remote_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/change_to_remote");

  client_enable_autoware_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/enable_autoware_control");

  client_enable_direct_control_ =
    raw_node_->create_client<ChangeOperationMode>("/api/operation_mode/disable_autoware_control");
}

template <typename T>
void callService(const rclcpp::Logger & logger, const typename rclcpp::Client<T>::SharedPtr client)
{
  auto req = std::make_shared<typename T::Request>();

  RCLCPP_DEBUG(logger, "client request");

  if (!client->service_is_ready()) {
    RCLCPP_DEBUG(logger, "client is unavailable");
    return;
  }

  client->async_send_request(req, [logger](typename rclcpp::Client<T>::SharedFuture result) {
    RCLCPP_DEBUG(
      logger, "Status: %d, %s", result.get()->status.code, result.get()->status.message.c_str());
  });
}

void OperationModeDebugPanel::onClickAutonomous()
{
  callService<ChangeOperationMode>(raw_node_->get_logger(), client_change_to_autonomous_);
}

void OperationModeDebugPanel::onClickStop()
{
  callService<ChangeOperationMode>(raw_node_->get_logger(), client_change_to_stop_);
}

void OperationModeDebugPanel::onClickLocal()
{
  callService<ChangeOperationMode>(raw_node_->get_logger(), client_change_to_local_);
}

void OperationModeDebugPanel::onClickRemote()
{
  callService<ChangeOperationMode>(raw_node_->get_logger(), client_change_to_remote_);
}

void OperationModeDebugPanel::onClickAutowareControl()
{
  callService<ChangeOperationMode>(raw_node_->get_logger(), client_enable_autoware_control_);
}

void OperationModeDebugPanel::onClickDirectControl()
{
  callService<ChangeOperationMode>(raw_node_->get_logger(), client_enable_direct_control_);
}

void OperationModeDebugPanel::onOperationMode(const OperationModeState::ConstSharedPtr msg)
{
  const auto updateLabel = [](QLabel * label, QString text, QString style) {
    label->setText(text);
    label->setStyleSheet(style);
  };

  const auto updateButton = [](QPushButton * button, bool available) {
    if (available) {
      button->setStyleSheet("color: black;");
    } else {
      button->setStyleSheet("color: white;");
    }
  };

  // Update current operation mode.

  QString state = "";
  QString style = "";

  switch (msg->mode) {
    case OperationModeState::AUTONOMOUS:
      state = "AUTONOMOUS";
      style = "background-color: #00FF00;";  // green
      break;

    case OperationModeState::LOCAL:
      state = "LOCAL";
      style = "background-color: #FFFF00;";  // yellow
      break;

    case OperationModeState::REMOTE:
      state = "REMOTE";
      style = "background-color: #FFFF00;";  // yellow
      break;

    case OperationModeState::STOP:
      state = "STOP";
      style = "background-color: #FFA500;";  // orange
      break;

    default:
      state = "UNKNOWN (" + QString::number(msg->mode) + ")";
      style = "background-color: #FF0000;";  // red
      break;
  }

  if (msg->is_in_transition) {
    state += "\n(TRANSITION)";
  }

  updateLabel(operation_mode_label_ptr_, state, style);

  // Update current control mode.

  if (msg->is_autoware_control_enabled) {
    updateLabel(control_mode_label_ptr_, "Enable", "background-color: #00FF00;");  // green
  } else {
    updateLabel(control_mode_label_ptr_, "Disable", "background-color: #FFFF00;");  // yellow
  }

  // Update operation mode available.

  updateButton(auto_button_ptr_, msg->is_autonomous_mode_available);
  updateButton(stop_button_ptr_, msg->is_stop_mode_available);
  updateButton(local_button_ptr_, msg->is_local_mode_available);
  updateButton(remote_button_ptr_, msg->is_remote_mode_available);

  // Update control mode available.

  updateButton(enable_button_ptr_, true);
  updateButton(disable_button_ptr_, true);
}

}  // namespace tier4_adapi_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tier4_adapi_rviz_plugins::OperationModeDebugPanel, rviz_common::Panel)
