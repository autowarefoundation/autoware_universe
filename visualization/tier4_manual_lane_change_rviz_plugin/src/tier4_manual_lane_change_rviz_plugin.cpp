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

#include "tier4_manual_lane_change_rviz_plugin.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <memory>
PLUGINLIB_EXPORT_CLASS(ManualLaneSelection, rviz_common::Panel)

using tier4_external_api_msgs::srv::SetPreferredLane;

ManualLaneSelection::ManualLaneSelection(QWidget * parent) : rviz_common::Panel(parent)
{
  auto * layout = new QHBoxLayout;

  left_button_ = new QPushButton("← Left");
  auto_button_ = new QPushButton(" Auto ");
  right_button_ = new QPushButton("Right →");

  layout->addWidget(left_button_);
  layout->addWidget(auto_button_);
  layout->addWidget(right_button_);
  setLayout(layout);

  node_ = std::make_shared<rclcpp::Node>("tier4_manual_lane_change_rviz_plugin");

  // Create the service client
  client_ = node_->create_client<SetPreferredLane>(
    "/planning/mission_planning/manual_lane_change_handler/set_preferred_lane");

  // Connect button signals
  connect(left_button_, &QPushButton::clicked, this, [this]() { send_lane_change_request(0); });
  connect(auto_button_, &QPushButton::clicked, this, [this]() { send_lane_change_request(2); });
  connect(right_button_, &QPushButton::clicked, this, [this]() { send_lane_change_request(1); });

  // Optional: Timer to spin the node (needed for service responses)
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  timer_->start(100);  // ms
}

void ManualLaneSelection::send_lane_change_request(uint8_t direction)
{
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    qWarning("ManualLaneSelection: Service not available");
    return;
  }

  auto request = std::make_shared<SetPreferredLane::Request>();
  request->lane_change_direction = direction;

  // Async call
  auto future = client_->async_send_request(
    request, [direction](rclcpp::Client<SetPreferredLane>::SharedFuture response) {
      const auto & res = response.get()->status;
      qInfo(
        "ManualLaneSelection: Sent %s lane change -> Success: %d, Message: %s",
        direction == SetPreferredLane::Request::LEFT    ? "LEFT"
        : direction == SetPreferredLane::Request::RIGHT ? "RIGHT"
                                                        : "AUTO",
        res.success, res.message.c_str());
    });
}
