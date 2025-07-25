#include "autoware_lane_change_panel/lane_change_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LaneChangePanel, rviz_common::Panel)

using autoware_planning_msgs::srv::SetLaneChangeOverride;

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

  // ✅ Create the service client
  client_ = node_->create_client<SetLaneChangeOverride>(
    "/planning/mission_planning/mission_planner/set_lane_change_override");

  // Connect button signals
  connect(left_button_, &QPushButton::clicked, this, &LaneChangePanel::onLeftClicked);
  connect(right_button_, &QPushButton::clicked, this, &LaneChangePanel::onRightClicked);

  // Optional: Timer to spin the node (needed for service responses)
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, [this]() { rclcpp::spin_some(node_); });
  timer_->start(100);  // ms
}

void LaneChangePanel::onLeftClicked()
{
  send_lane_change_request(0);  // 0 = LEFT
}

void LaneChangePanel::onRightClicked()
{
  send_lane_change_request(1);  // 1 = RIGHT
}

void LaneChangePanel::send_lane_change_request(uint8_t direction)
{
  if (!client_->wait_for_service(std::chrono::seconds(1))) {
    qWarning("LaneChangePanel: Service not available");
    return;
  }

  auto request = std::make_shared<SetLaneChangeOverride::Request>();
  request->lane_change_direction = direction;

  // Async call
  auto future = client_->async_send_request(request,
    [direction](rclcpp::Client<SetLaneChangeOverride>::SharedFuture response) {
      const auto & res = response.get()->status;
      qInfo("LaneChangePanel: Sent %s lane change -> Success: %d, Message: %s",
            direction == 0 ? "LEFT" : "RIGHT",
            res.success,
            res.message.c_str());
    });
}