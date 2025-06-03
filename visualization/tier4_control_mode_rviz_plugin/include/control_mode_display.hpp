// Copyright 2025 TIER IV, Inc.
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

#ifndef CONTROL_MODE_DISPLAY_HPP_
#define CONTROL_MODE_DISPLAY_HPP_

#include <QLabel>
#include <QVBoxLayout>
#include <QWidget>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>

#include <memory>
#include <string>

namespace rviz_plugins
{

class ControlModeDisplay : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ControlModeDisplay(QWidget * parent = nullptr);
  ~ControlModeDisplay() override;

protected:
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void processMessage(const autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr msg);
  std::string getModeString(uint8_t mode) const;
  void subscribe();
  void unsubscribe();

  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr subscription_;
  autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr last_msg_;
  QLabel * mode_label_;
};

}  // namespace rviz_plugins

#endif  // CONTROL_MODE_DISPLAY_HPP_
