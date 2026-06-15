// Copyright 2026 The Autoware Contributors
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

#ifndef DRIVING_MODE_MANAGER_HPP_
#define DRIVING_MODE_MANAGER_HPP_

#include "core/init.hpp"
#include "core/main.hpp"

#include <autoware_driving_mode_manager/plugin.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <pluginlib/class_loader.hpp>

#include <memory>

namespace autoware::driving_mode_manager
{

class DrivingModeManager : public rclcpp::Node
{
public:
  explicit DrivingModeManager(const rclcpp::NodeOptions & options);

private:
  diagnostic_updater::Updater diag_;
  rclcpp::TimerBase::SharedPtr timer_;

  void on_timer_init();
  void on_timer_main();

  pluginlib::ClassLoader<Plugin> loader_;
  std::unique_ptr<ManagerInit> init_;
  std::unique_ptr<ManagerMain> main_;
};

}  // namespace autoware::driving_mode_manager

#endif  // DRIVING_MODE_MANAGER_HPP_
