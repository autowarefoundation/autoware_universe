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

#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware_driving_mode_manager/plugin.hpp>
#include <pluginlib/class_loader.hpp>

#include <memory>

namespace autoware::driving_mode_manager
{

class DrivingModeManager : public autoware::agnocast_wrapper::Node
{
public:
  explicit DrivingModeManager(const rclcpp::NodeOptions & options);

private:
  autoware::agnocast_wrapper::diagnostic_updater::Updater diag_;
  AUTOWARE_TIMER_PTR timer_;
  double rate_;

  void on_timer_init();
  void on_timer_main();
  void on_diag(diagnostic_updater::DiagnosticStatusWrapper & stat);

  pluginlib::ClassLoader<Plugin> loader_;
  std::unique_ptr<ManagerInit> init_;
  std::unique_ptr<ManagerMain> main_;
};

}  // namespace autoware::driving_mode_manager

#endif  // DRIVING_MODE_MANAGER_HPP_
