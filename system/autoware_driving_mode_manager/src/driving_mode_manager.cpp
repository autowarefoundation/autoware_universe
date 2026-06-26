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

#include "driving_mode_manager.hpp"

#include "ros_interface.hpp"

#include <memory>
#include <stdexcept>
#include <string>

namespace autoware::driving_mode_manager
{

DrivingModeManager::DrivingModeManager(const rclcpp::NodeOptions & options)
: Node("driving_mode_manager", options),
  diag_(this, 0.1),
  loader_("autoware_driving_mode_manager", "autoware::driving_mode_manager::Plugin")
{
  rate_ = declare_parameter<double>("rate");
  diag_.setHardwareID("none");

  const auto plugin_name = declare_parameter<std::string>("plugin");
  if (!loader_.isClassAvailable(plugin_name)) {
    throw std::invalid_argument("unknown plugin: " + plugin_name);
  }
  const auto plugin = loader_.createSharedInstance(plugin_name);
  init_ = std::make_unique<ManagerInit>(std::make_unique<RosInterface>(this), plugin);

  const auto period = rclcpp::Rate(rate_).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer_init(); });
}

void DrivingModeManager::on_timer_init()
{
  init_->update();
  if (!init_->is_ready()) return;

  main_ = std::make_unique<ManagerMain>(*init_);
  init_.reset();

  const auto period = rclcpp::Rate(rate_).period();
  timer_->cancel();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer_main(); });
}

void DrivingModeManager::on_timer_main()
{
  main_->update();
}

}  // namespace autoware::driving_mode_manager

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::driving_mode_manager::DrivingModeManager)
