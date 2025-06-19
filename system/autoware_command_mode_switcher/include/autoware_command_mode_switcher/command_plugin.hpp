//  Copyright 2025 The Autoware Contributors
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

#ifndef AUTOWARE_COMMAND_MODE_SWITCHER__COMMAND_PLUGIN_HPP_
#define AUTOWARE_COMMAND_MODE_SWITCHER__COMMAND_PLUGIN_HPP_

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace autoware::command_mode_switcher
{

enum class MrmState {
  Normal,
  Operating,
  Succeeded,
  Failed,
};

struct SourceState
{
  bool enabled;
  bool disabled;
};

class CommandPlugin
{
public:
  virtual ~CommandPlugin() = default;
  virtual uint16_t mode() const = 0;
  virtual uint16_t source() const = 0;
  virtual bool autoware_control() const = 0;
  virtual void initialize() = 0;

  virtual SourceState update_source_state(bool request);
  virtual MrmState update_mrm_state() { return MrmState::Normal; }
  virtual bool get_transition_completed() { return true; }

  void construct(rclcpp::Node * node) { node_ = node; }
  void set_plugin_name(const std::string & plugin_name) { plugin_name_ = plugin_name; }
  std::string expand_param(const std::string & param_name) const
  {
    return "plugin_parameters." + plugin_name_ + "." + param_name;
  }

protected:
  rclcpp::Node * node_;

private:
  std::string plugin_name_;
};

}  // namespace autoware::command_mode_switcher

#endif  // AUTOWARE_COMMAND_MODE_SWITCHER__COMMAND_PLUGIN_HPP_
