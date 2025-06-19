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

#include "autoware_command_mode_decider/status.hpp"

#include <autoware_command_mode_types/modes.hpp>

#include <sstream>
#include <string>
#include <vector>

namespace autoware::command_mode_decider
{

CommandModeStatusItem::CommandModeStatusItem(uint16_t mode)
{
  message_.mode = mode;
  message_.mrm = StatusMessage::NORMAL;
  message_.transition_completed = false;
  message_.transition = false;
  message_.request = false;
  message_.vehicle_selected = false;
  message_.network_selected = false;
  message_.command_selected = false;
  message_.command_exclusive = false;
  message_.command_enabled = false;
  message_.command_disabled = false;
}

CommandModeStatusItem::CommandModeStatusItem(const StatusMessage & message)
{
  message_ = message;
}

bool CommandModeStatusItem::is_completed() const
{
  return !message_.transition && is_vehicle_ready();
}

bool CommandModeStatusItem::is_vehicle_ready() const
{
  return message_.vehicle_selected && is_network_ready();
}

bool CommandModeStatusItem::is_network_ready() const
{
  return message_.network_selected && message_.command_selected && message_.command_exclusive;
}

void CommandModeStatusTable::init(const std::vector<uint16_t> & modes)
{
  for (const auto & mode : modes) {
    items_[mode] = ModeItem();
  }
}

bool CommandModeStatusTable::ready() const
{
  // available and drivable are not necessary.
  for (const auto & [mode, item] : items_) {
    if (!item.status.stamp) return false;
  }
  return true;
}

void CommandModeStatusTable::set(const StatusMessage & item, const rclcpp::Time & stamp)
{
  const auto iter = items_.find(item.mode);
  if (iter != items_.end()) {
    iter->second.status.data = CommandModeStatusItem(item);
    iter->second.status.stamp = stamp;
  }
}

void CommandModeStatusTable::set(const AvailabilityMessage & item, const rclcpp::Time & stamp)
{
  const auto iter = items_.find(item.mode);
  if (iter != items_.end()) {
    iter->second.available.data = item.available;
    iter->second.available.stamp = stamp;
  }
}

void CommandModeStatusTable::set(uint16_t mode, bool drivable, const rclcpp::Time & stamp)
{
  const auto iter = items_.find(mode);
  if (iter != items_.end()) {
    iter->second.drivable.data = drivable;
    iter->second.drivable.stamp = stamp;
  }
}

void CommandModeStatusTable::check_timeout(const rclcpp::Time & now)
{
  const auto timeout = [](const rclcpp::Time & now, const std::optional<rclcpp::Time> & stamp) {
    if (!stamp) return false;
    const auto duration = (now - *stamp).seconds();
    return 1.0 < duration;
  };
  for (auto & [mode, item] : items_) {
    if (timeout(now, item.status.stamp)) {
      item.status.data = CommandModeStatusItem(mode);
      item.status.stamp = std::nullopt;
    }
    if (timeout(now, item.available.stamp)) {
      item.available.data = false;
      item.available.stamp = std::nullopt;
    }
    if (timeout(now, item.drivable.stamp)) {
      item.drivable.data = false;
      item.drivable.stamp = std::nullopt;
    }
  }
}

bool CommandModeStatusTable::available(uint16_t mode, bool is_manual) const
{
  const auto iter = items_.find(mode);
  if (iter == items_.end()) {
    return false;
  }
  return iter->second.available.data && (iter->second.drivable.data || is_manual);
}

const CommandModeStatusItem & CommandModeStatusTable::get(uint16_t mode) const
{
  const auto iter = items_.find(mode);
  if (iter == items_.end()) {
    return empty_status_;
  }
  return iter->second.status.data;
}

std::string CommandModeStatusTable::debug() const
{
  std::stringstream ss;
  ss << "===== command mode status table =====" << std::endl;
  for (const auto & [mode, item] : items_) {
    ss << mode << ": " << item.available.data << ", " << item.drivable.data << std::endl;
  }
  return ss.str();
}

}  // namespace autoware::command_mode_decider
