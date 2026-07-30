// Copyright 2022 Tier IV, Inc.
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

#ifndef AUTOWARE__RTC_INTERFACE__RTC_INTERFACE_HPP_
#define AUTOWARE__RTC_INTERFACE__RTC_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "tier4_rtc_msgs/msg/auto_mode_status.hpp"
#include "tier4_rtc_msgs/msg/command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_command.hpp"
#include "tier4_rtc_msgs/msg/cooperate_response.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status.hpp"
#include "tier4_rtc_msgs/msg/cooperate_status_array.hpp"
#include "tier4_rtc_msgs/msg/module.hpp"
#include "tier4_rtc_msgs/msg/state.hpp"
#include "tier4_rtc_msgs/srv/auto_mode.hpp"
#include "tier4_rtc_msgs/srv/cooperate_commands.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace autoware::rtc_interface
{
using tier4_rtc_msgs::msg::AutoModeStatus;
using tier4_rtc_msgs::msg::Command;
using tier4_rtc_msgs::msg::CooperateCommand;
using tier4_rtc_msgs::msg::CooperateResponse;
using tier4_rtc_msgs::msg::CooperateStatus;
using tier4_rtc_msgs::msg::CooperateStatusArray;
using tier4_rtc_msgs::msg::Module;
using tier4_rtc_msgs::msg::State;
using tier4_rtc_msgs::srv::AutoMode;
using tier4_rtc_msgs::srv::CooperateCommands;
using unique_identifier_msgs::msg::UUID;

/// @brief Publishes the decision status of a planning module and receives execution commands.
///
/// @tparam NodeT the node type owning this interface. Only the instantiations explicitly provided
/// by this package can be linked against: `rclcpp::Node` (aliased as `RTCInterface` below) and
/// `autoware::agnocast_wrapper::Node`.
template <class NodeT = rclcpp::Node>
class BasicRTCInterface
{
public:
  BasicRTCInterface(NodeT * node, const std::string & name, const bool enable_rtc = true);
  void publishCooperateStatus(const rclcpp::Time & stamp);
  /// @brief update the cooperate status of the module identified by the given UUID
  /// @param[in] uuid unique ID of the module
  /// @param[in] safe new value for the "safe" field
  /// @param[in] state new value for the "state" field
  /// @param[in] start_distance new value for the "start_distance" field
  /// @param[in] finish_distance new value for the "finish_distance" field
  /// @param[in] stamp new value for the "stamp" field
  /// @param[in] requested new value for the "requested" field (default to false)
  /// @param[in] override_rtc_auto_mode optional value of the "auto_mode" field (if not set, the
  /// "auto_mode" is only true if enable_rtc was false at initialization)
  void updateCooperateStatus(
    const UUID & uuid, const bool safe, const uint8_t state, const double start_distance,
    const double finish_distance, const rclcpp::Time & stamp, const bool requested = false,
    const std::optional<bool> & override_rtc_auto_mode = std::nullopt);
  void removeCooperateStatus(const UUID & uuid);
  void removeExpiredCooperateStatus();
  void clearCooperateStatus();
  bool isActivated(const UUID & uuid) const;
  bool isForceActivated(const UUID & uuid) const;
  bool isForceDeactivated(const UUID & UUID) const;
  bool isRegistered(const UUID & uuid) const;
  bool isRTCEnabled(const UUID & uuid) const;
  bool isTerminated(const UUID & uuid) const;
  void lockCommandUpdate();
  void unlockCommandUpdate();
  void print() const;

private:
  void onCooperateCommandService(
    const CooperateCommands::Request::SharedPtr request,
    const CooperateCommands::Response::SharedPtr responses);
  void onAutoModeService(
    const AutoMode::Request::SharedPtr request, const AutoMode::Response::SharedPtr response);
  void onTimer();
  std::vector<CooperateResponse> validateCooperateCommands(
    const std::vector<CooperateCommand> & commands);
  void updateCooperateCommandStatus(const std::vector<CooperateCommand> & commands);
  void removeStoredCommand(const UUID & uuid);
  rclcpp::Logger getLogger() const;
  bool isLocked() const;

  // The publisher handle type depends on NodeT: rclcpp::Node yields rclcpp::Publisher, while
  // autoware::agnocast_wrapper::Node yields its own publisher wrapper. Deduce it from the node
  // instead of naming it, so that this header stays independent of the wrapper.
  using CooperateStatusPublisherPtr =
    decltype(std::declval<NodeT *>()->template create_publisher<CooperateStatusArray>(
      std::string{}, 1));
  using AutoModeStatusPublisherPtr =
    decltype(std::declval<NodeT *>()->template create_publisher<AutoModeStatus>(std::string{}, 1));

  CooperateStatusPublisherPtr pub_statuses_;
  AutoModeStatusPublisherPtr pub_auto_mode_status_;
  // The service and timer handles are only held to keep them alive; they are never dereferenced
  // after construction. Holding them type-erased keeps the layout of this class independent of
  // NodeT, which matters because it is held by value in several planning modules.
  std::shared_ptr<void> srv_commands_;
  std::shared_ptr<void> srv_auto_mode_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::shared_ptr<void> timer_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  Module module_;
  CooperateStatusArray registered_status_;
  std::vector<CooperateCommand> stored_commands_;
  bool is_auto_mode_enabled_;
  bool is_locked_;

  std::string cooperate_status_namespace_ = "/planning/cooperate_status";
  std::string auto_mode_status_namespace_ = "/planning/auto_mode_status";
  std::string cooperate_commands_namespace_ = "/planning/cooperate_commands";
  std::string enable_auto_mode_namespace_ = "/planning/enable_auto_mode";

  mutable std::mutex mutex_;

public:
  friend class RTCInterfaceTest;
};

using RTCInterface = BasicRTCInterface<rclcpp::Node>;

}  // namespace autoware::rtc_interface

#endif  // AUTOWARE__RTC_INTERFACE__RTC_INTERFACE_HPP_
