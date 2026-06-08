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

#ifndef ROS_INTERFACE_HPP_
#define ROS_INTERFACE_HPP_

#include "type/interface.hpp"

#include <autoware_driving_mode_manager/msg/debug_request_modes.hpp>

#include <autoware_adapi_v1_msgs/msg/mrm_state.hpp>
#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_system_msgs/srv/change_autoware_control.hpp>
#include <autoware_system_msgs/srv/change_operation_mode.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <tier4_system_msgs/msg/command_filter_status.hpp>
#include <tier4_system_msgs/msg/command_source_status.hpp>
#include <tier4_system_msgs/msg/driving_mode_mrm_state.hpp>
#include <tier4_system_msgs/msg/driving_mode_request.hpp>
#include <tier4_system_msgs/msg/driving_mode_status.hpp>
#include <tier4_system_msgs/msg/trajectory_source_status.hpp>
#include <tier4_system_msgs/srv/change_command_filter.hpp>
#include <tier4_system_msgs/srv/change_command_source.hpp>
#include <tier4_system_msgs/srv/change_mrm_request.hpp>
#include <tier4_system_msgs/srv/change_trajectory_source.hpp>

namespace autoware::driving_mode_manager
{

class RosInterface : public Interface
{
public:
  explicit RosInterface(rclcpp::Node * node);
  void init(MainLogic * logic) override { logic_ = logic; }

  rclcpp::Time now() const override;
  void change_trajectory_source(const TrajectorySource & source) override;
  void change_command_source(const CommandSource & source) override;
  void change_command_filter(const CommandFilter & filter) override;
  void change_platform_mode(const PlatformMode & mode) override;
  void publish_operation_mode(const OperationModeState & state) const override;
  void publish_mrm_state(const MrmState & state) const override;
  void publish_driving_mode_request(const AutowareMode & mode) const override;
  void publish_debug_status(const DebugStatus & status) const override;
  void publish_debug_status(const RequestModes & request) const override;

private:
  using TrajectorySourceSrv = tier4_system_msgs::srv::ChangeTrajectorySource;
  using ChangeCommandSourceSrv = tier4_system_msgs::srv::ChangeCommandSource;
  using ChangeCommandFilterSrv = tier4_system_msgs::srv::ChangeCommandFilter;
  using ControlModeCommandSrv = autoware_vehicle_msgs::srv::ControlModeCommand;
  using OperationModeStateMsg = autoware_adapi_v1_msgs::msg::OperationModeState;
  using MrmStateMsg = autoware_adapi_v1_msgs::msg::MrmState;

  using DrivingModeRequest = tier4_system_msgs::msg::DrivingModeRequest;
  using DrivingModeStatus = tier4_system_msgs::msg::DrivingModeStatus;
  using DrivingModeMrmState = tier4_system_msgs::msg::DrivingModeMrmState;
  using TrajectorySourceMsg = tier4_system_msgs::msg::TrajectorySourceStatus;
  using CommandSourceMsg = tier4_system_msgs::msg::CommandSourceStatus;
  using CommandFilterMsg = tier4_system_msgs::msg::CommandFilterStatus;
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
  using ChangeOperationMode = autoware_system_msgs::srv::ChangeOperationMode;
  using ChangeAutowareControl = autoware_system_msgs::srv::ChangeAutowareControl;
  using ChangeMrmRequest = tier4_system_msgs::srv::ChangeMrmRequest;

  using DebugRequestModes = autoware_driving_mode_manager::msg::DebugRequestModes;

  MainLogic * logic_;

  rclcpp::Node * node_;
  rclcpp::Client<TrajectorySourceSrv>::SharedPtr cli_trajectory_source_;
  rclcpp::Client<ChangeCommandSourceSrv>::SharedPtr cli_command_source_;
  rclcpp::Client<ChangeCommandFilterSrv>::SharedPtr cli_command_filter_;
  rclcpp::Client<ControlModeCommandSrv>::SharedPtr cli_control_mode_command_;
  rclcpp::Publisher<OperationModeStateMsg>::SharedPtr pub_operation_mode_;
  rclcpp::Publisher<MrmStateMsg>::SharedPtr pub_mrm_state_;

  rclcpp::Subscription<DrivingModeStatus>::SharedPtr sub_driving_mode_status_;
  rclcpp::Subscription<DrivingModeMrmState>::SharedPtr sub_driving_mode_mrm_state_;
  rclcpp::Subscription<TrajectorySourceMsg>::SharedPtr sub_trajectory_source_;
  rclcpp::Subscription<CommandSourceMsg>::SharedPtr sub_command_source_;
  rclcpp::Subscription<CommandFilterMsg>::SharedPtr sub_command_filter_;
  rclcpp::Subscription<ControlModeReport>::SharedPtr sub_control_mode_report_;
  rclcpp::Service<ChangeOperationMode>::SharedPtr srv_operation_mode_;
  rclcpp::Service<ChangeAutowareControl>::SharedPtr srv_autoware_control_;
  rclcpp::Service<ChangeMrmRequest>::SharedPtr srv_mrm_request_;
  rclcpp::Publisher<DrivingModeRequest>::SharedPtr pub_driving_mode_request_;
  rclcpp::Publisher<DrivingModeStatus>::SharedPtr pub_driving_mode_status_;
  rclcpp::Publisher<DebugRequestModes>::SharedPtr pub_debug_request_;

  void on_driving_mode_status(const DrivingModeStatus & msg);
  void on_driving_mode_mrm_state(const DrivingModeMrmState & msg);
  void on_trajectory_source(const TrajectorySourceMsg & msg);
  void on_command_source(const CommandSourceMsg & msg);
  void on_command_filter(const CommandFilterMsg & msg);
  void on_control_mode_report(const ControlModeReport & msg);
  void on_change_operation_mode(
    ChangeOperationMode::Request::SharedPtr req, ChangeOperationMode::Response::SharedPtr res);
  void on_change_autoware_control(
    ChangeAutowareControl::Request::SharedPtr req, ChangeAutowareControl::Response::SharedPtr res);
  void on_change_mrm_request(
    ChangeMrmRequest::Request::SharedPtr req, ChangeMrmRequest::Response::SharedPtr res);
};

}  // namespace autoware::driving_mode_manager

#endif  // ROS_INTERFACE_HPP_
