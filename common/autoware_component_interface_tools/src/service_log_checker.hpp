// Copyright 2022 TIER IV, Inc.
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

#ifndef SERVICE_LOG_CHECKER_HPP_
#define SERVICE_LOG_CHECKER_HPP_

#include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <string>
#include <unordered_map>

namespace autoware::component_interface_tools
{
class ServiceLogChecker : public autoware::agnocast_wrapper::Node
{
public:
  explicit ServiceLogChecker(const rclcpp::NodeOptions & options);

private:
  using ServiceLog = tier4_system_msgs::msg::ServiceLog;
  AUTOWARE_SUBSCRIPTION_PTR(ServiceLog) sub_;
  autoware::agnocast_wrapper::diagnostic_updater::Updater diagnostics_;
  void on_service_log(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(ServiceLog) & msg);
  void set_success(const ServiceLog & msg);
  void set_error(const ServiceLog & msg, const std::string & log);
  void update_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  std::unordered_map<std::string, std::string> errors_;
};
}  // namespace autoware::component_interface_tools
#endif  // SERVICE_LOG_CHECKER_HPP_
