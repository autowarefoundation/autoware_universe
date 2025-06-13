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

#include "context.hpp"

#include <memory>
#include <string>
#include <utility>

namespace autoware::command_mode_switcher
{

SwitcherContext::SwitcherContext(rclcpp::Node & node) : node_(node)
{
  group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  cli_select_ = node.create_client<SelectCommandSource>(
    "~/source/select", rmw_qos_profile_services_default, group_);
}

bool SwitcherContext::select_source(const std::string & source)
{
  auto request = std::make_shared<SelectCommandSource::Request>();
  request->source = source;
  cli_select_->async_send_request(request);
  return true;

  /*
  const auto duration = std::chrono::duration<double, std::ratio<1>>(timeout.value());
  if (future.wait_for(duration) != std::future_status::ready) {
    interface_->log(ServiceLog::ERROR_TIMEOUT, SpecType::name);
    throw ServiceTimeout(SpecT::name);
  }
  future.wait();
  auto result = future.get();
  return result->status.success;
  */
}

}  // namespace autoware::command_mode_switcher
