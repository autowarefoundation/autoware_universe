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

#include "generic_service_divider/service_divider_plugin_base.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <tier4_system_msgs/srv/reset_redundancy_switcher.hpp>

#include <memory>
#include <string>
#include <vector>

namespace generic_service_divider
{

class ResetRedundancySwitcherDivider : public ServiceDividerPluginBase
{
public:
  void initialize(rclcpp::Node::SharedPtr node) override
  {
    node_ = node;
    input_name_ = node_->declare_parameter<std::string>(
      "reset_redundancy_switcher.input_service", "/system/election/reset");

    const auto output_names = node_->declare_parameter<std::vector<std::string>>(
      "reset_redundancy_switcher.output_services.names", std::vector<std::string>{});
    const auto output_primaries = node_->declare_parameter<std::vector<bool>>(
      "reset_redundancy_switcher.output_services.primaries", std::vector<bool>{});
    const auto output_timeouts = node_->declare_parameter<std::vector<int64_t>>(
      "reset_redundancy_switcher.output_services.timeouts_ms", std::vector<int64_t>{});

    for (size_t i = 0; i < output_names.size(); ++i) {
      OutputServiceConfig cfg;
      cfg.name = output_names[i];
      cfg.primary = (i < output_primaries.size()) ? output_primaries[i] : false;
      cfg.timeout_ms = (i < output_timeouts.size()) ? static_cast<int>(output_timeouts[i]) : 200;
      outputs_.push_back(cfg);
    }
  }

  std::string service_type() const override
  {
    return "tier4_system_msgs/srv/ResetRedundancySwitcher";
  }

  std::string input_service_name() const override { return input_name_; }

  std::vector<OutputServiceConfig> output_services() const override { return outputs_; }

  bool is_response_success(const void * response) const override
  {
    const auto * r =
      static_cast<const tier4_system_msgs::srv::ResetRedundancySwitcher::Response *>(response);
    return r->status.success;
  }

  std::shared_ptr<void> create_error_response(const std::string & message) const override
  {
    auto r = std::make_shared<tier4_system_msgs::srv::ResetRedundancySwitcher::Response>();
    r->status.success = false;
    r->status.code = autoware_common_msgs::msg::ResponseStatus::SERVICE_TIMEOUT;
    r->status.message = message;
    return r;
  }

private:
  std::string input_name_;
  std::vector<OutputServiceConfig> outputs_;
};

}  // namespace generic_service_divider

PLUGINLIB_EXPORT_CLASS(
  generic_service_divider::ResetRedundancySwitcherDivider,
  generic_service_divider::ServiceDividerPluginBase)
