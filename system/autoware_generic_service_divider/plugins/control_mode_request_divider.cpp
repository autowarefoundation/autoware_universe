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

#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <memory>
#include <string>
#include <vector>

namespace generic_service_divider
{

class ControlModeRequestDivider : public ServiceDividerPluginBase
{
public:
  void initialize(rclcpp::Node::SharedPtr node) override
  {
    node_ = node;
    input_name_ = node_->declare_parameter<std::string>(
      "control_mode_request.input_service", "/control/control_mode_request");

    const auto output_names = node_->declare_parameter<std::vector<std::string>>(
      "control_mode_request.output_services.names", std::vector<std::string>{});
    const auto output_primaries = node_->declare_parameter<std::vector<bool>>(
      "control_mode_request.output_services.primaries", std::vector<bool>{});
    const auto output_timeouts = node_->declare_parameter<std::vector<int64_t>>(
      "control_mode_request.output_services.timeouts_ms", std::vector<int64_t>{});

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
    return "autoware_vehicle_msgs/srv/ControlModeCommand";
  }

  std::string input_service_name() const override { return input_name_; }

  std::vector<OutputServiceConfig> output_services() const override { return outputs_; }

  bool is_response_success(const void * response) const override
  {
    const auto * r =
      static_cast<const autoware_vehicle_msgs::srv::ControlModeCommand::Response *>(response);
    return r->success;
  }

  std::shared_ptr<void> create_error_response(const std::string & /*message*/) const override
  {
    auto r = std::make_shared<autoware_vehicle_msgs::srv::ControlModeCommand::Response>();
    r->success = false;
    return r;
  }

private:
  std::string input_name_;
  std::vector<OutputServiceConfig> outputs_;
};

}  // namespace generic_service_divider

PLUGINLIB_EXPORT_CLASS(
  generic_service_divider::ControlModeRequestDivider,
  generic_service_divider::ServiceDividerPluginBase)
