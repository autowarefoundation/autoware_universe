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

#include "fail_safe.hpp"

#include <map>
#include <string>

namespace autoware::default_adapi
{

auto allow_dynamic_params(const rclcpp::NodeOptions & original_options)
{
  rclcpp::NodeOptions options(original_options);
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);
  return options;
}

FailSafeNode::FailSafeNode(const rclcpp::NodeOptions & options)
: Node("fail_safe", allow_dynamic_params(options))
{
  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_mrm_state_);
  adaptor.init_sub(sub_mrm_state_, this, &FailSafeNode::on_state);
  adaptor.init_srv(srv_mrm_description_, this, &FailSafeNode::on_mrm_description);
  prev_state_.state = MrmState::UNKNOWN;

  // Get mrm descriptions from parameters
  std::map<std::string, rclcpp::Parameter> param_descriptions;
  std::map<std::string, autoware_adapi_v1_msgs::msg::MrmDescription> map_descriptions;
  get_parameters("mrm_descriptions", param_descriptions);
  for (const auto & [key, value] : param_descriptions) {
    const auto pos = key.find_first_of('.');
    const auto name = key.substr(0, pos);
    const auto data = key.substr(pos + 1);
    if (data == "behavior") {
      map_descriptions[name].behavior = value.as_int();
    } else if (data == "description") {
      map_descriptions[name].description = value.as_string();
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "unknown parameter: " << key);
    }
  }
  for (const auto & [name, description] : map_descriptions) {
    descriptions_.push_back(description);
    descriptions_.back().name = name;
  }
}

void FailSafeNode::on_state(const MrmState::ConstSharedPtr msg)
{
  prev_state_.stamp = msg->stamp;
  if (prev_state_ != *msg) {
    prev_state_ = *msg;
    pub_mrm_state_->publish(*msg);
  }
}

void FailSafeNode::on_mrm_description(
  const MrmDescription::Request::SharedPtr, const MrmDescription::Response::SharedPtr res)
{
  res->descriptions = descriptions_;
  res->status.success = true;
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::FailSafeNode)
