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

#include "localization.hpp"

#include "utils/localization_conversion.hpp"

namespace autoware::default_adapi
{

LocalizationNode::LocalizationNode(const rclcpp::NodeOptions & options)
: Node("localization", options), diagnostics_(this)
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("state", this, &LocalizationNode::diagnose_state);

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  group_cli_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  adaptor.init_pub(pub_state_);
  adaptor.init_sub(sub_state_, this, &LocalizationNode::on_state);
  adaptor.init_cli(cli_initialize_);
  adaptor.init_srv(srv_initialize_, this, &LocalizationNode::on_initialize, group_cli_);

  state_.state = ImplState::Message::UNKNOWN;
}

void LocalizationNode::diagnose_state(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto message = std::to_string(state_.state);

  if (state_.state == ImplState::Message::INITIALIZED) {
    stat.summary(DiagnosticStatus::OK, message);
  } else {
    stat.summary(DiagnosticStatus::ERROR, message);
  }
}

void LocalizationNode::on_state(const ImplState::Message::ConstSharedPtr msg)
{
  state_ = *msg;
  pub_state_->publish(*msg);
}

void LocalizationNode::on_initialize(
  const autoware::adapi_specs::localization::Initialize::Service::Request::SharedPtr req,
  const autoware::adapi_specs::localization::Initialize::Service::Response::SharedPtr res)
{
  res->status = localization_conversion::convert_call(cli_initialize_, req);
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::LocalizationNode)
