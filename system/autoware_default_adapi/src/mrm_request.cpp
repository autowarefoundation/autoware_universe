// Copyright 2025 The Autoware Contributors
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

#include "mrm_request.hpp"

namespace autoware::default_adapi
{

MrmRequestNode::MrmRequestNode(const rclcpp::NodeOptions & options)
: Node("mrm_request", options), diagnostics_(this)
{
  diagnostics_.setHardwareID("none");
  diagnostics_.add("delegate", this, &MrmRequestNode::diagnose_delegate);

  const auto adaptor = autoware::component_interface_utils::NodeAdaptor(this);
  adaptor.init_pub(pub_mrm_request_list_);
  adaptor.init_srv(srv_send_mrm_request_, this, &MrmRequestNode::on_send_mrm_request);

  publish_mrm_request_list();
}

void MrmRequestNode::diagnose_delegate(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  bool requested = false;

  for (const auto & [sender, request] : mrm_requests_) {
    if (request.strategy == MrmRequestItem::DELEGATE) {
      requested = true;
      stat.add(sender, "");
    }
  }
  stat.summary(requested ? DiagnosticStatus::ERROR : DiagnosticStatus::OK, "");
}

void MrmRequestNode::publish_mrm_request_list()
{
  MrmRequestList::Message msg;
  msg.stamp = now();
  for (const auto & [sender, request] : mrm_requests_) {
    msg.requests.push_back(request);
  }
  pub_mrm_request_list_->publish(msg);
}

void MrmRequestNode::on_send_mrm_request(
  const SendMrmRequest::Service::Request::SharedPtr req,
  const SendMrmRequest::Service::Response::SharedPtr res)
{
  switch (req->request.strategy) {
    case MrmRequestItem::CANCEL:
      mrm_requests_.erase(req->request.sender);
      break;
    case MrmRequestItem::DELEGATE:
      mrm_requests_[req->request.sender] = req->request;
      break;
    default:
      res->status.success = false;
      res->status.message = "unknown strategy";
      return;
  }

  res->status.success = true;
  diagnostics_.force_update();
  publish_mrm_request_list();
}

}  // namespace autoware::default_adapi

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::default_adapi::MrmRequestNode)
