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

#ifndef MRM_REQUEST_HPP_
#define MRM_REQUEST_HPP_

#include <autoware/adapi_specs/fail_safe.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class MrmRequestNode : public rclcpp::Node
{
public:
  explicit MrmRequestNode(const rclcpp::NodeOptions & options);

private:
  using SendMrmRequest = autoware::adapi_specs::fail_safe::SendMrmRequest;
  using MrmRequestList = autoware::adapi_specs::fail_safe::MrmRequestList;
  using MrmRequestItem = autoware_adapi_v1_msgs::msg::MrmRequest;

  Srv<SendMrmRequest> srv_send_mrm_request_;
  Pub<MrmRequestList> pub_mrm_request_list_;

  void diagnose_delegate(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void publish_mrm_request_list();
  void on_send_mrm_request(
    const SendMrmRequest::Service::Request::SharedPtr req,
    const SendMrmRequest::Service::Response::SharedPtr res);

  diagnostic_updater::Updater diagnostics_;
  std::unordered_map<std::string, MrmRequestItem> mrm_requests_;
};

}  // namespace autoware::default_adapi

#endif  // MRM_REQUEST_HPP_
