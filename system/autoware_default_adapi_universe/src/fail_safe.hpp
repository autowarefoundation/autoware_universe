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

#ifndef FAIL_SAFE_HPP_
#define FAIL_SAFE_HPP_

#include <autoware/adapi_specs/fail_safe.hpp>
#include <autoware/component_interface_specs_universe/system.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

#include <vector>

namespace autoware::default_adapi
{

class FailSafeNode : public rclcpp::Node
{
public:
  explicit FailSafeNode(const rclcpp::NodeOptions & options);

private:
  using MrmDescription = autoware::adapi_specs::fail_safe::MrmDescription::Service;
  using MrmState = autoware::adapi_specs::fail_safe::MrmState::Message;
  Srv<autoware::adapi_specs::fail_safe::MrmDescription> srv_mrm_description_;
  Pub<autoware::adapi_specs::fail_safe::MrmState> pub_mrm_state_;
  Sub<autoware::component_interface_specs_universe::system::MrmState> sub_mrm_state_;
  MrmState prev_state_;
  void on_state(const MrmState::ConstSharedPtr msg);
  void on_mrm_description(
    const MrmDescription::Request::SharedPtr req, const MrmDescription::Response::SharedPtr res);

  std::vector<autoware_adapi_v1_msgs::msg::MrmDescription> descriptions_;
};

}  // namespace autoware::default_adapi

#endif  // FAIL_SAFE_HPP_
