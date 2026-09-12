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

#ifndef POSE_INITIALIZATION_REQUESTER_HPP_
#define POSE_INITIALIZATION_REQUESTER_HPP_

#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::pose_initialization_requester
{

class PoseInitializationRequester : public rclcpp::Node
{
public:
  explicit PoseInitializationRequester(const rclcpp::NodeOptions & options);

private:
  using Initialize = autoware::component_interface_specs::localization::Initialize;
  using State = autoware::component_interface_specs::localization::InitializationState;
  rclcpp::CallbackGroup::SharedPtr group_cli_;
  rclcpp::TimerBase::SharedPtr timer_;
  autoware::component_interface_utils::NodeAdaptor<rclcpp::Node> adaptor_{this};
  autoware::component_interface_utils::Client<Initialize>::SharedPtr cli_initialize_;
  autoware::component_interface_utils::Subscription<State>::SharedPtr sub_state_;

  void on_timer();
  State::Message state_;
};

}  // namespace autoware::pose_initialization_requester

#endif  // POSE_INITIALIZATION_REQUESTER_HPP_
