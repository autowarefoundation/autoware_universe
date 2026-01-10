// Copyright 2023 Autoware Foundation
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

#ifndef STOPPER__STOPPER_LIDAR_MARKER_HPP_
#define STOPPER__STOPPER_LIDAR_MARKER_HPP_

#include "stopper/base_stopper.hpp"

#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::pose_estimator_arbiter::stopper
{
class StopperLidarMarker : public BaseStopper
{
  using SetBool = std_srvs::srv::SetBool;

public:
  explicit StopperLidarMarker(
    rclcpp::Node * node, const std::shared_ptr<const SharedData> shared_data,
    const std::vector<std::string> & instance_names)
  : BaseStopper(node, shared_data)
  {
    for (const auto & instance_name : instance_names) {
      auto client = node->create_client<SetBool>(instance_name + "/trigger_node_srv");
      trigger_clients_.push_back(client);
    }
  }

  void set_enable(bool enabled) override
  {
    auto request = std::make_shared<SetBool::Request>();
    request->data = enabled;

    for (auto & client : trigger_clients_) {
      if (!client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "Service %s not available", client->get_service_name());
        continue;
      }
      client->async_send_request(request);
    }
  }

private:
  std::vector<rclcpp::Client<SetBool>::SharedPtr> trigger_clients_;
};
}  // namespace autoware::pose_estimator_arbiter::stopper

#endif  // STOPPER__STOPPER_LIDAR_MARKER_HPP_
