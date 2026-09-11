// Copyright 2021 Tier IV, Inc.
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

#ifndef TIER4_API_UTILS__RCLCPP__CLIENT_HPP_
#define TIER4_API_UTILS__RCLCPP__CLIENT_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/client.hpp"
#include "tier4_api_utils/types/response.hpp"

#include <chrono>
#include <string>
#include <utility>

namespace tier4_api_utils
{

/// Create the underlying client handle. Kept as a free function because Client deduces its handle
/// type from this call, which is what lets node types other than rclcpp::Node supply their own.
template <typename ServiceT, class NodeT>
auto create_client_handle(
  NodeT * node, const std::string & service_name, const rmw_qos_profile_t & qos_profile,
  rclcpp::CallbackGroup::SharedPtr group)
{
  return node->template create_client<ServiceT>(service_name, qos_profile, group);
}

template <typename ServiceT, class NodeT = rclcpp::Node>
class Client
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Client)

  using ClientHandle = decltype(create_client_handle<ServiceT>(
    std::declval<NodeT *>(), std::declval<const std::string &>(),
    std::declval<const rmw_qos_profile_t &>(), nullptr));

  /// Taken from the handle rather than spelled as ServiceT::Response::SharedPtr: a node type may
  /// hand the response over as a pointer to const.
  using SharedResponse = typename ClientHandle::element_type::SharedResponse;

  using ResponseStatus = tier4_external_api_msgs::msg::ResponseStatus;
  using AutowareServiceResult = std::pair<ResponseStatus, SharedResponse>;

  Client(ClientHandle client, const rclcpp::Logger & logger) : client_(client), logger_(logger) {}

  AutowareServiceResult call(
    const typename ServiceT::Request::SharedPtr & request,
    const std::chrono::nanoseconds & timeout = std::chrono::seconds(2))
  {
    RCLCPP_DEBUG(logger_, "client request");

    if (!client_->service_is_ready()) {
      RCLCPP_DEBUG(logger_, "client available");
      return {response_error("Internal service is not available."), {}};
    }

    auto future = client_->async_send_request(request);
    if (future.wait_for(timeout) != std::future_status::ready) {
      RCLCPP_DEBUG(logger_, "client timeout");
      return {response_error("Internal service has timed out."), {}};
    }

    RCLCPP_DEBUG(logger_, "client response");
    return {response_success(), future.get()};
  }

private:
  RCLCPP_DISABLE_COPY(Client)

  ClientHandle client_;
  rclcpp::Logger logger_;
};

}  // namespace tier4_api_utils

#endif  // TIER4_API_UTILS__RCLCPP__CLIENT_HPP_
