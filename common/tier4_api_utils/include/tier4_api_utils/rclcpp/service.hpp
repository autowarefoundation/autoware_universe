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

#ifndef TIER4_API_UTILS__RCLCPP__SERVICE_HPP_
#define TIER4_API_UTILS__RCLCPP__SERVICE_HPP_

#include "rclcpp/callback_group.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/service.hpp"

#include <functional>
#include <string>
#include <utility>

namespace tier4_api_utils
{

/// The callback shape wrap() produces, named so that the service handle type below is deduced
/// from it rather than from the caller's lambda type.
template <typename ServiceT>
using ServiceCallback = std::function<void(
  typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr)>;

/// Create the underlying service handle. Kept as a free function because Service deduces its
/// handle type from this call, which is what lets node types other than rclcpp::Node supply
/// their own handle.
template <typename ServiceT, class NodeT>
auto create_service_handle(
  NodeT * node, const std::string & service_name, ServiceCallback<ServiceT> callback,
  const rmw_qos_profile_t & qos_profile, rclcpp::CallbackGroup::SharedPtr group)
{
  return node->template create_service<ServiceT>(
    service_name, std::move(callback), qos_profile, group);
}

template <typename ServiceT, class NodeT = rclcpp::Node>
class Service
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Service)

  using ServiceHandle = decltype(create_service_handle<ServiceT>(
    std::declval<NodeT *>(), std::declval<const std::string &>(),
    std::declval<ServiceCallback<ServiceT>>(), std::declval<const rmw_qos_profile_t &>(), nullptr));

  explicit Service(ServiceHandle service) : service_(service) {}

  template <typename CallbackT>
  static auto wrap(CallbackT && callback, const rclcpp::Logger & logger)
  {
    auto wrapped_callback = [logger, callback](
                              typename ServiceT::Request::SharedPtr request,
                              typename ServiceT::Response::SharedPtr response) {
      RCLCPP_DEBUG(logger, "service request");
      callback(request, response);
      RCLCPP_DEBUG(logger, "service response");
    };
    return wrapped_callback;
  }

private:
  RCLCPP_DISABLE_COPY(Service)

  ServiceHandle service_;
};

}  // namespace tier4_api_utils

#endif  // TIER4_API_UTILS__RCLCPP__SERVICE_HPP_
