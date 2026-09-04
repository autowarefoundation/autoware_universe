// Copyright 2021, Open Source Robotics Foundation, Inc.
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

#include "generic_service_divider/generic_client.hpp"

#include "generic_service_divider/service_typesupport_helpers.hpp"
#include "rcl/client.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

#include <memory>
#include <string>
#include <utility>

namespace generic_service_divider
{

GenericClient::GenericClient(
  rclcpp::node_interfaces::NodeBaseInterface * node_base,
  rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
  const std::string & service_name, const std::string & service_type,
  rcl_client_options_t & client_options)
: rclcpp::ClientBase(node_base, std::move(node_graph))
{
  ts_lib_ = get_service_typesupport_library(service_type, "rosidl_typesupport_cpp");
  auto * service_ts =
    get_service_typesupport_handle(service_type, "rosidl_typesupport_cpp", ts_lib_);

  auto [intro_lib, service_members] = get_service_members(service_type);
  intro_ts_lib_ = std::move(intro_lib);
  request_members_ = service_members->request_members_;
  response_members_ = service_members->response_members_;

  rcl_ret_t ret = rcl_client_init(
    get_client_handle().get(), node_base->get_rcl_node_handle(), service_ts, service_name.c_str(),
    &client_options);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create client");
  }
}

std::shared_ptr<void> GenericClient::create_response()
{
  return allocate_message(response_members_);
}

std::shared_ptr<rmw_request_id_t> GenericClient::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void GenericClient::handle_response(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response)
{
  ResponseCallback callback;
  SharedFuture future;
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);
    auto it = pending_requests_.find(request_header->sequence_number);
    if (it == pending_requests_.end()) {
      return;
    }
    callback = std::move(it->second.callback);
    future = it->second.future;
    it->second.promise.set_value(std::move(response));
    pending_requests_.erase(it);
  }
  if (callback) {
    callback(future);
  }
}

GenericClient::SharedRequest GenericClient::create_request()
{
  return allocate_message(request_members_);
}

GenericClient::SharedFuture GenericClient::async_send_request(
  SharedRequest request, ResponseCallback callback)
{
  int64_t sequence_number;
  rcl_ret_t ret = rcl_send_request(get_client_handle().get(), request.get(), &sequence_number);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send request");
  }

  std::lock_guard<std::mutex> lock(pending_requests_mutex_);
  PendingRequest pr;
  pr.callback = std::move(callback);
  pr.future = pr.promise.get_future().share();
  auto future = pr.future;
  pending_requests_[sequence_number] = std::move(pr);
  return future;
}

GenericClient::SharedFuture GenericClient::async_send_request(SharedRequest request)
{
  return async_send_request(std::move(request), nullptr);
}

}  // namespace generic_service_divider
