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

#include "generic_service_divider/generic_service.hpp"

#include "generic_service_divider/service_typesupport_helpers.hpp"
#include "rcl/service.h"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

#include <memory>
#include <string>
#include <utility>

namespace generic_service_divider
{

GenericService::GenericService(
  std::shared_ptr<rcl_node_t> node_handle, const std::string & service_name,
  const std::string & service_type, GenericServiceCallback callback,
  rcl_service_options_t & service_options)
: rclcpp::ServiceBase(node_handle), callback_(std::move(callback))
{
  ts_lib_ = get_service_typesupport_library(service_type, "rosidl_typesupport_cpp");
  auto * service_ts =
    get_service_typesupport_handle(service_type, "rosidl_typesupport_cpp", ts_lib_);

  auto [intro_lib, service_members] = get_service_members(service_type);
  intro_ts_lib_ = std::move(intro_lib);
  request_members_ = service_members->request_members_;
  response_members_ = service_members->response_members_;

  // On Humble, ServiceBase does not allocate service_handle_; we must do it.
  service_handle_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t, [handle = node_handle, service_name](rcl_service_t * service) {
      if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_node_logger(handle.get()), "Error in destruction of rcl service handle: %s",
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete service;
    });
  *service_handle_ = rcl_get_zero_initialized_service();

  rcl_ret_t ret = rcl_service_init(
    service_handle_.get(), node_handle.get(), service_ts, service_name.c_str(), &service_options);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create service");
  }
}

std::shared_ptr<void> GenericService::create_request()
{
  const auto * members = request_members_;
  auto request = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  request_members_->init_function(request.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return request;
}

std::shared_ptr<rmw_request_id_t> GenericService::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void GenericService::handle_request(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request)
{
  callback_(shared_from_this(), std::move(request_header), std::move(request));
}

GenericService::SharedResponse GenericService::create_response()
{
  const auto * members = response_members_;
  auto response = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  response_members_->init_function(response.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return response;
}

void GenericService::send_response(rmw_request_id_t & req_id, SharedResponse response)
{
  rcl_ret_t ret = rcl_send_response(get_service_handle().get(), &req_id, response.get());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send service response");
  }
}

}  // namespace generic_service_divider
