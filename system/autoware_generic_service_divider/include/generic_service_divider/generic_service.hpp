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

#ifndef GENERIC_SERVICE_DIVIDER__GENERIC_SERVICE_HPP_
#define GENERIC_SERVICE_DIVIDER__GENERIC_SERVICE_HPP_

#include "rclcpp/service.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include <functional>
#include <memory>
#include <string>

namespace generic_service_divider
{

class GenericService;

using GenericServiceCallback = std::function<void(
  std::shared_ptr<GenericService>, std::shared_ptr<rmw_request_id_t>, std::shared_ptr<void>)>;

// TODO(Jazzy): On Jazzy+, adapt virtual method signatures to const reference parameters
// TODO(Jazzy): Replace with rclcpp::GenericService when available
class GenericService : public rclcpp::ServiceBase,
                       public std::enable_shared_from_this<GenericService>
{
public:
  using SharedRequest = std::shared_ptr<void>;
  using SharedResponse = std::shared_ptr<void>;

  GenericService(
    std::shared_ptr<rcl_node_t> node_handle, const std::string & service_name,
    const std::string & service_type, GenericServiceCallback callback,
    rcl_service_options_t & service_options);

  ~GenericService() override = default;

  std::shared_ptr<void> create_request() override;

  std::shared_ptr<rmw_request_id_t> create_request_header() override;

  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override;

  SharedResponse create_response();

  void send_response(rmw_request_id_t & req_id, SharedResponse response);

private:
  GenericServiceCallback callback_;
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  std::shared_ptr<rcpputils::SharedLibrary> intro_ts_lib_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members_;
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members_;
};

}  // namespace generic_service_divider

#endif  // GENERIC_SERVICE_DIVIDER__GENERIC_SERVICE_HPP_
