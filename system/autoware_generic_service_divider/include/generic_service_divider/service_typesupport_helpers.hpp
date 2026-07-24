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

#ifndef GENERIC_SERVICE_DIVIDER__SERVICE_TYPESUPPORT_HELPERS_HPP_
#define GENERIC_SERVICE_DIVIDER__SERVICE_TYPESUPPORT_HELPERS_HPP_

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"

#include <memory>
#include <string>
#include <tuple>

namespace generic_service_divider
{

std::shared_ptr<rcpputils::SharedLibrary> get_service_typesupport_library(
  const std::string & type, const std::string & typesupport_identifier);

const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier,
  std::shared_ptr<rcpputils::SharedLibrary> library);

std::tuple<
  std::shared_ptr<rcpputils::SharedLibrary>,
  const rosidl_typesupport_introspection_cpp::ServiceMembers *>
get_service_members(const std::string & type);

}  // namespace generic_service_divider

#endif  // GENERIC_SERVICE_DIVIDER__SERVICE_TYPESUPPORT_HELPERS_HPP_
