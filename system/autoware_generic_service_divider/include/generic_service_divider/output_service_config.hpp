// Copyright 2025 TIER IV, Inc.
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

#ifndef GENERIC_SERVICE_DIVIDER__OUTPUT_SERVICE_CONFIG_HPP_
#define GENERIC_SERVICE_DIVIDER__OUTPUT_SERVICE_CONFIG_HPP_

#include <string>

namespace generic_service_divider
{

struct OutputServiceConfig
{
  std::string name;
  bool primary{false};
  int timeout_ms{200};
};

}  // namespace generic_service_divider

#endif  // GENERIC_SERVICE_DIVIDER__OUTPUT_SERVICE_CONFIG_HPP_
