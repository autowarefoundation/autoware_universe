// Copyright 2025 The Autoware Contributors
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

#include "utils/logger.hpp"

#include <iostream>
#include <string>

namespace autoware::diagnostic_graph_aggregator
{

void StdLogger::info(const std::string & message)
{
  std::cout << message << std::endl;
}

void StdLogger::debug(const std::string & message)
{
  std::cout << message << std::endl;
}

}  // namespace autoware::diagnostic_graph_aggregator
