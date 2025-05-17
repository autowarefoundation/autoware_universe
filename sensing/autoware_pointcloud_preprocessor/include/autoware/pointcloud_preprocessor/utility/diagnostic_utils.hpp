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

#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <iomanip>
#include <sstream>
#include <string>

namespace autoware::pointcloud_preprocessor
{

inline std::string format_timestamp(double timestamp)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(9) << timestamp;
  return oss.str();
}

template <typename NodeT, typename ComponentT>
void setup_diagnostics(
  NodeT * node, diagnostic_updater::Updater & updater, ComponentT * component,
  void (ComponentT::*check_fn)(diagnostic_updater::DiagnosticStatusWrapper &))
{
  const std::string node_name = node->get_fully_qualified_name();
  updater.setHardwareID(node_name + "_checker");
  updater.add(node_name + "_status", component, check_fn);
}

}  // namespace autoware::pointcloud_preprocessor
