// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__TYPES__DIAGNOSTICS_HPP_
#define COMMON__TYPES__DIAGNOSTICS_HPP_

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>
#include <tier4_system_msgs/srv/reset_diag_graph.hpp>

namespace autoware::diagnostic_graph_aggregator
{

using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticLevel = DiagnosticStatus::_level_type;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;

using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
using DiagNodeStruct = tier4_system_msgs::msg::DiagNodeStruct;
using DiagNodeStatus = tier4_system_msgs::msg::DiagNodeStatus;
using DiagLeafStruct = tier4_system_msgs::msg::DiagLeafStruct;
using DiagLeafStatus = tier4_system_msgs::msg::DiagLeafStatus;
using DiagLinkStruct = tier4_system_msgs::msg::DiagLinkStruct;
using DiagLinkStatus = tier4_system_msgs::msg::DiagLinkStatus;
using ResetDiagGraph = tier4_system_msgs::srv::ResetDiagGraph;
}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__TYPES__DIAGNOSTICS_HPP_
