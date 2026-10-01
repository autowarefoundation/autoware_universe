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

#ifndef DIAGNOSTIC_GRAPH_MERGER__MERGE_HPP_
#define DIAGNOSTIC_GRAPH_MERGER__MERGE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

namespace autoware::diagnostic_graph_merger
{

class MergeNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit MergeNode(const rclcpp::NodeOptions & options);

private:
  using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;

  void on_timer();
  void on_struct1(const DiagGraphStruct::ConstSharedPtr msg);
  void on_struct2(const DiagGraphStruct::ConstSharedPtr msg);
  void on_status1(const DiagGraphStatus::ConstSharedPtr msg);
  void on_status2(const DiagGraphStatus::ConstSharedPtr msg);
  void update_struct();
  void update_status();

  AUTOWARE_TIMER_PTR timer_;
  AUTOWARE_PUBLISHER_PTR(DiagGraphStruct) pub_merged_struct_;
  AUTOWARE_PUBLISHER_PTR(DiagGraphStatus) pub_merged_status_;
  AUTOWARE_SUBSCRIPTION_PTR(DiagGraphStruct) sub_struct1_;
  AUTOWARE_SUBSCRIPTION_PTR(DiagGraphStatus) sub_status1_;
  AUTOWARE_SUBSCRIPTION_PTR(DiagGraphStruct) sub_struct2_;
  AUTOWARE_SUBSCRIPTION_PTR(DiagGraphStatus) sub_status2_;

  DiagGraphStruct::ConstSharedPtr struct1_;
  DiagGraphStatus::ConstSharedPtr status1_;
  DiagGraphStruct::ConstSharedPtr struct2_;
  DiagGraphStatus::ConstSharedPtr status2_;
};

}  // namespace autoware::diagnostic_graph_merger

#endif  // DIAGNOSTIC_GRAPH_MERGER__MERGE_HPP_
