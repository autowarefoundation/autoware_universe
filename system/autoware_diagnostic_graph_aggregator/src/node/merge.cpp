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

#include "merge.hpp"

namespace autoware::diagnostic_graph_aggregator
{

MergeNode::MergeNode(const rclcpp::NodeOptions & options) : Node("merge", options)
{
  const auto qos_struct = rclcpp::QoS(1).transient_local();
  const auto qos_status = rclcpp::QoS(5);

  pub_merged_struct_ = create_publisher<DiagGraphStruct>("~/output/struct", qos_struct);
  pub_merged_status_ = create_publisher<DiagGraphStatus>("~/output/status", qos_status);

  sub_struct1_ = create_subscription<DiagGraphStruct>(
    "~/input1/struct", qos_struct, std::bind(&MergeNode::on_struct1, this, std::placeholders::_1));
  sub_status1_ = create_subscription<DiagGraphStatus>(
    "~/input1/status", qos_status, std::bind(&MergeNode::on_status1, this, std::placeholders::_1));
  sub_struct2_ = create_subscription<DiagGraphStruct>(
    "~/input2/struct", qos_struct, std::bind(&MergeNode::on_struct2, this, std::placeholders::_1));
  sub_status2_ = create_subscription<DiagGraphStatus>(
    "~/input2/status", qos_status, std::bind(&MergeNode::on_status2, this, std::placeholders::_1));

  const auto rate = rclcpp::Rate(declare_parameter<double>("rate"));
  timer_ = rclcpp::create_timer(this, get_clock(), rate.period(), [this]() { on_timer(); });
}

void MergeNode::on_timer()
{
  update_status();
}

void MergeNode::on_struct1(const DiagGraphStruct::ConstSharedPtr msg)
{
  struct1_ = msg;
  update_struct();
}

void MergeNode::on_struct2(const DiagGraphStruct::ConstSharedPtr msg)
{
  struct2_ = msg;
  update_struct();
}

void MergeNode::on_status1(const DiagGraphStatus::ConstSharedPtr msg)
{
  status1_ = msg;
}

void MergeNode::on_status2(const DiagGraphStatus::ConstSharedPtr msg)
{
  status2_ = msg;
}

void MergeNode::update_struct()
{
  using tier4_system_msgs::msg::DiagLeafStruct;
  using tier4_system_msgs::msg::DiagLinkStruct;

  const auto shift_diag = [](const DiagLeafStruct & diag, size_t offset) {
    DiagLeafStruct result = diag;
    result.parent += offset;
    return result;
  };
  const auto shift_link = [](const DiagLinkStruct & link, size_t offset) {
    DiagLinkStruct result = link;
    result.parent += offset;
    result.child += offset;
    return result;
  };

  if (!struct1_) return;
  if (!struct2_) return;
  const auto offset = struct1_->nodes.size();

  DiagGraphStruct merged;
  merged.nodes.reserve(struct1_->nodes.size() + struct2_->nodes.size());
  merged.diags.reserve(struct1_->diags.size() + struct2_->diags.size());
  merged.links.reserve(struct1_->links.size() + struct2_->links.size());
  merged.stamp = now();
  merged.id = struct1_->id + "+" + struct2_->id;
  for (const auto & node : struct1_->nodes) merged.nodes.push_back(node);
  for (const auto & diag : struct1_->diags) merged.diags.push_back(diag);
  for (const auto & link : struct1_->links) merged.links.push_back(link);
  for (const auto & node : struct2_->nodes) merged.nodes.push_back(node);
  for (const auto & diag : struct2_->diags) merged.diags.push_back(shift_diag(diag, offset));
  for (const auto & link : struct2_->links) merged.links.push_back(shift_link(link, offset));
  pub_merged_struct_->publish(merged);
}

void MergeNode::update_status()
{
  if (!struct1_) return;
  if (!struct2_) return;
  if (!status1_) return;
  if (!status2_) return;
  if (struct1_->id != status1_->id) return;
  if (struct2_->id != status2_->id) return;

  DiagGraphStatus merged;
  merged.nodes.reserve(status1_->nodes.size() + status2_->nodes.size());
  merged.diags.reserve(status1_->diags.size() + status2_->diags.size());
  merged.stamp = now();
  merged.id = status1_->id + "+" + status2_->id;
  for (const auto & node : status1_->nodes) merged.nodes.push_back(node);
  for (const auto & diag : status1_->diags) merged.diags.push_back(diag);
  for (const auto & node : status2_->nodes) merged.nodes.push_back(node);
  for (const auto & diag : status2_->diags) merged.diags.push_back(diag);
  pub_merged_status_->publish(merged);
}

}  // namespace autoware::diagnostic_graph_aggregator

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_aggregator::MergeNode)
