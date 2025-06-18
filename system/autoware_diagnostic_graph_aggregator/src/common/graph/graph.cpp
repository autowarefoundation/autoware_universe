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

#include "graph/graph.hpp"

#include "config/entity.hpp"
#include "config/loader.hpp"
#include "graph/diags.hpp"
#include "graph/links.hpp"
#include "graph/nodes.hpp"
#include "graph/units.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

Graph::Graph(const std::string & path) : Graph(path, "", nullptr)
{
}

Graph::Graph(const std::string & path, const std::string & id, std::shared_ptr<Logger> logger)
{
  id_ = id;

  auto graph = ConfigLoader::Load(path, logger);
  alloc_nodes_ = std::move(graph.nodes);
  alloc_diags_ = std::move(graph.diags);
  alloc_ports_ = std::move(graph.ports);
  nodes_ = raws(alloc_nodes_);
  diags_ = raws(alloc_diags_);

  for (const auto & diag : diags_) {
    diag_dict_[diag->name()] = diag;
  }
}

Graph::~Graph()
{
}

void Graph::update(const rclcpp::Time & stamp)
{
  // Update the graph from the leaves. Note that the nodes are topological sorted.
  std::for_each(diags_.rbegin(), diags_.rend(), [stamp](auto & diag) { diag->update(stamp); });
  std::for_each(nodes_.rbegin(), nodes_.rend(), [stamp](auto & node) { node->update(stamp); });
}

bool Graph::update(const rclcpp::Time & stamp, const DiagnosticArray & array)
{
  // check message stamp delay
  const auto delay = (stamp - array.header.stamp).seconds();
  if (1.0 < delay) {
    return false;  // TODO(Takagi, Isamu): parameterize, output warning
  }
  // TODO(Takagi, Isamu): Check future stamp. Use now stamp instead of message stamp.

  for (const auto & status : array.status) {
    const auto iter = diag_dict_.find(status.name);
    if (iter != diag_dict_.end()) {
      iter->second->update(array.header.stamp, status);
    } else {
      unknown_diags_[status.name] = status;
    }
  }
  return true;
}

DiagGraphStruct Graph::create_struct_msg(const rclcpp::Time & stamp) const
{
  DiagGraphStruct msg;
  msg.stamp = stamp;
  msg.id = id_;
  for (const auto & node : nodes_) msg.nodes.push_back(node->create_struct());
  for (const auto & diag : diags_) msg.diags.push_back(diag->create_struct());

  for (const auto & parent : nodes_) {
    for (const auto & child : parent->child_units()) {
      if (dynamic_cast<const NodeUnit *>(child)) {
        DiagLinkStruct link;
        link.parent = parent->index();
        link.child = child->index();
        msg.links.push_back(link);
      }
    }
  }
  return msg;
}

DiagGraphStatus Graph::create_status_msg(const rclcpp::Time & stamp) const
{
  DiagGraphStatus msg;
  msg.stamp = stamp;
  msg.id = id_;
  for (const auto & node : nodes_) msg.nodes.push_back(node->create_status());
  for (const auto & diag : diags_) msg.diags.push_back(diag->create_status());
  return msg;
}

DiagnosticArray Graph::create_unknown_msg(const rclcpp::Time & stamp) const
{
  DiagnosticArray msg;
  msg.header.stamp = stamp;
  for (const auto & [name, diag] : unknown_diags_) msg.status.push_back(diag);
  return msg;
}

void Graph::reset()
{
  for (const auto & node : nodes_) node->reset();
}

}  // namespace autoware::diagnostic_graph_aggregator
