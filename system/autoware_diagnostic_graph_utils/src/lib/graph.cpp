// Copyright 2024 The Autoware Contributors
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

#include "autoware/diagnostic_graph_utils/graph.hpp"

#include <vector>

namespace autoware::diagnostic_graph_utils
{

std::vector<DiagUnit *> DiagUnit::child_units() const
{
  std::vector<DiagUnit *> result;
  for (const auto & link : child_links()) {
    result.push_back(link->child());
  }
  return result;
}

std::vector<DiagNode *> DiagUnit::child_nodes() const
{
  std::vector<DiagNode *> result;
  for (const auto & link : child_links()) {
    if (const auto node = dynamic_cast<DiagNode *>(link->child())) {
      result.push_back(node);
    }
  }
  return result;
}

std::vector<DiagLink *> DiagUnit::child_links() const
{
  return child_links_;
}

DiagUnit::DiagnosticStatus DiagNode::create_diagnostic_status() const
{
  DiagnosticStatus status;
  status.level = level();
  status.name = path();
  return status;
}

DiagUnit::DiagnosticStatus DiagLeaf::create_diagnostic_status() const
{
  DiagnosticStatus status;
  status.level = level();
  status.name = name();
  status.message = status_.message;
  status.hardware_id = status_.hardware_id;
  status.values = status_.values;
  return status;
}

bool DiagGraph::create(const DiagGraphStruct & msg)
{
  units_.clear();
  nodes_.clear();
  diags_.clear();
  links_.clear();

  created_stamp_ = msg.stamp;
  id_ = msg.id;

  int index = 0;
  for (const auto & node : msg.nodes) {
    auto unit = nodes_.emplace_back(std::make_unique<DiagNode>(index++, node)).get();
    units_.push_back(unit);
  }
  for (const auto & diag : msg.diags) {
    auto unit = diags_.emplace_back(std::make_unique<DiagLeaf>(index++, diag)).get();
    units_.push_back(unit);
    DiagUnit * p = nodes_.at(diag.parent).get();
    DiagUnit * c = unit;
    p->add_child(links_.emplace_back(std::make_unique<DiagLink>(p, c)).get());
  }
  for (const auto & link : msg.links) {
    DiagUnit * p = nodes_.at(link.parent).get();
    DiagUnit * c = nodes_.at(link.child).get();
    p->add_child(links_.emplace_back(std::make_unique<DiagLink>(p, c)).get());
  }
  return true;
}

bool DiagGraph::update(const DiagGraphStatus & msg)
{
  if (id_ != msg.id) return false;
  updated_stamp_ = msg.stamp;
  for (size_t i = 0; i < msg.nodes.size(); ++i) nodes_[i]->update(msg.nodes[i]);
  for (size_t i = 0; i < msg.diags.size(); ++i) diags_[i]->update(msg.diags[i]);
  return true;
}

template <class T, class U>
void extend_ptrs(std::vector<T *> & result, const std::vector<std::unique_ptr<U>> & list)
{
  for (const auto & item : list) result.push_back(item.get());
}

template <class T>
std::vector<T *> create_ptrs(const std::vector<std::unique_ptr<T>> & list)
{
  std::vector<T *> result;
  extend_ptrs(result, list);
  return result;
}

std::vector<DiagUnit *> DiagGraph::units() const
{
  return units_;
}

std::vector<DiagNode *> DiagGraph::nodes() const
{
  return create_ptrs<DiagNode>(nodes_);
}

std::vector<DiagLeaf *> DiagGraph::diags() const
{
  return create_ptrs<DiagLeaf>(diags_);
}

std::vector<DiagLink *> DiagGraph::links() const
{
  return create_ptrs<DiagLink>(links_);
}

}  // namespace autoware::diagnostic_graph_utils
