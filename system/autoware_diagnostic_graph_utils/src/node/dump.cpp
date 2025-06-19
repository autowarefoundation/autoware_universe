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

#include "dump.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_utils
{

DumpNode::DumpNode(const rclcpp::NodeOptions & options) : Node("dump", options)
{
  using std::placeholders::_1;
  sub_graph_.register_create_callback(std::bind(&DumpNode::on_create, this, _1));
  sub_graph_.register_update_callback(std::bind(&DumpNode::on_update, this, _1));
  sub_graph_.subscribe(*this, 1);
}

void DumpNode::on_create(DiagGraph::ConstSharedPtr graph)
{
  const auto join = [](const std::vector<DiagUnit *> & units) {
    std::string result;
    for (size_t i = 0; i < units.size(); ++i) {
      result += (i ? " " : "") + std::to_string(units.at(i)->index());
    }
    return result;
  };

  const auto nodes = graph->nodes();
  const auto diags = graph->diags();

  // Clear all data when graph changes.
  node_header_.clear();
  diag_header_.clear();
  border_.clear();
  node_table_.clear();
  diag_table_.clear();

  TableLine node_title;
  node_title.index = "Node";
  node_title.level = "Level";
  node_title.input = "Input";
  node_title.latch = "Latch";
  node_title.data = "Path";
  node_title.type = "Type";
  node_title.links = "Children";

  TableLine diag_title;
  diag_title.index = "Diag";
  diag_title.level = "Level";
  diag_title.input = "input";
  diag_title.latch = "     ";
  diag_title.data = "Name";
  diag_title.type = "    ";
  node_title.links = "Children";

  // Set static information.
  for (const auto & node : nodes) {
    TableLine line;
    line.index = std::to_string(node->index());
    line.data = node->path();
    line.type = node->type();
    line.links = join(node->child_units());
    node_table_.push_back(line);
  }
  for (const auto & diag : diags) {
    TableLine line;
    line.index = std::to_string(diag->index());
    line.data = diag->name();
    line.links = join(diag->child_units());
    diag_table_.push_back(line);
  }

  // Calculate table cell width.
  width_.index = std::max(node_title.index.length(), diag_title.index.length());
  width_.level = std::max(node_title.level.length(), diag_title.level.length());
  width_.input = std::max(node_title.input.length(), diag_title.input.length());
  width_.latch = std::max(node_title.latch.length(), diag_title.latch.length());
  width_.data = std::max(node_title.data.length(), diag_title.data.length());
  width_.type = std::max(node_title.type.length(), diag_title.type.length());
  width_.links = std::max(node_title.links.length(), diag_title.links.length());

  for (const auto & line : node_table_) {
    width_.data = std::max(width_.data, line.data.length());
    width_.type = std::max(width_.type, line.type.length());
    width_.links = std::max(width_.links, line.links.length());
  }
  for (const auto & line : diag_table_) {
    width_.data = std::max(width_.data, line.data.length());
    width_.type = std::max(width_.type, line.type.length());
    width_.links = std::max(width_.links, line.links.length());
  }

  // Create header and border.
  diag_header_ = diag_title.str(width_);
  node_header_ = node_title.str(width_);
  border_ = "|";
  border_ += std::string(width_.index + 2, '-') + "|";
  border_ += std::string(width_.level + 2, '-') + "|";
  border_ += std::string(width_.input + 2, '-') + "|";
  border_ += std::string(width_.latch + 2, '-') + "|";
  border_ += std::string(width_.data + 2, '-') + "|";
  border_ += std::string(width_.type + 2, '-') + "|";
  border_ += std::string(width_.links + 2, '-') + "|";
}

void DumpNode::on_update(DiagGraph::ConstSharedPtr graph)
{
  const auto text_level = [](DiagUnit::DiagnosticLevel level) {
    if (level == DiagUnit::DiagnosticStatus::OK) return "OK   ";
    if (level == DiagUnit::DiagnosticStatus::WARN) return "WARN ";
    if (level == DiagUnit::DiagnosticStatus::ERROR) return "ERROR";
    if (level == DiagUnit::DiagnosticStatus::STALE) return "STALE";
    return "-----";
  };

  const auto nodes = graph->nodes();
  const auto diags = graph->diags();

  std::cout << border_ << std::endl;
  std::cout << node_header_ << std::endl;
  std::cout << border_ << std::endl;
  for (size_t i = 0; i < nodes.size(); ++i) {
    auto & node = nodes.at(i);
    auto & line = node_table_.at(i);
    line.level = text_level(node->level());
    line.input = text_level(node->input_level());
    line.latch = text_level(node->latch_level());
    std::cout << line.str(width_) << std::endl;
  }

  std::cout << border_ << std::endl;
  std::cout << diag_header_ << std::endl;
  std::cout << border_ << std::endl;
  for (size_t i = 0; i < diags.size(); ++i) {
    auto & diag = diags.at(i);
    auto & line = diag_table_.at(i);
    line.level = text_level(diag->level());
    line.input = text_level(diag->input_level());
    std::cout << line.str(width_) << std::endl;
  }
}

std::string TableLine::str(const TableWidth & width) const
{
  std::ostringstream ss;
  ss << "| " << std::right << std::setw(width.index) << index << " ";
  ss << "| " << std::left << std::setw(width.level) << level << " ";
  ss << "| " << std::left << std::setw(width.input) << input << " ";
  ss << "| " << std::left << std::setw(width.latch) << latch << " ";
  ss << "| " << std::left << std::setw(width.data) << data << " ";
  ss << "| " << std::left << std::setw(width.type) << type << " ";
  ss << "| " << std::left << std::setw(width.links) << links << " |";
  return ss.str();
}

}  // namespace autoware::diagnostic_graph_utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_utils::DumpNode)
