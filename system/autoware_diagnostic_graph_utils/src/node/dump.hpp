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

#ifndef NODE__DUMP_HPP_
#define NODE__DUMP_HPP_

#include "autoware/diagnostic_graph_utils/subscription.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_utils
{

struct TableWidth
{
  size_t index;
  size_t level;
  size_t input;
  size_t latch;
  size_t data;
  size_t type;
  size_t links;
};

struct TableLine
{
  std::string str(const TableWidth & width) const;
  std::string index;
  std::string level;
  std::string input;
  std::string latch;
  std::string type;
  std::string data;   // node path or diag name
  std::string links;  // node children
};

class DumpNode : public rclcpp::Node
{
public:
  explicit DumpNode(const rclcpp::NodeOptions & options);

private:
  void on_create(DiagGraph::ConstSharedPtr graph);
  void on_update(DiagGraph::ConstSharedPtr graph);
  DiagGraphSubscription sub_graph_;

  TableWidth width_;
  std::string node_header_;
  std::string diag_header_;
  std::string border_;
  std::vector<TableLine> node_table_;
  std::vector<TableLine> diag_table_;
};

}  // namespace autoware::diagnostic_graph_utils

#endif  // NODE__DUMP_HPP_
