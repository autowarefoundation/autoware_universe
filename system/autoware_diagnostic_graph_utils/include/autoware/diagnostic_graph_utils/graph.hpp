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

#ifndef AUTOWARE__DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_
#define AUTOWARE__DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_

#include <rclcpp/time.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_status.hpp>
#include <tier4_system_msgs/msg/diag_graph_struct.hpp>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::diagnostic_graph_utils
{

class DiagGraph;
class DiagUnit;
class DiagNode;
class DiagLeaf;
class DiagLink;

class DiagUnit
{
public:
  using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
  using DiagnosticLevel = DiagnosticStatus::_level_type;

  explicit DiagUnit(int index) : index_(index) {}
  virtual ~DiagUnit() = default;
  virtual DiagnosticStatus create_diagnostic_status() const = 0;
  virtual DiagnosticLevel level() const = 0;
  virtual std::string path_or_name() const = 0;

  int index() const { return index_; }
  std::vector<DiagUnit *> child_units() const;
  std::vector<DiagLink *> child_links() const;
  void add_child(DiagLink * link) { child_links_.push_back(link); }

private:
  int index_;
  std::vector<DiagLink *> child_links_;
};

class DiagLink
{
public:
  DiagLink(DiagUnit * parent, DiagUnit * child)
  {
    parent_ = parent;
    child_ = child;
  }
  DiagUnit * parent() const { return parent_; }
  DiagUnit * child() const { return child_; }

private:
  DiagUnit * parent_;
  DiagUnit * child_;
};

class DiagNode : public DiagUnit
{
public:
  using DiagNodeStruct = tier4_system_msgs::msg::DiagNodeStruct;
  using DiagNodeStatus = tier4_system_msgs::msg::DiagNodeStatus;

  explicit DiagNode(int index, const DiagNodeStruct & msg) : DiagUnit(index), struct_(msg) {}
  DiagnosticStatus create_diagnostic_status() const override;
  DiagnosticLevel level() const override { return status_.level; }
  DiagnosticLevel input_level() const { return status_.input_level; }
  DiagnosticLevel latch_level() const { return status_.latch_level; }

  void update(const DiagNodeStatus & msg) { status_ = msg; }
  std::string type() const { return struct_.type; }
  std::string path() const { return struct_.path; }
  std::string path_or_name() const override { return path(); }

private:
  DiagNodeStruct struct_;
  DiagNodeStatus status_;
};

class DiagLeaf : public DiagUnit
{
public:
  using DiagLeafStruct = tier4_system_msgs::msg::DiagLeafStruct;
  using DiagLeafStatus = tier4_system_msgs::msg::DiagLeafStatus;

  explicit DiagLeaf(int index, const DiagLeafStruct & msg) : DiagUnit(index), struct_(msg) {}
  DiagnosticStatus create_diagnostic_status() const override;
  DiagnosticLevel level() const override { return status_.level; }
  DiagnosticLevel input_level() const { return status_.input_level; }

  void update(const DiagLeafStatus & msg) { status_ = msg; }
  std::string name() const { return struct_.name; }
  std::string path_or_name() const override { return name(); }

private:
  DiagLeafStruct struct_;
  DiagLeafStatus status_;
};

class DiagGraph
{
public:
  using DiagGraphStruct = tier4_system_msgs::msg::DiagGraphStruct;
  using DiagGraphStatus = tier4_system_msgs::msg::DiagGraphStatus;
  using SharedPtr = std::shared_ptr<DiagGraph>;
  using ConstSharedPtr = std::shared_ptr<const DiagGraph>;
  bool create(const DiagGraphStruct & msg);
  bool update(const DiagGraphStatus & msg);
  rclcpp::Time created_stamp() const { return created_stamp_; }
  rclcpp::Time updated_stamp() const { return updated_stamp_; }
  std::string id() const { return id_; }
  std::vector<DiagUnit *> units() const;
  std::vector<DiagNode *> nodes() const;
  std::vector<DiagLeaf *> diags() const;
  std::vector<DiagLink *> links() const;

private:
  rclcpp::Time created_stamp_;
  rclcpp::Time updated_stamp_;
  std::string id_;
  std::vector<std::unique_ptr<DiagNode>> nodes_;
  std::vector<std::unique_ptr<DiagLeaf>> diags_;
  std::vector<std::unique_ptr<DiagLink>> links_;
  std::vector<DiagUnit *> units_;
};

}  // namespace autoware::diagnostic_graph_utils

#endif  // AUTOWARE__DIAGNOSTIC_GRAPH_UTILS__GRAPH_HPP_
