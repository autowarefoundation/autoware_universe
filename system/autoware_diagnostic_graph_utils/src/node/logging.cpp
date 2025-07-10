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

#include "logging.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_utils
{

LoggingNode::LoggingNode(const rclcpp::NodeOptions & options) : Node("logging", options)
{
  root_path_ = declare_parameter<std::string>("root_path");
  max_depth_ = declare_parameter<int>("max_depth");
  enable_terminal_log_ = declare_parameter<bool>("enable_terminal_log");
  ignore_dependent_error_ = declare_parameter<bool>("ignore_dependent_error");

  using std::placeholders::_1;
  sub_graph_.register_create_callback(std::bind(&LoggingNode::on_create, this, _1));
  sub_graph_.subscribe(*this, 1);

  pub_error_graph_text_ =
    create_publisher<StringStamped>("~/debug/error_graph_text", rclcpp::QoS(1));

  const auto period = rclcpp::Rate(declare_parameter<double>("show_rate")).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { on_timer(); });
}

void LoggingNode::on_create(DiagGraph::ConstSharedPtr graph)
{
  // Search root node.
  root_node_ = nullptr;
  for (const auto & node : graph->nodes()) {
    if (node->path() == root_path_) {
      root_node_ = node;
      return;
    }
  }
  RCLCPP_ERROR_STREAM(get_logger(), "root unit is not found: " << root_path_);
}

void LoggingNode::on_timer()
{
  static const auto prefix_message = "The target mode is not available for the following reasons:";
  if (root_node_ && root_node_->level() != DiagUnit::DiagnosticStatus::OK) {
    dump_text_.str("");
    dump_text_.clear(std::stringstream::goodbit);
    dump_unit(root_node_, 0, "");
    const auto error_graph_text = dump_text_.str();

    // show on terminal
    if (enable_terminal_log_ && error_graph_text != prev_error_graph_text_) {
      RCLCPP_WARN_STREAM_THROTTLE(
        get_logger(), *get_clock(), 3000 /* ms */,
        prefix_message << std::endl
                       << error_graph_text);
    }

    // publish debug topic
    StringStamped error_graph_message;
    error_graph_message.stamp = now();
    error_graph_message.data = error_graph_text;
    pub_error_graph_text_->publish(error_graph_message);

    // update previous value
    prev_error_graph_text_ = error_graph_text;
  } else {
    const std::string error_graph_text{""};

    // publish debug topic
    StringStamped error_graph_message;
    error_graph_message.stamp = now();
    pub_error_graph_text_->publish(error_graph_message);

    // show on terminal
    if (enable_terminal_log_ && error_graph_text != prev_error_graph_text_) {
      RCLCPP_INFO_STREAM(get_logger(), "The target mode is available now.");
    }

    // update previous value
    prev_error_graph_text_ = error_graph_text;
  }
}

void LoggingNode::dump_unit(DiagNode * node, int depth, const std::string & indent)
{
  const auto text_level = [](DiagUnit::DiagnosticLevel level) {
    if (level == DiagUnit::DiagnosticStatus::OK) return "OK   ";
    if (level == DiagUnit::DiagnosticStatus::WARN) return "WARN ";
    if (level == DiagUnit::DiagnosticStatus::ERROR) return "ERROR";
    if (level == DiagUnit::DiagnosticStatus::STALE) return "STALE";
    return "-----";
  };

  if (max_depth_ < depth) {
    return;
  }
  if (node->level() == DiagUnit::DiagnosticStatus::OK) {
    return;
  }
  if (node->is_dependent() && ignore_dependent_error_) {
    return;
  }

  std::string path = node->path();
  if (path.empty()) {
    path = "[anonymous group]";
  }

  dump_text_ << indent << "- " + path << " " << text_level(node->level()) << std::endl;
  for (const auto & child : node->child_nodes()) {
    dump_unit(child, depth + 1, indent + "    ");
  }
}

}  // namespace autoware::diagnostic_graph_utils

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::diagnostic_graph_utils::LoggingNode)
