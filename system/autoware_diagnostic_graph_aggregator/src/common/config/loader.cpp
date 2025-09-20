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

#include "config/loader.hpp"

#include "config/entity.hpp"
#include "config/errors.hpp"
#include "config/substitutions.hpp"
#include "graph/diags.hpp"
#include "graph/links.hpp"
#include "graph/nodes.hpp"
#include "graph/units.hpp"

#include <deque>
#include <filesystem>
#include <memory>
#include <queue>
#include <regex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// cspell: ignore yamls

namespace autoware::diagnostic_graph_aggregator
{

ConfigLoader::ConfigLoader(std::shared_ptr<Logger> logger)
{
  logger_ = logger ? logger : std::make_shared<DummyLogger>();
}

ConfigLoader::~ConfigLoader()
{
}

GraphData ConfigLoader::Load(const std::string & path, std::shared_ptr<Logger> logger)
{
  ConfigLoader loader(logger);
  loader.load(path);
  return loader.take();
}

GraphData ConfigLoader::take()
{
  return GraphData{std::move(files_), std::move(nodes_), std::move(diags_), std::move(ports_)};
}

void ConfigLoader::load(const std::string & path)
{
  // Load functions.
  logger_->info("Load step: load_file_tree");
  load_file_tree(path);
  logger_->info("Load step: make_node_units");
  make_node_units();
  logger_->info("Load step: make_diag_units");
  make_diag_units();
  logger_->info("Load step: resolve_links");
  resolve_links();
  logger_->info("Load step: topological_sort");
  topological_sort();

  // Edit functions.
  logger_->info("Load step: apply_remove_edits");
  apply_remove_edits();

  // Finalize functions.
  logger_->info("Load step: finalize");
  finalize();
  logger_->info("Load step: validate");
  validate();
  logger_->info("Load step: completed");
}

FileData * ConfigLoader::load_file(const FileContext & context, const std::string & path)
{
  const auto resolved_path = substitutions::evaluate(path, context);
  const auto [iter, success] = context.visited->insert(resolved_path);
  if (!success) {
    throw SameFileFound(resolved_path);
  }
  if (!std::filesystem::exists(resolved_path)) {
    throw FileNotFound(resolved_path);
  }

  const auto load_files = [this](const FileContext & context, ConfigYaml yaml) {
    std::vector<FileData *> result;
    for (ConfigYaml node : yaml.optional("files").list()) {
      result.push_back(load_file(context, node.required("path").text()));
    }
    return result;
  };

  const auto result = files_.emplace_back(std::make_unique<FileData>()).get();
  result->original_path = path;
  result->resolved_path = resolved_path;
  result->yaml = ConfigYaml::LoadFile(result->resolved_path);
  result->files = load_files(FileContext{resolved_path, context.visited}, result->yaml);
  return result;
}

BaseUnit * ConfigLoader::load_unit(ConfigYaml yaml)
{
  const auto type = yaml.required("type").text();
  if (type == "link") {
    return links_.emplace_back(std::make_unique<LinkUnit>(yaml)).get();
  }
  ParserWithAccess parser(type, yaml);
  const auto result = nodes_.emplace_back(std::make_unique<NodeUnit>(parser)).get();
  for (auto & unit : parser.take_temps()) temps_.push_back(std::move(unit));
  for (auto & port : parser.take_ports()) ports_.push_back(std::move(port));
  return result;
}

BaseUnit * ConfigLoader::load_diag(ConfigYaml yaml, const std::string & name)
{
  return diags_.emplace_back(std::make_unique<DiagUnit>(yaml, name)).get();
}

void ConfigLoader::load_file_tree(const std::string & path)
{
  const auto visited = std::make_shared<std::unordered_set<std::string>>();
  root_file_ = load_file(FileContext{"ROOT_FILE", visited}, path);
}

void ConfigLoader::make_node_units()
{
  for (const auto & file : files_) {
    const auto yamls = file->yaml.optional("units").list();
    for (ConfigYaml yaml : yamls) {
      load_unit(yaml);
    }
  }

  // The vector size increases dynamically by the load_unit function.
  for (size_t i = 0; i < ports_.size(); ++i) {
    const auto & port = ports_.at(i);
    for (auto & unit : port->units_) {
      if (const auto temp = dynamic_cast<TempNode *>(unit)) {
        unit = load_unit(temp->yaml());  // Replace temp unit link.
      }
    }
  }

  // Remove replaced units.
  temps_ = release<TempNode>(std::move(temps_));
}

void ConfigLoader::make_diag_units()
{
  const auto take_diag_name = [](ConfigYaml yaml) {
    const auto diag_node = yaml.required("node").text();
    const auto diag_name = yaml.required("name").text();
    const auto sep = diag_node.empty() ? "" : ": ";
    return diag_node + sep + diag_name;
  };

  // The diag units with the same name will be merged.
  std::unordered_set<std::string> names;
  for (const auto & port : ports_) {
    for (auto & unit : port->units_) {
      if (const auto temp = dynamic_cast<TempDiag *>(unit)) {
        const auto yaml = temp->yaml();
        const auto name = take_diag_name(yaml);
        const auto [iter, success] = names.insert(name);
        if (!success) {
          throw SameDiagFound(name);
        }
        unit = load_diag(yaml, name);  // Replace temp unit link.
      }
    }
  }

  // Remove replaced units.
  temps_ = release<TempDiag>(std::move(temps_));
}

void ConfigLoader::resolve_links()
{
  // Create mapping from path to unit.
  std::unordered_map<std::string, BaseUnit *> paths;
  for (const auto & node : nodes_) {
    const auto path = node->path();
    if (!path.empty()) {
      const auto [iter, success] = paths.insert(std::make_pair(path, node.get()));
      if (!success) {
        throw PathConflict(path);
      }
    }
  }
  for (const auto & link : links_) {
    const auto path = link->path();
    if (!path.empty()) {
      const auto [iter, success] = paths.insert(std::make_pair(path, link.get()));
      if (!success) {
        throw PathConflict(path);
      }
    }
  }

  // Create mapping from link to unit.
  std::unordered_map<LinkUnit *, BaseUnit *> links;
  for (auto & link : raws(links_)) {
    const auto target = link->link();
    if (!paths.count(target)) {
      throw PathNotFound(target);
    }
    links[link] = paths.at(target);
  }

  // Resolve link units.
  for (const auto & start : raws(links_)) {
    std::unordered_set<LinkUnit *> visited;
    BaseUnit * unit = start;
    while (const auto link = dynamic_cast<LinkUnit *>(unit)) {
      const auto [iter, success] = visited.insert(link);
      if (!success) {
        throw LinkLoopFound(link->link());
      }
      unit = links.at(link);
    }
    for (const auto & link : visited) {
      links[link] = unit;
    }
  }
  for (const auto & port : ports_) {
    for (auto & unit : port->units_) {
      // TODO(Takagi, Isamu): remove cppcheck-suppress after fixing the issue.
      // cppcheck-suppress constVariablePointer
      if (const auto link = dynamic_cast<LinkUnit *>(unit)) {
        unit = links.at(link);  // Replace link unit link.
      }
    }
  }

  // Remove replaced link units.
  links_.clear();
}

void ConfigLoader::topological_sort()
{
  std::deque<NodeUnit *> result;
  {
    std::unordered_map<NodeUnit *, int> degrees;
    std::deque<NodeUnit *> buffer;

    // Count degrees of each node.
    for (const auto & node : raws(nodes_)) {
      for (const auto & child : node->child_nodes()) {
        ++degrees[child];
      }
    }

    // Find initial nodes that are zero degrees.
    // Do not use "at" function because the zero degree nodes are not set.
    for (const auto & node : raws(nodes_)) {
      if (degrees[node] == 0) {
        buffer.push_back(node);
      }
    }

    // Sort by topological order.
    while (!buffer.empty()) {
      const auto node = buffer.front();
      buffer.pop_front();
      for (const auto & child : node->child_nodes()) {
        if (--degrees[child] == 0) {
          buffer.push_back(child);
        }
      }
      result.push_back(node);
    }
  }

  // Detect circulation because the result does not include the nodes on the loop.
  if (result.size() != nodes_.size()) {
    throw UnitLoopFound("detect unit loop");
  }

  // Update the nodes order by the topological sort result.
  std::unordered_map<NodeUnit *, std::unique_ptr<NodeUnit>> mapping;
  for (auto & node : nodes_) {
    mapping[node.get()] = std::move(node);
  }
  std::vector<std::unique_ptr<NodeUnit>> nodes;
  for (auto & node : result) {
    nodes.push_back(std::move(mapping.at(node)));
  }
  nodes_ = std::move(nodes);
}

void ConfigLoader::apply_remove_edits()
{
  const auto list_edits = [this]() {
    std::vector<ConfigYaml> result;
    for (const auto & file : files_) {
      const auto edits = file->yaml.optional("edits").list();
      result.insert(result.end(), edits.begin(), edits.end());
    }
    return result;
  };

  // List all node paths.
  std::unordered_map<std::string, bool> remove_paths;
  for (const auto & node : nodes_) {
    const auto path = node->path();
    if (!path.empty()) {
      remove_paths[path] = false;
    }
  }

  // Mark target paths to be removed.
  const auto remove_by_regex = [&remove_paths](ConfigYaml & yaml) {
    const auto raw_regex = yaml.optional("regex").text("");
    if (raw_regex.empty()) {
      return false;
    }
    const std::regex regex{raw_regex};
    bool is_any_removed = false;
    for (const auto & [path, flag] : remove_paths) {
      if (std::regex_match(path, regex)) {
        remove_paths[path] = true;
        is_any_removed = true;
      }
    }
    if (!is_any_removed) {
      throw PathNotFound(raw_regex);
    } else {
      return true;
    }
  };
  const auto remove_by_fullpath = [&remove_paths](ConfigYaml & yaml) {
    const auto path = yaml.required("path").text();
    if (remove_paths.count(path) == 0) {
      throw PathNotFound(path);
    } else {
      remove_paths[path] = true;
      return true;
    }
  };
  for (ConfigYaml yaml : list_edits()) {
    const auto type = yaml.required("type").text();
    if (type == "remove") {
      remove_by_regex(yaml) || remove_by_fullpath(yaml);
    }
  }

  // Mark target nodes to be removed.
  std::unordered_set<BaseUnit *> remove_nodes;
  for (auto & node : raws(nodes_)) {
    const auto path = node->path();
    if (!path.empty() && remove_paths.at(path)) {
      remove_nodes.insert(node);
    }
  }

  // Remove ports used by the target nodes.
  {
    std::unordered_set<LinkPort *> remove_ports;
    for (const auto & node : remove_nodes) {
      for (const auto & port : node->ports()) {
        remove_ports.insert(port);
      }
    }
    ports_ = filter(std::move(ports_), remove_ports);
  }

  // Remove links to the target nodes.
  for (const auto & port : ports_) {
    port->units_ = filter(port->units_, remove_nodes);
  }

  // Remove target nodes.
  nodes_ = filter(std::move(nodes_), remove_nodes);
}

void ConfigLoader::finalize()
{
  std::unordered_map<BaseUnit *, std::vector<BaseUnit *>> parents;
  for (const auto & parent : raws(nodes_)) {
    for (const auto & child : parent->child_units()) {
      parents[child].push_back(parent);
    }
  }

  int index = 0;
  for (const auto & node : raws(nodes_)) node->finalize(index++, parents[node]);
  for (const auto & diag : raws(diags_)) diag->finalize(index++, parents[diag]);
}

void ConfigLoader::validate()
{
  std::unordered_set<BaseUnit *> units;
  for (const auto & node : raws(nodes_)) units.insert(node);
  for (const auto & diag : raws(diags_)) units.insert(diag);

  int dead_links = 0;
  for (const auto & port : ports_) {
    for (const auto & unit : port->iterate()) {
      if (!units.count(unit)) {
        ++dead_links;
      }
    }
  }

  std::unordered_map<BaseUnit *, int> unit_used;
  for (const auto & port : ports_) {
    for (const auto & unit : port->iterate()) {
      ++unit_used[unit];
    }
  }

  std::unordered_map<LinkPort *, int> port_used;
  for (const auto & node : nodes_) {
    for (const auto & port : node->ports()) {
      ++port_used[port];
    }
  }

  std::ostringstream ss;
  ss << "==================== validate ====================" << std::endl;
  ss << "temps.size: " << temps_.size() << std::endl;
  ss << "links.size: " << links_.size() << std::endl;
  ss << "Dead links: " << dead_links << std::endl;
  ss << "Unused diags: " << std::endl;
  for (const auto & diag : raws(diags_)) {
    if (unit_used.count(diag) == 0) {
      ss << " - " << diag->name() << std::endl;
    }
  }
  ss << "Unused ports: " << std::endl;
  for (const auto & port : raws(ports_)) {
    if (port_used.count(port) == 0) {
      ss << " - " << port << std::endl;
    }
  }
  ss << "===================================================" << std::endl;
  logger_->debug(ss.str());
}

}  // namespace autoware::diagnostic_graph_aggregator
