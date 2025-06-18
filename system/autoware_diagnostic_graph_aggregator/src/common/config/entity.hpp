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

#ifndef COMMON__CONFIG__ENTITY_HPP_
#define COMMON__CONFIG__ENTITY_HPP_

#include "config/yaml.hpp"
#include "types/forward.hpp"

#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

// cspell: ignore yamls

namespace autoware::diagnostic_graph_aggregator
{

struct GraphData
{
  std::vector<std::unique_ptr<FileData>> files;
  std::vector<std::unique_ptr<NodeUnit>> nodes;
  std::vector<std::unique_ptr<DiagUnit>> diags;
  std::vector<std::unique_ptr<LinkPort>> ports;
};

struct FileData
{
  std::string original_path;
  std::string resolved_path;
  std::vector<FileData *> files;
  std::vector<NodeUnit *> nodes;
  ConfigYaml yaml;
};

struct FileContext
{
  std::string file;
  std::shared_ptr<std::unordered_set<std::string>> visited;
};

class Parser
{
public:
  Parser(const std::string & type, ConfigYaml yaml);
  ~Parser();
  auto type() const { return type_; }
  auto yaml() const { return yaml_; }
  LinkList * parse_node_list(const std::vector<ConfigYaml> & yamls);
  LinkItem * parse_node_item(const ConfigYaml & yaml);
  LinkItem * parse_diag_item(const ConfigYaml & yaml);
  LinkItem * parse_link_node(const std::string & link);

protected:
  std::string type_;
  ConfigYaml yaml_;
  std::vector<std::unique_ptr<TempUnit>> temps_;
  std::vector<std::unique_ptr<LinkPort>> ports_;
};

class ParserWithAccess : public Parser
{
public:
  using Parser::Parser;
  std::vector<std::unique_ptr<TempUnit>> take_temps();
  std::vector<std::unique_ptr<LinkPort>> take_ports();
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__ENTITY_HPP_
