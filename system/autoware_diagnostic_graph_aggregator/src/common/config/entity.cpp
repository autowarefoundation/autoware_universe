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

#include "config/entity.hpp"

#include "graph/links.hpp"
#include "graph/units.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

// cspell: ignore yamls

namespace autoware::diagnostic_graph_aggregator
{

Parser::Parser(const std::string & type, ConfigYaml yaml) : type_(type), yaml_(yaml)
{
}

Parser::~Parser()
{
}

LinkList * Parser::parse_node_list(const std::vector<ConfigYaml> & yamls)
{
  std::vector<BaseUnit *> units;
  for (const auto & yaml : yamls) {
    units.push_back(temps_.emplace_back(std::make_unique<TempNode>(yaml)).get());
  }
  auto list = std::make_unique<LinkList>(units);
  auto port = list.get();
  ports_.push_back(std::move(list));
  return port;
}

LinkItem * Parser::parse_node_item(const ConfigYaml & yaml)
{
  auto unit = temps_.emplace_back(std::make_unique<TempNode>(yaml)).get();
  auto item = std::make_unique<LinkItem>(unit);
  auto port = item.get();
  ports_.push_back(std::move(item));
  return port;
}

LinkItem * Parser::parse_diag_item(const ConfigYaml & yaml)
{
  auto unit = temps_.emplace_back(std::make_unique<TempDiag>(yaml)).get();
  auto item = std::make_unique<LinkItem>(unit);
  auto port = item.get();
  ports_.push_back(std::move(item));
  return port;
}

std::vector<std::unique_ptr<TempUnit>> ParserWithAccess::take_temps()
{
  return std::move(temps_);
}

std::vector<std::unique_ptr<LinkPort>> ParserWithAccess::take_ports()
{
  return std::move(ports_);
}

LinkItem * Parser::parse_link_node(const std::string & link)
{
  YAML::Node yaml;
  yaml["type"] = "link";
  yaml["link"] = link;
  return parse_node_item(ConfigYaml(yaml));
}

}  // namespace autoware::diagnostic_graph_aggregator
