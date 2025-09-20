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

#include "config/yaml.hpp"

#include "config/errors.hpp"

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

ConfigYaml ConfigYaml::LoadFile(const std::string & path)
{
  return ConfigYaml(YAML::LoadFile(path));
}

ConfigYaml::ConfigYaml(const YAML::Node yaml)
{
  yaml_ = yaml;
}

std::string ConfigYaml::str() const
{
  return YAML::Dump(yaml_);
}

ConfigYaml ConfigYaml::required(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  if (!yaml_[name]) {
    throw FieldNotFound(name);
  }
  const auto node = yaml_[name];
  yaml_.remove(name);
  return ConfigYaml(node);
}

ConfigYaml ConfigYaml::optional(const std::string & name)
{
  // TODO(Takagi, Isamu): check map type.
  if (!yaml_[name]) {
    return ConfigYaml(YAML::Node(YAML::NodeType::Undefined));
  }
  const auto node = yaml_[name];
  yaml_.remove(name);
  return ConfigYaml(node);
}

std::vector<ConfigYaml> ConfigYaml::list() const
{
  if (yaml_.IsDefined() && !yaml_.IsSequence()) {
    throw InvalidType("list");
  }
  std::vector<ConfigYaml> result;
  for (const auto & node : yaml_) {
    result.push_back(ConfigYaml(node));
  }
  return result;
}

std::vector<ConfigYaml> ConfigYaml::list(const std::vector<ConfigYaml> & value) const
{
  return yaml_.IsDefined() ? list() : value;
}

bool ConfigYaml::exists() const
{
  return yaml_.IsDefined();
}

std::string ConfigYaml::text() const
{
  return yaml_.as<std::string>();
}

std::string ConfigYaml::text(const std::string & value) const
{
  return yaml_.as<std::string>(value);
}

double ConfigYaml::float64() const
{
  return yaml_.as<double>();
}

double ConfigYaml::float64(double value) const
{
  return yaml_.as<double>(value);
}

}  // namespace autoware::diagnostic_graph_aggregator
