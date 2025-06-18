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

#ifndef COMMON__CONFIG__YAML_HPP_
#define COMMON__CONFIG__YAML_HPP_

#include <yaml-cpp/yaml.h>

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class ConfigYaml
{
public:
  static ConfigYaml LoadFile(const std::string & path);
  explicit ConfigYaml(const YAML::Node yaml = {});
  std::string str() const;

  ConfigYaml required(const std::string & name);
  ConfigYaml optional(const std::string & name);
  bool exists() const;

  std::vector<ConfigYaml> list() const;
  std::vector<ConfigYaml> list(const std::vector<ConfigYaml> & value) const;
  std::string text() const;
  std::string text(const std::string & value) const;
  double float64() const;
  double float64(double value) const;

private:
  YAML::Node yaml_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__CONFIG__YAML_HPP_
