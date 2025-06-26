// Copyright 2023 The Autoware Contributors
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

#ifndef COMMON__GRAPH__UNITS_HPP_
#define COMMON__GRAPH__UNITS_HPP_

#include "config/yaml.hpp"
#include "types/diagnostics.hpp"
#include "types/forward.hpp"

#include <rclcpp/time.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class BaseUnit
{
public:
  virtual ~BaseUnit() = default;
  virtual DiagnosticLevel level() const = 0;
  virtual std::vector<LinkPort *> ports() const = 0;
  virtual std::string debug() const { return "NotImplemented"; }

  void finalize(int index, std::vector<BaseUnit *> parents);
  int index() const { return index_; }
  std::vector<BaseUnit *> child_units() const;
  std::vector<NodeUnit *> child_nodes() const;
  std::vector<BaseUnit *> parent_units() const;

private:
  int index_;
  std::vector<BaseUnit *> parents_;
};

class TempUnit : public BaseUnit
{
public:
  DiagnosticLevel level() const override { throw std::logic_error("TempUnit"); }
  std::vector<LinkPort *> ports() const override { return {}; }
};

class TempNode : public TempUnit
{
public:
  explicit TempNode(ConfigYaml yaml) { yaml_ = yaml; }
  ConfigYaml yaml() const { return yaml_; }

private:
  ConfigYaml yaml_;
};

class TempDiag : public TempUnit
{
public:
  explicit TempDiag(ConfigYaml yaml) { yaml_ = yaml; }
  ConfigYaml yaml() const { return yaml_; }

private:
  ConfigYaml yaml_;
};

class LinkUnit : public TempUnit
{
public:
  explicit LinkUnit(ConfigYaml yaml);
  std::string path() const { return path_; }
  std::string link() const { return link_; }

private:
  std::string path_;
  std::string link_;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__UNITS_HPP_
