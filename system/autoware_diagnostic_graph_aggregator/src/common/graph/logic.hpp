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

#ifndef COMMON__GRAPH__LOGIC_HPP_
#define COMMON__GRAPH__LOGIC_HPP_

#include "types/diagnostics.hpp"
#include "types/forward.hpp"

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class Logic
{
public:
  virtual ~Logic() = default;
  virtual std::string type() const = 0;
  virtual std::vector<LinkPort *> ports() const = 0;
  virtual DiagnosticLevel level() const = 0;
};

class LogicFactory
{
public:
  using Function = std::function<std::unique_ptr<Logic>(Parser & parser)>;
  static std::unique_ptr<Logic> Create(Parser & parser);
  static void Register(const std::string & type, Function function);

private:
  static inline std::unordered_map<std::string, Function> logics_;
};

template <typename T>
class RegisterLogic
{
public:
  explicit RegisterLogic(const std::string & type)
  {
    LogicFactory::Register(type, [](Parser & parser) { return std::make_unique<T>(parser); });
  }
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__GRAPH__LOGIC_HPP_
