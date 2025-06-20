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

#ifndef COMMON__LOGICS__CONST_HPP_
#define COMMON__LOGICS__CONST_HPP_

#include "graph/logic.hpp"

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class ConstLogic : public Logic
{
public:
  explicit ConstLogic(Parser & parser);
};

class ConstOkLogic : public ConstLogic
{
public:
  using ConstLogic::ConstLogic;
  std::string type() const override { return "ok"; }
  std::vector<LinkPort *> ports() const override { return {}; }
  DiagnosticLevel level() const override;
};

class ConstWarnLogic : public ConstLogic
{
public:
  using ConstLogic::ConstLogic;
  std::string type() const override { return "warn"; }
  std::vector<LinkPort *> ports() const override { return {}; }
  DiagnosticLevel level() const override;
};

class ConstErrorLogic : public ConstLogic
{
public:
  using ConstLogic::ConstLogic;
  std::string type() const override { return "error"; }
  std::vector<LinkPort *> ports() const override { return {}; }
  DiagnosticLevel level() const override;
};

class ConstStaleLogic : public ConstLogic
{
public:
  using ConstLogic::ConstLogic;
  std::string type() const override { return "stale"; }
  std::vector<LinkPort *> ports() const override { return {}; }
  DiagnosticLevel level() const override;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__LOGICS__CONST_HPP_
