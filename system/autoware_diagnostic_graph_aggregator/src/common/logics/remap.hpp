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

#ifndef COMMON__LOGICS__REMAP_HPP_
#define COMMON__LOGICS__REMAP_HPP_

#include "graph/logic.hpp"

#include <string>
#include <vector>

namespace autoware::diagnostic_graph_aggregator
{

class RemapLogic : public Logic
{
public:
  explicit RemapLogic(Parser & parser);
  std::vector<LinkPort *> ports() const override;

protected:
  LinkItem * link_;
};

class WarnToOkLogic : public RemapLogic
{
public:
  using RemapLogic::RemapLogic;
  std::string type() const override { return "warn-to-ok"; }
  DiagnosticLevel level() const override;
};

class WarnToErrorLogic : public RemapLogic
{
public:
  using RemapLogic::RemapLogic;
  std::string type() const override { return "warn-to-error"; }
  DiagnosticLevel level() const override;
};

}  // namespace autoware::diagnostic_graph_aggregator

#endif  // COMMON__LOGICS__REMAP_HPP_
