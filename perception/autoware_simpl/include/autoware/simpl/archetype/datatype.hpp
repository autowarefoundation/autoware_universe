// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__SIMPL__ARCHETYPE__DATATYPE_HPP_
#define AUTOWARE__SIMPL__ARCHETYPE__DATATYPE_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/map.hpp"

#include <vector>
namespace autoware::simpl::archetype
{
struct Point2D
{
  double x;
  double y;

  Point2D(double _x, double _y) : x(_x), y(_y) {}
};

using AgentStates = std::vector<AgentState>;
using AgentHistories = std::vector<AgentHistory>;
using MapPoints = std::vector<MapPoint>;
using RpeTensor = std::vector<float>;
using NodePoints = std::vector<Point2D>;
}  // namespace autoware::simpl::archetype
#endif  // AUTOWARE__SIMPL__ARCHETYPE__DATATYPE_HPP_
