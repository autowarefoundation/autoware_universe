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

#ifndef TYPES_HPP_
#define TYPES_HPP_

#include "type_alias.hpp"

namespace autoware::surround_obstacle_checker
{

struct StopObstacle
{
  bool is_point_cloud;
  double nearest_distance;
  Point nearest_point;
  UUID uuid;
};

}  // namespace autoware::surround_obstacle_checker

#endif  // TYPES_HPP_
