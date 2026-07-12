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

#ifndef AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_CONTEXT_HPP_
#define AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_CONTEXT_HPP_
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

namespace autoware::trajectory_modifier
{
struct TrajectoryModifierContext
{
  explicit TrajectoryModifierContext(autoware::agnocast_wrapper::Node * node)
  : vehicle_info(autoware::vehicle_info_utils::VehicleInfoUtils(*node).getVehicleInfo()),
    tf_buffer{node->get_clock()},
    tf_listener{tf_buffer, *node}
  {
  }

  autoware::vehicle_info_utils::VehicleInfo vehicle_info;
  autoware::agnocast_wrapper::Buffer tf_buffer;
  autoware::agnocast_wrapper::TransformListener tf_listener;
};
}  // namespace autoware::trajectory_modifier
#endif  // AUTOWARE__TRAJECTORY_PROCESSOR__TRAJECTORY_MODIFIER_CONTEXT_HPP_
