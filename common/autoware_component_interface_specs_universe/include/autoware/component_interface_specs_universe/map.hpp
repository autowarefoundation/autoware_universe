// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__MAP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__MAP_HPP_

#include <autoware/component_interface_specs/map.hpp>

// Re-export the core specs so universe consumers keep resolving the canonical
// (single-version-authority) type; core is the sole definition and version authority.
// PointCloudMap is intentionally NOT re-exported: its spec (/map/point_cloud_map)
// is a never-published topic with no universe consumer, and it is excluded from the
// versioned Specs tuple anyway, since it is a raw, high-bandwidth topic that
// core deliberately keeps out of the versioned spec set.
namespace autoware::component_interface_specs_universe::map
{
using autoware::component_interface_specs::map::GetDifferentialPointCloudMap;
using autoware::component_interface_specs::map::GetPartialPointCloudMap;
using autoware::component_interface_specs::map::MapProjectorInfo;
using autoware::component_interface_specs::map::Specs;
using autoware::component_interface_specs::map::VectorMap;
using autoware::component_interface_specs::map::version;
}  // namespace autoware::component_interface_specs_universe::map

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__MAP_HPP_
