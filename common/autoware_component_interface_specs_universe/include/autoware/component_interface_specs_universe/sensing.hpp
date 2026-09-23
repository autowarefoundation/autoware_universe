// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__SENSING_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__SENSING_HPP_

#include <autoware/component_interface_specs/sensing.hpp>

// Re-export the core specs so universe consumers keep resolving the canonical
// (single-version-authority) type; core is the sole definition and version authority.
namespace autoware::component_interface_specs_universe::sensing
{
using autoware::component_interface_specs::sensing::Specs;
using autoware::component_interface_specs::sensing::VehicleVelocityConverterTwist;
using autoware::component_interface_specs::sensing::version;
}  // namespace autoware::component_interface_specs_universe::sensing

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__SENSING_HPP_
