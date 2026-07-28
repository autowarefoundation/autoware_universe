// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__SYSTEM_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__SYSTEM_HPP_

#include <autoware/component_interface_specs/system.hpp>

// Re-export the core specs so universe consumers keep resolving the canonical
// (single-version-authority) type; core is the sole definition and version authority.
// MrmState was promoted from universe to core; existing consumers keep
// compiling unchanged via this re-export, since the type identity is preserved.
namespace autoware::component_interface_specs_universe::system
{
using autoware::component_interface_specs::system::ChangeAutowareControl;
using autoware::component_interface_specs::system::ChangeOperationMode;
using autoware::component_interface_specs::system::MrmState;
using autoware::component_interface_specs::system::OperationModeState;
using autoware::component_interface_specs::system::Specs;
using autoware::component_interface_specs::system::version;
}  // namespace autoware::component_interface_specs_universe::system

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS_UNIVERSE__SYSTEM_HPP_
