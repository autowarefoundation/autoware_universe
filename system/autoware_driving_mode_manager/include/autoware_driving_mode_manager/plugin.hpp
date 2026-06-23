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

#ifndef AUTOWARE_DRIVING_MODE_MANAGER__PLUGIN_HPP_
#define AUTOWARE_DRIVING_MODE_MANAGER__PLUGIN_HPP_

#include "config.hpp"
#include "types.hpp"

namespace autoware::driving_mode_manager
{

class Plugin
{
public:
  virtual ~Plugin() = default;
  virtual AutowareMode decide(const RequestModes & modes, const AutowareModeSet & availables) = 0;
  virtual void setup(DrivingModeConfigInterface & config) const = 0;
};

}  // namespace autoware::driving_mode_manager

#endif  // AUTOWARE_DRIVING_MODE_MANAGER__PLUGIN_HPP_
