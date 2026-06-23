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

#ifndef PLUGIN__DEFAULT_HPP_
#define PLUGIN__DEFAULT_HPP_

#include <autoware_driving_mode_manager/plugin.hpp>

namespace autoware::driving_mode_manager
{

class DefaultPlugin : public Plugin
{
public:
  AutowareMode decide(const RequestModes & modes, const AutowareModeSet & availables) override;
  void setup(DrivingModeConfigInterface & config) const override;
};

}  // namespace autoware::driving_mode_manager

#endif  // PLUGIN__DEFAULT_HPP_
