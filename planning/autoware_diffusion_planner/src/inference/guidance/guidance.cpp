// Copyright 2026 TIER IV, Inc.
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

#include "autoware/diffusion_planner/inference/guidance/guidance.hpp"

namespace autoware::diffusion_planner
{

void Guidance::set_enabled(bool enabled)
{
  enabled_ = enabled;
}

bool Guidance::is_enabled() const
{
  return enabled_;
}

}  // namespace autoware::diffusion_planner
