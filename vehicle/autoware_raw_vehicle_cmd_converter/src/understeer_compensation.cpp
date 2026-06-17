//  Copyright 2026 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "autoware_raw_vehicle_cmd_converter/understeer_compensation.hpp"

namespace autoware::raw_vehicle_cmd_converter
{

void UndersteerCompensation::setUndersteerParams(const double k_us, const double wheelbase)
{
  k_us_ = k_us;
  wheelbase_ = wheelbase;
}

double UndersteerCompensation::calculateUndersteerRatio(const double vel) const
{
  if (vel <= 0.0) return 1.0;
  return 1.0 + k_us_ * vel * vel / wheelbase_;
}

double UndersteerCompensation::calculateSteeringTireState(
  const double vel, const double steer_status) const
{
  return steer_status / calculateUndersteerRatio(vel);
}

}  // namespace autoware::raw_vehicle_cmd_converter
