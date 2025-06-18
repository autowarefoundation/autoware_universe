// Copyright 2021 Tier IV, Inc.
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

#include "autoware/kinematic_evaluator/metrics/kinematic_metrics.hpp"

namespace autoware::kinematic_diagnostics
{
namespace metrics
{

autoware_utils::Accumulator<double> updateVelocityStats(
  const double & value, const autoware_utils::Accumulator<double> stat_prev)
{
  autoware_utils::Accumulator<double> stat(stat_prev);
  stat.add(value);
  return stat;
}

}  // namespace metrics
}  // namespace autoware::kinematic_diagnostics
