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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_UTILS_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_UTILS_HPP_

#include "autoware_trajectory_validator/msg/metric_report.hpp"
#include "autoware_trajectory_validator/msg/risk_level.hpp"

#include <algorithm>
#include <vector>

namespace autoware::trajectory_validator
{
using autoware_trajectory_validator::msg::MetricReport;
using autoware_trajectory_validator::msg::RiskLevel;
using RiskLevelType = RiskLevel::_level_type;

/**
 * @brief Returns the worst risk level from a vector of RiskLevel messages.
 * @param risks A vector of RiskLevel messages.
 * @return The worst risk level.
 */
inline RiskLevelType worst_risk_level(const std::vector<RiskLevel> & risks)
{
  RiskLevelType worst = RiskLevel::SAFE;
  for (const auto & risk : risks) {
    worst = std::max(worst, risk.level);
  }
  return worst;
}

/**
 * @brief Returns the worst risk level from a vector of MetricReport messages.
 * @param metrics A vector of MetricReport messages.
 * @return The worst risk level.
 */
inline RiskLevelType worst_risk_level(const std::vector<MetricReport> & metrics)
{
  RiskLevelType worst = RiskLevel::SAFE;
  for (const auto & metric : metrics) {
    worst = std::max(worst, metric.risk.level);
  }
  return worst;
}
}  // namespace autoware::trajectory_validator
#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__DETAIL__RISK_UTILS_HPP_
