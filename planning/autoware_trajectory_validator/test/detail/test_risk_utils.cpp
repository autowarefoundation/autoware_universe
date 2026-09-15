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

#include "autoware/trajectory_validator/detail/risk_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace autoware::trajectory_validator
{
namespace
{

RiskLevel make_risk(const RiskLevelType level)
{
  RiskLevel risk;
  risk.level = level;
  return risk;
}

MetricReport make_metric(const RiskLevelType level)
{
  MetricReport metric;
  metric.risk = make_risk(level);
  return metric;
}

TEST(RiskUtilsTest, RiskLevelArrayReturnsSafeForEmptyInput)
{
  EXPECT_EQ(worst_risk_level(std::vector<RiskLevel>{}), RiskLevel::SAFE);
}

TEST(RiskUtilsTest, RiskLevelArrayReturnsWorstRisk)
{
  const std::vector<RiskLevel> risks{
    make_risk(RiskLevel::SAFE), make_risk(RiskLevel::HIGH_CAUTION),
    make_risk(RiskLevel::LOW_CAUTION)};

  EXPECT_EQ(worst_risk_level(risks), RiskLevel::HIGH_CAUTION);
}

TEST(RiskUtilsTest, RiskLevelArrayTreatsFatalAsWorst)
{
  const std::vector<RiskLevel> risks{
    make_risk(RiskLevel::DANGER), make_risk(RiskLevel::FATAL), make_risk(RiskLevel::HIGH_CAUTION)};

  EXPECT_EQ(worst_risk_level(risks), RiskLevel::FATAL);
}

TEST(RiskUtilsTest, MetricReportArrayReturnsSafeForEmptyInput)
{
  EXPECT_EQ(worst_risk_level(std::vector<MetricReport>{}), RiskLevel::SAFE);
}

TEST(RiskUtilsTest, MetricReportArrayReturnsWorstRisk)
{
  const std::vector<MetricReport> metrics{
    make_metric(RiskLevel::LOW_CAUTION), make_metric(RiskLevel::DANGER),
    make_metric(RiskLevel::HIGH_CAUTION)};

  EXPECT_EQ(worst_risk_level(metrics), RiskLevel::DANGER);
}

TEST(RiskUtilsTest, MetricReportArrayTreatsFatalAsWorst)
{
  const std::vector<MetricReport> metrics{
    make_metric(RiskLevel::SAFE), make_metric(RiskLevel::DANGER), make_metric(RiskLevel::FATAL)};

  EXPECT_EQ(worst_risk_level(metrics), RiskLevel::FATAL);
}

}  // namespace
}  // namespace autoware::trajectory_validator
