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

#include "diagnostic_test_utils.hpp"

#include <gtest/gtest.h>

#include <vector>

class ShadowModeTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

// Test 11: Shadow filter has a configured_action -> excluded from aggregation -> status OK
TEST_F(ShadowModeTest, ShadowValidatorStatusPublishedAtOk)
{
  DiagHarness h("t11_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["shadow_mode_filter"][Action::MODERATE] = "shadow_mode_filter_status";

  // active_filter_names does not contain "shadow_mode_filter" -> excluded from aggregation
  auto diag = make_diag(*h.node, filter_map, "", {"enabled_filter"});

  auto candidate1 = make_report({make_metric("shadow_mode_filter", RiskLevel::DANGER)});
  std::vector<ValidationReport> reports = {candidate1};

  h.run(diag, reports);

  auto status = h.find("shadow_mode_filter_status");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::OK);
}

// Test 12: Active SAFE + shadow DANGER -> enabled_filter_status OK, shadow_mode_filter_status OK
TEST_F(ShadowModeTest, ActiveOkShadowDangerBothPublishOk)
{
  DiagHarness h("t12_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["enabled_filter"][Action::MODERATE] = "enabled_filter_status";
  filter_map["shadow_mode_filter"][Action::MODERATE] = "shadow_mode_filter_status";

  // only "enabled_filter" is active; "shadow_mode_filter" is excluded from action aggregation
  auto diag = make_diag(*h.node, filter_map, "", {"enabled_filter"});

  auto candidate1 = make_report(
    {make_metric("enabled_filter", RiskLevel::SAFE),
     make_metric("shadow_mode_filter", RiskLevel::DANGER)});
  std::vector<ValidationReport> reports = {candidate1};

  h.run(diag, reports);

  // enabled_filter SAFE -> action NONE -> OK
  // shadow_mode_filter excluded from aggregation -> OK
  auto se = h.find("enabled_filter_status");
  auto ss = h.find("shadow_mode_filter_status");
  ASSERT_TRUE(se.has_value());
  ASSERT_TRUE(ss.has_value());
  EXPECT_EQ(se->level, DiagnosticStatus::OK);
  EXPECT_EQ(ss->level, DiagnosticStatus::OK);
}

// Test 13: Active DANGER + shadow DANGER -> enabled_filter_status ERROR, shadow_mode_filter_status
// OK
TEST_F(ShadowModeTest, ActiveDangerShadowDangerOnlyActiveFires)
{
  DiagHarness h("t13_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["enabled_filter"][Action::MODERATE] = "enabled_filter_status";
  filter_map["shadow_mode_filter"][Action::MODERATE] = "shadow_mode_filter_status";

  // active_filter_names contains only "enabled_filter" -> "shadow_mode_filter" is excluded
  auto diag = make_diag(*h.node, filter_map, "", {"enabled_filter"});

  auto candidate1 = make_report(
    {make_metric("enabled_filter", RiskLevel::DANGER),
     make_metric("shadow_mode_filter", RiskLevel::DANGER)});
  std::vector<ValidationReport> reports = {candidate1};

  h.run(diag, reports);

  // enabled_filter DANGER -> MODERATE action -> ERROR
  // shadow_mode_filter excluded from aggregation -> OK
  auto se = h.find("enabled_filter_status");
  auto ss = h.find("shadow_mode_filter_status");
  ASSERT_TRUE(se.has_value());
  ASSERT_TRUE(ss.has_value());
  EXPECT_EQ(se->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(ss->level, DiagnosticStatus::OK);
}
