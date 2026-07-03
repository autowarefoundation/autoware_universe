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

#include <string>
#include <vector>

class TrajectoryValidatorDiagnosticTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static FilterConfiguredActionsMap single_map(
    const std::string & filter, const std::string & status_name)
  {
    FilterConfiguredActionsMap filter_map;
    filter_map[filter][Action::MODERATE] = status_name;
    return filter_map;
  }
};

// Test 3: Validator passes on one candidate (SAFE) and fails on another -> min action NONE -> OK
TEST_F(TrajectoryValidatorDiagnosticTest, ValidatorPassesOnOneCandidateStatusOk)
{
  DiagHarness h("t3_node");
  auto diag = make_diag(*h.node, single_map("filter_1", "status_1"), "");

  auto candidate1 = make_report({make_metric("filter_1", RiskLevel::SAFE)});
  auto candidate2 = make_report({make_metric("filter_1", RiskLevel::HIGH_CAUTION)});
  std::vector<ValidationReport> reports = {candidate1, candidate2};

  h.run(diag, reports);

  auto status = h.find("status_1");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::OK);
}

// Test 4: Best trajectory has one configured_action validator at DANGER -> status fires at ERROR
TEST_F(TrajectoryValidatorDiagnosticTest, BestHasBindingValidatorAtError)
{
  DiagHarness h("t4_node");
  auto diag = make_diag(*h.node, single_map("filter_1", "status_1"), "");

  auto candidate1 = make_report({make_metric("filter_1", RiskLevel::DANGER)});
  std::vector<ValidationReport> reports = {candidate1};

  h.run(diag, reports);

  auto status = h.find("status_1");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
}

// Test 6: Two filters configured_action at MODERATE on the best trajectory -> both fire at ERROR
TEST_F(TrajectoryValidatorDiagnosticTest, TwoValidatorsBindingBothFire)
{
  DiagHarness h("t6_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["filter_1"][Action::MODERATE] = "status_1";
  filter_map["filter_2"][Action::MODERATE] = "status_2";

  auto diag = make_diag(*h.node, filter_map, "");

  auto candidate1 = make_report(
    {make_metric("filter_1", RiskLevel::DANGER), make_metric("filter_2", RiskLevel::DANGER)});
  std::vector<ValidationReport> reports = {candidate1};

  h.run(diag, reports);

  auto sa = h.find("status_1");
  auto sb = h.find("status_2");
  ASSERT_TRUE(sa.has_value());
  ASSERT_TRUE(sb.has_value());
  EXPECT_EQ(sa->level, DiagnosticStatus::ERROR);
  EXPECT_EQ(sb->level, DiagnosticStatus::ERROR);
}

// Test 6b: Filter fails on ALL candidates -> min action MODERATE -> status fires at ERROR
TEST_F(TrajectoryValidatorDiagnosticTest, ValidatorFailsOnAllCandidatesFires)
{
  DiagHarness h("t6b_node");
  auto diag = make_diag(*h.node, single_map("filter_1", "status_1"), "");

  auto candidate1 = make_report({make_metric("filter_1", RiskLevel::DANGER)});
  auto candidate2 = make_report({make_metric("filter_1", RiskLevel::DANGER)});
  std::vector<ValidationReport> reports = {candidate1, candidate2};

  h.run(diag, reports);

  auto status = h.find("status_1");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
}

// Test 6c: Each filter fails on a different candidate -> per-filter min is NONE -> neither fires
TEST_F(TrajectoryValidatorDiagnosticTest, EachValidatorFailsOnDifferentCandidateNeitherFires)
{
  DiagHarness h("t6c_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["filter_1"][Action::MODERATE] = "status_1";
  filter_map["filter_2"][Action::MODERATE] = "status_2";
  auto diag = make_diag(*h.node, filter_map, "");

  auto candidate1 = make_report(
    {make_metric("filter_1", RiskLevel::DANGER),  // filter_1 fails here
     make_metric("filter_2", RiskLevel::SAFE)});  // filter_2 passes here
  auto candidate2 = make_report(
    {make_metric("filter_1", RiskLevel::SAFE),      // filter_1 passes here
     make_metric("filter_2", RiskLevel::DANGER)});  // filter_2 fails here
  std::vector<ValidationReport> reports = {candidate1, candidate2};

  h.run(diag, reports);

  auto sa = h.find("status_1");
  auto sb = h.find("status_2");
  ASSERT_TRUE(sa.has_value());
  ASSERT_TRUE(sb.has_value());
  EXPECT_EQ(sa->level, DiagnosticStatus::OK);  // min action NONE (passes on candidate2)
  EXPECT_EQ(sb->level, DiagnosticStatus::OK);  // min action NONE (passes on candidate1)
}

// Test 6d: filter_1 fails on all candidates, filter_2 has one safe candidate -> only filter_1 fires
TEST_F(TrajectoryValidatorDiagnosticTest, OneAlwaysFailsOtherHasSafeCandidateOnlyAlwaysFailFires)
{
  DiagHarness h("t6d_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["filter_1"][Action::MODERATE] = "status_1";
  filter_map["filter_2"][Action::MODERATE] = "status_2";
  auto diag = make_diag(*h.node, filter_map, "");

  auto candidate1 = make_report(
    {make_metric("filter_1", RiskLevel::DANGER),    // filter_1 fails
     make_metric("filter_2", RiskLevel::DANGER)});  // filter_2 fails
  auto candidate2 = make_report(
    {make_metric("filter_1", RiskLevel::DANGER),  // filter_1 fails
     make_metric("filter_2", RiskLevel::SAFE)});  // filter_2 passes
  std::vector<ValidationReport> reports = {candidate1, candidate2};

  h.run(diag, reports);

  auto sa = h.find("status_1");
  auto sb = h.find("status_2");
  ASSERT_TRUE(sa.has_value());
  ASSERT_TRUE(sb.has_value());
  EXPECT_EQ(sa->level, DiagnosticStatus::ERROR);  // min action MODERATE (fails on both)
  EXPECT_EQ(sb->level, DiagnosticStatus::OK);     // min action NONE (passes on candidate2)
}

// Test 7: DANGER cycle then SAFE cycle -> status returns to OK
TEST_F(TrajectoryValidatorDiagnosticTest, Renew)
{
  DiagHarness h("t7_node");
  auto diag = make_diag(*h.node, single_map("filter_1", "status_1"), "");

  auto error_candidate = make_report({make_metric("filter_1", RiskLevel::DANGER)});
  h.run(diag, {error_candidate});
  {
    auto status = h.find("status_1");
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
  }

  auto ok_candidate = make_report({make_metric("filter_1", RiskLevel::SAFE)});
  h.run(diag, {ok_candidate});
  {
    auto status = h.find("status_1");
    ASSERT_TRUE(status.has_value());
    EXPECT_EQ(status->level, DiagnosticStatus::OK);
  }
}

// Test 8: (filter, action) bound to empty name -> no DiagnosticsInterface created, nothing fires
TEST_F(TrajectoryValidatorDiagnosticTest, EmptyNameNoStatusFired)
{
  DiagHarness h("t8_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["filter_1"][Action::MODERATE] = "";  // empty name — no interface created

  auto diag = make_diag(*h.node, filter_map, "");

  auto candidate1 = make_report({make_metric("filter_1", RiskLevel::DANGER)});

  spin_until(h.node, [&h]() { return h.sub->get_publisher_count() > 0; });
  h.received.clear();
  diag.update_and_publish({candidate1}, h.node->get_clock()->now());
  rclcpp::spin_some(h.node);
  rclcpp::spin_some(h.node);

  EXPECT_TRUE(h.all_statuses().empty());
}

// Test 9: No reports -> no_candidates_diag_status_name fires at ERROR
TEST_F(TrajectoryValidatorDiagnosticTest, NoReportsFiresNoCandidateName)
{
  DiagHarness h("t9_node");
  auto diag = make_diag(*h.node, FilterConfiguredActionsMap{}, "no_candidate_status");

  h.run(diag, {});

  auto status = h.find("no_candidate_status");
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->level, DiagnosticStatus::ERROR);
}

// Test 9b: No reports, empty no_candidates_diag_status_name -> nothing fires
TEST_F(TrajectoryValidatorDiagnosticTest, NoReportsEmptyNoCandidateNameNothing)
{
  DiagHarness h("t9b_node");
  auto diag = make_diag(*h.node, FilterConfiguredActionsMap{}, "");

  spin_until(h.node, [&h]() { return h.sub->get_publisher_count() > 0; });
  h.received.clear();
  diag.update_and_publish({}, h.node->get_clock()->now());
  rclcpp::spin_some(h.node);
  rclcpp::spin_some(h.node);

  EXPECT_TRUE(h.all_statuses().empty());
}

// Test 10: Two filters with distinct names -> both published every cycle (at OK when passing)
TEST_F(TrajectoryValidatorDiagnosticTest, TwoValidatorsDistinctNamesPublishedEachCycle)
{
  DiagHarness h("t10_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["filter_1"][Action::MODERATE] = "status_1";
  filter_map["filter_2"][Action::MODERATE] = "status_2";
  auto diag = make_diag(*h.node, filter_map, "");

  auto candidate1 = make_report(
    {make_metric("filter_1", RiskLevel::SAFE), make_metric("filter_2", RiskLevel::SAFE)});
  std::vector<ValidationReport> reports = {candidate1};

  h.run(diag, reports);

  auto sa = h.find("status_1");
  auto sb = h.find("status_2");
  ASSERT_TRUE(sa.has_value()) << "status_1 must be published every cycle";
  ASSERT_TRUE(sb.has_value()) << "status_2 must be published every cycle";
  EXPECT_EQ(sa->level, DiagnosticStatus::OK);
  EXPECT_EQ(sb->level, DiagnosticStatus::OK);
}

// Test 10b: One filter with MODERATE + EMERGENCY configured_actions fires only the matching level.
// DANGER -> MODERATE action: only MODERATE fires. FATAL -> EMERGENCY action: only EMERGENCY fires.
TEST_F(TrajectoryValidatorDiagnosticTest, MultiBindingSameValidatorThresholdExactMatch)
{
  DiagHarness h("t10b_node");
  FilterConfiguredActionsMap filter_map;
  filter_map["filter_1"][Action::MODERATE] = "status_moderate";
  filter_map["filter_1"][Action::EMERGENCY] = "status_emergency";
  auto diag = make_diag(*h.node, filter_map, "");

  // DANGER -> MODERATE action: only status_moderate fires
  auto candidate_danger = make_report({make_metric("filter_1", RiskLevel::DANGER)});
  h.run(diag, {candidate_danger});
  {
    auto sm = h.find("status_moderate");
    auto se = h.find("status_emergency");
    ASSERT_TRUE(sm.has_value());
    ASSERT_TRUE(se.has_value());
    EXPECT_EQ(sm->level, DiagnosticStatus::ERROR);  // exact match: MODERATE fires
    EXPECT_EQ(se->level, DiagnosticStatus::OK);     // no match: EMERGENCY does not fire
  }

  // FATAL -> EMERGENCY action: only status_emergency fires
  auto candidate_fatal = make_report({make_metric("filter_1", RiskLevel::FATAL)});
  h.run(diag, {candidate_fatal});
  {
    auto sm = h.find("status_moderate");
    auto se = h.find("status_emergency");
    ASSERT_TRUE(sm.has_value());
    ASSERT_TRUE(se.has_value());
    EXPECT_EQ(sm->level, DiagnosticStatus::OK);     // no match: MODERATE does not fire
    EXPECT_EQ(se->level, DiagnosticStatus::ERROR);  // exact match: EMERGENCY fires
  }
}
