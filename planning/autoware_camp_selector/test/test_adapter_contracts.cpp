#include <autoware/camp_selector/adapter_contracts.hpp>

#include <gtest/gtest.h>

#include <limits>

using namespace autoware::camp_selector;

TEST(AdapterContract, OnlyPublishedSelectedRowAdvancesContinuity)
{
  PublishedPlanLedger<int> ledger;
  const auto id = ledger.stage(1.0, {10, 20, 70});
  EXPECT_FALSE(ledger.previous());
  EXPECT_EQ(*ledger.candidate(id, 2), 70);
  EXPECT_FALSE(ledger.confirm_published(id + 1, 2));
  EXPECT_FALSE(ledger.confirm_published(id, 3));
  EXPECT_TRUE(ledger.confirm_published(id, 2));
  EXPECT_EQ(ledger.previous()->states, 70);
  EXPECT_DOUBLE_EQ(ledger.previous()->origin_seconds, 1.0);
  EXPECT_FALSE(ledger.confirm_published(id, 0));
}

TEST(AdapterContract, SupersededFailedAndResetPoolsCannotCommit)
{
  PublishedPlanLedger<int> ledger;
  const auto first = ledger.stage(10.0, {0, 7});
  const auto newer = ledger.stage(10.1, {1, 8});
  EXPECT_FALSE(ledger.confirm_published(first, 1));
  EXPECT_TRUE(ledger.confirm_published(newer, 1));
  const auto failed = ledger.stage(10.2, {2, 9});
  ledger.discard_pending();
  EXPECT_FALSE(ledger.confirm_published(failed, 1));
  EXPECT_EQ(ledger.previous()->states, 8);
  EXPECT_THROW(ledger.stage(9.0, {1}), std::invalid_argument);
  ledger.reset_episode();
  const auto replay = ledger.stage(10.1, {3, 4});
  EXPECT_GT(replay, failed);
  EXPECT_FALSE(ledger.confirm_published(newer, 1));
  EXPECT_FALSE(ledger.previous());
  EXPECT_TRUE(ledger.confirm_published(replay, 1));
  EXPECT_EQ(ledger.previous()->states, 4);
}

TEST(AdapterContract, RejectUnsupportedGeneratorChanges)
{
  const std::vector<double> temperatures{0, 1, 1, 1, 1, 1, 1, 1};
  EXPECT_NO_THROW(validate_fixed_dp_contract(8, 8, temperatures, false, false, "multi_step"));
  EXPECT_THROW(
    validate_fixed_dp_contract(8, 8, temperatures, true, false, "multi_step"),
    std::invalid_argument);
  EXPECT_THROW(
    validate_fixed_dp_contract(8, 8, temperatures, false, true, "multi_step"),
    std::invalid_argument);
  EXPECT_THROW(
    validate_fixed_dp_contract(8, 8, temperatures, false, false, "single_step"),
    std::invalid_argument);
  EXPECT_THROW(
    validate_fixed_dp_contract(8, 1, temperatures, false, false, "multi_step"),
    std::invalid_argument);
  auto reordered = temperatures;
  reordered[0] = 1;
  reordered[1] = 0;
  EXPECT_THROW(
    validate_fixed_dp_contract(8, 8, reordered, false, false, "multi_step"), std::invalid_argument);
}

static std::unordered_map<std::string, std::vector<float>> raw_inputs()
{
  return {
    {"lanes",
     std::vector<float>(2 * NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM, 0)},
    {"route_lanes",
     std::vector<float>(2 * NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM, 0)},
    {"route_lanes_speed_limit", std::vector<float>(2 * NUM_SEGMENTS_IN_ROUTE, 13.9F)}};
}

TEST(AdapterContract, CaptureFirstBatchWithoutNormalizingOrReadingLaterBatch)
{
  auto inputs = raw_inputs();
  inputs.at("lanes")[0] = 12.5F;
  inputs.at("route_lanes")[0] = -3.0F;
  inputs.at("route_lanes")[NUM_SEGMENTS_IN_ROUTE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM + 10] =
    1.0F;
  auto captured = capture_raw_tensor_context(inputs);
  EXPECT_EQ(captured.lanes.size(), NUM_SEGMENTS_IN_LANE * POINTS_PER_SEGMENT * SEGMENT_POINT_DIM);
  EXPECT_EQ(captured.route_speed_limits.size(), NUM_SEGMENTS_IN_ROUTE);
  EXPECT_FLOAT_EQ(captured.lanes[0], 12.5F);
  EXPECT_FLOAT_EQ(captured.route_lanes[0], -3.0F);
  EXPECT_FLOAT_EQ(captured.route_speed_limits[0], 13.9F);
  EXPECT_FALSE(captured.route_has_traffic_light);
  for (int channel = 8; channel <= 11; ++channel) {
    auto light = raw_inputs();
    light.at("route_lanes")[channel] = 1;
    EXPECT_TRUE(capture_raw_tensor_context(light).route_has_traffic_light);
  }
  auto no_light = raw_inputs();
  no_light.at("route_lanes")[12] = 1;
  EXPECT_FALSE(capture_raw_tensor_context(no_light).route_has_traffic_light);
}

TEST(AdapterContract, MissingAndTruncatedTensorsAreRejected)
{
  auto inputs = raw_inputs();
  inputs.erase("lanes");
  EXPECT_THROW(capture_raw_tensor_context(inputs), std::out_of_range);
  inputs = raw_inputs();
  inputs.at("route_lanes_speed_limit").resize(24);
  EXPECT_THROW(capture_raw_tensor_context(inputs), std::invalid_argument);
  inputs = raw_inputs();
  inputs.at("route_lanes").resize(2);
  EXPECT_THROW(capture_raw_tensor_context(inputs), std::invalid_argument);
}
