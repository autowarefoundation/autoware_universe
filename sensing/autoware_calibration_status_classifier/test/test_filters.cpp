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

#include "autoware/calibration_status_classifier/calibration_status_classifier_filters.hpp"
#include "autoware/calibration_status_classifier/filter.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <utility>
#include <vector>

namespace autoware::calibration_status_classifier
{

class FiltersTest : public ::testing::Test
{
};

TEST_F(FiltersTest, DefaultConstructorAllPassed)
{
  CalibrationStatusClassifierFilters filters;
  auto result = filters.evaluate(100.0);
  EXPECT_TRUE(result.all_passed);
  EXPECT_TRUE(result.filter_results.empty());
}

TEST_F(FiltersTest, SingleLinearVelocityFilterBelowThreshold)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 0.0);
  vel->update(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.0);

  CalibrationStatusClassifierFilters filters(std::move(vel));
  auto result = filters.evaluate(100.0);
  EXPECT_TRUE(result.all_passed);
  ASSERT_EQ(result.filter_results.size(), 1u);
  EXPECT_EQ(result.filter_results[0].name, "Linear velocity");
  EXPECT_EQ(result.filter_results[0].unit, "m/s");
  EXPECT_TRUE(result.filter_results[0].is_passed);
  EXPECT_FALSE(result.filter_results[0].is_threshold_exceeded);
}

TEST_F(FiltersTest, SingleLinearVelocityFilterAboveThreshold)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 0.0);
  vel->update(LinearVelocityInfo{2.0, 0.0, 0.0}, 100.0);

  CalibrationStatusClassifierFilters filters(std::move(vel));
  auto result = filters.evaluate(100.0);
  EXPECT_FALSE(result.all_passed);
  ASSERT_EQ(result.filter_results.size(), 1u);
  EXPECT_FALSE(result.filter_results[0].is_passed);
  EXPECT_TRUE(result.filter_results[0].is_threshold_exceeded);
}

TEST_F(FiltersTest, AngularVelocityFilterBasic)
{
  auto ang = std::make_unique<AngularVelocityFilter>(0.1, 0.0);
  ang->update(AngularVelocityInfo{0.05, 0.0, 0.0}, 100.0);

  CalibrationStatusClassifierFilters filters(std::move(ang));
  auto result = filters.evaluate(100.0);
  EXPECT_TRUE(result.all_passed);
  ASSERT_EQ(result.filter_results.size(), 1u);
  EXPECT_EQ(result.filter_results[0].name, "Angular velocity");
  EXPECT_EQ(result.filter_results[0].unit, "rad/s");
}

TEST_F(FiltersTest, ObjectsFilterBasic)
{
  auto obj = std::make_unique<ObjectsFilter>(5, 0.0);
  obj->update({{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}}, 100.0);

  CalibrationStatusClassifierFilters filters(std::move(obj));
  auto result = filters.evaluate(100.0);
  EXPECT_TRUE(result.all_passed);
  ASSERT_EQ(result.filter_results.size(), 1u);
  EXPECT_EQ(result.filter_results[0].name, "Object count");
  EXPECT_EQ(result.filter_results[0].unit, "");
}

TEST_F(FiltersTest, TimeoutDebounce)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 2.0);

  // Exceed threshold
  vel->update(LinearVelocityInfo{2.0, 0.0, 0.0}, 100.0);
  EXPECT_FALSE(vel->is_passed(100.0));

  // Drop below threshold but within timeout
  vel->update(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.5);
  EXPECT_FALSE(vel->is_passed(100.5));

  // Still within timeout
  EXPECT_FALSE(vel->is_passed(101.5));

  // Past timeout
  EXPECT_TRUE(vel->is_passed(103.0));
}

TEST_F(FiltersTest, ZeroTimeoutImmediateRecovery)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 0.0);

  vel->update(LinearVelocityInfo{2.0, 0.0, 0.0}, 100.0);
  EXPECT_FALSE(vel->is_passed(100.0));

  vel->update(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.1);
  EXPECT_TRUE(vel->is_passed(100.1));
}

TEST_F(FiltersTest, NeverExceededAlwaysPassed)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 5.0);

  // Never exceeded - immediately passes (no last_exceeded_time_)
  vel->update(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.0);
}

TEST_F(FiltersTest, MultipleFilters)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 0.0);
  auto ang = std::make_unique<AngularVelocityFilter>(0.1, 0.0);

  vel->update(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.0);
  ang->update(AngularVelocityInfo{0.05, 0.0, 0.0}, 100.0);

  CalibrationStatusClassifierFilters filters(std::move(vel), std::move(ang));
  auto result = filters.evaluate(100.0);
  EXPECT_TRUE(result.all_passed);
  EXPECT_EQ(result.filter_results.size(), 2u);
}

TEST_F(FiltersTest, MultipleFiltersOneFailing)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 0.0);
  auto ang = std::make_unique<AngularVelocityFilter>(0.1, 0.0);

  vel->update(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.0);
  ang->update(AngularVelocityInfo{0.5, 0.0, 0.0}, 100.0);  // exceeds 0.1

  CalibrationStatusClassifierFilters filters(std::move(vel), std::move(ang));
  auto result = filters.evaluate(100.0);
  EXPECT_FALSE(result.all_passed);
}

TEST_F(FiltersTest, GetTypedFilter)
{
  std::vector<std::unique_ptr<Filter>> vec;
  vec.push_back(std::make_unique<LinearVelocityFilter>(1.0, 0.0));
  vec.push_back(std::make_unique<ObjectsFilter>(5, 0.0));

  CalibrationStatusClassifierFilters filters(std::move(vec));

  auto * vel = filters.get<LinearVelocityFilter>();
  ASSERT_NE(vel, nullptr);

  auto * obj = filters.get<ObjectsFilter>();
  ASSERT_NE(obj, nullptr);

  auto * ang = filters.get<AngularVelocityFilter>();
  EXPECT_EQ(ang, nullptr);
}

TEST_F(FiltersTest, ResultFieldsAccuracy)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.5, 3.0);
  vel->update(LinearVelocityInfo{0.8, 0.0, 0.0}, 100.0);

  auto result = vel->get_result(100.5);

  EXPECT_EQ(result.name, "Linear velocity");
  EXPECT_EQ(result.unit, "m/s");
  EXPECT_TRUE(result.is_passed);
  EXPECT_FALSE(result.is_threshold_exceeded);
  EXPECT_DOUBLE_EQ(result.current_value, 0.8);
  EXPECT_DOUBLE_EQ(result.threshold, 1.5);
  EXPECT_DOUBLE_EQ(result.timeout_sec, 3.0);
  EXPECT_NEAR(result.state_age, 0.5, 1e-9);
}

TEST_F(FiltersTest, ObjectsFilterStoresDetailedObjects)
{
  auto obj = std::make_unique<ObjectsFilter>(2, 0.0);
  std::vector<ObjectInfo> objects = {{1.0, 2.0, 3.0}, {4.0, 5.0, 6.0}, {7.0, 8.0, 9.0}};
  obj->update(objects, 100.0);

  ASSERT_EQ(obj->objects().size(), 3u);
  EXPECT_DOUBLE_EQ(obj->objects()[0].x, 1.0);
  EXPECT_DOUBLE_EQ(obj->objects()[1].y, 5.0);
  EXPECT_DOUBLE_EQ(obj->objects()[2].z, 9.0);

  // 3 objects exceeds threshold of 2
  EXPECT_FALSE(obj->is_passed(100.0));
}

TEST_F(FiltersTest, ThreadSafeUpdateAndEvaluate)
{
  auto vel = std::make_unique<LinearVelocityFilter>(1.0, 0.0);
  auto ang = std::make_unique<AngularVelocityFilter>(0.1, 0.0);

  CalibrationStatusClassifierFilters filters(std::move(vel), std::move(ang));

  // Use thread-safe update template
  filters.update<LinearVelocityFilter>(LinearVelocityInfo{0.5, 0.0, 0.0}, 100.0);
  filters.update<AngularVelocityFilter>(AngularVelocityInfo{0.05, 0.0, 0.0}, 100.0);

  auto result = filters.evaluate(100.0);
  EXPECT_TRUE(result.all_passed);
  EXPECT_EQ(result.filter_results.size(), 2u);
}

}  // namespace autoware::calibration_status_classifier
