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

#include "autoware/multi_object_tracker/object_model/object_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::multi_object_tracker::object_model
{
namespace
{
double initial_vel_long_stddev(const ObjectModel & model)
{
  return std::sqrt(model.initial_covariance.vel_long);
}
}  // namespace

TEST(ObjectModelTest, InitialVelocityStddevIsBoundedByProcessLimit)
{
  for (const auto * model :
       {&general_vehicle, &normal_vehicle, &big_vehicle, &bicycle, &pedestrian}) {
    EXPECT_GT(model->initial_covariance.vel_long, 0.0);
    EXPECT_LE(initial_vel_long_stddev(*model), model->process_limit.vel_long_max);
  }
}
}  // namespace autoware::multi_object_tracker::object_model
