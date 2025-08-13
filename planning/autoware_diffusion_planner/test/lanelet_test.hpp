// Copyright 2025 TIER IV, Inc.
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

#ifndef LANELET_TEST_HPP_
#define LANELET_TEST_HPP_

#include "autoware/diffusion_planner/conversion/lanelet.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>

#include <memory>
#include <vector>

namespace autoware::diffusion_planner::test
{

class LaneletTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a mock lanelet map
    lanelet_map_ = std::make_shared<lanelet::LaneletMap>();

    // Add a lanelet with a centerline
    centerline_ = lanelet::LineString3d(
      lanelet::utils::getId(), {
                                 lanelet::Point3d(lanelet::utils::getId(), 0.0, 0.0, 0.0),
                                 lanelet::Point3d(lanelet::utils::getId(), 10.0, 0.0, 0.0),
                                 lanelet::Point3d(lanelet::utils::getId(), 20.0, 0.0, 0.0),
                               });
    lanelet_ = lanelet::Lanelet(lanelet::utils::getId(), centerline_, centerline_);
    lanelet_.setAttribute("subtype", "road");
    lanelet_map_->add(lanelet_);
  }

  lanelet::LaneletMapPtr lanelet_map_;
  lanelet::LineString3d centerline_;
  lanelet::Lanelet lanelet_;
};

}  // namespace autoware::diffusion_planner::test

#endif  // LANELET_TEST_HPP_
