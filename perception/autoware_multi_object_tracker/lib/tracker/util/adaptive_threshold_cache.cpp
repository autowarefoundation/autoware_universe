// Copyright 2025 Tier IV, Inc.
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

#include "autoware/multi_object_tracker/tracker/util/adaptive_threshold_cache.hpp"

#include <algorithm>
#include <cmath>

namespace autoware::multi_object_tracker
{

AdaptiveThresholdCache::AdaptiveThresholdCache()
{
  initializeDistanceInfluenceTable();
  initializeBEVAreaInfluenceTable();
}

void AdaptiveThresholdCache::initializeDistanceInfluenceTable()
{
  // distance influence as polynomial curve
  // range from 0.11 to 4.18 (capped at MAX_DISTANCE)
  constexpr double K0 = 0.000121;
  constexpr double K1 = 0.00896;
  constexpr double OFFSET = 0.110;

  distance_influence_table_.resize(DISTANCE_BIN_NUM);
  for (size_t i = 0; i < DISTANCE_BIN_NUM; ++i) {
    double distance = static_cast<double>(i) * DISTANCE_BIN;
    distance_influence_table_[i] = K0 * distance * distance + K1 * distance + OFFSET;
  }
}

void AdaptiveThresholdCache::initializeBEVAreaInfluenceTable()
{
  // bev area influence as normalized quadratic curve
  // range from 0 to 1 (capped at MAX_BEV_AREA)
  bev_area_influence_table_.resize(BEV_AREA_BIN_NUM);
  for (size_t i = 0; i < BEV_AREA_BIN_NUM; ++i) {
    double area = static_cast<double>(i);
    bev_area_influence_table_[i] = 0.03 * area * area / MAX_BEV_AREA;
  }
}

double AdaptiveThresholdCache::getDistanceInfluence(double distance_sq) const
{
  size_t index = static_cast<size_t>(distance_sq * DISTANCE_SQ_BIN_INV);
  index = std::min(index, DISTANCE_BIN_NUM - 1);
  return distance_influence_table_[index];
}

double AdaptiveThresholdCache::getBEVAreaInfluence(double bev_area) const
{
  size_t index = static_cast<size_t>(bev_area);
  index = std::min(index, BEV_AREA_BIN_NUM - 1);
  return bev_area_influence_table_[index];
}

}  // namespace autoware::multi_object_tracker
