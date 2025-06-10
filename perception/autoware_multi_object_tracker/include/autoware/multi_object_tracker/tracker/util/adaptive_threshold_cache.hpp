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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UTIL__ADAPTIVE_THRESHOLD_CACHE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UTIL__ADAPTIVE_THRESHOLD_CACHE_HPP_

#include <algorithm>
#include <vector>

namespace autoware::multi_object_tracker
{

class AdaptiveThresholdCache
{
public:
  AdaptiveThresholdCache();

  double getDistanceInfluence(double distance_sq) const;
  double getBEVAreaInfluence(double bev_area) const;

private:
  static constexpr double DISTANCE_BIN = 5.0;  // meter
  static constexpr double DISTANCE_SQ_BIN = DISTANCE_BIN * DISTANCE_BIN;
  static constexpr double DISTANCE_SQ_BIN_INV = 1.0 / DISTANCE_SQ_BIN;
  static constexpr double MAX_DISTANCE = 150.0;  // meter
  static constexpr size_t DISTANCE_BIN_NUM =
    static_cast<size_t>(MAX_DISTANCE * MAX_DISTANCE * DISTANCE_SQ_BIN_INV) + 1;
  static constexpr size_t BEV_AREA_BIN_NUM = 21;  // bin width 1 m^2
  static constexpr double MAX_BEV_AREA = 20.0;    // m^2

  std::vector<double> distance_influence_table_;
  std::vector<double> bev_area_influence_table_;

  void initializeDistanceInfluenceTable();
  void initializeBEVAreaInfluenceTable();
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__UTIL__ADAPTIVE_THRESHOLD_CACHE_HPP_
