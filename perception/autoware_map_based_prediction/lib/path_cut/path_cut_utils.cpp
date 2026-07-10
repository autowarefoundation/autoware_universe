// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/path_cut/path_cut_utils.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>

#include <algorithm>
#include <limits>

namespace autoware::map_based_prediction::path_cut
{
using autoware_perception_msgs::msg::ObjectClassification;

double max_deceleration_for_label(const MaxDecelerationParams & params, const uint8_t label)
{
  switch (label) {
    case ObjectClassification::CAR:
    case ObjectClassification::BUS:
    case ObjectClassification::TRUCK:
    case ObjectClassification::TRAILER:
      return params.vehicle;
    case ObjectClassification::MOTORCYCLE:
      return params.motorcycle;
    case ObjectClassification::PEDESTRIAN:
      return params.pedestrian;
    case ObjectClassification::BICYCLE:
      return params.bicycle;
    default:
      return params.vehicle;
  }
}

double distance_to_stop_with_max_deceleration(const double speed, const double max_deceleration)
{
  if (speed <= 0.0) {
    return 0.0;
  }

  if (max_deceleration <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  return (speed * speed) / (2.0 * max_deceleration);
}

bool can_stop_before_the_line(
  const double distance_to_line, const double speed, const double max_deceleration)
{
  if (max_deceleration <= 0.0) return false;

  return distance_to_stop_with_max_deceleration(speed, max_deceleration) <= distance_to_line;
}

PredictedPath force_cut_at_index(const PredictedPath & path, const size_t last_kept_index)
{
  PredictedPath cut = path;
  const size_t keep_count = std::min(last_kept_index + 1, cut.path.size());
  cut.path.resize(keep_count);
  return cut;
}

}  // namespace autoware::map_based_prediction::path_cut
