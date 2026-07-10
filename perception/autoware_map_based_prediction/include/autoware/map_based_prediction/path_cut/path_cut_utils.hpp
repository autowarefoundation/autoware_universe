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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PATH_CUT__PATH_CUT_UTILS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PATH_CUT__PATH_CUT_UTILS_HPP_

#include <autoware_perception_msgs/msg/predicted_path.hpp>

#include <cstddef>
#include <cstdint>

namespace autoware::map_based_prediction::path_cut
{
using autoware_perception_msgs::msg::PredictedPath;

struct MaxDecelerationParams
{
  double vehicle{3.0};
  double pedestrian{0.5};
  double bicycle{1.0};
  double motorcycle{2.0};
};

double max_deceleration_for_label(const MaxDecelerationParams & params, uint8_t label);

double distance_to_stop_with_max_deceleration(double speed, double max_deceleration);

bool can_stop_before_the_line(double distance_to_line, double speed, double max_deceleration);

PredictedPath force_cut_at_index(const PredictedPath & path, size_t last_kept_index);

}  // namespace autoware::map_based_prediction::path_cut

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PATH_CUT__PATH_CUT_UTILS_HPP_
