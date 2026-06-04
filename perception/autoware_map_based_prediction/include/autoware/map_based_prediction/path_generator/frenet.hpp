// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PATH_GENERATOR__FRENET_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PATH_GENERATOR__FRENET_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <vector>

namespace autoware::map_based_prediction
{

struct FrenetPoint
{
  double s;
  double d;
  float s_vel;
  float d_vel;
  float s_acc;
  float d_acc;
};

using FrenetPath = std::vector<FrenetPoint>;

FrenetPoint getFrenetPoint(
  const TrackedObject & object, const geometry_msgs::msg::Pose & ref_pose, const double duration,
  const double speed_limit, bool use_vehicle_acceleration,
  double acceleration_exponential_half_life);

FrenetPath generateFrenetPath(
  const FrenetPoint & current_point, const FrenetPoint & target_point, const double max_length,
  const double duration, const double lateral_duration, const double sampling_time_interval);

PosePath interpolateReferencePath(
  const PosePath & base_path, const FrenetPath & frenet_predicted_path);

PredictedPath convertToPredictedPath(
  const TrackedObject & object, const FrenetPath & frenet_predicted_path, const PosePath & ref_path,
  const double sampling_time_interval);

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PATH_GENERATOR__FRENET_HPP_
