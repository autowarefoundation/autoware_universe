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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PARAMS_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PARAMS_HPP_

namespace autoware::map_based_prediction
{

struct PredictionTimeHorizonParams
{
  double vehicle{15.0};
  double pedestrian{10.0};
  double unknown{10.0};
};

struct NodeParams
{
  double object_buffer_time_length{2.0};
  bool remember_lost_crosswalk_users{false};
  double prediction_time_horizon_unknown{10.0};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PARAMS_HPP_
