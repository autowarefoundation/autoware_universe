// Copyright 2024 TIER IV, inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__FENCE_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__FENCE_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <memory>

namespace autoware::map_based_prediction
{

class FenceModule
{
public:
  FenceModule() = default;

  void buildFromMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  /// @p predicted_path_ls holds the path positions up to the crosswalk arrival index.
  [[nodiscard]] bool doesPathCrossAnyFenceBeforeCrosswalk(
    const lanelet::BasicLineString2d & predicted_path_ls) const;

  /// @p predicted_path_ls holds all path positions of @p predicted_path.
  [[nodiscard]] PredictedPath cutPathBeforeFences(
    const PredictedPath & predicted_path,
    const lanelet::BasicLineString2d & predicted_path_ls) const;

private:
  lanelet::LaneletMapUPtr fence_layer_{nullptr};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__FENCE_HPP_
