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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__VEGETATION_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__VEGETATION_HPP_

#include "autoware/map_based_prediction/path_generator/path_generator.hpp"

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <memory>
#include <vector>

namespace autoware::map_based_prediction
{

class VegetationModule
{
public:
  VegetationModule() = default;

  void buildFromMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr);

  [[nodiscard]] bool doesPathCrossAnyVegetationBeforeCrosswalk(
    const PredictedPathWithArrivalIndex & predicted_path,
    const autoware_perception_msgs::msg::Shape & object_shape) const;

  [[nodiscard]] PredictedPath cutPathsCrossingVegetation(
    const PredictedPath & predicted_path,
    const autoware_perception_msgs::msg::Shape & object_shape) const;

private:
  lanelet::LaneletMapConstUPtr vegetation_layer_{nullptr};
};

}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PREDICTOR_VRU__VEGETATION_HPP_
