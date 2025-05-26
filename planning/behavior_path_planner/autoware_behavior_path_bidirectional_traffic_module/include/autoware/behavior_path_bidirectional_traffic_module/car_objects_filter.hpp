// Copyright 2024 TIER IV, Inc.
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

#ifndef AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__CAR_OBJECTS_FILTER_HPP_
#define AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__CAR_OBJECTS_FILTER_HPP_

#include <autoware_perception_msgs/msg/predicted_objects.hpp>

#include <vector>

namespace autoware::behavior_path_planner
{

class CarObjectsFilter
{
public:
  CarObjectsFilter(const int & window_size, const double & threshold);

  [[nodiscard]] std::vector<autoware_perception_msgs::msg::PredictedObjects> update(
    const std::vector<autoware_perception_msgs::msg::PredictedObjects> & dynamic_objects) const;

private:
  const int window_size_;
  const double threshold_;
};

};  // namespace autoware::behavior_path_planner

#endif  // AUTOWARE__BEHAVIOR_PATH_BIDIRECTIONAL_TRAFFIC_MODULE__CAR_OBJECTS_FILTER_HPP_
