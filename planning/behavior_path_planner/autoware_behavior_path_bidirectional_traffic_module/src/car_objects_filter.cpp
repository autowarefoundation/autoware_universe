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

#include "autoware/behavior_path_bidirectional_traffic_module/car_objects_filter.hpp"

#include <vector>

namespace autoware::behavior_path_planner
{

CarObjectsFilter::CarObjectsFilter(const int & window_size, const double & threshold)
: window_size_(window_size), threshold_(threshold)
{
}

std::vector<autoware_perception_msgs::msg::PredictedObjects> CarObjectsFilter::update(
  const std::vector<autoware_perception_msgs::msg::PredictedObjects> & dynamic_objects) const
{
  auto filtered_objects = dynamic_objects;
  return filtered_objects;
}

};  // namespace autoware::behavior_path_planner
