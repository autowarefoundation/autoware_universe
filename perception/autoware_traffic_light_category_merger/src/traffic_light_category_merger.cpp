// Copyright 2026 TIER IV, Inc.
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

#include "traffic_light_category_merger.hpp"

namespace autoware::traffic_light
{

TrafficLightArray TrafficLightCategoryMerger::merge(
  const TrafficLightArray & car_signals, const TrafficLightArray & pedestrian_signals)
{
  TrafficLightArray output;
  output.header = car_signals.header;
  output.signals.insert(
    output.signals.end(), car_signals.signals.begin(), car_signals.signals.end());
  output.signals.insert(
    output.signals.end(), pedestrian_signals.signals.begin(), pedestrian_signals.signals.end());
  return output;
}

}  // namespace autoware::traffic_light
