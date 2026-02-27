// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef TEST_UTILS_HPP_
#define TEST_UTILS_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <memory>

namespace utils
{
inline std::shared_ptr<lanelet::LaneletMap> create_map(lanelet::Id light_id, double stop_line_x)
{
  // 1. Create Stop Line
  lanelet::Point3d sl1(lanelet::utils::getId(), stop_line_x, -5, 0);
  lanelet::Point3d sl2(lanelet::utils::getId(), stop_line_x, 5, 0);
  lanelet::LineString3d stop_line(lanelet::utils::getId(), {sl1, sl2});

  // 2. Create Traffic Light Shape (Dummy visual)
  lanelet::Point3d light_pt(lanelet::utils::getId(), stop_line_x + 5, 5, 5);
  lanelet::LineString3d light_shape(lanelet::utils::getId(), {light_pt});

  // 3. Create Regulatory Element
  auto traffic_light_re =
    lanelet::TrafficLight::make(light_id, lanelet::AttributeMap(), {light_shape}, stop_line);

  // 4. Create Lanelet Boundaries
  lanelet::Point3d l1(lanelet::utils::getId(), 0, -5, 0);
  lanelet::Point3d l2(lanelet::utils::getId(), 20, -5, 0);
  lanelet::Point3d r1(lanelet::utils::getId(), 0, 5, 0);
  lanelet::Point3d r2(lanelet::utils::getId(), 20, 5, 0);

  lanelet::LineString3d left(lanelet::utils::getId(), {l1, l2});
  lanelet::LineString3d right(lanelet::utils::getId(), {r1, r2});

  // 5. Create Lanelet and add RE
  lanelet::Lanelet lanelet(lanelet::utils::getId(), left, right);
  lanelet.addRegulatoryElement(traffic_light_re);
  // 6. Create the map
  return lanelet::utils::createMap({lanelet});
}
}  // namespace utils

#endif  // TEST_UTILS_HPP_
