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

#ifndef MAP_BASED_SIGNAL_FILTER_HPP_
#define MAP_BASED_SIGNAL_FILTER_HPP_

#include <tier4_perception_msgs/msg/traffic_light_element.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi.hpp>

#include <lanelet2_core/Forward.h>

#include <map>
#include <set>
#include <utility>
#include <vector>

namespace autoware::traffic_light
{

/**
 * @brief Rule-based filter that constrains ML predictions to the (color, shape) combinations
 * declared on a traffic light's `light_bulbs` linestring in the lanelet2 map.
 *
 * A prediction whose (color, shape) pair is not encoded on the map is treated as an ML error
 * and dropped from the element list. UNKNOWN color/shape elements always pass through so the
 * classifier can still signal low confidence.
 *
 * If the lanelet2 map does not carry `light_bulbs` for a given traffic light id, the filter
 * is a no-op for that id — we cannot filter what we cannot describe.
 */
class MapBasedSignalFilter
{
public:
  using IdType = tier4_perception_msgs::msg::TrafficLightRoi::_traffic_light_id_type;
  using ColorType = tier4_perception_msgs::msg::TrafficLightElement::_color_type;
  using ShapeType = tier4_perception_msgs::msg::TrafficLightElement::_shape_type;
  using ColorShape = std::pair<ColorType, ShapeType>;

  explicit MapBasedSignalFilter(const lanelet::LaneletMapConstPtr & lanelet_map_ptr);

  /**
   * @brief Whether the map declares any bulbs for the given traffic light linestring id.
   */
  bool has_rules_for(IdType traffic_light_id) const;

  /**
   * @brief Filter the element list in place. Elements whose (color, shape) is absent from the
   * map's declared bulbs are erased. UNKNOWN color/shape elements pass through unchanged.
   *
   * @return true if any element was removed.
   */
  bool filter_elements(
    IdType traffic_light_id,
    std::vector<tier4_perception_msgs::msg::TrafficLightElement> & elements) const;

  /**
   * @brief Test hook. Returns the set of (color, shape) pairs declared for a traffic light id,
   * or an empty set if the id has no declared bulbs. Use has_rules_for() to distinguish
   * "no rules" from "empty result".
   */
  std::set<ColorShape> get_allowed_bulbs(IdType traffic_light_id) const;

private:
  std::map<IdType, std::set<ColorShape>> allowed_bulbs_;
};

}  // namespace autoware::traffic_light

#endif  // MAP_BASED_SIGNAL_FILTER_HPP_
