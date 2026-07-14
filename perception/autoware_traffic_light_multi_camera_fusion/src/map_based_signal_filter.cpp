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

#include "map_based_signal_filter.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <algorithm>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

namespace
{

using T4Element = tier4_perception_msgs::msg::TrafficLightElement;

std::optional<T4Element::_color_type> convert_map_color_to_tl_color(const std::string & color)
{
  if (color == "red") return T4Element::RED;
  if (color == "yellow") return T4Element::AMBER;
  if (color == "green") return T4Element::GREEN;
  if (color == "white") return T4Element::WHITE;
  return std::nullopt;
}

std::optional<T4Element::_shape_type> convert_map_arrow_to_tl_shape(const std::string & arrow)
{
  // circle bulb does not have an arrow attribute
  // so we will handle a missing arrow as a circle
  if (arrow == "none" || arrow == "circle") return T4Element::CIRCLE;
  if (arrow == "left") return T4Element::LEFT_ARROW;
  if (arrow == "right") return T4Element::RIGHT_ARROW;
  if (arrow == "up" || arrow == "straight") return T4Element::UP_ARROW;
  if (arrow == "down") return T4Element::DOWN_ARROW;
  if (arrow == "up_left") return T4Element::UP_LEFT_ARROW;
  if (arrow == "up_right") return T4Element::UP_RIGHT_ARROW;
  if (arrow == "down_left") return T4Element::DOWN_LEFT_ARROW;
  if (arrow == "down_right") return T4Element::DOWN_RIGHT_ARROW;
  if (arrow == "cross") return T4Element::CROSS;
  return std::nullopt;
}

std::optional<MapBasedSignalFilter::IdType> parse_traffic_light_id(const std::string & value)
{
  try {
    return static_cast<MapBasedSignalFilter::IdType>(std::stoll(value));
  } catch (const std::invalid_argument &) {
    return std::nullopt;
  } catch (const std::out_of_range &) {
    return std::nullopt;
  }
}

}  // namespace

MapBasedSignalFilter::MapBasedSignalFilter(const lanelet::LaneletMapConstPtr & lanelet_map_ptr)
{
  if (!lanelet_map_ptr) {
    return;
  }

  const lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr);
  const std::vector<lanelet::AutowareTrafficLightConstPtr> traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);

  for (const auto & tl : traffic_lights) {
    for (const auto & light_bulbs_ls : tl->lightBulbs()) {
      if (!light_bulbs_ls.hasAttribute("traffic_light_id")) {
        continue;
      }
      const auto tl_id_opt =
        parse_traffic_light_id(light_bulbs_ls.attribute("traffic_light_id").value());
      if (!tl_id_opt) {
        continue;
      }
      const IdType tl_id = *tl_id_opt;

      for (const auto & point : light_bulbs_ls) {
        if (!point.hasAttribute("color")) {
          continue;
        }
        const auto color = convert_map_color_to_tl_color(point.attribute("color").value());
        if (!color) {
          continue;
        }
        const auto shape = convert_map_arrow_to_tl_shape(point.attributeOr("arrow", "none"));
        if (!shape) {
          continue;
        }
        allowed_bulbs_[tl_id].emplace(*color, *shape);
      }
    }
  }
}

bool MapBasedSignalFilter::has_rules_for(IdType traffic_light_id) const
{
  const auto it = allowed_bulbs_.find(traffic_light_id);
  return it != allowed_bulbs_.end() && !it->second.empty();
}

bool MapBasedSignalFilter::filter_elements(
  IdType traffic_light_id, std::vector<T4Element> & elements) const
{
  const auto it = allowed_bulbs_.find(traffic_light_id);
  if (it == allowed_bulbs_.end() || it->second.empty()) {
    return false;
  }
  const auto & allowed = it->second;

  const auto original_size = elements.size();
  elements.erase(
    std::remove_if(
      elements.begin(), elements.end(),
      [&allowed](const T4Element & element) {
        // preserve UNKNOWN elements
        if (element.color == T4Element::UNKNOWN || element.shape == T4Element::UNKNOWN) {
          return false;
        }
        return allowed.count({element.color, element.shape}) == 0;
      }),
    elements.end());
  return elements.size() != original_size;
}

std::set<MapBasedSignalFilter::ColorShape> MapBasedSignalFilter::get_allowed_bulbs(
  IdType traffic_light_id) const
{
  const auto it = allowed_bulbs_.find(traffic_light_id);
  if (it == allowed_bulbs_.end()) {
    return {};
  }
  return it->second;
}

}  // namespace autoware::traffic_light
