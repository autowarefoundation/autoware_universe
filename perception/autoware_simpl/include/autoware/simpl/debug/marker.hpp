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

#ifndef AUTOWARE__SIMPL__DEBUG__MARKER_HPP_
#define AUTOWARE__SIMPL__DEBUG__MARKER_HPP_

#include "autoware/simpl/archetype/agent.hpp"
#include "autoware/simpl/archetype/map.hpp"
#include "autoware/simpl/archetype/polyline.hpp"
#include "autoware/simpl/processing/preprocessor.hpp"

#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::simpl::debug
{
/**
 * @brief Create a history marker array.
 *
 * @param history_map Key-value mapping of agent ID and its history.
 * @param header ROS message header.
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray create_history_marker_array(
  const std::unordered_map<std::string, archetype::AgentHistory> & history_map,
  const std_msgs::msg::Header & header);

/**
 * @brief Create a polyline marker array.
 *
 * @param map_points Vector of polylines.
 * @param header ROS message header.
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray create_polyline_marker_array(
  const std::vector<archetype::Polyline> & polylines, const std_msgs::msg::Header & header);

/**
 * @brief Create a polyline marker array.
 *
 * @param polylines Vector of polylines.
 * @param header ROS message header.
 * @param node_centers Node centers.
 * @param node_vectors Node vectors.
 * @return visualization_msgs::msg::MarkerArray
 */
visualization_msgs::msg::MarkerArray create_processed_map_marker_array(
  const std::vector<archetype::Polyline> & polylines, const processing::NodePoints & node_centers,
  const processing::NodePoints & node_vectors, const std_msgs::msg::Header & header);
}  // namespace autoware::simpl::debug
#endif  // AUTOWARE__SIMPL__DEBUG__MARKER_HPP_
