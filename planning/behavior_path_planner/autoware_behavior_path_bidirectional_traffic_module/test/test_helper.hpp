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

#ifndef TEST_HELPER_HPP_
#define TEST_HELPER_HPP_

#include "autoware/behavior_path_bidirectional_traffic_module/bidirectional_lanelets.hpp"
#include "autoware/route_handler/route_handler.hpp"
#include "autoware_test_utils/autoware_test_utils.hpp"
#include "autoware_test_utils/mock_data_parser.hpp"

#include <autoware_internal_planning_msgs/msg/path_point_with_lane_id.hpp>

#include <memory>
#include <string>
#include <string_view>

inline auto load_bidirectional_lanelets_from_map(
  std::string_view package_name, std::string_view map_path)
{
  const auto map_bin_msg = autoware::test_utils::make_map_bin_msg(
    autoware::test_utils::get_absolute_path_to_lanelet_map(package_name.data(), map_path.data()),
    0.5);

  // create route handler
  auto route_handler = std::make_shared<autoware::route_handler::RouteHandler>(map_bin_msg);
  const lanelet::LaneletMap & map = *route_handler->getLaneletMapPtr();

  // get bidirectional lanelets
  auto get_next_lanelets = [&](const lanelet::ConstLanelet & lanelet) {
    return route_handler->getNextLanelets(lanelet);
  };
  auto get_prev_lanelets = [&](const lanelet::ConstLanelet & lanelet) {
    return route_handler->getPreviousLanelets(lanelet);
  };

  // get bidirectional lanelets
  return autoware::behavior_path_planner::ConnectedBidirectionalLanelets::
    search_bidirectional_lanes_on_map(map, get_next_lanelets, get_prev_lanelets);
}

inline auto load_path(std::string_view package_name, std::string_view path_path)
{
  // Load YAML file
  const auto path_dir = ament_index_cpp::get_package_share_directory(package_name.data()) + "/" +
                        std::string(path_path);
  YAML::Node node = YAML::LoadFile(path_dir);

  // Convert to PathWithLaneId
  return autoware::test_utils::parse<autoware_internal_planning_msgs::msg::PathWithLaneId>(node);
}

#endif  // TEST_HELPER_HPP_
