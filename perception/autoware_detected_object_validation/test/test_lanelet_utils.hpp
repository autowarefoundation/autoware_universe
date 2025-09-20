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
#ifndef TEST_LANELET_UTILS_HPP_
#define TEST_LANELET_UTILS_HPP_

#include <autoware_lanelet2_extension/utility/message_conversion.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>

#include <memory>

/**
 * @brief Create a simple LaneletMapBin message containing a single 50[m] straight lanelet.
 *
 * The lanelet has left boundary at y=+1.5 and right boundary at y=-1.5,
 * forming a lane of width 3.0[m] and length 50.0[m].
 *
 * @return autoware_map_msgs::msg::LaneletMapBin The generated map message.
 */
inline autoware_map_msgs::msg::LaneletMapBin createSimpleLaneletMapMsg()
{
  // 1) Create left boundary (y = +1.5).
  //    This is a straight line from (0, +1.5, 0) to (50, +1.5, 0).
  lanelet::Point3d p1_left(lanelet::utils::getId(), 0.0, 1.5, 0.0);
  lanelet::Point3d p2_left(lanelet::utils::getId(), 50.0, 1.5, 0.0);
  lanelet::LineString3d ls_left(lanelet::utils::getId(), {p1_left, p2_left});
  ls_left.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RoadBorder;

  // 2) Create right boundary (y = -1.5).
  //    This is a straight line from (0, -1.5, 0) to (50, -1.5, 0).
  lanelet::Point3d p1_right(lanelet::utils::getId(), 0.0, -1.5, 0.0);
  lanelet::Point3d p2_right(lanelet::utils::getId(), 50.0, -1.5, 0.0);
  lanelet::LineString3d ls_right(lanelet::utils::getId(), {p1_right, p2_right});
  ls_right.attributes()[lanelet::AttributeName::Type] = lanelet::AttributeValueString::RoadBorder;

  // 3) Create a lanelet by specifying the left and right boundaries,
  //    then set the subtype to "Road".
  lanelet::Lanelet lanelet_section(lanelet::utils::getId(), ls_left, ls_right);
  lanelet_section.attributes()[lanelet::AttributeName::Subtype] =
    lanelet::AttributeValueString::Road;

  // 4) Create a LaneletMap and add the lanelet to it.
  auto lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet_map->add(lanelet_section);

  // 5) Convert the LaneletMap to a LaneletMapBin message.
  autoware_map_msgs::msg::LaneletMapBin map_bin_msg;
  lanelet::utils::conversion::toBinMsg(lanelet_map, &map_bin_msg);

  // 6) Set the frame_id in the header.
  map_bin_msg.header.frame_id = "map";

  return map_bin_msg;
}

#endif  // TEST_LANELET_UTILS_HPP_
