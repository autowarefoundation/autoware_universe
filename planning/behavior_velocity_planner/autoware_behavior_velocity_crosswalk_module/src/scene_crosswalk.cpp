// Copyright 2020 Tier IV, Inc.
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

#include "scene_crosswalk.hpp"

#include <autoware/behavior_velocity_planner_common/util/util.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <autoware/universe_utils/math/unit_conversion.hpp>

#include <cmath>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace
{
void sortCrosswalksByDistance(
  const PathWithLaneId & ego_path, const geometry_msgs::msg::Point & ego_pos,
  lanelet::ConstLanelets & crosswalks)
{
const auto compare = [&](const lanelet::ConstLanelet & l1, const lanelet::ConstLanelet & l2) {
    const auto l1_end_points_on_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, l1.polygon2d().basicPolygon(), ego_pos);
    const auto l2_end_points_on_crosswalk =
      getPathEndPointsOnCrosswalk(ego_path, l2.polygon2d().basicPolygon(), ego_pos);

    if (!l1_end_points_on_crosswalk || !l2_end_points_on_crosswalk) {
      if (!l1_end_points_on_crosswalk && !l2_end_points_on_crosswalk) {
        return false;
      }
      return l1_end_points_on_crosswalk ? true : false;
    }

    const auto dist_l1 =
      calcSignedArcLength(ego_path.points, size_t(0), l1_end_points_on_crosswalk->first);
    const auto dist_l2 =
      calcSignedArcLength(ego_path.points, size_t(0), l2_end_points_on_crosswalk->first);

    if (std::abs(dist_l1 - dist_l2) < 1e-6) {
      return l1.id() < l2.id();
    }

    return dist_l1 < dist_l2;
  };

  std::sort(crosswalks.begin(), crosswalks.end(), compare);
}
}  // namespace
}  // namespace autoware::behavior_velocity_planner
