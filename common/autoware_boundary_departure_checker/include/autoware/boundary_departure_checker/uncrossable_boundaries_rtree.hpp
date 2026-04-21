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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARIES_RTREE_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARIES_RTREE_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <lanelet2_core/LaneletMap.h>

#include <string>
#include <vector>

namespace autoware::boundary_departure_checker
{
/**
 * @brief Class to handle construction and querying of an R-tree for uncrossable boundary segments.
 */
class UncrossableBoundariesRTree
{
public:
  /**
   * @brief Constructor that builds the R-tree from a LaneletMap.
   * @param[in] lanelet_map input map containing all line strings
   * @param[in] boundary_types_to_detect list of boundary type names to consider as uncrossable
   */
  UncrossableBoundariesRTree(
    const lanelet::LaneletMap & lanelet_map,
    const std::vector<std::string> & boundary_types_to_detect);

  /**
   * @brief Retrieve a 3D line segment from the Lanelet2 map.
   * @param[in] lanelet_map_ptr pointer to the Lanelet2 map
   * @param[in] seg_id identifier for the segment
   * @return corresponding 3D segment
   */
  autoware_utils_geometry::Segment3d get_segment_3d_from_id(
    const lanelet::LaneletMapPtr & lanelet_map_ptr, const IdxForRTreeSegment & seg_id) const;

  /**
   * @brief Query the R-tree for segments near a point.
   * @param[in] point query point
   * @param[in] max_results maximum number of results to return
   * @return list of nearby segments
   */
  std::vector<SegmentWithIdx> query(const Point2d & point, size_t max_results) const;

  /**
   * @brief Check if the R-tree is empty.
   * @return true if empty
   */
  bool empty() const { return rtree_.empty(); }

  /**
   * @brief Check if a line string matches one of the uncrossable boundary types.
   * @param[in] boundary_types_to_detect list of boundary type strings to match
   * @param[in] ls lanelet line string to inspect
   * @return true if the line string is uncrossable
   */
  static bool is_uncrossable_type(
    const std::vector<std::string> & boundary_types_to_detect,
    const lanelet::ConstLineString3d & ls);

private:
  autoware::boundary_departure_checker::UncrossableBoundsRTree rtree_;
};

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__UNCROSSABLE_BOUNDARIES_RTREE_HPP_
