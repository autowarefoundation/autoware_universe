// Copyright 2026 Contributor
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

#ifndef AUTOWARE__CANDIDATE_BANK__CANDIDATE__MAP_CONFLICT_CONTEXT_HPP_
#define AUTOWARE__CANDIDATE_BANK__CANDIDATE__MAP_CONFLICT_CONTEXT_HPP_

#include <geometry_msgs/msg/point.hpp>

#include <cstdint>
#include <vector>

namespace autoware::candidate_bank
{

// CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_BEGIN
// Map-only data prepared by LaneSegmentContext.  CandidateManager intentionally receives
// plain geometry/topology snapshots instead of Lanelet2 objects so the safety evaluator remains
// testable without owning the HD-map subsystem.
struct CandidateMapLaneGeometry
{
  std::int64_t lane_id{0};
  bool is_route_lane{false};
  double route_s_start_m{0.0};
  std::vector<geometry_msgs::msg::Point> centerline_map;
};

struct CandidateMapConflictApproach
{
  std::int64_t lane_id{0};
  // Arc length from this lane's start to the conflict point while following the map topology.
  // For the lane containing the conflict this is simply s(conflict) on that lane.
  double distance_from_lane_start_to_conflict_m{0.0};
};

struct CandidateMapConflictPoint
{
  std::int64_t ego_route_lane_id{0};
  std::int64_t competing_lane_id{0};
  geometry_msgs::msg::Point point_map;
  double ego_route_distance_m{0.0};
  std::vector<CandidateMapConflictApproach> approaches;
};

struct CandidateMapConflictContext
{
  // valid=true means route/map geometry was available.  A valid context may legitimately contain
  // zero crossing conflicts on a straight non-intersecting road.
  bool valid{false};
  std::vector<CandidateMapLaneGeometry> lanes;
  std::vector<CandidateMapConflictPoint> conflicts;
};
// CANDIDATE_BANK_MAP_CONFLICT_FILTER_V1_END

}  // namespace autoware::candidate_bank

#endif  // AUTOWARE__CANDIDATE_BANK__CANDIDATE__MAP_CONFLICT_CONTEXT_HPP_
