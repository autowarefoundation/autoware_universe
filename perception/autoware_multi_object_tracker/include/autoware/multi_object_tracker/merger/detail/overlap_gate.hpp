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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_GATE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_GATE_HPP_

#include "autoware/multi_object_tracker/merger/detail/overlap_types.hpp"

#include <cstddef>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker::detail
{

// Discover every tracker pair whose bounding boxes could overlap, via the multi-circle gate.
// The gate covers each tracker's oriented bounding box with a chain of equal circles along its
// longer axis; a pair whose circles stay farther apart than (radius_a + radius_b + gate_margin)
// cannot overlap. The circle union is a superset of the box, so the gate admits every pair the
// geometric redundancy test could accept.
std::vector<std::pair<size_t, size_t>> findCandidatePairs(
  const std::vector<TrackerSnapshot> & snapshots);

}  // namespace autoware::multi_object_tracker::detail

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_GATE_HPP_
