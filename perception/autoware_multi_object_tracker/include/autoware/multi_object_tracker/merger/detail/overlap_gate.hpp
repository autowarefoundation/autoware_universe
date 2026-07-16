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

// Spatial gate: emits a superset of the pairs that can satisfy the redundancy test, including
// disjoint unknown-unknown pairs within unknown_pair_max_gap.
std::vector<std::pair<size_t, size_t>> findCandidatePairs(
  const std::vector<TrackerSnapshot> & snapshots, const double unknown_pair_max_gap);

}  // namespace autoware::multi_object_tracker::detail

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__MERGER__DETAIL__OVERLAP_GATE_HPP_
