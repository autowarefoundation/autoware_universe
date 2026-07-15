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

#include <autoware_lane_event_classifier/lane_following/checker.hpp>
#include <magic_enum.hpp>

#include <string_view>

namespace autoware::lane_event_classifier
{

std::string_view to_string(LaneFollowingReason reason)
{
  return magic_enum::enum_name(reason);
}

// Stub: the lane-following check logic is added in a follow-up PR. Until then the ego is always
// reported as following, so the node publishes LANE_FOLLOWING every cycle.
LaneFollowingResult LaneFollowingChecker::evaluate() const
{
  return {true, LaneFollowingReason::no_reference_lane};
}

}  // namespace autoware::lane_event_classifier
