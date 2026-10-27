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

#pragma once

#include <autoware_sensing_msgs/msg/concatenated_point_cloud_info.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>

namespace autoware::pointcloud_preprocessor
{

// The matching strategy, typed once for the whole pipeline. The enumerator values are pinned to
// the ConcatenatedPointCloudInfo STRATEGY_* message constants, which are the wire-format source of
// truth, so recording the strategy into the info message is a static_cast (no name map).
enum class MatchingStrategyType : uint8_t {
  naive = autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_NAIVE,
  advanced = autoware_sensing_msgs::msg::ConcatenatedPointCloudInfo::STRATEGY_ADVANCED,
};

/// Parse a strategy name as it appears at the string boundaries (the node's
/// "matching_strategy.type" parameter, the Python bindings' matching_strategy argument).
/// Everything past the boundary passes MatchingStrategyType around instead of strings.
/// @throws std::invalid_argument for an unknown name.
inline MatchingStrategyType parse_matching_strategy(const std::string & name)
{
  if (name == "naive") return MatchingStrategyType::naive;
  if (name == "advanced") return MatchingStrategyType::advanced;
  throw std::invalid_argument("unknown matching_strategy: '" + name + "'");
}

}  // namespace autoware::pointcloud_preprocessor
