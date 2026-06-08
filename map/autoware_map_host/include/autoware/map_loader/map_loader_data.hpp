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

#ifndef AUTOWARE__MAP_LOADER__MAP_LOADER_DATA_HPP_
#define AUTOWARE__MAP_LOADER__MAP_LOADER_DATA_HPP_

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <optional>

namespace autoware::map_loader
{

struct MapLoaderData
{
  std::optional<autoware_map_msgs::msg::MapProjectorInfo> projector_info;
  std::optional<autoware_map_msgs::msg::LaneletMapBin> vector_map_msg;
  lanelet::LaneletMapPtr lanelet_map_ptr;
};

}  // namespace autoware::map_loader

#endif  // AUTOWARE__MAP_LOADER__MAP_LOADER_DATA_HPP_
