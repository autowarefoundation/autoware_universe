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

#ifndef AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__PARAMETERS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__PARAMETERS_HPP_

namespace autoware::planning_validator
{

struct DirectionCheckFlags
{
  bool enable{};
  bool check_oncoming_lanes{};
  bool check_crossing_lanes{};
};

struct PointcloudParams
{
  double height_buffer{};
  double min_height{};

  struct VoxelGridFilterParams
  {
    double x{};
    double y{};
    double z{};
  } voxel_grid_filter;

  struct ClusteringParams
  {
    double tolerance{};
    double min_height{};
    int min_size{};
    int max_size{};
  } clustering;
};

struct CollisionCheckerParams
{
  bool enable{};
  bool is_critical{};
  double detection_range{};
  double ttc_threshold{};

  DirectionCheckFlags right_turn;
  DirectionCheckFlags left_turn;

  PointcloudParams pointcloud;
};

}  // namespace autoware::planning_validator

#endif  // AUTOWARE__PLANNING_VALIDATOR_COLLISION_CHECKER__PARAMETERS_HPP_
