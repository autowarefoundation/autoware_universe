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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VALID_TRAJECTORY_FILTER_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VALID_TRAJECTORY_FILTER_HPP_

#include "autoware/trajectory_safety_filter/safety_filter_interface.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{

// Parameters specific to ValidTrajectory filter
struct ValidTrajectoryParams
{
  double max_distance_from_trajectory = 5.0;  // meters
  bool check_finite_values = true;
  bool check_trajectory_length = true;
  size_t min_trajectory_length = 2;
};

class ValidTrajectory : public SafetyFilterInterface
{
public:
  ValidTrajectory() : SafetyFilterInterface("ValidTrajectory") {}

  bool filter_trajectory(TrajectoryPoints & traj_points, const FilterContext & context) override;

  void initialize(const std::string & name) override;
  void set_parameters(const std::unordered_map<std::string, std::any> & params) override;

private:
  ValidTrajectoryParams params_;

  bool check_finite(const TrajectoryPoint & point) const;
  bool check_trajectory_validity(const TrajectoryPoints & points) const;
  bool check_off_track(
    const TrajectoryPoints & points, const geometry_msgs::msg::Point & ego_position) const;
};

}  // namespace autoware::trajectory_safety_filter::plugin

#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__FILTERS__VALID_TRAJECTORY_FILTER_HPP_
