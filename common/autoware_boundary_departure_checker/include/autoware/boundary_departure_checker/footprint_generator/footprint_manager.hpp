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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINT_GENERATOR__FOOTPRINT_MANAGER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINT_GENERATOR__FOOTPRINT_MANAGER_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/footprint_generator.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::boundary_departure_checker
{

class FootprintManager
{
public:
  explicit FootprintManager(const std::vector<FootprintType> & footprint_types);

  [[nodiscard]] std::unordered_map<FootprintType, Footprints> generate_all(
    const TrajectoryPoints & pred_traj, const vehicle_info_utils::VehicleInfo & info,
    const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov, const Param & param) const;

  [[nodiscard]] const std::vector<FootprintType> & get_footprint_type_order() const
  {
    return footprint_type_order_;
  }

private:
  std::vector<std::unique_ptr<FootprintGenerator>> generator_;
  std::vector<FootprintType> footprint_type_order_;
};

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINT_GENERATOR__FOOTPRINT_MANAGER_HPP_
