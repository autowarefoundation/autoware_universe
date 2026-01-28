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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__ABNORMALITIES__FOOTPRINT_GENERATOR_MANAGER_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__ABNORMALITIES__FOOTPRINT_GENERATOR_MANAGER_HPP_

#include "autoware/boundary_departure_checker/abnormalities/footprint_generator.hpp"
#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <memory>
#include <vector>

namespace autoware::boundary_departure_checker
{

class FootprintGeneratorManager
{
public:
  explicit FootprintGeneratorManager(const Param & param);

  [[nodiscard]] std::vector<Footprints> generate_all(
    const TrajectoryPoints & pred_traj, const SteeringReport & steering,
    const vehicle_info_utils::VehicleInfo & info, const Param & param,
    const FootprintMargin & uncertainty_fp_margin) const;

  [[nodiscard]] const std::vector<AbnormalityType> & get_ordered_types() const
  {
    return ordered_types_;
  }

private:
  std::vector<std::unique_ptr<FootprintGenerator>> generators_;
  std::vector<AbnormalityType> ordered_types_;
};

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__ABNORMALITIES__FOOTPRINT_GENERATOR_MANAGER_HPP_
