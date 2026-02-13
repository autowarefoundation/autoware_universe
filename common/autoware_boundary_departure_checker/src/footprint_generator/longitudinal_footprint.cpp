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

#include "autoware/boundary_departure_checker/footprint_generator/longitudinal_footprint.hpp"

#include "autoware/boundary_departure_checker/utils.hpp"

namespace autoware::boundary_departure_checker
{
Footprints LongitudinalFootprintGenerator::generate(
  const TrajectoryPoints & pred_traj, const vehicle_info_utils::VehicleInfo & info,
  const Param & param, const FootprintMargin & uncertainty_fp_margin)
{
  const auto longitudinal_config_opt =
    param.get_abnormality_config<LongitudinalConfig>(FootprintType::LONGITUDINAL);
  if (longitudinal_config_opt) {
    return utils::create_vehicle_footprints(
      pred_traj, info, uncertainty_fp_margin, longitudinal_config_opt->get());
  }
  return utils::create_vehicle_footprints(pred_traj, info, uncertainty_fp_margin);
}
}  // namespace autoware::boundary_departure_checker
