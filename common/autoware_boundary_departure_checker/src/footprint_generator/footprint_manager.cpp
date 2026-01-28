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

#include "autoware/boundary_departure_checker/footprint_generator/footprint_manager.hpp"

#include "autoware/boundary_departure_checker/footprint_generator/localization_footprint.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/longitudinal_footprint.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/normal_footprint.hpp"
#include "autoware/boundary_departure_checker/footprint_generator/steering_footprint.hpp"

#include <memory>
#include <vector>

namespace autoware::boundary_departure_checker
{

FootprintManager::FootprintManager(
  const std::vector<FootprintType> & footprint_types)
{
  // Always add NORMAL first
  generators_.push_back(std::make_unique<NormalFootprintGenerator>());
  ordered_types_.push_back(FootprintType::NORMAL);

  for (const auto footprint_type : footprint_types) {
    if (footprint_type == FootprintType::NORMAL) {
      continue;
    }

    if (footprint_type == FootprintType::LOCALIZATION) {
      generators_.push_back(std::make_unique<LocalizationFootprintGenerator>());
      ordered_types_.push_back(footprint_type);
    } else if (footprint_type == FootprintType::LONGITUDINAL) {
      generators_.push_back(std::make_unique<LongitudinalFootprintGenerator>());
      ordered_types_.push_back(footprint_type);
    } else if (
      footprint_type == FootprintType::STEERING_ACCELERATED ||
      footprint_type == FootprintType::STEERING_STUCK ||
      footprint_type == FootprintType::STEERING_SUDDEN_LEFT ||
      footprint_type == FootprintType::STEERING_SUDDEN_RIGHT) {
      generators_.push_back(std::make_unique<SteeringFootprintGenerator>(footprint_type));
      ordered_types_.push_back(footprint_type);
    }
  }
}

std::vector<Footprints> FootprintManager::generate_all(
  const TrajectoryPoints & pred_traj, const SteeringReport & steering,
  const vehicle_info_utils::VehicleInfo & info, const Param & param,
  const FootprintMargin & uncertainty_fp_margin) const
{
  std::vector<Footprints> all_footprints;
  all_footprints.reserve(generators_.size());
  for (const auto & generator : generators_) {
    all_footprints.push_back(
      generator->generate(pred_traj, steering, info, param, uncertainty_fp_margin));
  }
  return all_footprints;
}

}  // namespace autoware::boundary_departure_checker
