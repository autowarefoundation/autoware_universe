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

#include "autoware/boundary_departure_checker/abnormalities/footprint_generator_manager.hpp"

#include "autoware/boundary_departure_checker/abnormalities/localization_footprint_generator.hpp"
#include "autoware/boundary_departure_checker/abnormalities/longitudinal_footprint_generator.hpp"
#include "autoware/boundary_departure_checker/abnormalities/normal_footprint_generator.hpp"
#include "autoware/boundary_departure_checker/abnormalities/steering_footprint_generator.hpp"

#include <memory>
#include <vector>

namespace autoware::boundary_departure_checker
{

FootprintGeneratorManager::FootprintGeneratorManager(const Param & param)
{
  // Always add NORMAL first
  generators_.push_back(std::make_unique<NormalFootprintGenerator>());
  ordered_types_.push_back(AbnormalityType::NORMAL);

  for (const auto abnormality_type : param.abnormality_types_to_compensate) {
    if (abnormality_type == AbnormalityType::NORMAL) {
      continue;
    }

    if (abnormality_type == AbnormalityType::LOCALIZATION) {
      generators_.push_back(std::make_unique<LocalizationFootprintGenerator>());
      ordered_types_.push_back(abnormality_type);
    } else if (abnormality_type == AbnormalityType::LONGITUDINAL) {
      generators_.push_back(std::make_unique<LongitudinalFootprintGenerator>());
      ordered_types_.push_back(abnormality_type);
    } else if (
      abnormality_type == AbnormalityType::STEERING_ACCELERATED ||
      abnormality_type == AbnormalityType::STEERING_STUCK ||
      abnormality_type == AbnormalityType::STEERING_SUDDEN_LEFT ||
      abnormality_type == AbnormalityType::STEERING_SUDDEN_RIGHT) {
      generators_.push_back(std::make_unique<SteeringFootprintGenerator>(abnormality_type));
      ordered_types_.push_back(abnormality_type);
    }
  }
}

std::vector<Footprints> FootprintGeneratorManager::generate_all(
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
