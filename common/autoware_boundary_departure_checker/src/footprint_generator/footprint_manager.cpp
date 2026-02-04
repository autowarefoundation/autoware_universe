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
#include "autoware/boundary_departure_checker/utils.hpp"

#include <memory>
#include <unordered_map>
#include <vector>

namespace autoware::boundary_departure_checker
{

FootprintManager::FootprintManager(const std::vector<FootprintType> & footprint_types)
{
  // Always add NORMAL first
  generator_.push_back(std::make_unique<NormalFootprintGenerator>());
  footprint_type_order_.push_back(FootprintType::NORMAL);

  for (const auto footprint_type : footprint_types) {
    if (footprint_type == FootprintType::NORMAL) {
      continue;
    }

    if (footprint_type == FootprintType::LOCALIZATION) {
      generator_.push_back(std::make_unique<LocalizationFootprintGenerator>());
      footprint_type_order_.push_back(footprint_type);
    } else if (footprint_type == FootprintType::LONGITUDINAL) {
      generator_.push_back(std::make_unique<LongitudinalFootprintGenerator>());
      footprint_type_order_.push_back(footprint_type);
    } else if (
      footprint_type == FootprintType::STEERING_ACCELERATED ||
      footprint_type == FootprintType::STEERING_STUCK ||
      footprint_type == FootprintType::STEERING_SUDDEN_LEFT ||
      footprint_type == FootprintType::STEERING_SUDDEN_RIGHT) {
      generator_.push_back(std::make_unique<SteeringFootprintGenerator>(footprint_type));
      footprint_type_order_.push_back(footprint_type);
    }
  }
}

std::unordered_map<FootprintType, Footprints> FootprintManager::generate_all(
  const TrajectoryPoints & pred_traj, const vehicle_info_utils::VehicleInfo & info,
  const geometry_msgs::msg::PoseWithCovariance & curr_pose_with_cov, const Param & param) const
{
  std::unordered_map<FootprintType, Footprints> generated_footprints;

  const auto uncertainty_fp_margin =
    utils::calc_margin_from_covariance(curr_pose_with_cov, param.footprint_extra_margin);

  generated_footprints.reserve(generator_.size());
  for (const auto & generator : generator_) {
    generated_footprints.insert(
      {generator->get_type(), generator->generate(pred_traj, info, param, uncertainty_fp_margin)});
  }
  return generated_footprints;
}

}  // namespace autoware::boundary_departure_checker
