// Copyright 2026 Xinchen Lin
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

#ifndef AUTOWARE__TRAJECTORY_RANKER__CAMP_RANKER_HPP_
#define AUTOWARE__TRAJECTORY_RANKER__CAMP_RANKER_HPP_

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

namespace autoware::trajectory_ranker
{

inline constexpr std::size_t kCampAtomCount = 16;
inline constexpr std::array<std::string_view, kCampAtomCount> kCampAtomNames{
  "predicted_obb_collision_exposure_fraction",
  "ttc_deficit_0_95s",
  "dynamic_clearance_buffer_deficit",
  "overspeed_integral_m2_per_s",
  "full_footprint_road_exit_severity_s",
  "reverse_progress_severity_m",
  "red_light_crossing_exposure_fraction",
  "red_stopping_margin_m2_s",
  "route_progress_shortfall_m",
  "longitudinal_acceleration_energy_s",
  "lateral_acceleration_energy_s",
  "yaw_rate_energy_s",
  "yaw_acceleration_energy_s",
  "longitudinal_jerk_energy_s",
  "jerk_magnitude_energy_s",
  "previous_plan_execution_transition_rms"};

enum class CampAtomStatus { Observed, NotApplicable, TypedMissing };

using CampAtomVector = std::array<double, kCampAtomCount>;
using CampStatusPattern = std::array<CampAtomStatus, kCampAtomCount>;

struct CampPatternWeights
{
  CampStatusPattern status;
  CampAtomVector weights;
};

struct CampFixedWeightModel
{
  std::size_t candidate_pool_k;
  std::vector<std::string> atom_names;
  CampAtomVector scales;
  std::array<double, 3> transition_component_scales;
  std::vector<CampPatternWeights> patterns;
};

struct CampRankingResult
{
  std::size_t selected_index;
  std::vector<double> costs;
};

CampAtomStatus camp_atom_status_from_string(std::string_view status);

CampFixedWeightModel load_camp_fixed_weight_model(const std::filesystem::path & path);

CampRankingResult rank_camp_candidates(
  const CampFixedWeightModel & model, const CampStatusPattern & status,
  const std::vector<CampAtomVector> & raw_atoms);

}  // namespace autoware::trajectory_ranker

#endif  // AUTOWARE__TRAJECTORY_RANKER__CAMP_RANKER_HPP_
