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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CAMP_ATOM_MATERIALIZER_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CAMP_ATOM_MATERIALIZER_HPP_

#include "autoware/diffusion_planner/conversion/lanelet.hpp"

#include <Eigen/Core>
#include <autoware/trajectory_ranker/camp_ranker.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace autoware::diffusion_planner
{

inline constexpr std::size_t kCampHorizonSteps = 80;
inline constexpr std::size_t kCampActorCount = 32;

struct CampActorShape
{
  bool observed{false};
  double length_m{0.0};
  double width_m{0.0};
};

struct CampPlanState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double longitudinal_velocity_mps{0.0};
};

using CampWorldPlan = std::array<CampPlanState, kCampHorizonSteps>;

struct CampPreviousPlan
{
  double origin_seconds{0.0};
  CampWorldPlan states;
};

struct CampTensorContext
{
  std::vector<float> lanes;
  std::vector<float> route_lanes;
  std::vector<float> route_speed_limits;
  bool route_has_traffic_light{false};
};

struct CampAtomMaterializationInput
{
  std::vector<float> denormalized_predictions;
  std::int64_t batch_size{0};
  std::int64_t agent_count{0};
  std::array<CampActorShape, kCampActorCount> actor_shapes{};
  CampTensorContext tensor_context;
  const LaneletMap * lanelet_map{nullptr};
  Eigen::Matrix4d ego_to_map{Eigen::Matrix4d::Identity()};
  double ego_wheelbase_m{0.0};
  double ego_length_m{0.0};
  double ego_width_m{0.0};
  double origin_seconds{0.0};
  std::optional<CampPreviousPlan> previous_plan;
};

struct CampAtomMaterializationResult
{
  trajectory_ranker::CampStatusPattern status;
  std::vector<trajectory_ranker::CampAtomVector> raw_atoms;
  std::vector<CampWorldPlan> candidate_world_plans;
};

CampAtomMaterializationResult materialize_camp_atoms(
  const CampAtomMaterializationInput & input,
  const std::array<double, 3> & transition_component_scales);

}  // namespace autoware::diffusion_planner

#endif  // AUTOWARE__DIFFUSION_PLANNER__CAMP_ATOM_MATERIALIZER_HPP_
