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

#ifndef AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__UNCROSSABLE_BOUNDARY_DEPARTURE_HPP_
#define AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__UNCROSSABLE_BOUNDARY_DEPARTURE_HPP_

#include "autoware/trajectory_validator/detail/uuid_hash.hpp"
#include "autoware/trajectory_validator/validator_interface.hpp"

#include <autoware/boundary_departure_checker/uncrossable_boundary_checker.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
namespace autoware::trajectory_validator::plugin::safety
{
class UncrossableBoundaryDepartureFilter : public plugin::ValidatorInterface
{
public:
  UncrossableBoundaryDepartureFilter() : ValidatorInterface("uncrossable_boundary_departure_filter")
  {
  }

  result_t is_feasible(
    const CandidateTrajectory & candidate_trajectory, const FilterContext & context) final;

  void update_parameters(const validator::Params & params) final;

private:
  std::unique_ptr<boundary_departure_checker::UncrossableBoundaryChecker> checker_;
  boundary_departure_checker::UncrossableBoundaryDepartureParam params_;

  // Per-generator hysteresis state, keyed by the generator UUID's raw 16 bytes. Each generator's
  // trajectory keeps its own hysteresis so one trajectory cannot remove another via the
  // shared ON/OFF time buffers.
  std::unordered_map<std::array<uint8_t, 16>, boundary_departure_checker::HysteresisState, UuidHash>
    hysteresis_states_;

  tl::expected<void, std::string> validate_filter_context(const FilterContext & context) const;
};
}  // namespace autoware::trajectory_validator::plugin::safety

#endif  // AUTOWARE__TRAJECTORY_VALIDATOR__FILTERS__SAFETY__UNCROSSABLE_BOUNDARY_DEPARTURE_HPP_
