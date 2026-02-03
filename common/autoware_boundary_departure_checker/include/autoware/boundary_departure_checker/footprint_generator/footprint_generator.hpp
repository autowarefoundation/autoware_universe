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

#ifndef AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINT_GENERATOR__FOOTPRINT_GENERATOR_HPP_
#define AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINT_GENERATOR__FOOTPRINT_GENERATOR_HPP_

#include "autoware/boundary_departure_checker/data_structs.hpp"
#include "autoware/boundary_departure_checker/parameters.hpp"
#include "autoware/boundary_departure_checker/type_alias.hpp"

#include <autoware_vehicle_info_utils/vehicle_info.hpp>

namespace autoware::boundary_departure_checker
{

// footprint_generator.hpp
class FootprintGenerator
{
public:
  virtual ~FootprintGenerator() = default;

  [[nodiscard]] FootprintType get_type() const { return type_; }

  virtual Footprints generate(
    const TrajectoryPoints & pred_traj, const vehicle_info_utils::VehicleInfo & info,
    const Param & param, const FootprintMargin & uncertainty_fp_margin) = 0;

  FootprintGenerator(const FootprintGenerator &) = default;
  FootprintGenerator(FootprintGenerator &&) = delete;
  FootprintGenerator & operator=(const FootprintGenerator &) = default;
  FootprintGenerator & operator=(FootprintGenerator &&) = delete;

protected:
  explicit FootprintGenerator(const FootprintType type) : type_(type) {}
  FootprintType type_;
};

}  // namespace autoware::boundary_departure_checker

#endif  // AUTOWARE__BOUNDARY_DEPARTURE_CHECKER__FOOTPRINT_GENERATOR__FOOTPRINT_GENERATOR_HPP_
