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

#ifndef AUTOWARE__TENSORRT_ONEPLANNER__DIMENSIONS_HPP_
#define AUTOWARE__TENSORRT_ONEPLANNER__DIMENSIONS_HPP_

#include <autoware/diffusion_planner/dimensions.hpp>

#include <array>
#include <cstdint>

namespace autoware::tensorrt_oneplanner
{

// The planner-conditioning tensors are identical to the diffusion planner's
// (autoware::diffusion_planner::*_SHAPE). Only the prediction output differs:
// OnePlanner is ego-only (P=1), no neighbor-prediction head.
inline constexpr std::array<int64_t, 4> EGO_PREDICTION_SHAPE = {
  1, 1, autoware::diffusion_planner::OUTPUT_T, autoware::diffusion_planner::POSE_DIM};

// Expected major_version in the OnePlanner normalization JSON (args file).
inline constexpr int WEIGHT_MAJOR_VERSION = 1;

}  // namespace autoware::tensorrt_oneplanner

#endif  // AUTOWARE__TENSORRT_ONEPLANNER__DIMENSIONS_HPP_
