// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__DIFFUSION_PLANNER__CONSTANTS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__CONSTANTS_HPP_

namespace autoware::diffusion_planner::constants
{

// Velocity thresholds
constexpr float MOVING_VELOCITY_THRESHOLD_MPS = 0.2f;

// Time constants
constexpr double PREDICTION_TIME_STEP_S = 0.1;
constexpr double TIMER_PERIOD_S = 0.2;
constexpr int LOG_THROTTLE_INTERVAL_MS = 5000;

// Geometric constants
constexpr float LANE_MASK_RANGE_M = 100.0f;
constexpr double BACKWARD_PATH_LENGTH_M = 0.0;
constexpr double FORWARD_PATH_LENGTH_M = 150.0;

// Visualization parameters
struct VisualizationParams
{
  static constexpr double DEBUG_MARKER_LIFETIME_S = 0.2;
  static constexpr float ROUTE_MARKER_ALPHA = 0.8f;
  static constexpr float MAP_MARKER_ALPHA = 0.8f;
};

}  // namespace autoware::diffusion_planner::constants

#endif  // AUTOWARE__DIFFUSION_PLANNER__CONSTANTS_HPP_
