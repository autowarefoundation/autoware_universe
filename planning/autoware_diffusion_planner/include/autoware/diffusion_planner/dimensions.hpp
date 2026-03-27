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

#ifndef AUTOWARE__DIFFUSION_PLANNER__DIMENSIONS_HPP_
#define AUTOWARE__DIFFUSION_PLANNER__DIMENSIONS_HPP_

#include "autoware/diffusion_planner/conversion/lanelet.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace autoware::diffusion_planner
{
inline constexpr int64_t NUM_SEGMENTS_IN_LANE = 140;
inline constexpr int64_t NUM_SEGMENTS_IN_ROUTE = 25;
inline constexpr int64_t NUM_POLYGONS = 10;
inline constexpr int64_t NUM_LINE_STRINGS = 60;
inline constexpr int64_t MAX_NUM_NEIGHBORS = 32;
inline constexpr int64_t MAX_NUM_AGENTS = MAX_NUM_NEIGHBORS + 1;  // Including ego
inline constexpr int64_t POINTS_PER_SEGMENT = 20;
inline constexpr int64_t POINTS_PER_POLYGON = 40;
inline constexpr int64_t POINTS_PER_LINE_STRING = 20;
// Number of columns in a segment matrix
// (X,Y,dX,dY,LeftBoundX,LeftBoundY,RightBoundX,RightBoundY,TrafficLightEncoding(Dim5),Speed Limit)
inline constexpr int64_t TRAFFIC_LIGHT_ONE_HOT_DIM = 5;

inline constexpr int64_t EGO_AGENT_PAST_IDX_X = 0;
inline constexpr int64_t EGO_AGENT_PAST_IDX_Y = 1;
inline constexpr int64_t EGO_AGENT_PAST_IDX_COS = 2;
inline constexpr int64_t EGO_AGENT_PAST_IDX_SIN = 3;

// Index for each field
inline constexpr int64_t X = 0;
inline constexpr int64_t Y = 1;
inline constexpr int64_t dX = 2;
inline constexpr int64_t dY = 3;
inline constexpr int64_t LB_X = 4;
inline constexpr int64_t LB_Y = 5;
inline constexpr int64_t RB_X = 6;
inline constexpr int64_t RB_Y = 7;
inline constexpr int64_t TRAFFIC_LIGHT = 8;
inline constexpr int64_t TRAFFIC_LIGHT_GREEN = 8;
inline constexpr int64_t TRAFFIC_LIGHT_YELLOW = 9;
inline constexpr int64_t TRAFFIC_LIGHT_RED = 10;
inline constexpr int64_t TRAFFIC_LIGHT_WHITE = 11;
inline constexpr int64_t TRAFFIC_LIGHT_NO_TRAFFIC_LIGHT = 12;
inline constexpr int64_t LINE_TYPE_LEFT_START = 13;
inline constexpr int64_t LINE_TYPE_RIGHT_START = LINE_TYPE_LEFT_START + LINE_TYPE_NUM;
inline constexpr int64_t SEGMENT_POINT_DIM = LINE_TYPE_RIGHT_START + LINE_TYPE_NUM;

inline constexpr int64_t INPUT_T = 30;
inline constexpr int64_t INPUT_T_WITH_CURRENT = INPUT_T + 1;  // Including current time step
inline constexpr int64_t OUTPUT_T = 80;                       // Output timestamp number
inline constexpr int64_t POSE_DIM = 4;                        // x, y, cos(yaw), sin(yaw)
inline constexpr std::array<int64_t, 4> OUTPUT_SHAPE = {1, MAX_NUM_AGENTS, OUTPUT_T, POSE_DIM};

inline constexpr int64_t TURN_INDICATOR_OUTPUT_NONE = 0;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_DISABLE = 1;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_ENABLE_LEFT = 2;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_ENABLE_RIGHT = 3;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_KEEP = 4;
inline constexpr int64_t TURN_INDICATOR_OUTPUT_DIM = 5;
inline constexpr std::array<int64_t, 2> TURN_INDICATOR_LOGIT_SHAPE = {1, TURN_INDICATOR_OUTPUT_DIM};

inline constexpr std::array<int64_t, 4> SAMPLED_TRAJECTORIES_SHAPE = {
  1, MAX_NUM_AGENTS, OUTPUT_T + 1, POSE_DIM};
inline constexpr std::array<int64_t, 3> EGO_HISTORY_SHAPE = {1, INPUT_T + 1, POSE_DIM};
inline constexpr std::array<int64_t, 2> EGO_CURRENT_STATE_SHAPE = {1, 10};
inline constexpr std::array<int64_t, 4> NEIGHBOR_SHAPE = {1, MAX_NUM_NEIGHBORS, INPUT_T + 1, 11};
inline constexpr std::array<int64_t, 3> STATIC_OBJECTS_SHAPE = {1, 5, 10};
inline constexpr std::array<int64_t, 4> LANES_SHAPE = {
  1, NUM_SEGMENTS_IN_LANE, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM};
inline constexpr std::array<int64_t, 3> LANES_HAS_SPEED_LIMIT_SHAPE = {1, NUM_SEGMENTS_IN_LANE, 1};
inline constexpr std::array<int64_t, 3> LANES_SPEED_LIMIT_SHAPE = {1, NUM_SEGMENTS_IN_LANE, 1};
inline constexpr std::array<int64_t, 4> ROUTE_LANES_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, POINTS_PER_SEGMENT, SEGMENT_POINT_DIM};
inline constexpr std::array<int64_t, 3> ROUTE_LANES_HAS_SPEED_LIMIT_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, 1};
inline constexpr std::array<int64_t, 3> ROUTE_LANES_SPEED_LIMIT_SHAPE = {
  1, NUM_SEGMENTS_IN_ROUTE, 1};
inline constexpr std::array<int64_t, 4> POLYGONS_SHAPE = {
  1, NUM_POLYGONS, POINTS_PER_POLYGON, 2 + POLYGON_TYPE_NUM};
inline constexpr std::array<int64_t, 4> LINE_STRINGS_SHAPE = {
  1, NUM_LINE_STRINGS, POINTS_PER_LINE_STRING, 2 + LINE_STRING_TYPE_NUM};
inline constexpr std::array<int64_t, 2> GOAL_POSE_SHAPE = {1, POSE_DIM};
inline constexpr std::array<int64_t, 2> EGO_SHAPE_SHAPE = {1, 3};
inline constexpr std::array<int64_t, 2> TURN_INDICATORS_SHAPE = {1, INPUT_T + 1};
inline constexpr std::array<int64_t, 2> DELAY_SHAPE = {1, 1};
}  // namespace autoware::diffusion_planner
#endif  // AUTOWARE__DIFFUSION_PLANNER__DIMENSIONS_HPP_
