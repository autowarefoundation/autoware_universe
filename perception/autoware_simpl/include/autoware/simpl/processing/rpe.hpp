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

#ifndef AUTOWARE__SIMPL__PROCESSING__RPE_HPP_
#define AUTOWARE__SIMPL__PROCESSING__RPE_HPP_

#include "autoware/simpl/archetype/datatype.hpp"

#include <cmath>

namespace autoware::simpl::processing
{
/**
 * @brief Perform cosine encoding.
 */
inline double encode_cosine(const archetype::Point2D & v1, const archetype::Point2D & v2)
{
  const double v1_norm = std::hypot(v1.x, v1.y);
  const double v2_norm = std::hypot(v2.x, v2.y);
  return (v1.x * v2.x + v1.y * v2.y) / std::clamp(v1_norm * v2_norm, 1e-6, 1e9);
}

/**
 * @brief Perform cosine encoding.
 */
inline double encode_cosine(const archetype::Point2D & v1, const double v2_x, const double v2_y)
{
  const double v1_norm = std::hypot(v1.x, v1.y);
  const double v2_norm = std::hypot(v2_x, v2_y);
  return (v1.x * v2_x + v1.y * v2_y) / std::clamp(v1_norm * v2_norm, 1e-6, 1e9);
}

/**
 * @brief Perform sine encoding.
 */
inline double encode_sine(const archetype::Point2D & v1, const archetype::Point2D & v2)
{
  const double v1_norm = std::hypot(v1.x, v1.y);
  const double v2_norm = std::hypot(v2.x, v2.y);
  return (v1.x * v2.x - v1.y * v2.y) / std::clamp(v1_norm * v2_norm, 1e-6, 1e9);
}

/**
 * @brief Perform sine encoding.
 */
inline double encode_sine(const archetype::Point2D & v1, const double v2_x, const double v2_y)
{
  const double v1_norm = std::hypot(v1.x, v1.y);
  const double v2_norm = std::hypot(v2_x, v2_y);
  return (v1.x * v2_x - v1.y * v2_y) / std::clamp(v1_norm * v2_norm, 1e-6, 1e9);
}
}  // namespace autoware::simpl::processing
#endif  // AUTOWARE__SIMPL__PROCESSING__RPE_HPP_
