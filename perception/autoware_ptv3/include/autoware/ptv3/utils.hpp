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

#ifndef AUTOWARE__PTV3__UTILS_HPP_
#define AUTOWARE__PTV3__UTILS_HPP_

#include <stdexcept>

namespace autoware::ptv3
{

/**
 * @brief Box record shared by CUDA decode and ROS conversion.
 */
struct Box3D
{
  int label{-1};
  float score{0.0f};
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float length{0.0f};
  float width{0.0f};
  float height{0.0f};
  float yaw{0.0f};
  float vel_x{0.0f};
  float vel_y{0.0f};
};

// cspell: ignore divup
template <typename T1, typename T2>
unsigned int divup(const T1 a, const T2 b)
{
  if (a == 0) {
    throw std::runtime_error("A dividend of divup isn't positive.");
  }
  if (b == 0) {
    throw std::runtime_error("A divisor of divup isn't positive.");
  }

  return (a + b - 1) / b;
}

}  // namespace autoware::ptv3

#endif  // AUTOWARE__PTV3__UTILS_HPP_
