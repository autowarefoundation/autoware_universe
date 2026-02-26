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

#ifndef AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__INTERPOLATION_HPP_
#define AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__INTERPOLATION_HPP_

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spline_interpolation.hpp>

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
/**
 * @brief Type alias for the interpolation function.
 */
using interpolation_fn = std::function<std::vector<double>(
  const std::vector<double> &, const std::vector<double> &, const std::vector<double> &)>;

/**
 * @brief Convert a string name to an interpolation function.
 *
 * @param name The name of the interpolation function.
 * @return The interpolation function.
 */
interpolation_fn to_interpolator(const std::string & name);
}  // namespace autoware::predicted_path_postprocessor::processor
#endif  // AUTOWARE__PREDICTED_PATH_POSTPROCESSOR__PROCESSOR__INTERPOLATION_HPP_
