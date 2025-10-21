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

#include "autoware/predicted_path_postprocessor/processor/interpolation.hpp"

#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
interpolation_fn to_interpolator(const std::string & name)
{
  using kv_type = std::vector<double>;
  if (name == "linear") {
    return static_cast<kv_type (*)(const kv_type &, const kv_type &, const kv_type &)>(
      interpolation::lerp);
  } else if (name == "spline") {
    return interpolation::spline;
  } else if (name == "spline_by_akima") {
    return interpolation::splineByAkima;
  } else {
    throw std::invalid_argument("Invalid interpolation name: " + name);
  }
}
}  // namespace autoware::predicted_path_postprocessor::processor
