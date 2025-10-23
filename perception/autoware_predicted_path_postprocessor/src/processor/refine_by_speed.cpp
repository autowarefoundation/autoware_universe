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

#include "autoware/predicted_path_postprocessor/processor/refine_by_speed.hpp"

#include "autoware/predicted_path_postprocessor/processor/result.hpp"

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware/interpolation/spline_interpolation.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
namespace
{
/**
 * @brief Convert a string to an interpolation function.
 *
 * @param name The name of the interpolation function.
 * @return The interpolation function.
 */
RefineBySpeed::interpolation_fn to_interpolator(const std::string & name)
{
  using kv_type = RefineBySpeed::kv_type;

  if (name == "linear") {
    // needs to cast overloads
    return static_cast<kv_type (*)(const kv_type &, const kv_type &, const kv_type &)>(
      interpolation::lerp);
  } else if (name == "spline") {
    return interpolation::spline;
  } else if (name == "spline_by_akima") {
    return interpolation::splineByAkima;
  } else {
    throw std::invalid_argument("Invalid interpolation type: " + name);
  }
}
}  // namespace

RefineBySpeed::RefineBySpeed(rclcpp::Node * node_ptr, const std::string & processor_name)
: ProcessorInterface(processor_name)
{
  speed_threshold_ = node_ptr->declare_parameter<double>(processor_name + ".speed_threshold");

  auto interpolation = node_ptr->declare_parameter<std::string>(processor_name + ".interpolation");
  interpolator_ = to_interpolator(std::move(interpolation));
}

RefineBySpeed::result_type RefineBySpeed::process(target_type & target, const Context &)
{
  const auto speed = std::abs(target.kinematics.initial_twist_with_covariance.twist.linear.x);
  // skip if the speed is higher than the threshold
  if (speed > speed_threshold_) {
    return make_ok<error_type>();
  }

  // Refine the predicted path based on the current speed
  for (auto & mode : target.kinematics.predicted_paths) {
    // Refine the path based on the current speed
    const auto delta_t = rclcpp::Duration(mode.time_step).seconds();

    if (delta_t <= 0.0) {
      continue;
    }

    auto & waypoints = mode.path;
    const auto num_waypoints = waypoints.size();

    if (num_waypoints < 2) {
      continue;
    }

    // containers of values and keys
    constexpr double epsilon = 1e-6;
    std::vector<double> base_xs({waypoints[0].position.x});
    std::vector<double> base_ys({waypoints[0].position.y});
    std::vector<double> base_zs({waypoints[0].position.z});
    std::vector<double> base_keys({0.0});
    std::vector<double> query_keys({0.0});
    for (size_t i = 1; i < num_waypoints; ++i) {
      // push values only if the distance is greater than epsilon to ensure monotonically increasing
      const auto distance =
        autoware_utils_geometry::calc_distance2d(waypoints[i - 1], waypoints[i]);
      if (distance > epsilon) {
        base_xs.push_back(waypoints[i].position.x);
        base_ys.push_back(waypoints[i].position.y);
        base_zs.push_back(waypoints[i].position.z);
        base_keys.push_back(base_keys.back() + distance);
      }
      query_keys.push_back(query_keys.back() + speed * delta_t);
    }

    const auto s_max = base_keys.back();
    // skip if the path is too short
    if (s_max <= 1e-6 || base_keys.size() < 2) {
      continue;
    }

    // clip values from 0.0 to s_max
    std::transform(
      query_keys.begin(), query_keys.end(), query_keys.begin(),
      [s_max](const auto & s) { return std::clamp(s, 0.0, s_max); });

    const auto query_xs = interpolator_(base_keys, base_xs, query_keys);
    const auto query_ys = interpolator_(base_keys, base_ys, query_keys);
    const auto query_zs = interpolator_(base_keys, base_zs, query_keys);

    // NOTE: waypoints[0] is the center position of the object, so we skip it
    for (size_t i = 1; i < num_waypoints; ++i) {
      waypoints[i].position =
        autoware_utils_geometry::create_point(query_xs[i], query_ys[i], query_zs[i]);

      const auto yaw = autoware_utils_geometry::calc_azimuth_angle(
        waypoints[i - 1].position, waypoints[i].position);
      waypoints[i].orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
    }
  }
  return make_ok<error_type>();
}
}  // namespace autoware::predicted_path_postprocessor::processor
