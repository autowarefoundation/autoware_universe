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

#include <autoware/interpolation/linear_interpolation.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::predicted_path_postprocessor::processor
{
RefineBySpeed::RefineBySpeed(rclcpp::Node * node_ptr, const std::string & processor_name)
: ProcessorInterface(processor_name)
{
  load_config(
    node_ptr, processor_name, [](rclcpp::Node * node_ptr, const std::string & processor_name) {
      node_ptr->declare_parameter<double>(processor_name + ".speed_threshold", 1.0);
    });

  speed_threshold_ = node_ptr->get_parameter(processor_name + ".speed_threshold").as_double();
}

void RefineBySpeed::process(
  autoware_perception_msgs::msg::PredictedObject & target, const Context &)
{
  const auto speed = std::abs(target.kinematics.initial_twist_with_covariance.twist.linear.x);
  // skip if the speed is higher than the threshold
  if (speed > speed_threshold_) {
    return;
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

    // skip if the path is too short
    if (num_waypoints < 2) {
      continue;
    }

    // containers of base values and keys
    std::vector<double> base_path_x(num_waypoints);
    std::vector<double> base_path_y(num_waypoints);
    std::vector<double> base_path_z(num_waypoints);
    std::vector<double> base_path_s(num_waypoints, 0.0);

    // container of query keys
    std::vector<double> query_path_s(num_waypoints, 0.0);
    for (size_t i = 0; i < num_waypoints; ++i) {
      base_path_x[i] = waypoints[i].position.x;
      base_path_y[i] = waypoints[i].position.y;
      base_path_z[i] = waypoints[i].position.z;
      if (i >= 1) {
        base_path_s[i] = base_path_s[i - 1] +
                         autoware_utils_geometry::calc_distance2d(waypoints[i - 1], waypoints[i]);

        query_path_s[i] = query_path_s[i - 1] + speed * delta_t;
      }
    }

    const auto s_max = base_path_s.back();
    // skip if the path is too short
    if (s_max <= 1e-6) {
      continue;
    }

    // clip values from 0.0 to s_max
    std::transform(
      query_path_s.begin(), query_path_s.end(), query_path_s.begin(),
      [s_max](const auto & s) { return std::clamp(s, 0.0, s_max); });

    const auto interpolated_x = interpolation::lerp(base_path_s, base_path_x, query_path_s);
    const auto interpolated_y = interpolation::lerp(base_path_s, base_path_y, query_path_s);
    const auto interpolated_z = interpolation::lerp(base_path_s, base_path_z, query_path_s);

    // NOTE: waypoints[0] is the center position of the object, so we skip it
    for (size_t i = 1; i < num_waypoints; ++i) {
      waypoints[i].position = autoware_utils_geometry::create_point(
        interpolated_x[i], interpolated_y[i], interpolated_z[i]);
      auto yaw = autoware_utils_geometry::calc_azimuth_angle(
        waypoints[i - 1].position, waypoints[i].position);
      waypoints[i].orientation = autoware_utils_geometry::create_quaternion_from_yaw(yaw);
    }
  }
}
}  // namespace autoware::predicted_path_postprocessor::processor
