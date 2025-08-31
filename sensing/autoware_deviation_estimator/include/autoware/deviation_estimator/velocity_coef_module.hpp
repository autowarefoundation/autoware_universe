// Copyright 2018-2019 Autoware Foundation
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

#ifndef AUTOWARE__DEVIATION_ESTIMATOR__VELOCITY_COEF_MODULE_HPP_
#define AUTOWARE__DEVIATION_ESTIMATOR__VELOCITY_COEF_MODULE_HPP_

#include "autoware/deviation_estimator/utils.hpp"

#include "autoware_internal_debug_msgs/msg/float64_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <utility>
#include <vector>

namespace autoware::deviation_estimator
{

class VelocityCoefModule
{
public:
  VelocityCoefModule() = default;
  void update_coef(const TrajectoryData & traj_data);
  double get_coef() const;
  double get_coef_std() const;
  bool empty() const;

private:
  std::vector<double> coef_vx_list_;
  std::pair<double, double> coef_vx_;
};

}  // namespace autoware::deviation_estimator

#endif  // AUTOWARE__DEVIATION_ESTIMATOR__VELOCITY_COEF_MODULE_HPP_
