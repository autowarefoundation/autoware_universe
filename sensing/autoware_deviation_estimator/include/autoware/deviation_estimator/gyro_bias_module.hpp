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

#ifndef AUTOWARE__DEVIATION_ESTIMATOR__GYRO_BIAS_MODULE_HPP_
#define AUTOWARE__DEVIATION_ESTIMATOR__GYRO_BIAS_MODULE_HPP_

#include "autoware/deviation_estimator/utils.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <utility>
#include <vector>

namespace autoware::deviation_estimator
{

class GyroBiasModule
{
public:
  GyroBiasModule() = default;
  void update_bias(const TrajectoryData & traj_data);
  geometry_msgs::msg::Vector3 get_bias_base_link() const;
  geometry_msgs::msg::Vector3 get_bias_std() const;
  bool empty() const;

private:
  std::vector<geometry_msgs::msg::Vector3> gyro_bias_list_;
  std::pair<geometry_msgs::msg::Vector3, geometry_msgs::msg::Vector3> gyro_bias_pair_;
};

}  // namespace autoware::deviation_estimator

#endif  // AUTOWARE__DEVIATION_ESTIMATOR__GYRO_BIAS_MODULE_HPP_
