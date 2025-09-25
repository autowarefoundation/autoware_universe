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

#ifndef AUTOWARE__TRAJECTORY_SAFETY_FILTER__SAFETY_FILTER_INTERFACE_HPP_
#define AUTOWARE__TRAJECTORY_SAFETY_FILTER__SAFETY_FILTER_INTERFACE_HPP_

#include "autoware/trajectory_safety_filter/filter_context.hpp"

#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <any>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::trajectory_safety_filter::plugin
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using VehicleInfo = autoware::vehicle_info_utils::VehicleInfo;

class SafetyFilterInterface
{
public:
  explicit SafetyFilterInterface(std::string name) : name_(std::move(name)) {}
  SafetyFilterInterface(std::string name, const VehicleInfo & vehicle_info)
  : name_(std::move(name)), vehicle_info_ptr_(std::make_shared<VehicleInfo>(vehicle_info))
  {
  }
  virtual ~SafetyFilterInterface() = default;
  SafetyFilterInterface(const SafetyFilterInterface &) = delete;
  SafetyFilterInterface & operator=(const SafetyFilterInterface &) = delete;
  SafetyFilterInterface(SafetyFilterInterface &&) = delete;
  SafetyFilterInterface & operator=(SafetyFilterInterface &&) = delete;

  // Main filter method with context for plugin-specific data
  virtual bool is_feasible(const TrajectoryPoints & traj_points, const FilterContext & context) = 0;

  // Set parameters directly (for testing and runtime configuration)
  virtual void set_parameters(const std::unordered_map<std::string, std::any> & params) = 0;

  // Set vehicle info
  virtual void set_vehicle_info(const VehicleInfo & vehicle_info)
  {
    vehicle_info_ptr_ = std::make_shared<VehicleInfo>(vehicle_info);
  }

  std::string get_name() const { return name_; }

protected:
  std::string name_;
  std::shared_ptr<VehicleInfo> vehicle_info_ptr_;
};
}  // namespace autoware::trajectory_safety_filter::plugin

// NOLINTNEXTLINE
#endif  // AUTOWARE__TRAJECTORY_SAFETY_FILTER__SAFETY_FILTER_INTERFACE_HPP_
