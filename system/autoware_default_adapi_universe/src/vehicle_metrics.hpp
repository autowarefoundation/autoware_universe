// Copyright 2025 The Autoware Contributors
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

#ifndef VEHICLE_METRICS_HPP_
#define VEHICLE_METRICS_HPP_

#include <autoware/adapi_specs/vehicle.hpp>
#include <autoware/component_interface_specs_universe/vehicle.hpp>
#include <rclcpp/rclcpp.hpp>

// This file should be included after messages.
#include "utils/types.hpp"

namespace autoware::default_adapi
{

class VehicleMetricsNode : public rclcpp::Node
{
public:
  explicit VehicleMetricsNode(const rclcpp::NodeOptions & options);

private:
  using VehicleMetrics = autoware::adapi_specs::vehicle::VehicleMetrics;
  using EnergyStatus = autoware::component_interface_specs_universe::vehicle::EnergyStatus;
  void on_timer();

  rclcpp::TimerBase::SharedPtr timer_;
  Pub<VehicleMetrics> pub_metrics_;
  Sub<EnergyStatus> sub_energy_;
  EnergyStatus::Message::ConstSharedPtr energy_;
};

}  // namespace autoware::default_adapi

#endif  // VEHICLE_METRICS_HPP_
