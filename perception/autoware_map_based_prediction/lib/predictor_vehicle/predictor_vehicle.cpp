// Copyright 2021 Tier IV, Inc.
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

#include "autoware/map_based_prediction/predictor_vehicle/predictor_vehicle.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <memory>

namespace autoware::map_based_prediction
{

PredictorVehicle::PredictorVehicle(rclcpp::Node & node) : node_(node)
{
}

void PredictorVehicle::setParams(const Params & params)
{
  const bool recreate_generator =
    !path_generator_ ||
    std::abs(params_.prediction_sampling_time_interval - params.prediction_sampling_time_interval) >
      1e-9;

  params_ = params;

  if (recreate_generator) {
    path_generator_ = std::make_shared<PathGenerator>(params_.prediction_sampling_time_interval);
    if (time_keeper_) path_generator_->setTimeKeeper(time_keeper_);
  }

  path_generator_->setUseVehicleAcceleration(params_.use_vehicle_acceleration);
  path_generator_->setAccelerationHalfLife(params_.acceleration_exponential_half_life);
}

void PredictorVehicle::setLaneletMap(
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr,
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr,
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr)
{
  lanelet_map_ptr_ = lanelet_map_ptr;
  routing_graph_ptr_ = routing_graph_ptr;
  traffic_rules_ptr_ = traffic_rules_ptr;
  lru_cache_of_convert_path_type_.clear();
}

void PredictorVehicle::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = time_keeper_ptr;
  if (path_generator_) path_generator_->setTimeKeeper(time_keeper_);
}

void PredictorVehicle::removeOldHistory(double current_time, double buffer_time)
{
  utils::removeOldObjectsHistory(current_time, buffer_time, road_users_history_);
}

}  // namespace autoware::map_based_prediction
