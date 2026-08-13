// Copyright 2021 TIER IV, Inc.
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

#ifndef AUTOWARE__MAP_BASED_PREDICTION__PATH_GENERATOR__PATH_GENERATOR_HPP_
#define AUTOWARE__MAP_BASED_PREDICTION__PATH_GENERATOR__PATH_GENERATOR_HPP_

#include "autoware/map_based_prediction/data_structure.hpp"

#include <Eigen/Eigen>
#include <autoware_utils/system/time_keeper.hpp>

#include <memory>
#include <vector>

namespace autoware::map_based_prediction
{

struct CrosswalkEdgePoints
{
  Eigen::Vector2d front_center_point;
  Eigen::Vector2d front_right_point;
  Eigen::Vector2d front_left_point;
  Eigen::Vector2d back_center_point;
  Eigen::Vector2d back_right_point;
  Eigen::Vector2d back_left_point;

  void swap()
  {
    const Eigen::Vector2d tmp_center_point = front_center_point;
    const Eigen::Vector2d tmp_right_point = front_right_point;
    const Eigen::Vector2d tmp_left_point = front_left_point;

    front_center_point = back_center_point;
    front_right_point = back_right_point;
    front_left_point = back_left_point;

    back_center_point = tmp_center_point;
    back_right_point = tmp_right_point;
    back_left_point = tmp_left_point;
  }
};

struct PredictedPathWithArrivalIndex : PredictedPath
{
  size_t arrival_index{};
};

class PathGenerator
{
public:
  explicit PathGenerator(const double sampling_time_interval);
  PathGenerator(const double sampling_time_interval, const double min_crosswalk_user_velocity);

  void setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr);

  PredictedPath generatePathForNonVehicleObject(
    const TrackedObject & object, const double duration) const;

  PredictedPath generatePathForLowSpeedVehicle(
    const TrackedObject & object, const double duration) const;

  PredictedPath generatePathForOffLaneVehicle(
    const TrackedObject & object, const double duration) const;

  PredictedPath generatePathForOnLaneVehicle(
    const TrackedObject & object, const PosePath & ref_path, const double duration,
    const double lateral_duration, const double path_width = 0.0,
    const double speed_limit = 0.0) const;

  [[nodiscard]] PredictedPathWithArrivalIndex generatePathForCrosswalkUser(
    const TrackedObject & object, const CrosswalkEdgePoints & reachable_crosswalk,
    const double duration) const;

  PredictedPath generatePathToTargetPoint(
    const TrackedObject & object, const Eigen::Vector2d & point) const;

  void setUseVehicleAcceleration(const bool use_vehicle_acceleration)
  {
    use_vehicle_acceleration_ = use_vehicle_acceleration;
  }

  void setAccelerationHalfLife(const double acceleration_exponential_half_life)
  {
    acceleration_exponential_half_life_ = acceleration_exponential_half_life;
  }

private:
  // Parameters
  double sampling_time_interval_;
  double min_crosswalk_user_velocity_;
  bool use_vehicle_acceleration_;
  double acceleration_exponential_half_life_;

  std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;

  // Member functions
  PredictedPath generateStraightPath(const TrackedObject & object, const double duration) const;

  PredictedPath generatePolynomialPath(
    const TrackedObject & object, const PosePath & ref_path, const double duration,
    const double lateral_duration, const double path_width, const double backlash_width,
    const double speed_limit = 0.0) const;
};
}  // namespace autoware::map_based_prediction

#endif  // AUTOWARE__MAP_BASED_PREDICTION__PATH_GENERATOR__PATH_GENERATOR_HPP_
