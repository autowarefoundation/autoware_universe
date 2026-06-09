// Copyright 2020 TIER IV, Inc.
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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__VEHICLE_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__VEHICLE_TRACKER_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/vehicle_extend_manager.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <optional>

namespace autoware::multi_object_tracker
{

// Vehicle update strategy type for conditioned updates
enum class UpdateStrategyType { FRONT_WHEEL_UPDATE, REAR_WHEEL_UPDATE, WEAK_UPDATE };

struct UpdateStrategy
{
  UpdateStrategyType type;
  geometry_msgs::msg::Point anchor_point;  // Anchor point for the update (used for
                                           // FRONT_WHEEL_UPDATE and REAR_WHEEL_UPDATE)
};

class VehicleTracker : public Tracker
{
private:
  rclcpp::Logger logger_;

  object_model::ObjectModel object_model_;

  double velocity_deviation_threshold_;

  BicycleMotionModel motion_model_;
  using IDX = BicycleMotionModel::IDX;

  VehicleExtendManager extend_manager_;

  // EKF kinematic update — selects update variant based on data availability.
  bool updateKinematics(
    const types::DynamicObject & object, const types::InputChannel & channel_info);
  // Wheel-anchor EKF update (front or rear) plus z/height updates.
  bool updateWheelKinematics(
    const UpdateStrategy & strategy, const types::DynamicObject & measurement);

public:
  VehicleTracker(
    const object_model::ObjectModel & object_model, const rclcpp::Time & time,
    const types::DynamicObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const types::InputChannel & channel_info) override;

  bool conditionedUpdate(
    const types::DynamicObject & measurement, const types::DynamicObject & prediction,
    const autoware_perception_msgs::msg::Shape & tracker_shape,
    const rclcpp::Time & measurement_time, const types::InputChannel & channel_info) override;

  bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const override;

  void setObjectShape(const autoware_perception_msgs::msg::Shape & shape) override;
  void mergeFootprintFrom(
    const geometry_msgs::msg::Polygon & footprint,
    const geometry_msgs::msg::Pose & src_pose) override;

  // Clusters (trust_extension=false) have unreliable bbox orientation — always use conditioned.
  UpdatePath selectUpdatePath(
    bool trust_extension, bool has_significant_shape_change) const override
  {
    if (!trust_extension) return UpdatePath::CONDITIONED;
    return has_significant_shape_change ? UpdatePath::TRY_EXTENSION : UpdatePath::NORMAL;
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__VEHICLE_TRACKER_HPP_
