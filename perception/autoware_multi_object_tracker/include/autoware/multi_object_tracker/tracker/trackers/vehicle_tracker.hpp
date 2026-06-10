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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__VEHICLE_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__VEHICLE_TRACKER_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"
#include "autoware/multi_object_tracker/tracker/shape_model/vehicle_shape_model.hpp"
#include "autoware/multi_object_tracker/tracker/trackers/tracker_base.hpp"
#include "autoware/multi_object_tracker/tracker/update/vehicle_update_strategy.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <optional>

namespace autoware::multi_object_tracker
{

class VehicleTracker : public Tracker
{
private:
  rclcpp::Logger logger_;

  object_model::ObjectModel object_model_;

  double velocity_deviation_threshold_;

  BicycleMotionModel motion_model_;
  using IDX = BicycleMotionModel::IDX;

  VehicleShapeModel shape_model_;

  // Tracks which end of the vehicle was used as anchor in the last conditioned update.
  // Consumed by setObjectShape() so UnstableShapeFilter commits the new length correctly.
  BicycleMotionModel::LengthUpdateAnchor shape_update_anchor_;

  // EKF kinematic update — selects update variant based on data availability.
  bool updateKinematics(
    const types::DynamicObject & object, const types::InputChannel & channel_info);
  // Wheel-anchor EKF update (front or rear) plus z/height updates.
  // Also records the anchor in shape_update_anchor_.
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

  bool getMotionState(
    const rclcpp::Time & time, geometry_msgs::msg::Pose & pose, std::array<double, 36> & pose_cov,
    geometry_msgs::msg::Twist & twist, std::array<double, 36> & twist_cov) const override;
  rclcpp::Time getStateTime() const override { return motion_model_.getLastUpdateTime(); }

  ShapeModelBase & getShapeModel() override { return shape_model_; }
  const ShapeModelBase & getShapeModel() const override { return shape_model_; }
  void assembleShapeTo(types::DynamicObject & output, bool to_publish) const override;

  // Overridden because the committed shape also drives the motion-model length (and anchor).
  // mergeFootprintFrom() is handled by the base via getShapeModel().mergeFrom().
  void setObjectShape(const autoware_perception_msgs::msg::Shape & shape) override;

  // Clusters (trust_extension=false) have unreliable bbox orientation — always use conditioned.
  UpdatePath selectUpdatePath(
    bool trust_extension, bool has_significant_shape_change) const override
  {
    if (!trust_extension) return UpdatePath::CONDITIONED;
    return has_significant_shape_change ? UpdatePath::TRY_EXTENSION : UpdatePath::NORMAL;
  }
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__TRACKERS__VEHICLE_TRACKER_HPP_
