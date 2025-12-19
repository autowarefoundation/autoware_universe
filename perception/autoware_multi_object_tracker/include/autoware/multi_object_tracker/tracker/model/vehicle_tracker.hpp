// Copyright 2020 Tier IV, Inc.
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
//
//
// Author: v1.0 Yukihiro Saito
//

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__VEHICLE_TRACKER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__VEHICLE_TRACKER_HPP_

#include "autoware/multi_object_tracker/object_model/object_model.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/tracker/motion_model/bicycle_motion_model.hpp"

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

  // determine anchor point for shape updates by last update strategy
  BicycleMotionModel::LengthUpdateAnchor shape_update_anchor_;  // Default: CENTER

public:
  VehicleTracker(
    const object_model::ObjectModel & object_model, const rclcpp::Time & time,
    const types::DynamicObject & object);

  bool predict(const rclcpp::Time & time) override;
  bool measure(
    const types::DynamicObject & object, const rclcpp::Time & time,
    const types::InputChannel & channel_info) override;
  bool measureWithPose(
    const types::DynamicObject & object, const types::InputChannel & channel_info);

  bool conditionedUpdate(
    const types::DynamicObject & measurement, const types::DynamicObject & prediction,
    const autoware_perception_msgs::msg::Shape & tracker_shape,
    const rclcpp::Time & measurement_time, const types::InputChannel & channel_info) override;

  bool getTrackedObject(
    const rclcpp::Time & time, types::DynamicObject & object,
    const bool to_publish = false) const override;

  void setObjectShape(const autoware_perception_msgs::msg::Shape & shape) override;

  const double ALIGNMENT_RATIO_THRESHOLD = 0.09;  // 9% of length as alignment tolerance
  UpdateStrategy determineUpdateStrategy(
    const types::DynamicObject & measurement, const types::DynamicObject & prediction) const;

private:
  // Helper structs for determineUpdateStrategy
  struct EdgePositions
  {
    double front_x, front_y;
    double rear_x, rear_y;
  };

  enum class Edge { FRONT, REAR };
  struct EdgeAlignment
  {
    double min_alignment_distance;
    Edge aligned_pred_edge;
    Edge aligned_meas_edge;
  };

  // Helper functions for determineUpdateStrategy
  EdgePositions calculateEdgeCenters(const types::DynamicObject & obj) const;
  EdgeAlignment findAlignedEdges(
    const EdgePositions & meas_edges, const types::DynamicObject & prediction) const;
  geometry_msgs::msg::Point calculateAnchorPoint(
    const EdgeAlignment & alignment, const types::DynamicObject & measurement) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__TRACKER__MODEL__VEHICLE_TRACKER_HPP_
