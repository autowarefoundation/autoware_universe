// Copyright 2026 TIER IV, Inc.
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

#ifndef AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_NODE_HPP_
#define AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_NODE_HPP_

#include "autoware/proximity_hazard_object_checker/proximity_hazard_object_checker.hpp"

#include <autoware_proximity_hazard_object_checker/msg/proximity_hazard_objects.hpp>
#include <autoware_utils_rclcpp/polling_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

namespace autoware::proximity_hazard_object_checker
{
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_proximity_hazard_object_checker::msg::ProximityHazardObjects;
using nav_msgs::msg::Odometry;

class ProximityHazardObjectCheckerNode : public rclcpp::Node
{
public:
  explicit ProximityHazardObjectCheckerNode(const rclcpp::NodeOptions & options);

private:
  void on_timer();
  void publish_sector_markers(const std::string & frame_id);

  autoware_utils_rclcpp::InterProcessPollingSubscriber<Odometry> sub_odometry_{
    this, "~/input/odometry"};
  autoware_utils_rclcpp::InterProcessPollingSubscriber<PredictedObjects> sub_objects_{
    this, "~/input/objects"};
  rclcpp::Publisher<ProximityHazardObjects>::SharedPtr pub_hazards_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::shared_ptr<proximity_hazard_object::ParamListener> param_listener_;
  autoware_utils_geometry::LinearRing2d vehicle_footprint_;

  std::unique_ptr<ProximityHazardObjectChecker> impl_;
};

}  // namespace autoware::proximity_hazard_object_checker

#endif  // AUTOWARE__PROXIMITY_HAZARD_OBJECT_CHECKER__PROXIMITY_HAZARD_OBJECT_CHECKER_NODE_HPP_
