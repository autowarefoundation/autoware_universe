// Copyright 2026 Autoware Foundation
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

#ifndef AUTOWARE__AVOIDANCE_TARGET_DETECTOR__NODE_HPP_
#define AUTOWARE__AVOIDANCE_TARGET_DETECTOR__NODE_HPP_

#include "autoware/avoidance_target_detector/boundary.hpp"
#include "autoware/avoidance_target_detector/object_filtering.hpp"
#include "autoware/avoidance_target_detector/parameter.hpp"
#include "autoware_utils/ros/polling_subscriber.hpp"

#include <autoware/trajectory/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

namespace autoware::avoidance_target_detector
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObjects;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::Path;
using autoware_planning_msgs::msg::Trajectory;
using autoware_planning_msgs::msg::TrajectoryPoint;
using visualization_msgs::msg::MarkerArray;

/** ROS 2 node that detects and publishes avoidance target objects. */
class AvoidanceTargetDetectorNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the avoidance target detector node.
   * @param node_options Node options for component loading.
   */
  explicit AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options);

private:
  /**
   * @brief Callback for incoming predicted objects.
   * @param msg Predicted objects message.
   */
  void on_objects(const PredictedObjects::ConstSharedPtr msg);

  /**
   * @brief Callback for incoming tracked objects.
   * @param msg Tracked objects message.
   */
  void on_tracked_objects(const TrackedObjects::ConstSharedPtr msg);

  void update_ego_trajectory(const TrajectoryPoint & ego_point);

  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr sub_tracked_objects_;
  autoware_utils::InterProcessPollingSubscriber<Trajectory> sub_trajectory_{
    this, "~/input/trajectory"};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletRoute, autoware_utils::polling_policy::Newest>
    sub_route_{this, "~/input/route", rclcpp::QoS{1}.transient_local()};
  autoware_utils::InterProcessPollingSubscriber<
    LaneletMapBin, autoware_utils::polling_policy::Newest>
    sub_lanelet_map_{this, "~/input/lanelet_map_bin", rclcpp::QoS{1}.transient_local()};

  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_avoidance_targets_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_driving_along_vehicles_;
  rclcpp::Publisher<TrackedObjects>::SharedPtr pub_tracked_avoidance_targets_;
  rclcpp::Publisher<TrackedObjects>::SharedPtr pub_tracked_driving_along_vehicles_;
  rclcpp::Publisher<Path>::SharedPtr pub_drivable_area_path_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_near_segment_polygon_;

  std::shared_ptr<ExtendedRouteHandler> extended_route_handler_;
  PredictedObjectSelector object_selector_;
  TrackedObjectSelector tracked_object_selector_;

  LaneletMapBin::ConstSharedPtr map_bin_;
  Trajectory::ConstSharedPtr trajectory_;
  LaneletRoute::ConstSharedPtr route_;

  autoware::experimental::trajectory::Trajectory<TrajectoryPoint> ego_trajectory_;
  bool ego_trajectory_built_{false};
};

}  // namespace autoware::avoidance_target_detector

#endif  // AUTOWARE__AVOIDANCE_TARGET_DETECTOR__NODE_HPP_
