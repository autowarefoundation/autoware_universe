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

#include "autoware/avoidance_target_detector/node.hpp"

#include <autoware/marker_utils/marker_conversion.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <memory>
#include <vector>

namespace autoware::avoidance_target_detector
{

/**
 * @brief Construct the avoidance target detector node.
 * @param node_options Node options for component loading.
 */
AvoidanceTargetDetectorNode::AvoidanceTargetDetectorNode(const rclcpp::NodeOptions & node_options)
: Node{"avoidance_target_detector", node_options},
  sub_objects_{create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&AvoidanceTargetDetectorNode::on_objects, this, std::placeholders::_1))},
  sub_tracked_objects_{create_subscription<TrackedObjects>(
    "~/input/tracked_objects", rclcpp::QoS{1},
    std::bind(&AvoidanceTargetDetectorNode::on_tracked_objects, this, std::placeholders::_1))},
  pub_avoidance_targets_{create_publisher<PredictedObjects>("~/output/avoidance_targets", 1)},
  pub_driving_along_vehicles_{
    create_publisher<PredictedObjects>("~/output/driving_along_vehicles", 1)},
  pub_tracked_avoidance_targets_{
    create_publisher<TrackedObjects>("~/output/tracked_avoidance_targets", 1)},
  pub_tracked_driving_along_vehicles_{
    create_publisher<TrackedObjects>("~/output/tracked_driving_along_vehicles", 1)},
  pub_drivable_area_path_{create_publisher<Path>("~/output/drivable_area", 1)},
  pub_near_segment_polygon_{
    create_publisher<MarkerArray>("~/debug/near_segment_polygon", rclcpp::QoS{1}.transient_local())}
{
  declare_parameter<bool>("use_extended_route_bounds", true);
}

void AvoidanceTargetDetectorNode::update_ego_trajectory(const TrajectoryPoint & ego_point)
{
  namespace aw_trajectory = autoware::experimental::trajectory;
  constexpr double k_max_ego_trajectory_length_m = 100.0;
  constexpr std::size_t k_max_ego_trajectory_points = 100;

  std::vector<TrajectoryPoint> points;
  if (ego_trajectory_built_) {
    points = ego_trajectory_.restore();
  }
  points.push_back(ego_point);

  while (points.size() > k_max_ego_trajectory_points) {
    points.erase(points.begin());
  }

  const auto built = aw_trajectory::Trajectory<TrajectoryPoint>::Builder{}.build(points);
  if (!built) {
    return;
  }

  ego_trajectory_ = *built;
  ego_trajectory_built_ = true;

  while (ego_trajectory_.length() > k_max_ego_trajectory_length_m) {
    const double excess = ego_trajectory_.length() - k_max_ego_trajectory_length_m;
    ego_trajectory_.crop(excess, k_max_ego_trajectory_length_m);
  }
}

/**
 * @brief Callback for incoming predicted objects.
 * @param msg Predicted objects message.
 */
void AvoidanceTargetDetectorNode::on_objects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  bool map_or_route_updated = false;

  if (const auto route_msg = sub_route_.take_data()) {
    if (!route_msg->segments.empty()) {
      route_ = route_msg;
      map_or_route_updated = true;
    }
  }

  if (const auto map_msg = sub_lanelet_map_.take_data()) {
    map_bin_ = map_msg;
    map_or_route_updated = true;
  }

  if (map_or_route_updated && route_ && map_bin_) {
    extended_route_handler_ = std::make_shared<ExtendedRouteHandler>(*map_bin_, *route_);
    extended_route_handler_->create_map();
    extended_route_handler_->export_debug_map();
  }

  if (!route_ || !extended_route_handler_) {
    return;
  }

  trajectory_ = sub_trajectory_.take_data();
  const Trajectory trajectory_msg = trajectory_ ? *trajectory_ : Trajectory{};

  if (!trajectory_ || !extended_route_handler_->getOriginalRouteHandler()->isHandlerReady()) {
    return;
  }

  if (!trajectory_->points.empty()) {
    update_ego_trajectory(trajectory_->points.front());
  }

  const auto use_extended_bounds = get_parameter("use_extended_route_bounds").as_bool();
  const auto & route_bounds = use_extended_bounds
                                ? extended_route_handler_->get_extended_route_bounds()
                                : extended_route_handler_->get_original_route_bounds();
  pub_drivable_area_path_->publish(to_path_msg(route_bounds, *trajectory_));

  if (ego_trajectory_built_ && !trajectory_msg.points.empty()) {
    const auto ego_points = ego_trajectory_.restore();
    if (!ego_points.empty()) {
      const auto near_segment_polygon = extended_route_handler_->get_near_segment_polygon(
        ego_points.front().pose.position, trajectory_msg.points.back().pose.position);
      if (near_segment_polygon.size() >= 3) {
        using autoware_utils_visualization::create_marker_color;
        using autoware_utils_visualization::create_marker_scale;
        pub_near_segment_polygon_->publish(
          autoware::experimental::marker_utils::create_lanelet_polygon_marker_array(
            near_segment_polygon, get_clock()->now(), "near_segment_polygon", 0,
            create_marker_scale(0.4, 0.0, 0.0), create_marker_color(1.0, 0.5, 0.0, 0.7), 0.0));
      }
    }
  }

  object_selector_.update_objects(
    get_clock()->now(), *msg, trajectory_msg, *extended_route_handler_, ego_trajectory_,
    ego_trajectory_built_);

  const auto avoidance_targets =
    object_selector_.get_avoidance_targets(*msg, trajectory_msg, route_bounds);

  pub_avoidance_targets_->publish(avoidance_targets);

  PredictedObjects driving_along_vehicles;
  if (ego_trajectory_built_ && !trajectory_msg.points.empty()) {
    driving_along_vehicles = object_selector_.get_driving_along_vehicles(
      *msg, *extended_route_handler_, ego_trajectory_, trajectory_msg);
  }
  pub_driving_along_vehicles_->publish(driving_along_vehicles);
}

/**
 * @brief Callback for incoming tracked objects.
 * @param msg Tracked objects message.
 * @details Reuses the route handler and ego trajectory maintained by on_objects().
 */
void AvoidanceTargetDetectorNode::on_tracked_objects(const TrackedObjects::ConstSharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (!route_ || !extended_route_handler_ || !trajectory_) {
    return;
  }

  if (!extended_route_handler_->getOriginalRouteHandler()->isHandlerReady()) {
    return;
  }

  const Trajectory trajectory_msg = *trajectory_;

  const auto use_extended_bounds = get_parameter("use_extended_route_bounds").as_bool();
  const auto & route_bounds = use_extended_bounds
                                ? extended_route_handler_->get_extended_route_bounds()
                                : extended_route_handler_->get_original_route_bounds();

  tracked_object_selector_.update_objects(
    get_clock()->now(), *msg, trajectory_msg, *extended_route_handler_, ego_trajectory_,
    ego_trajectory_built_);

  const auto avoidance_targets =
    tracked_object_selector_.get_avoidance_targets(*msg, trajectory_msg, route_bounds);

  pub_tracked_avoidance_targets_->publish(avoidance_targets);

  TrackedObjects driving_along_vehicles;
  if (ego_trajectory_built_ && !trajectory_msg.points.empty()) {
    driving_along_vehicles = tracked_object_selector_.get_driving_along_vehicles(
      *msg, *extended_route_handler_, ego_trajectory_, trajectory_msg);
  }
  pub_tracked_driving_along_vehicles_->publish(driving_along_vehicles);
}

}  // namespace autoware::avoidance_target_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::avoidance_target_detector::AvoidanceTargetDetectorNode)
