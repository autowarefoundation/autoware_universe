// Copyright 2025 Instituto de Telecomunições-Porto Branch, Inc. All rights reserved.
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

#include "autoware/spheric_collision_detector/spheric_collision_detector_node.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace spheric_collision_detector
{
SphericCollisionDetectorNode::SphericCollisionDetectorNode(const rclcpp::NodeOptions & node_options)
: Node("spheric_collision_detector_node", node_options), updater_(this)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter<double>("update_rate");

  // Core Parameter
  param_.delay_time = declare_parameter<double>("delay_time");
  param_.max_deceleration = declare_parameter<double>("max_deceleration");
  // Dynamic Reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&SphericCollisionDetectorNode::paramCallback, this, _1));

  // Core
  spheric_collision_detector_ = std::make_unique<SphericCollisionDetector>(*this);
  spheric_collision_detector_->setParam(param_);

  // Subscriber
  self_pose_listener_ = std::make_shared<autoware_utils::SelfPoseListener>(this);
  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);

  sub_object_recognition_ = create_subscription<autoware_perception_msgs::msg::DetectedObjects>(
    "input/object_recognition", rclcpp::QoS{1},
    std::bind(&SphericCollisionDetectorNode::onObjectRecognition, this, _1));

  sub_predicted_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/predicted_trajectory", 1,
    std::bind(&SphericCollisionDetectorNode::onPredictedTrajectory, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", 1, std::bind(&SphericCollisionDetectorNode::onOdom, this, _1));

  // Publisher
  debug_publisher_ = std::make_shared<autoware_utils::DebugPublisher>(this, "debug/marker");
  time_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(this);

  // Diagnostic Updater
  updater_.setHardwareID("spheric_collision_detector");

  updater_.add(
    "spheric_collision_detector", this, &SphericCollisionDetectorNode::checkCollisionStatus);

  // Wait for first self pose
  self_pose_listener_->wait_for_first_pose();

  // Timer
  initTimer(1.0 / node_param_.update_rate);
}

void SphericCollisionDetectorNode::onObjectRecognition(
  const autoware_perception_msgs::msg::DetectedObjects::SharedPtr msg)
{
  object_recognition_ = msg;
}

void SphericCollisionDetectorNode::onPredictedTrajectory(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  predicted_trajectory_ = msg;
}

void SphericCollisionDetectorNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ = std::make_shared<geometry_msgs::msg::Twist>(msg->twist.twist);
}

void SphericCollisionDetectorNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&SphericCollisionDetectorNode::onTimer, this));
}

bool SphericCollisionDetectorNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "scd: waiting for current_pose...");
    return false;
  }

  if (!object_recognition_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "scd: waiting for object_recognition msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "scd: waiting for predicted_trajectory msg...");
    return false;
  }

  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "scd: waiting for current_twist msg...");
    return false;
  }

  return true;
}

bool SphericCollisionDetectorNode::isDataTimeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp).seconds() - now.seconds();
  if (pose_time_diff > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "scd: pose is timeout...");
    return true;
  }

  return false;
}

void SphericCollisionDetectorNode::onTimer()
{
  current_pose_ = self_pose_listener_->get_current_pose();

  if (object_recognition_) {
    const auto & header = object_recognition_->header;
    try {
      object_recognition_transform_ = tf_buffer_.lookupTransform(
        "map", header.frame_id, header.stamp, rclcpp::Duration::from_seconds(0.01));
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "scd: Could not transform map to %s: %s", header.frame_id.c_str(),
        ex.what());
      return;
    }
  }

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  input_.current_pose = current_pose_;
  input_.object_recognition = object_recognition_;
  input_.predicted_trajectory = predicted_trajectory_;
  input_.current_twist = current_twist_;
  input_.object_recognition_transform = object_recognition_transform_;
  output_.will_collide = false;

  output_ = spheric_collision_detector_->update(input_);

  if (output_.will_collide) {
    RCLCPP_INFO(this->get_logger(), "scd: collision-time: %3ld", output_.collision_elapsed_time);
  }
  // Force an immediate update as the state of the node has changed
  updater_.force_update();

  debug_publisher_->publish("marker_array", createMarkerArray());
  time_publisher_->publish(output_.processing_time_map);
}

rcl_interfaces::msg::SetParametersResult SphericCollisionDetectorNode::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    using autoware_utils::update_param;
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(parameters, "update_rate", p.update_rate);
    }

    auto & p = param_;

    update_param(parameters, "delay_time", p.delay_time);
    update_param(parameters, "max_deceleration", p.max_deceleration);

    if (spheric_collision_detector_) {
      spheric_collision_detector_->setParam(param_);
    }
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }
  return result;
}

void SphericCollisionDetectorNode::checkCollisionStatus(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (output_.will_collide) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "scd: vehicle will collide with obstacles";
  }

  stat.summary(level, msg);
}

visualization_msgs::msg::MarkerArray SphericCollisionDetectorNode::createMarkerArray() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  addResampledTrajectoryMarkers(marker_array);
  addVehiclePassingAreaMarkers(marker_array);
  addObstacleMarkers(marker_array);

  return marker_array;
}

void SphericCollisionDetectorNode::addResampledTrajectoryMarkers(
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (output_.resampled_trajectory.points.size() < 2) {
    return;  // early return avoids nesting
  }

  // Line
  auto line_marker = autoware_utils::create_default_marker(
    "map", this->now(), "scd_resampled_trajectory_line", 0,
    visualization_msgs::msg::Marker::LINE_STRIP, autoware_utils::create_marker_scale(0.05, 0, 0),
    autoware_utils::create_marker_color(1.0, 1.0, 1.0, 0.999));

  for (const auto & p : output_.resampled_trajectory.points) {
    line_marker.points.push_back(p.pose.position);
    line_marker.colors.push_back(line_marker.color);
  }

  marker_array.markers.push_back(line_marker);

  // Points
  auto point_marker = autoware_utils::create_default_marker(
    "map", this->now(), "scd_resampled_trajectory_points", 0,
    visualization_msgs::msg::Marker::SPHERE_LIST,
    autoware_utils::create_marker_scale(0.1, 0.1, 0.1),
    autoware_utils::create_marker_color(0.0, 0.0, 0.0, 0.999));

  for (const auto & p : output_.resampled_trajectory.points) {
    point_marker.points.push_back(p.pose.position);
    point_marker.colors.push_back(point_marker.color);
  }

  marker_array.markers.push_back(point_marker);
}

void SphericCollisionDetectorNode::addVehiclePassingAreaMarkers(
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (output_.vehicle_passing_areas.empty()) {
    return;
  }

  const auto color_ok = autoware_utils::create_marker_color(1.0, 1.0, 0.0, 0.5);
  const auto color_will_collide = autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.3);
  const auto color = output_.will_collide ? color_will_collide : color_ok;

  auto marker = autoware_utils::create_default_marker(
    "map", this->now(), "scd_ego_passing_area", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
    autoware_utils::create_marker_scale(0.05, 0.05, 0.05), color);

  for (const auto & area : output_.vehicle_passing_areas) {
    const auto & c = area->center_;
    const auto dm = 2.0 * area->radius_;

    marker.scale.x = dm;
    marker.scale.y = dm;
    marker.scale.z = dm;

    marker.points.push_back(toMsg(Eigen::Vector3d(c.x(), c.y(), c.z())));
  }

  marker_array.markers.push_back(marker);
}

void SphericCollisionDetectorNode::addObstacleMarkers(
  visualization_msgs::msg::MarkerArray & marker_array) const
{
  if (output_.obstacles.empty()) {
    return;
  }

  auto marker = autoware_utils::create_default_marker(
    "map", this->now(), "scd_obstacle_spheres", 0, visualization_msgs::msg::Marker::SPHERE_LIST,
    autoware_utils::create_marker_scale(0.03, 0.03, 0.03),
    autoware_utils::create_marker_color(1.0, 1.0, 0.0, 0.5));

  for (const auto & obstacle : output_.obstacles) {
    for (const auto & sphere : obstacle) {
      const auto & c = sphere->center_;
      const auto dm = 2.0 * sphere->radius_;

      marker.scale.x = dm;
      marker.scale.y = dm;
      marker.scale.z = dm;

      marker.points.push_back(toMsg(Eigen::Vector3d(c.x(), c.y(), c.z())));
    }
  }

  marker_array.markers.push_back(marker);
}

}  // namespace spheric_collision_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(spheric_collision_detector::SphericCollisionDetectorNode)
