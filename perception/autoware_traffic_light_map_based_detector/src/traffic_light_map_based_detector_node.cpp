// Copyright 2023 TIER IV, Inc.
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

#define EIGEN_MPL2_ONLY

#include "traffic_light_map_based_detector_node.hpp"

#include <Eigen/Core>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light
{
MapBasedDetector::MapBasedDetector(const rclcpp::NodeOptions & node_options)
: Node("traffic_light_map_based_detector", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  // detector config
  detector_config_ = {
    this->declare_parameter<double>("max_vibration_pitch"),
    this->declare_parameter<double>("max_vibration_yaw"),
    this->declare_parameter<double>("max_vibration_height"),
    this->declare_parameter<double>("max_vibration_width"),
    this->declare_parameter<double>("max_vibration_depth"),
    this->declare_parameter<double>("max_detection_range"),
    this->declare_parameter<double>("car_traffic_light_max_angle_range"),
    this->declare_parameter<double>("pedestrian_traffic_light_max_angle_range")};
  // transform sampling config
  transform_sampling_config_ = {
    this->declare_parameter<double>("min_timestamp_offset"),
    this->declare_parameter<double>("max_timestamp_offset")};

  if (
    transform_sampling_config_.max_timestamp_offset <
    transform_sampling_config_.min_timestamp_offset) {
    throw std::invalid_argument(
      "max_timestamp_offset must be greater than or equal to min_timestamp_offset");
  }

  // subscribers
  map_sub_ = create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedDetector::mapCallback, this, _1));
  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "~/input/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&MapBasedDetector::cameraInfoCallback, this, _1));
  route_sub_ = create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "~/input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&MapBasedDetector::routeCallback, this, _1));

  // publishers
  roi_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/output/rois", 1);
  expect_roi_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::TrafficLightRoiArray>("~/expect/rois", 1);
  viz_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);
}

bool MapBasedDetector::getTransform(
  const rclcpp::Time & t, const std::string & frame_id, tf2::Transform & tf) const
{
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_.lookupTransform("map", frame_id, t, rclcpp::Duration::from_seconds(0.2));
    tf2::fromMsg(transform.transform, tf);
  } catch (const tf2::TransformException & ex) {
    return false;
  }
  return true;
}

void MapBasedDetector::cameraInfoCallback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg)
{
  if (!detector_) {
    return;
  }

  /* Camera pose in the period*/
  std::vector<tf2::Transform> tf_map2camera_vec;
  rclcpp::Time t1 = rclcpp::Time(input_msg->header.stamp) +
                    rclcpp::Duration::from_seconds(transform_sampling_config_.min_timestamp_offset);
  rclcpp::Time t2 = rclcpp::Time(input_msg->header.stamp) +
                    rclcpp::Duration::from_seconds(transform_sampling_config_.max_timestamp_offset);
  rclcpp::Duration interval = rclcpp::Duration::from_seconds(0.01);
  for (auto t = t1; t <= t2; t += interval) {
    tf2::Transform tf;
    if (getTransform(t, input_msg->header.frame_id, tf)) {
      tf_map2camera_vec.push_back(tf);
    }
  }
  /* camera pose at the exact moment*/
  tf2::Transform tf_map2camera;
  if (!getTransform(
        rclcpp::Time(input_msg->header.stamp), input_msg->header.frame_id, tf_map2camera)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "failed to get transform from map frame to camera frame");
    return;
  }
  if (tf_map2camera_vec.empty()) {
    tf_map2camera_vec.push_back(tf_map2camera);
  }

  auto result = detector_->detect(tf_map2camera_vec, tf_map2camera, *input_msg);

  roi_pub_->publish(result.rough_rois);
  expect_roi_pub_->publish(result.expect_rois);
  viz_pub_->publish(result.markers);
}

void MapBasedDetector::mapCallback(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg)
{
  detector_ = std::make_unique<TrafficLightMapBasedDetector>(detector_config_, *input_msg);
}

void MapBasedDetector::routeCallback(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr input_msg)
{
  if (!detector_) {
    RCLCPP_WARN(get_logger(), "failed to set traffic lights in route: map not received");
    return;
  }
  auto error = detector_->setRoute(*input_msg);
  if (error) {
    RCLCPP_ERROR(get_logger(), "%s", error->message.c_str());
  }
}
}  // namespace autoware::traffic_light

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::traffic_light::MapBasedDetector)
