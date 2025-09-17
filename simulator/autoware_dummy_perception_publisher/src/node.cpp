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

#include "autoware/dummy_perception_publisher/node.hpp"

#include "autoware/trajectory/interpolator/akima_spline.hpp"
#include "autoware/trajectory/interpolator/interpolator.hpp"
#include "autoware/trajectory/pose.hpp"
#include "autoware/trajectory/trajectory_point.hpp"
#include "autoware_utils_geometry/geometry.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <autoware_perception_msgs/msg/detail/tracked_objects__struct.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <optional>
#include <set>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::dummy_perception_publisher
{

using autoware::experimental::trajectory::interpolator::AkimaSpline;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_simulation_msgs::msg::DummyObject;
using InterpolationTrajectory = autoware::experimental::trajectory::Trajectory<Pose>;

ObjectInfo::ObjectInfo(
  const tier4_simulation_msgs::msg::DummyObject & object, const rclcpp::Time & current_time)
: length(object.shape.dimensions.x),
  width(object.shape.dimensions.y),
  height(object.shape.dimensions.z),
  std_dev_x(std::sqrt(object.initial_state.pose_covariance.covariance[0])),
  std_dev_y(std::sqrt(object.initial_state.pose_covariance.covariance[7])),
  std_dev_z(std::sqrt(object.initial_state.pose_covariance.covariance[14])),
  std_dev_yaw(std::sqrt(object.initial_state.pose_covariance.covariance[35])),
  twist_covariance_(object.initial_state.twist_covariance),
  pose_covariance_(object.initial_state.pose_covariance)
{
  const auto current_pose = calculateStraightLinePosition(object, current_time);

  // calculate tf from map to moved_object
  Transform ros_map2moved_object;
  ros_map2moved_object.translation.x = current_pose.position.x;
  ros_map2moved_object.translation.y = current_pose.position.y;
  ros_map2moved_object.translation.z = current_pose.position.z;
  ros_map2moved_object.rotation = current_pose.orientation;
  tf2::fromMsg(ros_map2moved_object, tf_map2moved_object);

  // set twist and pose information
  const double initial_vel = std::clamp(
    object.initial_state.twist_covariance.twist.linear.x, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));
  const double initial_acc = object.initial_state.accel_covariance.accel.linear.x;
  const double elapsed_time = current_time.seconds() - rclcpp::Time(object.header.stamp).seconds();
  double current_vel = initial_vel + initial_acc * elapsed_time;
  if (initial_acc != 0.0) {
    current_vel = std::clamp(
      current_vel, static_cast<double>(object.min_velocity),
      static_cast<double>(object.max_velocity));
  }

  stopAtZeroVelocity(current_vel, initial_vel, initial_acc);

  twist_covariance_.twist.linear.x = current_vel;
  pose_covariance_.pose = current_pose;
}

ObjectInfo::ObjectInfo(
  const tier4_simulation_msgs::msg::DummyObject & object, const PredictedObject & predicted_object,
  const rclcpp::Time & predicted_time, const rclcpp::Time & current_time,
  const double switch_time_threshold)
: length(object.shape.dimensions.x),
  width(object.shape.dimensions.y),
  height(object.shape.dimensions.z),
  std_dev_x(std::sqrt(object.initial_state.pose_covariance.covariance[0])),
  std_dev_y(std::sqrt(object.initial_state.pose_covariance.covariance[7])),
  std_dev_z(std::sqrt(object.initial_state.pose_covariance.covariance[14])),
  std_dev_yaw(std::sqrt(object.initial_state.pose_covariance.covariance[35])),
  twist_covariance_(object.initial_state.twist_covariance),
  pose_covariance_(object.initial_state.pose_covariance)
{
  // Check if threshold time has passed since object creation
  const double time_since_creation = (current_time - rclcpp::Time(object.header.stamp)).seconds();
  // Use straight-line movement for first switch_time_threshold seconds, then switch to predicted
  // path
  if (
    time_since_creation < switch_time_threshold ||
    predicted_object.kinematics.predicted_paths.empty()) {
    // Reuse the logic from the other constructor
    *this = ObjectInfo(object, current_time);
    return;
  }

  const auto interpolated_pose =
    calculateTrajectoryBasedPosition(object, predicted_object, predicted_time, current_time);

  // Update pose and transform
  pose_covariance_.pose = interpolated_pose;
  tf2::fromMsg(interpolated_pose, tf_map2moved_object);

  // Use dummy object's velocity consistently
  twist_covariance_.twist.linear.x = object.initial_state.twist_covariance.twist.linear.x;
}

void ObjectInfo::stopAtZeroVelocity(double & current_vel, double initial_vel, double initial_acc)
{
  // stop at zero velocity
  if (initial_acc < 0 && initial_vel > 0) {
    current_vel = std::max(current_vel, 0.0);
  }
  if (initial_acc > 0 && initial_vel < 0) {
    current_vel = std::min(current_vel, 0.0);
  }
}

Pose ObjectInfo::calculateStraightLinePosition(
  const tier4_simulation_msgs::msg::DummyObject & object, const rclcpp::Time & current_time)
{
  const auto & initial_pose = object.initial_state.pose_covariance.pose;
  const double initial_vel = std::clamp(
    object.initial_state.twist_covariance.twist.linear.x, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));
  const double initial_acc = object.initial_state.accel_covariance.accel.linear.x;

  const double elapsed_time = current_time.seconds() - rclcpp::Time(object.header.stamp).seconds();

  double move_distance{0.0};
  double current_vel = initial_vel + initial_acc * elapsed_time;
  if (initial_acc == 0.0) {
    move_distance = initial_vel * elapsed_time;
    return autoware_utils_geometry::calc_offset_pose(initial_pose, move_distance, 0.0, 0.0);
  }

  stopAtZeroVelocity(current_vel, initial_vel, initial_acc);
  // clamp velocity within min/max
  current_vel = std::clamp(
    current_vel, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));

  // add distance on acceleration or deceleration
  move_distance = (std::pow(current_vel, 2) - std::pow(initial_vel, 2)) * 0.5 / initial_acc;

  // add distance after reaching max_velocity
  if (initial_acc > 0) {
    const double time_to_reach_max_vel =
      std::max(static_cast<double>(object.max_velocity) - initial_vel, 0.0) / initial_acc;
    move_distance += static_cast<double>(object.max_velocity) *
                     std::max(elapsed_time - time_to_reach_max_vel, 0.0);
  }

  // add distance after reaching min_velocity
  if (initial_acc < 0) {
    const double time_to_reach_min_vel =
      std::min(static_cast<double>(object.min_velocity) - initial_vel, 0.0) / initial_acc;
    move_distance += static_cast<double>(object.min_velocity) *
                     std::max(elapsed_time - time_to_reach_min_vel, 0.0);
  }

  return autoware_utils_geometry::calc_offset_pose(initial_pose, move_distance, 0.0, 0.0);
}

Pose ObjectInfo::calculateTrajectoryBasedPosition(
  const tier4_simulation_msgs::msg::DummyObject & object, const PredictedObject & predicted_object,
  const rclcpp::Time & predicted_time, const rclcpp::Time & current_time)
{
  // Select first path (which has been ordered based on random or highest confidence strategy in
  // findMatchingPredictedObject)
  const auto & selected_path = predicted_object.kinematics.predicted_paths.front();

  // Calculate elapsed time from predicted object timestamp to current time
  const double elapsed_time = (current_time - predicted_time).seconds();

  // Calculate distance traveled based on elapsed time and dummy object speed
  const double speed = object.initial_state.twist_covariance.twist.linear.x;
  const double distance_traveled = speed * elapsed_time;

  if (distance_traveled <= 0.0 || selected_path.path.empty()) {
    return predicted_object.kinematics.initial_pose_with_covariance.pose;
  }

  if (selected_path.path.size() < 2) {  // Fallback to last pose if path has only one point
    return selected_path.path.back();
  }

  const auto interpolated_pose = std::invoke([&]() {
    auto trajectory_interpolation_util =
      InterpolationTrajectory::Builder{}
        .set_xy_interpolator<AkimaSpline>()  // Set interpolator for x-y plane
        .build(selected_path.path);

    if (!trajectory_interpolation_util) {
      // Fallback to last pose if failed to build interpolation trajectory
      return object.initial_state.pose_covariance.pose;
    }
    trajectory_interpolation_util->align_orientation_with_trajectory_direction();

    const auto total_length = trajectory_interpolation_util->length();
    // Check if the distance traveled exceeds the path length (extrapolation)
    if (distance_traveled >= total_length) {
      const double overshoot_distance = distance_traveled - total_length;

      // Use the last two points to determine direction and extrapolate
      const auto & second_last_pose = selected_path.path[selected_path.path.size() - 2];
      const auto & last_pose = selected_path.path.back();

      // Calculate direction vector from second-last to last pose
      const double dx = last_pose.position.x - second_last_pose.position.x;
      const double dy = last_pose.position.y - second_last_pose.position.y;
      const double dz = last_pose.position.z - second_last_pose.position.z;
      const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (segment_length < std::numeric_limits<double>::epsilon()) {  // Fallback to last pose if
                                                                      // segment length is zero
        return last_pose;
      }

      // Normalize direction vector
      const double dir_x = dx / segment_length;
      const double dir_y = dy / segment_length;
      const double dir_z = dz / segment_length;

      // Extrapolate position
      Pose interpolated_pose;
      interpolated_pose.position.x = last_pose.position.x + dir_x * overshoot_distance;
      interpolated_pose.position.y = last_pose.position.y + dir_y * overshoot_distance;
      interpolated_pose.position.z = last_pose.position.z + dir_z * overshoot_distance;

      // Keep the last orientation
      interpolated_pose.orientation = last_pose.orientation;
      return interpolated_pose;
    }

    // Interpolation within the path
    return trajectory_interpolation_util->compute(distance_traveled);
  });

  return interpolated_pose;
}

TrackedObject ObjectInfo::toTrackedObject(
  const tier4_simulation_msgs::msg::DummyObject & object) const
{
  TrackedObject tracked_object;
  tracked_object.kinematics.pose_with_covariance = pose_covariance_;
  tracked_object.kinematics.twist_with_covariance = twist_covariance_;
  tracked_object.classification.push_back(object.classification);
  tracked_object.shape.type = object.shape.type;
  tracked_object.shape.dimensions.x = length;
  tracked_object.shape.dimensions.y = width;
  tracked_object.shape.dimensions.z = height;
  tracked_object.object_id = object.id;
  tracked_object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN;
  return tracked_object;
}

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode()
: Node("dummy_perception_publisher"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  visible_range_ = this->declare_parameter("visible_range", 100.0);
  detection_successful_rate_ = this->declare_parameter("detection_successful_rate", 0.8);
  enable_ray_tracing_ = this->declare_parameter("enable_ray_tracing", true);
  use_object_recognition_ = this->declare_parameter("use_object_recognition", true);
  use_base_link_z_ = this->declare_parameter("use_base_link_z", true);
  const bool object_centric_pointcloud =
    this->declare_parameter("object_centric_pointcloud", false);
  publish_ground_truth_objects_ = this->declare_parameter("publish_ground_truth", false);
  const unsigned int random_seed =
    static_cast<unsigned int>(this->declare_parameter("random_seed", 0));
  const bool use_fixed_random_seed = this->declare_parameter("use_fixed_random_seed", false);

  if (object_centric_pointcloud) {
    pointcloud_creator_ =
      std::unique_ptr<PointCloudCreator>(new ObjectCentricPointCloudCreator(enable_ray_tracing_));
  } else {
    pointcloud_creator_ =
      std::unique_ptr<PointCloudCreator>(new EgoCentricPointCloudCreator(visible_range_));
  }

  // parameters for vehicle centric point cloud generation
  angle_increment_ = this->declare_parameter("angle_increment", 0.25 * M_PI / 180.0);

  if (use_fixed_random_seed) {
    random_generator_.seed(random_seed);
  } else {
    std::random_device seed_gen;
    random_generator_.seed(seed_gen());
  }

  // Initialize path selection distribution
  path_selection_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

  // Declare prediction parameters
  predicted_path_delay_ = this->declare_parameter("predicted_path_delay", 2.0);
  min_keep_duration_ = this->declare_parameter("min_keep_duration", 3.0);
  switch_time_threshold_ = this->declare_parameter("switch_time_threshold", 2.0);

  // Initialize vehicle parameters
  vehicle_params_ = {
    this->declare_parameter("vehicle.max_remapping_distance", 2.0),
    this->declare_parameter("vehicle.max_speed_difference_ratio", 1.05),
    this->declare_parameter("vehicle.min_speed_ratio", 0.5),
    this->declare_parameter("vehicle.max_speed_ratio", 1.5),
    this->declare_parameter("vehicle.speed_check_threshold", 1.0),
    this->declare_parameter("vehicle.path_selection_strategy", std::string("highest_confidence"))};
  // Initialize pedestrian parameters
  pedestrian_params_ = {
    this->declare_parameter("pedestrian.max_remapping_distance", 3.0),
    this->declare_parameter("pedestrian.max_speed_difference_ratio", 1.3),
    this->declare_parameter("pedestrian.min_speed_ratio", 0.3),
    this->declare_parameter("pedestrian.max_speed_ratio", 2.0),
    this->declare_parameter("pedestrian.speed_check_threshold", 0.5),
    this->declare_parameter("pedestrian.path_selection_strategy", std::string("random"))};

  // create subscriber and publisher
  rclcpp::QoS qos{1};
  qos.transient_local();
  detected_object_with_feature_pub_ =
    this->create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      "output/dynamic_object", qos);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output/points_raw", qos);
  object_sub_ = this->create_subscription<tier4_simulation_msgs::msg::DummyObject>(
    "input/object", 100,
    std::bind(&DummyPerceptionPublisherNode::objectCallback, this, std::placeholders::_1));
  predicted_objects_sub_ = this->create_subscription<PredictedObjects>(
    "input/predicted_objects", 100,
    std::bind(
      &DummyPerceptionPublisherNode::predictedObjectsCallback, this, std::placeholders::_1));

  // optional ground truth publisher
  if (publish_ground_truth_objects_) {
    ground_truth_objects_pub_ =
      this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "~/output/debug/ground_truth_objects", qos);
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DummyPerceptionPublisherNode::timerCallback, this));
}

void DummyPerceptionPublisherNode::timerCallback()
{
  // output msgs
  tier4_perception_msgs::msg::DetectedObjectsWithFeature output_dynamic_object_msg;
  autoware_perception_msgs::msg::TrackedObjects output_ground_truth_objects_msg;
  PoseStamped output_moved_object_pose;
  sensor_msgs::msg::PointCloud2 output_pointcloud_msg;
  std_msgs::msg::Header header;
  rclcpp::Time current_time = this->now();

  // avoid terminal contamination.
  static rclcpp::Time failed_tf_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
  if ((this->now() - failed_tf_time).seconds() < 5.0) {
    return;
  }

  std::string error;
  if (!tf_buffer_.canTransform("base_link", /*src*/ "map", tf2::TimePointZero, &error)) {
    failed_tf_time = this->now();
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000, "map->base_link is not available yet");
    return;
  }

  tf2::Transform tf_base_link2map;
  try {
    TransformStamped ros_base_link2map;
    ros_base_link2map = tf_buffer_.lookupTransform(
      /*target*/ "base_link", /*src*/ "map", current_time, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
    return;
  }

  std::vector<size_t> selected_indices{};
  std::vector<ObjectInfo> obj_infos;
  static std::uniform_real_distribution<> detection_successful_random(0.0, 1.0);
  for (size_t i = 0; i < objects_.size(); ++i) {
    if (detection_successful_rate_ >= detection_successful_random(random_generator_)) {
      selected_indices.push_back(i);
    }

    // Try to find matching predicted object
    const auto & object = objects_.at(i);
    auto predicted_object_pair = findMatchingPredictedObject(object.id, current_time);
    const auto & predicted_object = predicted_object_pair.first;
    const auto & predicted_time = predicted_object_pair.second;

    const bool matched_predicted =
      (!predicted_object.object_id.uuid.empty() &&
       !predicted_object.kinematics.predicted_paths.empty() && predicted_time.nanoseconds() > 0);

    ObjectInfo obj_info = [&]() {
      if (object.action == tier4_simulation_msgs::msg::DummyObject::PREDICT) {
        if (matched_predicted) {
          return ObjectInfo(
            object, predicted_object, predicted_time, current_time, switch_time_threshold_);
        }

        // Check if we have a last used prediction for this object
        const auto & dummy_uuid_str = autoware_utils_uuid::to_hex_string(object.id);
        auto info_it = dummy_predicted_info_map_.find(dummy_uuid_str);

        if (
          info_it != dummy_predicted_info_map_.end() &&
          info_it->second.last_used_prediction.has_value() &&
          info_it->second.last_used_prediction_time.has_value()) {
          RCLCPP_DEBUG(
            rclcpp::get_logger("dummy_perception_publisher"),
            "Using last known prediction for lost object with ID: %s", dummy_uuid_str.c_str());
          return ObjectInfo(
            object, info_it->second.last_used_prediction.value(),
            info_it->second.last_used_prediction_time.value(), current_time,
            switch_time_threshold_);
        }

        RCLCPP_DEBUG(
          rclcpp::get_logger("dummy_perception_publisher"),
          "No matching predicted object found for dummy object with ID: %s",
          dummy_uuid_str.c_str());
      }

      // Use straight-line motion (original constructor) for all other actions
      return ObjectInfo(object, current_time);
    }();
    obj_infos.push_back(obj_info);

    // Update last known position based on calculated ObjectInfo position
    const auto & dummy_uuid_str = autoware_utils_uuid::to_hex_string(object.id);
    dummy_predicted_info_map_[dummy_uuid_str].last_known_position =
      obj_info.pose_covariance_.pose.position;

    // Track object creation time if not already tracked
    auto & info = dummy_predicted_info_map_[dummy_uuid_str];
    if (!info.creation_timestamp.has_value()) {
      info.creation_timestamp = rclcpp::Time(object.header.stamp);
    }
  }

  // publish ground truth
  // add Tracked Object
  if (publish_ground_truth_objects_) {
    for (size_t i = 0; i < objects_.size(); ++i) {
      const auto & object = objects_[i];
      // Use the same ObjectInfo as calculated above for consistency
      const auto & object_info = obj_infos[i];
      TrackedObject gt_tracked_object = object_info.toTrackedObject(object);
      gt_tracked_object.existence_probability = 1.0;
      output_ground_truth_objects_msg.objects.push_back(gt_tracked_object);
    }
  }

  // publish noised detected objects
  pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr detected_merged_pointcloud_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  if (objects_.empty()) {
    pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);
  } else {
    pointcloud_creator_->create_pointclouds(
      obj_infos, tf_base_link2map, random_generator_, merged_pointcloud_ptr);
    pcl::toROSMsg(*merged_pointcloud_ptr, output_pointcloud_msg);
  }
  if (!selected_indices.empty()) {
    std::vector<ObjectInfo> detected_obj_infos;
    for (const auto selected_idx : selected_indices) {
      // Use the same ObjectInfo as calculated above for consistency
      const auto & detected_obj_info = obj_infos[selected_idx];
      tf2::toMsg(detected_obj_info.tf_map2moved_object, output_moved_object_pose.pose);
      detected_obj_infos.push_back(detected_obj_info);
    }

    const auto pointclouds = pointcloud_creator_->create_pointclouds(
      detected_obj_infos, tf_base_link2map, random_generator_, detected_merged_pointcloud_ptr);

    std::vector<size_t> delete_idxs;
    for (size_t i = 0; i < selected_indices.size(); ++i) {
      const auto pointcloud = pointclouds[i];
      const size_t selected_idx = selected_indices[i];
      const auto & object = objects_.at(selected_idx);
      const auto & object_info = obj_infos[selected_idx];
      // dynamic object
      std::normal_distribution<> x_random(0.0, object_info.std_dev_x);
      std::normal_distribution<> y_random(0.0, object_info.std_dev_y);
      std::normal_distribution<> yaw_random(0.0, object_info.std_dev_yaw);
      tf2::Quaternion noised_quat;
      noised_quat.setRPY(0, 0, yaw_random(random_generator_));
      tf2::Transform tf_moved_object2noised_moved_object(
        noised_quat, tf2::Vector3(x_random(random_generator_), y_random(random_generator_), 0.0));
      tf2::Transform tf_base_link2noised_moved_object;
      tf_base_link2noised_moved_object =
        tf_base_link2map * object_info.tf_map2moved_object * tf_moved_object2noised_moved_object;

      // add DetectedObjectWithFeature
      tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
      feature_object.object.classification.push_back(object.classification);
      feature_object.object.kinematics.pose_with_covariance = object.initial_state.pose_covariance;
      feature_object.object.kinematics.twist_with_covariance =
        object.initial_state.twist_covariance;
      feature_object.object.kinematics.orientation_availability =
        autoware_perception_msgs::msg::DetectedObjectKinematics::SIGN_UNKNOWN;
      feature_object.object.kinematics.has_twist = false;
      tf2::toMsg(
        tf_base_link2noised_moved_object,
        feature_object.object.kinematics.pose_with_covariance.pose);
      feature_object.object.shape = object.shape;
      pcl::toROSMsg(*pointcloud, feature_object.feature.cluster);
      output_dynamic_object_msg.feature_objects.push_back(feature_object);

      // check delete idx
      tf2::Transform tf_base_link2moved_object;
      tf_base_link2moved_object = tf_base_link2map * object_info.tf_map2moved_object;
      double dist = std::sqrt(
        tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
        tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
      if (visible_range_ < dist) {
        delete_idxs.push_back(selected_idx);
      }
    }

    // delete
    for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx) {
      objects_.erase(objects_.begin() + delete_idxs.at(delete_idx));
    }
  }

  // create output header
  output_moved_object_pose.header.frame_id = "map";
  output_moved_object_pose.header.stamp = current_time;
  output_dynamic_object_msg.header.frame_id = "base_link";
  output_dynamic_object_msg.header.stamp = current_time;
  output_pointcloud_msg.header.frame_id = "base_link";
  output_pointcloud_msg.header.stamp = current_time;
  output_ground_truth_objects_msg.header.frame_id = "map";
  output_ground_truth_objects_msg.header.stamp = current_time;

  // publish
  pointcloud_pub_->publish(output_pointcloud_msg);
  if (use_object_recognition_) {
    detected_object_with_feature_pub_->publish(output_dynamic_object_msg);
  }
  if (publish_ground_truth_objects_) {
    ground_truth_objects_pub_->publish(output_ground_truth_objects_msg);
  }
}

void DummyPerceptionPublisherNode::objectCallback(
  const tier4_simulation_msgs::msg::DummyObject::ConstSharedPtr msg)
{
  switch (msg->action) {
    case tier4_simulation_msgs::msg::DummyObject::PREDICT:
    case tier4_simulation_msgs::msg::DummyObject::ADD: {
      tf2::Transform tf_input2map;
      tf2::Transform tf_input2object_origin;
      tf2::Transform tf_map2object_origin;
      try {
        TransformStamped ros_input2map;
        ros_input2map = tf_buffer_.lookupTransform(
          /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
          rclcpp::Duration::from_seconds(0.5));
        tf2::fromMsg(ros_input2map.transform, tf_input2map);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
        return;
      }
      tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
      tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
      tier4_simulation_msgs::msg::DummyObject object;
      object = *msg;
      tf2::toMsg(tf_map2object_origin, object.initial_state.pose_covariance.pose);

      // Use base_link Z
      if (use_base_link_z_) {
        TransformStamped ros_map2base_link;
        try {
          ros_map2base_link = tf_buffer_.lookupTransform(
            "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
          object.initial_state.pose_covariance.pose.position.z =
            ros_map2base_link.transform.translation.z + 0.5 * object.shape.dimensions.z;
        } catch (tf2::TransformException & ex) {
          RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
          return;
        }
      }

      objects_.push_back(object);
      break;
    }
    case tier4_simulation_msgs::msg::DummyObject::DELETE: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          objects_.erase(objects_.begin() + i);
          break;
        }
      }
      break;
    }
    case tier4_simulation_msgs::msg::DummyObject::MODIFY: {
      for (size_t i = 0; i < objects_.size(); ++i) {
        if (objects_.at(i).id.uuid == msg->id.uuid) {
          tf2::Transform tf_input2map;
          tf2::Transform tf_input2object_origin;
          tf2::Transform tf_map2object_origin;
          try {
            TransformStamped ros_input2map;
            ros_input2map = tf_buffer_.lookupTransform(
              /*target*/ msg->header.frame_id, /*src*/ "map", msg->header.stamp,
              rclcpp::Duration::from_seconds(0.5));
            tf2::fromMsg(ros_input2map.transform, tf_input2map);
          } catch (tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
            return;
          }
          tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
          tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
          tier4_simulation_msgs::msg::DummyObject object;
          objects_.at(i) = *msg;
          tf2::toMsg(tf_map2object_origin, objects_.at(i).initial_state.pose_covariance.pose);
          if (use_base_link_z_) {
            // Use base_link Z
            TransformStamped ros_map2base_link;
            try {
              ros_map2base_link = tf_buffer_.lookupTransform(
                "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
              objects_.at(i).initial_state.pose_covariance.pose.position.z =
                ros_map2base_link.transform.translation.z + 0.5 * objects_.at(i).shape.dimensions.z;
            } catch (tf2::TransformException & ex) {
              RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
              return;
            }
          }

          break;
        }
      }
      break;
    }
    case tier4_simulation_msgs::msg::DummyObject::DELETEALL: {
      objects_.clear();
      break;
    }
  }
}

void DummyPerceptionPublisherNode::predictedObjectsCallback(
  const PredictedObjects::ConstSharedPtr msg)
{
  // Add to buffer, removing oldest if necessary
  if (predicted_objects_buffer_.size() >= MAX_BUFFER_SIZE) {
    predicted_objects_buffer_.pop_front();
  }
  predicted_objects_buffer_.push_back(*msg);

  // Update the dummy-to-predicted mapping based on euclidean distance
  updateDummyToPredictedMapping(objects_, *msg);
}

std::pair<PredictedObject, rclcpp::Time> DummyPerceptionPublisherNode::findMatchingPredictedObject(
  const unique_identifier_msgs::msg::UUID & object_id, const rclcpp::Time & current_time)
{
  PredictedObject empty_object;
  rclcpp::Time empty_time(0, 0, RCL_ROS_TIME);

  const auto & obj_uuid_str = autoware_utils_uuid::to_hex_string(object_id);

  // Check if this dummy object is mapped to a predicted object
  auto mapping_it = dummy_predicted_info_map_.find(obj_uuid_str);
  if (mapping_it == dummy_predicted_info_map_.end() || mapping_it->second.predicted_uuid.empty()) {
    return std::make_pair(empty_object, empty_time);
  }

  const std::string & mapped_predicted_uuid = mapping_it->second.predicted_uuid;

  // Check if we should keep using the current prediction for at least min_keep_duration
  // Check if we have a last used prediction and if we should keep using it
  auto info_it = dummy_predicted_info_map_.find(obj_uuid_str);
  if (info_it == dummy_predicted_info_map_.end()) {
    // Create entry if it doesn't exist (needed for later updates)
    dummy_predicted_info_map_[obj_uuid_str];
    info_it = dummy_predicted_info_map_.find(obj_uuid_str);
  }

  if (
    info_it->second.last_used_prediction.has_value() &&
    info_it->second.prediction_update_timestamp.has_value()) {
    const double time_since_last_update =
      (current_time - info_it->second.prediction_update_timestamp.value()).seconds();

    // If less than min_keep_duration_ has passed since last update, keep using the same prediction
    if (time_since_last_update < min_keep_duration_) {
      if (info_it->second.last_used_prediction_time.has_value()) {
        return std::make_pair(
          info_it->second.last_used_prediction.value(),
          info_it->second.last_used_prediction_time.value());
      }
    }
  }

  // Time to update: find the closest prediction in the past
  for (auto it = predicted_objects_buffer_.rbegin(); it != predicted_objects_buffer_.rend(); ++it) {
    const auto & predicted_objects_msg = *it;
    const rclcpp::Time msg_time(predicted_objects_msg.header.stamp);

    // Skip future messages
    if (msg_time > current_time) {
      continue;
    }

    // Look for the mapped predicted object UUID
    for (const auto & predicted_object : predicted_objects_msg.objects) {
      unique_identifier_msgs::msg::UUID pred_obj_uuid;
      pred_obj_uuid.uuid = predicted_object.object_id.uuid;
      const auto & pred_obj_uuid_str = autoware_utils_uuid::to_hex_string(pred_obj_uuid);

      if (pred_obj_uuid_str == mapped_predicted_uuid) {
        // Apply path selection strategy based on object type configuration
        PredictedObject modified_predicted_object = predicted_object;

        // Check if this is a pedestrian object
        const bool is_pedestrian = std::any_of(
          predicted_object.classification.begin(), predicted_object.classification.end(),
          [](const auto & classification) {
            return classification.label ==
                   autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN;
          });

        // Determine path selection strategy based on object type
        const std::string path_strategy = is_pedestrian ? pedestrian_params_.path_selection_strategy
                                                        : vehicle_params_.path_selection_strategy;

        if (!predicted_object.kinematics.predicted_paths.empty()) {
          auto & paths = modified_predicted_object.kinematics.predicted_paths;
          if (path_strategy == "random") {
            // Randomly select a path index
            const size_t num_paths = predicted_object.kinematics.predicted_paths.size();
            std::uniform_int_distribution<size_t> path_index_dist(0, num_paths - 1);
            const size_t random_path_index = path_index_dist(random_generator_);
            // Reorder paths to put the randomly selected path first
            std::swap(paths[0], paths[random_path_index]);

            RCLCPP_DEBUG(
              rclcpp::get_logger("dummy_perception_publisher"),
              "Randomly selected path %zu out of %zu for %s object %s", random_path_index,
              num_paths, is_pedestrian ? "pedestrian" : "vehicle", obj_uuid_str.c_str());
          } else if (path_strategy == "highest_confidence") {
            // Find path with highest confidence and move it to first position
            auto max_confidence_it = std::max_element(
              paths.begin(), paths.end(),
              [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

            if (max_confidence_it != paths.begin()) {
              std::swap(paths[0], *max_confidence_it);
            }

            RCLCPP_DEBUG(
              rclcpp::get_logger("dummy_perception_publisher"),
              "Selected most likely path (confidence: %.3f) for %s object %s", paths[0].confidence,
              is_pedestrian ? "pedestrian" : "vehicle", obj_uuid_str.c_str());
          }
        }

        // Store this as the new prediction to use for some seconds
        dummy_predicted_info_map_[obj_uuid_str].last_used_prediction = modified_predicted_object;
        dummy_predicted_info_map_[obj_uuid_str].last_used_prediction_time = msg_time;
        dummy_predicted_info_map_[obj_uuid_str].prediction_update_timestamp = current_time;

        return std::make_pair(modified_predicted_object, msg_time);
      }
    }
  }

  return std::make_pair(empty_object, empty_time);
}

std::set<std::string> DummyPerceptionPublisherNode::collectAvailablePredictedUUIDs(
  const PredictedObjects & predicted_objects, std::map<std::string, Point> & predicted_positions)
{
  std::set<std::string> available_predicted_uuids;

  for (const auto & pred_obj : predicted_objects.objects) {
    const auto pred_uuid_str = autoware_utils_uuid::to_hex_string(pred_obj.object_id);
    available_predicted_uuids.insert(pred_uuid_str);
    predicted_positions[pred_uuid_str] =
      pred_obj.kinematics.initial_pose_with_covariance.pose.position;
  }

  return available_predicted_uuids;
}

std::vector<std::string> DummyPerceptionPublisherNode::findDisappearedPredictedObjectUUIDs(
  std::set<std::string> & available_predicted_uuids)
{
  std::vector<std::string> dummy_objects_to_remap;

  for (const auto & mapping : dummy_predicted_info_map_) {
    const std::string & dummy_uuid = mapping.first;
    const std::string & predicted_uuid = mapping.second.predicted_uuid;

    // Skip entries without a predicted UUID mapping
    if (predicted_uuid.empty()) {
      continue;
    }

    // If the predicted object ID no longer exists, mark dummy for remapping
    if (available_predicted_uuids.find(predicted_uuid) == available_predicted_uuids.end()) {
      dummy_objects_to_remap.push_back(dummy_uuid);
    } else {
      // Remove already assigned predicted objects from available set
      available_predicted_uuids.erase(predicted_uuid);
    }
  }

  return dummy_objects_to_remap;
}

std::map<std::string, Point> DummyPerceptionPublisherNode::collectDummyObjectPositions(
  const std::vector<tier4_simulation_msgs::msg::DummyObject> & dummy_objects,
  const rclcpp::Time & current_time, std::vector<std::string> & unmapped_dummy_uuids)
{
  std::map<std::string, Point> dummy_positions;

  for (const auto & dummy_obj : dummy_objects) {
    const auto dummy_uuid_str = autoware_utils_uuid::to_hex_string(dummy_obj.id);

    // Use last known position if available (which includes straight-line calculated position)
    // Otherwise calculate current position using straight-line model
    auto info_it = dummy_predicted_info_map_.find(dummy_uuid_str);

    dummy_positions[dummy_uuid_str] =
      (info_it != dummy_predicted_info_map_.end() &&
       info_it->second.last_known_position.has_value())
        ? info_it->second.last_known_position.value()
        : ObjectInfo::calculateStraightLinePosition(dummy_obj, current_time).position;

    if (info_it == dummy_predicted_info_map_.end() || info_it->second.predicted_uuid.empty()) {
      unmapped_dummy_uuids.push_back(dummy_uuid_str);
    }
  }

  return dummy_positions;
}

std::optional<std::string> DummyPerceptionPublisherNode::findBestPredictedObjectMatch(
  const std::string & dummy_uuid, const Point & dummy_position,
  const std::set<std::string> & available_predicted_uuids,
  const std::map<std::string, Point> & predicted_positions,
  const PredictedObjects & predicted_objects)
{
  // Get the best matching predicted object based on several metrics
  std::string closest_pred_uuid;
  double min_distance = std::numeric_limits<double>::max();

  // Iterate over all available predicted objects to find the best match
  for (const auto & pred_uuid : available_predicted_uuids) {
    const auto & pred_pos = predicted_positions.at(pred_uuid);
    auto pred_obj = std::find_if(
      predicted_objects.objects.begin(), predicted_objects.objects.end(),
      [&pred_uuid](const auto & obj) {
        return autoware_utils_uuid::to_hex_string(obj.object_id) == pred_uuid;
      });

    // Handle case where predicted object is not found (should not happen)
    if (pred_obj == predicted_objects.objects.end()) {
      continue;
    }

    // Find the actual predicted object for validation
    const PredictedObject & candidate_pred_obj = *pred_obj;

    // In case of multiple valid candidates, choose the closest one
    double distance = calculateEuclideanDistance(dummy_position, pred_pos);

    // Check if there is a valid remapping candidate based on position and speed
    if (
      distance >= min_distance ||
      !isValidRemappingCandidate(candidate_pred_obj, dummy_uuid, dummy_position)) {
      continue;
    }

    min_distance = distance;
    closest_pred_uuid = pred_uuid;
  }

  if (closest_pred_uuid.empty()) {
    return std::nullopt;
  }

  return closest_pred_uuid;
}

void DummyPerceptionPublisherNode::createRemappingsForDisappearedObjects(
  const std::vector<std::string> & dummy_objects_to_remap,
  std::set<std::string> & available_predicted_uuids,
  const std::map<std::string, Point> & predicted_positions,
  const std::map<std::string, Point> & dummy_positions, const PredictedObjects & predicted_objects)
{
  const rclcpp::Time current_time = this->now();

  // First, remove old mappings
  for (const auto & dummy_uuid : dummy_objects_to_remap) {
    auto it = dummy_predicted_info_map_.find(dummy_uuid);
    if (it != dummy_predicted_info_map_.end()) {
      it->second.predicted_uuid.clear();
    }
  }

  // Find best matches for all objects that need remapping
  std::vector<std::pair<std::string, double>> mapping_candidates;

  for (const auto & dummy_uuid : dummy_objects_to_remap) {
    // Use last known position if available, otherwise use current position
    Point remapping_position;
    auto current_pos_it = dummy_positions.find(dummy_uuid);
    if (current_pos_it == dummy_positions.end()) {
      continue;
    }
    auto info_it = dummy_predicted_info_map_.find(dummy_uuid);

    remapping_position = (info_it != dummy_predicted_info_map_.end() &&
                          info_it->second.last_known_position.has_value())
                           ? info_it->second.last_known_position.value()
                           : current_pos_it->second;
    // Find closest available predicted object for remapping
    auto best_match = findBestPredictedObjectMatch(
      dummy_uuid, remapping_position, available_predicted_uuids, predicted_positions,
      predicted_objects);

    if (best_match) {
      const double distance =
        calculateEuclideanDistance(remapping_position, predicted_positions.at(*best_match));
      mapping_candidates.emplace_back(dummy_uuid + ":" + *best_match, distance);
    }
  }

  // Sort candidates by distance to ensure closest matches get priority
  // This is because multiple dummy objects may have the same best match
  std::sort(
    mapping_candidates.begin(), mapping_candidates.end(),
    [](const auto & a, const auto & b) { return a.second < b.second; });

  // Create mappings in order of proximity, ensuring one-to-one mapping
  for (const auto & candidate : mapping_candidates) {
    const std::string & combined = candidate.first;
    const size_t colon_pos = combined.find(':');
    const std::string dummy_uuid = combined.substr(0, colon_pos);
    const std::string predicted_uuid = combined.substr(colon_pos + 1);

    // Only create mapping if predicted object is still available
    if (available_predicted_uuids.find(predicted_uuid) != available_predicted_uuids.end()) {
      dummy_predicted_info_map_[dummy_uuid].predicted_uuid = predicted_uuid;
      dummy_mapping_timestamps_[dummy_uuid] = current_time;
      available_predicted_uuids.erase(predicted_uuid);
    }
  }
}

void DummyPerceptionPublisherNode::updateDummyToPredictedMapping(
  const std::vector<tier4_simulation_msgs::msg::DummyObject> & dummy_objects,
  const PredictedObjects & predicted_objects)
{
  const rclcpp::Time current_time = this->now();

  // Create sets of available UUIDs
  std::map<std::string, Point> predicted_positions;
  std::set<std::string> available_predicted_uuids =
    collectAvailablePredictedUUIDs(predicted_objects, predicted_positions);

  // Check for disappeared predicted objects and mark dummy objects for remapping
  std::vector<std::string> dummy_objects_to_remap =
    findDisappearedPredictedObjectUUIDs(available_predicted_uuids);

  // Update dummy object positions and find unmapped dummy objects
  std::vector<std::string> unmapped_dummy_uuids;
  std::map<std::string, Point> dummy_positions =
    collectDummyObjectPositions(dummy_objects, current_time, unmapped_dummy_uuids);

  // Handle remapping for dummy objects whose predicted objects disappeared
  createRemappingsForDisappearedObjects(
    dummy_objects_to_remap, available_predicted_uuids, predicted_positions, dummy_positions,
    predicted_objects);

  // Map unmapped dummy objects to closest available predicted objects
  for (const auto & dummy_uuid : unmapped_dummy_uuids) {
    if (available_predicted_uuids.empty()) {
      break;
    }

    const auto & dummy_pos = dummy_positions[dummy_uuid];
    auto best_match = findBestPredictedObjectMatch(
      dummy_uuid, dummy_pos, available_predicted_uuids, predicted_positions, predicted_objects);

    if (best_match) {
      dummy_predicted_info_map_[dummy_uuid].predicted_uuid = *best_match;
      dummy_mapping_timestamps_[dummy_uuid] = current_time;
      available_predicted_uuids.erase(*best_match);
    }
  }

  std::set<std::string> current_dummy_uuids;
  for (const auto & dummy_obj : dummy_objects) {
    current_dummy_uuids.insert(autoware_utils_uuid::to_hex_string(dummy_obj.id));
  }

  // Clean up mappings for dummy objects that no longer exist
  for (auto it = dummy_predicted_info_map_.begin(); it != dummy_predicted_info_map_.end();) {
    if (current_dummy_uuids.find(it->first) != current_dummy_uuids.end()) {
      ++it;
      continue;
    }
    dummy_mapping_timestamps_.erase(it->first);
    it = dummy_predicted_info_map_.erase(it);
  }

  // Update last known positions for all dummy objects
  for (const auto & dummy_obj : dummy_objects) {
    const auto dummy_uuid_str = autoware_utils_uuid::to_hex_string(dummy_obj.id);
    dummy_predicted_info_map_[dummy_uuid_str].last_known_position =
      dummy_obj.initial_state.pose_covariance.pose.position;
  }
}

double DummyPerceptionPublisherNode::calculateEuclideanDistance(
  const Point & pos1, const Point & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double dz = pos1.z - pos2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool DummyPerceptionPublisherNode::isValidRemappingCandidate(
  const PredictedObject & candidate_prediction, const std::string & dummy_uuid_str,
  const Point & expected_position)
{
  // Perform various checks to validate if the candidate predicted object is a good match
  // for the dummy object
  auto dummy_it =
    std::find_if(objects_.begin(), objects_.end(), [&dummy_uuid_str](const auto & obj) {
      const auto uuid = autoware_utils_uuid::to_hex_string(obj.id);
      return uuid == dummy_uuid_str;
    });

  if (dummy_it == objects_.end()) {
    return false;
  }

  const auto & dummy_object = *dummy_it;
  // Check if this is a pedestrian object

  bool is_pedestrian =
    (dummy_object.classification.label ==
     autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN);

  // Use class-specific thresholds
  // Pedestrians: more lenient due to unpredictable movement patterns
  // Vehicles: stricter for more predictable movement
  const double max_remapping_distance = is_pedestrian ? pedestrian_params_.max_remapping_distance
                                                      : vehicle_params_.max_remapping_distance;
  const double max_speed_difference_ratio = is_pedestrian
                                              ? pedestrian_params_.max_speed_difference_ratio
                                              : vehicle_params_.max_speed_difference_ratio;

  // Check if candidate has predicted paths
  if (candidate_prediction.kinematics.predicted_paths.empty()) {
    return false;
  }

  // Get dummy object speed for comparison (reuse dummy_it from above)
  const double dummy_speed = dummy_object.initial_state.twist_covariance.twist.linear.x;

  // Get candidate predicted object speed
  const auto & candidate_twist =
    candidate_prediction.kinematics.initial_twist_with_covariance.twist;
  const double candidate_speed = std::sqrt(
    candidate_twist.linear.x * candidate_twist.linear.x +
    candidate_twist.linear.y * candidate_twist.linear.y);

  // Speed bounds check - more lenient for pedestrians
  const double min_speed_ratio =
    is_pedestrian ? pedestrian_params_.min_speed_ratio : vehicle_params_.min_speed_ratio;
  const double max_speed_ratio =
    is_pedestrian ? pedestrian_params_.max_speed_ratio : vehicle_params_.max_speed_ratio;
  const double speed_check_threshold = is_pedestrian ? pedestrian_params_.speed_check_threshold
                                                     : vehicle_params_.speed_check_threshold;

  auto is_within_speed_bounds = [&](double speed) {
    return speed >= min_speed_ratio * dummy_speed && speed <= max_speed_ratio * dummy_speed;
  };

  // Reject if candidate speed is too different when dummy speed is significant
  if (dummy_speed > speed_check_threshold && !is_within_speed_bounds(candidate_speed)) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Rejecting remapping candidate for object %s (%s) due to speed difference: dummy=%fm/s, "
      "candidate=%fm/s",
      dummy_uuid_str.c_str(), is_pedestrian ? "pedestrian" : "vehicle", dummy_speed,
      candidate_speed);
    return false;
  }

  // Compare speeds if both are significant
  if (dummy_speed > 0.1 && candidate_speed > 0.1) {
    const double speed_ratio =
      std::max(dummy_speed / candidate_speed, candidate_speed / dummy_speed);
    if (speed_ratio > max_speed_difference_ratio) {
      RCLCPP_DEBUG(
        rclcpp::get_logger("dummy_perception_publisher"),
        "Rejecting remapping candidate for object %s (%s) due to speed difference: %fx (dummy: "
        "%fm/s, "
        "candidate: %fm/s, max_ratio: %f)",
        dummy_uuid_str.c_str(), is_pedestrian ? "pedestrian" : "vehicle", speed_ratio, dummy_speed,
        candidate_speed, max_speed_difference_ratio);
      return false;
    }
  }

  // Calculate expected position based on last known trajectory if available
  Point comparison_position = expected_position;
  auto info_it = dummy_predicted_info_map_.find(dummy_uuid_str);
  if (
    info_it == dummy_predicted_info_map_.end() ||
    !info_it->second.last_used_prediction.has_value()) {
    return true;  // No last prediction, allow the match
  }

  // Calculate where the dummy object should be based on its last known trajectory
  const auto & last_trajectory =
    info_it->second.last_used_prediction.value().kinematics.predicted_paths.at(0);
  const auto expected_pos = calculateExpectedPosition(last_trajectory, dummy_uuid_str);
  if (expected_pos.has_value()) {
    comparison_position = expected_pos.value();
  }

  // Check position similarity
  const auto & candidate_pos =
    candidate_prediction.kinematics.initial_pose_with_covariance.pose.position;
  const double distance = calculateEuclideanDistance(comparison_position, candidate_pos);

  if (distance > max_remapping_distance) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("dummy_perception_publisher"),
      "Rejecting remapping candidate for object %s (%s) due to large distance: %fm > %fm "
      "(expected: %f, %f, "
      "candidate: %f, %f)",
      dummy_uuid_str.c_str(), is_pedestrian ? "pedestrian" : "vehicle", distance,
      max_remapping_distance, comparison_position.x, comparison_position.y, candidate_pos.x,
      candidate_pos.y);
    return false;
  }

  return true;
}

std::optional<Point> DummyPerceptionPublisherNode::calculateExpectedPosition(
  const autoware_perception_msgs::msg::PredictedPath & last_prediction,
  const std::string & dummy_uuid_str)
{
  // Check if we have predicted paths
  if (last_prediction.path.empty()) {
    return std::nullopt;
  }

  // Find the dummy object by UUID
  auto dummy_it =
    std::find_if(objects_.begin(), objects_.end(), [&dummy_uuid_str](const auto & obj) {
      const auto uuid = autoware_utils_uuid::to_hex_string(obj.id);
      return uuid == dummy_uuid_str;
    });

  if (dummy_it == objects_.end()) {
    return std::nullopt;
  }

  const auto & dummy_object = *dummy_it;

  const auto & selected_path = last_prediction;

  if (selected_path.path.empty()) {
    return std::nullopt;
  }

  // If only one point in path, return that point
  if (selected_path.path.size() < 2) {
    return selected_path.path.back().position;
  }

  // Calculate elapsed time from last prediction to current time
  const auto current_time = get_clock()->now();
  auto info_it = dummy_predicted_info_map_.find(dummy_uuid_str);
  if (
    info_it == dummy_predicted_info_map_.end() ||
    !info_it->second.last_used_prediction_time.has_value()) {
    return std::nullopt;
  }

  const double elapsed_time =
    (current_time - info_it->second.last_used_prediction_time.value()).seconds();

  // Calculate distance traveled based on elapsed time and dummy object speed
  const double speed = dummy_object.initial_state.twist_covariance.twist.linear.x;
  const double distance_traveled = speed * elapsed_time;

  // Calculate cumulative distances along the path
  std::vector<double> cumulative_distances;
  cumulative_distances.reserve(selected_path.path.size());
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < selected_path.path.size(); ++i) {
    const auto & prev_pose = selected_path.path.at(i - 1);
    const auto & curr_pose = selected_path.path.at(i);

    const double dx = curr_pose.position.x - prev_pose.position.x;
    const double dy = curr_pose.position.y - prev_pose.position.y;
    const double dz = curr_pose.position.z - prev_pose.position.z;
    const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    cumulative_distances.push_back(cumulative_distances.back() + segment_length);
  }

  Point expected_position;

  if (distance_traveled <= 0.0) {
    // No movement, use initial position
    return std::nullopt;
  }
  if (distance_traveled >= cumulative_distances.back()) {
    // Extrapolate beyond the path end
    // Use the last two points to determine direction and extrapolate
    const auto & second_last_pose = selected_path.path.at(selected_path.path.size() - 2);
    const auto & last_pose = selected_path.path.back();

    // Calculate direction vector from second-last to last pose
    const double dx = last_pose.position.x - second_last_pose.position.x;
    const double dy = last_pose.position.y - second_last_pose.position.y;
    const double dz = last_pose.position.z - second_last_pose.position.z;
    const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (segment_length < std::numeric_limits<double>::epsilon()) {
      return last_pose.position;
    }
    // Normalize direction vector
    const double dir_x = dx / segment_length;
    const double dir_y = dy / segment_length;
    const double dir_z = dz / segment_length;

    // Extrapolate position
    const double overshoot_distance = distance_traveled - cumulative_distances.back();
    expected_position.x = last_pose.position.x + dir_x * overshoot_distance;
    expected_position.y = last_pose.position.y + dir_y * overshoot_distance;
    expected_position.z = last_pose.position.z + dir_z * overshoot_distance;
    return expected_position;
  }

  // Interpolate along the path
  for (size_t i = 1; i < cumulative_distances.size(); ++i) {
    if (distance_traveled > cumulative_distances.at(i)) {
      continue;
    }
    // Interpolate between path points i-1 and i
    const double segment_start_distance = cumulative_distances.at(i - 1);
    const double segment_end_distance = cumulative_distances.at(i);
    const double segment_length = segment_end_distance - segment_start_distance;

    if (segment_length < std::numeric_limits<double>::epsilon()) {
      return selected_path.path.at(i - 1).position;
    }
    const double interpolation_factor =
      (distance_traveled - segment_start_distance) / segment_length;

    const auto & start_pose = selected_path.path.at(i - 1);
    const auto & end_pose = selected_path.path.at(1);

    expected_position.x =
      start_pose.position.x + interpolation_factor * (end_pose.position.x - start_pose.position.x);
    expected_position.y =
      start_pose.position.y + interpolation_factor * (end_pose.position.y - start_pose.position.y);
    expected_position.z =
      start_pose.position.z + interpolation_factor * (end_pose.position.z - start_pose.position.z);
    return expected_position;
  }
  return selected_path.path.back().position;
}
}  // namespace autoware::dummy_perception_publisher
