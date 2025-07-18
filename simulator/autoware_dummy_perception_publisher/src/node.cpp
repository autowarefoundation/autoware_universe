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

#include "autoware_utils_geometry/geometry.hpp"

#include <autoware_utils_uuid/uuid_helper.hpp>

#include <pcl/filters/voxel_grid_occlusion_estimation.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::dummy_perception_publisher
{

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

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
  // calculate current pose
  const auto & initial_pose = object.initial_state.pose_covariance.pose;
  const double initial_vel = std::clamp(
    object.initial_state.twist_covariance.twist.linear.x, static_cast<double>(object.min_velocity),
    static_cast<double>(object.max_velocity));
  const double initial_acc = object.initial_state.accel_covariance.accel.linear.x;

  const double elapsed_time = current_time.seconds() - rclcpp::Time(object.header.stamp).seconds();

  double move_distance;
  double current_vel = initial_vel + initial_acc * elapsed_time;
  if (initial_acc == 0.0) {
    move_distance = initial_vel * elapsed_time;
  } else {
    if (initial_acc < 0 && 0 < initial_vel) {
      current_vel = std::max(current_vel, 0.0);
    }
    if (0 < initial_acc && initial_vel < 0) {
      current_vel = std::min(current_vel, 0.0);
    }

    // add distance on acceleration or deceleration
    current_vel = std::clamp(
      current_vel, static_cast<double>(object.min_velocity),
      static_cast<double>(object.max_velocity));
    move_distance = (std::pow(current_vel, 2) - std::pow(initial_vel, 2)) * 0.5 / initial_acc;

    // add distance after reaching max_velocity
    if (0 < initial_acc) {
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
  }

  const auto current_pose =
    autoware_utils_geometry::calc_offset_pose(initial_pose, move_distance, 0.0, 0.0);

  // calculate tf from map to moved_object
  geometry_msgs::msg::Transform ros_map2moved_object;
  ros_map2moved_object.translation.x = current_pose.position.x;
  ros_map2moved_object.translation.y = current_pose.position.y;
  ros_map2moved_object.translation.z = current_pose.position.z;
  ros_map2moved_object.rotation = current_pose.orientation;
  tf2::fromMsg(ros_map2moved_object, tf_map2moved_object);
  // set twist and pose information
  twist_covariance_.twist.linear.x = current_vel;
  pose_covariance_.pose = current_pose;
}

ObjectInfo::ObjectInfo(
  const tier4_simulation_msgs::msg::DummyObject & object,
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const rclcpp::Time & predicted_time, const rclcpp::Time & current_time)
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
  // Check if 2 seconds have passed since object creation
  const double time_since_creation = (current_time - rclcpp::Time(object.header.stamp)).seconds();
  const double PREDICTED_PATH_DELAY = 2.0;  // seconds

  // Use straight-line movement for first 2 seconds, then switch to predicted path
  if (
    time_since_creation < PREDICTED_PATH_DELAY ||
    predicted_object.kinematics.predicted_paths.empty()) {
    // Reuse the logic from the other constructor
    *this = ObjectInfo(object, current_time);
    return;
  }

  // Find path with highest confidence
  auto best_path_it = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });

  const auto & best_path = *best_path_it;

  // Calculate elapsed time from predicted object timestamp to current time
  const double elapsed_time = (current_time - predicted_time).seconds();

  // Calculate distance traveled based on elapsed time and dummy object speed
  const double speed = object.initial_state.twist_covariance.twist.linear.x;
  const double distance_traveled = speed * elapsed_time;

  // Calculate cumulative distances along the path
  std::vector<double> cumulative_distances;
  cumulative_distances.push_back(0.0);

  for (size_t i = 1; i < best_path.path.size(); ++i) {
    const auto & prev_pose = best_path.path[i - 1];
    const auto & curr_pose = best_path.path[i];

    const double dx = curr_pose.position.x - prev_pose.position.x;
    const double dy = curr_pose.position.y - prev_pose.position.y;
    const double dz = curr_pose.position.z - prev_pose.position.z;
    const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);

    cumulative_distances.push_back(cumulative_distances.back() + segment_length);
  }

  geometry_msgs::msg::Pose interpolated_pose;

  if (distance_traveled <= 0.0 || best_path.path.empty()) {
    // Use initial pose if no distance traveled or path is empty
    interpolated_pose = predicted_object.kinematics.initial_pose_with_covariance.pose;
  } else if (distance_traveled >= cumulative_distances.back()) {
    // Extrapolate beyond the path end
    const double overshoot_distance = distance_traveled - cumulative_distances.back();
    
    if (best_path.path.size() >= 2) {
      // Use the last two points to determine direction and extrapolate
      const auto & second_last_pose = best_path.path[best_path.path.size() - 2];
      const auto & last_pose = best_path.path.back();
      
      // Calculate direction vector from second-last to last pose
      const double dx = last_pose.position.x - second_last_pose.position.x;
      const double dy = last_pose.position.y - second_last_pose.position.y;
      const double dz = last_pose.position.z - second_last_pose.position.z;
      const double segment_length = std::sqrt(dx * dx + dy * dy + dz * dz);
      
      if (segment_length > 0.0) {
        // Normalize direction vector
        const double dir_x = dx / segment_length;
        const double dir_y = dy / segment_length;
        const double dir_z = dz / segment_length;
        
        // Extrapolate position
        interpolated_pose.position.x = last_pose.position.x + dir_x * overshoot_distance;
        interpolated_pose.position.y = last_pose.position.y + dir_y * overshoot_distance;
        interpolated_pose.position.z = last_pose.position.z + dir_z * overshoot_distance;
        
        // Keep the last orientation
        interpolated_pose.orientation = last_pose.orientation;
      } else {
        // Fallback to last pose if segment length is zero
        interpolated_pose = last_pose;
      }
    } else {
      // Fallback to last pose if path has only one point
      interpolated_pose = best_path.path.back();
    }
  } else {
    // Find which segment the distance falls into
    size_t segment_index = 0;
    for (size_t i = 1; i < cumulative_distances.size(); ++i) {
      if (distance_traveled <= cumulative_distances[i]) {
        segment_index = i - 1;
        break;
      }
    }
    // Calculate interpolation ratio within the segment
    const double segment_start_distance = cumulative_distances[segment_index];
    const double segment_end_distance = cumulative_distances[segment_index + 1];
    const double segment_length = segment_end_distance - segment_start_distance;

    const double interpolation_ratio =
      (segment_length > 0.0) ? (distance_traveled - segment_start_distance) / segment_length : 0.0;

    // Interpolate between the two poses
    const auto & pose1 = best_path.path[segment_index];
    const auto & pose2 = best_path.path[segment_index + 1];

    // Linear interpolation for position
    interpolated_pose.position.x =
      pose1.position.x + (pose2.position.x - pose1.position.x) * interpolation_ratio;
    interpolated_pose.position.y =
      pose1.position.y + (pose2.position.y - pose1.position.y) * interpolation_ratio;
    interpolated_pose.position.z =
      pose1.position.z + (pose2.position.z - pose1.position.z) * interpolation_ratio;

    // Spherical linear interpolation for orientation
    tf2::Quaternion q1, q2;
    tf2::fromMsg(pose1.orientation, q1);
    tf2::fromMsg(pose2.orientation, q2);
    tf2::Quaternion q_interpolated = q1.slerp(q2, interpolation_ratio);
    interpolated_pose.orientation = tf2::toMsg(q_interpolated);
  }

  // Update pose and transform
  pose_covariance_.pose = interpolated_pose;
  tf2::fromMsg(interpolated_pose, tf_map2moved_object);

  // Use dummy object's velocity consistently
  twist_covariance_.twist.linear.x = object.initial_state.twist_covariance.twist.linear.x;
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
  predicted_objects_sub_ =
    this->create_subscription<autoware_perception_msgs::msg::PredictedObjects>(
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
  geometry_msgs::msg::PoseStamped output_moved_object_pose;
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
    geometry_msgs::msg::TransformStamped ros_base_link2map;
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
      if (matched_predicted) {
        return ObjectInfo(object, predicted_object, predicted_time, current_time);
      }
      
      // Check if we have a last used prediction for this object
      const auto & dummy_uuid_str = autoware_utils_uuid::to_hex_string(object.id);
      auto last_used_pred_it = dummy_last_used_predictions_.find(dummy_uuid_str);
      auto last_used_time_it = dummy_last_used_prediction_times_.find(dummy_uuid_str);
      
      if (last_used_pred_it != dummy_last_used_predictions_.end() && 
          last_used_time_it != dummy_last_used_prediction_times_.end()) {
        std::cerr << "Using last known prediction for lost object with ID: " << dummy_uuid_str << std::endl;
        return ObjectInfo(object, last_used_pred_it->second, last_used_time_it->second, current_time);
      }

      std::cerr << "No matching predicted object found for dummy object with ID: " << dummy_uuid_str << std::endl;
      // No last known prediction, use original constructor
      return ObjectInfo(object, current_time);
    }();
    obj_infos.push_back(obj_info);

    // Update last known position based on calculated ObjectInfo position
    const auto & dummy_uuid_str = autoware_utils_uuid::to_hex_string(object.id);
    dummy_last_known_positions_[dummy_uuid_str] = obj_info.pose_covariance_.pose.position;

    // Track object creation time if not already tracked
    if (dummy_creation_timestamps_.find(dummy_uuid_str) == dummy_creation_timestamps_.end()) {
      dummy_creation_timestamps_[dummy_uuid_str] = rclcpp::Time(object.header.stamp);
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
    case tier4_simulation_msgs::msg::DummyObject::ADD: {
      tf2::Transform tf_input2map;
      tf2::Transform tf_input2object_origin;
      tf2::Transform tf_map2object_origin;
      try {
        geometry_msgs::msg::TransformStamped ros_input2map;
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
        geometry_msgs::msg::TransformStamped ros_map2base_link;
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
            geometry_msgs::msg::TransformStamped ros_input2map;
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
            geometry_msgs::msg::TransformStamped ros_map2base_link;
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
  const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg)
{
  // Add to buffer, removing oldest if necessary
  if (predicted_objects_buffer_.size() >= MAX_BUFFER_SIZE) {
    predicted_objects_buffer_.pop_front();
  }
  predicted_objects_buffer_.push_back(*msg);

  // Update the dummy-to-predicted mapping based on euclidean distance
  updateDummyToPredictedMapping(objects_, *msg);
}

std::pair<autoware_perception_msgs::msg::PredictedObject, rclcpp::Time>
DummyPerceptionPublisherNode::findMatchingPredictedObject(
  const unique_identifier_msgs::msg::UUID & object_id, const rclcpp::Time & current_time)
{
  autoware_perception_msgs::msg::PredictedObject empty_object;
  rclcpp::Time empty_time(0, 0, RCL_ROS_TIME);

  const auto & obj_uuid_str = autoware_utils_uuid::to_hex_string(object_id);

  // Check if this dummy object is mapped to a predicted object
  auto mapping_it = dummy_to_predicted_uuid_map_.find(obj_uuid_str);
  if (mapping_it == dummy_to_predicted_uuid_map_.end()) {
    return std::make_pair(empty_object, empty_time);
  }

  const std::string & mapped_predicted_uuid = mapping_it->second;

  // Check if we should keep using the current prediction for at least 1 second
  const double MIN_KEEP_DURATION = 3.0;  // seconds

  // Check if we have a last used prediction and if we should keep using it
  auto last_used_pred_it = dummy_last_used_predictions_.find(obj_uuid_str);
  auto last_update_time_it = dummy_prediction_update_timestamps_.find(obj_uuid_str);

  if (
    last_used_pred_it != dummy_last_used_predictions_.end() &&
    last_update_time_it != dummy_prediction_update_timestamps_.end()) {
    const double time_since_last_update = (current_time - last_update_time_it->second).seconds();

    // If less than 1 second has passed since last update, keep using the same prediction
    if (time_since_last_update < MIN_KEEP_DURATION) {
      auto last_used_time_it = dummy_last_used_prediction_times_.find(obj_uuid_str);
      if (last_used_time_it != dummy_last_used_prediction_times_.end()) {
        return std::make_pair(last_used_pred_it->second, last_used_time_it->second);
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
        // Validate trajectory against current prediction if available
        auto current_pred_it = dummy_last_used_predictions_.find(obj_uuid_str);
        if (current_pred_it != dummy_last_used_predictions_.end()) {
          // Check if the new trajectory is valid compared to current one
          if (!isTrajectoryValid(current_pred_it->second, predicted_object, obj_uuid_str)) {
            // Skip this prediction update and keep using the current one
            std::cerr << "Skipping trajectory update for object " << obj_uuid_str << " due to validation failure" << std::endl;
            return std::make_pair(current_pred_it->second, dummy_last_used_prediction_times_[obj_uuid_str]);
          }
        }
        
        // Store this as the new prediction to use for the next 1 second
        dummy_last_used_predictions_[obj_uuid_str] = predicted_object;
        dummy_last_used_prediction_times_[obj_uuid_str] = msg_time;
        dummy_prediction_update_timestamps_[obj_uuid_str] = current_time;

        return std::make_pair(predicted_object, msg_time);
      }
    }
  }

  return std::make_pair(empty_object, empty_time);
}

void DummyPerceptionPublisherNode::updateDummyToPredictedMapping(
  const std::vector<tier4_simulation_msgs::msg::DummyObject> & dummy_objects,
  const autoware_perception_msgs::msg::PredictedObjects & predicted_objects)
{
  const rclcpp::Time current_time = this->now();

  // Create sets of available UUIDs
  std::set<std::string> available_predicted_uuids;
  std::map<std::string, geometry_msgs::msg::Point> predicted_positions;

  for (const auto & pred_obj : predicted_objects.objects) {
    const auto pred_uuid_str = autoware_utils_uuid::to_hex_string(pred_obj.object_id);
    available_predicted_uuids.insert(pred_uuid_str);
    predicted_positions[pred_uuid_str] =
      pred_obj.kinematics.initial_pose_with_covariance.pose.position;
  }

  // Check for disappeared predicted objects and mark dummy objects for remapping
  std::vector<std::string> dummy_objects_to_remap;
  for (const auto & mapping : dummy_to_predicted_uuid_map_) {
    const std::string & dummy_uuid = mapping.first;
    const std::string & predicted_uuid = mapping.second;

    // If the predicted object ID no longer exists, mark dummy for remapping
    if (available_predicted_uuids.find(predicted_uuid) == available_predicted_uuids.end()) {
      dummy_objects_to_remap.push_back(dummy_uuid);
    } else {
      // Remove already assigned predicted objects from available set
      available_predicted_uuids.erase(predicted_uuid);
    }
  }

  // Update dummy object positions and find unmapped dummy objects
  std::vector<std::string> unmapped_dummy_uuids;
  std::map<std::string, geometry_msgs::msg::Point> dummy_positions;

  for (const auto & dummy_obj : dummy_objects) {
    const auto dummy_uuid_str = autoware_utils_uuid::to_hex_string(dummy_obj.id);
    dummy_positions[dummy_uuid_str] = dummy_obj.initial_state.pose_covariance.pose.position;

    if (dummy_to_predicted_uuid_map_.find(dummy_uuid_str) == dummy_to_predicted_uuid_map_.end()) {
      unmapped_dummy_uuids.push_back(dummy_uuid_str);
    }
  }

  // Handle remapping for dummy objects whose predicted objects disappeared
  // First, remove old mappings
  for (const auto & dummy_uuid : dummy_objects_to_remap) {
    dummy_to_predicted_uuid_map_.erase(dummy_uuid);
  }

  // Then, find best matches for all objects that need remapping
  std::vector<std::pair<std::string, std::string>> new_mappings;   // dummy_uuid -> predicted_uuid
  std::vector<std::pair<std::string, double>> mapping_candidates;  // dummy_uuid -> distance

  for (const auto & dummy_uuid : dummy_objects_to_remap) {
    // Use last known position if available, otherwise use current position
    geometry_msgs::msg::Point remapping_position;
    auto last_pos_it = dummy_last_known_positions_.find(dummy_uuid);
    if (last_pos_it != dummy_last_known_positions_.end()) {
      remapping_position = last_pos_it->second;
    } else {
      // Fallback to current position if last known position is not available
      auto current_pos_it = dummy_positions.find(dummy_uuid);
      if (current_pos_it != dummy_positions.end()) {
        remapping_position = current_pos_it->second;
      } else {
        // Skip if we can't find any position for this dummy object
        continue;
      }
    }

    // Find closest available predicted object for remapping
    std::string closest_pred_uuid;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto & pred_uuid : available_predicted_uuids) {
      const auto & pred_pos = predicted_positions[pred_uuid];
      double distance = calculateEuclideanDistance(remapping_position, pred_pos);

      if (distance < min_distance) {
        min_distance = distance;
        closest_pred_uuid = pred_uuid;
      }
    }

    // Store candidate mapping with distance for prioritization
    if (!closest_pred_uuid.empty()) {
      mapping_candidates.emplace_back(dummy_uuid + ":" + closest_pred_uuid, min_distance);
    }
  }

  // Sort candidates by distance to ensure closest matches get priority
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
      dummy_to_predicted_uuid_map_[dummy_uuid] = predicted_uuid;
      dummy_mapping_timestamps_[dummy_uuid] = current_time;
      available_predicted_uuids.erase(predicted_uuid);
    }
  }

  // Map unmapped dummy objects to closest available predicted objects
  for (const auto & dummy_uuid : unmapped_dummy_uuids) {
    if (available_predicted_uuids.empty()) {
      break;
    }

    const auto & dummy_pos = dummy_positions[dummy_uuid];
    std::string closest_pred_uuid;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto & pred_uuid : available_predicted_uuids) {
      const auto & pred_pos = predicted_positions[pred_uuid];
      double distance = calculateEuclideanDistance(dummy_pos, pred_pos);

      if (distance < min_distance) {
        min_distance = distance;
        closest_pred_uuid = pred_uuid;
      }
    }

    if (!closest_pred_uuid.empty()) {
      dummy_to_predicted_uuid_map_[dummy_uuid] = closest_pred_uuid;
      dummy_mapping_timestamps_[dummy_uuid] = current_time;
      available_predicted_uuids.erase(closest_pred_uuid);
    }
  }

  // Clean up mappings for dummy objects that no longer exist
  std::set<std::string> current_dummy_uuids;
  for (const auto & dummy_obj : dummy_objects) {
    current_dummy_uuids.insert(autoware_utils_uuid::to_hex_string(dummy_obj.id));
  }

  for (auto it = dummy_to_predicted_uuid_map_.begin(); it != dummy_to_predicted_uuid_map_.end();) {
    if (current_dummy_uuids.find(it->first) == current_dummy_uuids.end()) {
      dummy_mapping_timestamps_.erase(it->first);
      dummy_last_known_positions_.erase(it->first);
      dummy_creation_timestamps_.erase(it->first);
      dummy_last_used_predictions_.erase(it->first);
      dummy_last_used_prediction_times_.erase(it->first);
      dummy_prediction_update_timestamps_.erase(it->first);
      it = dummy_to_predicted_uuid_map_.erase(it);
    } else {
      ++it;
    }
  }

  // Update last known positions for all dummy objects
  for (const auto & dummy_obj : dummy_objects) {
    const auto dummy_uuid_str = autoware_utils_uuid::to_hex_string(dummy_obj.id);
    dummy_last_known_positions_[dummy_uuid_str] =
      dummy_obj.initial_state.pose_covariance.pose.position;
  }
}

double DummyPerceptionPublisherNode::calculateEuclideanDistance(
  const geometry_msgs::msg::Point & pos1, const geometry_msgs::msg::Point & pos2)
{
  double dx = pos1.x - pos2.x;
  double dy = pos1.y - pos2.y;
  double dz = pos1.z - pos2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool DummyPerceptionPublisherNode::isTrajectoryValid(
  const autoware_perception_msgs::msg::PredictedObject & current_prediction,
  const autoware_perception_msgs::msg::PredictedObject & new_prediction,
  const std::string & dummy_uuid_str)
{
  // Maximum acceptable yaw change in radians (90 degrees)
  const double MAX_YAW_CHANGE = M_PI / 2.0;
  
  // Maximum acceptable direction change in radians (120 degrees)
  const double MAX_DIRECTION_CHANGE = 2.0 * M_PI / 3.0;
  
  // Maximum acceptable velocity change ratio (500% change)
  const double MAX_VELOCITY_CHANGE_RATIO = 5.0;
  
  // Maximum acceptable path length change ratio (300% change)
  const double MAX_PATH_LENGTH_CHANGE_RATIO = 3.0;
  
  // If current prediction is empty, accept any new prediction
  if (current_prediction.kinematics.predicted_paths.empty()) {
    return true;
  }
  
  // If new prediction is empty, reject it
  if (new_prediction.kinematics.predicted_paths.empty()) {
    return false;
  }
  
  // Get the highest confidence paths from both predictions
  const auto & current_path = *std::max_element(
    current_prediction.kinematics.predicted_paths.begin(),
    current_prediction.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
    
  const auto & new_path = *std::max_element(
    new_prediction.kinematics.predicted_paths.begin(),
    new_prediction.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  
  // Check yaw angle change
  if (!current_path.path.empty() && !new_path.path.empty()) {
    // Get yaw from quaternion
    auto extractYaw = [](const geometry_msgs::msg::Quaternion & q) {
      tf2::Quaternion tf_q;
      tf2::fromMsg(q, tf_q);
      double roll, pitch, yaw;
      tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
      return yaw;
    };
    
    const double current_yaw = extractYaw(current_path.path[0].orientation);
    const double new_yaw = extractYaw(new_path.path[0].orientation);
    
    // Calculate yaw difference, handling wrap-around
    double yaw_diff = std::abs(new_yaw - current_yaw);
    if (yaw_diff > M_PI) {
      yaw_diff = 2.0 * M_PI - yaw_diff;
    }
    
    if (yaw_diff > MAX_YAW_CHANGE) {
      std::cerr << "Rejecting trajectory for object " << dummy_uuid_str 
                << " due to large yaw change: " << yaw_diff << " rad" << std::endl;
      return false;
    }
  }
  
  // Check direction change using velocity vectors
  const auto & current_vel = current_prediction.kinematics.initial_twist_with_covariance.twist.linear;
  const auto & new_vel = new_prediction.kinematics.initial_twist_with_covariance.twist.linear;
  
  // Calculate current and new velocity magnitudes
  const double current_speed = std::sqrt(current_vel.x * current_vel.x + current_vel.y * current_vel.y);
  const double new_speed = std::sqrt(new_vel.x * new_vel.x + new_vel.y * new_vel.y);
  
  // Check velocity magnitude change
  if (current_speed > 0.1 && new_speed > 0.1) {  // Only check if both speeds are significant
    const double speed_ratio = std::max(current_speed / new_speed, new_speed / current_speed);
    if (speed_ratio > MAX_VELOCITY_CHANGE_RATIO) {
      std::cerr << "Rejecting trajectory for object " << dummy_uuid_str 
                << " due to large speed change: " << speed_ratio << "x" << std::endl;
      return false;
    }
    
    // Check direction change using dot product
    const double dot_product = (current_vel.x * new_vel.x + current_vel.y * new_vel.y);
    const double cos_angle = dot_product / (current_speed * new_speed);
    
    // Clamp to avoid numerical issues
    const double clamped_cos = std::max(-1.0, std::min(1.0, cos_angle));
    const double direction_change = std::acos(clamped_cos);
    
    if (direction_change > MAX_DIRECTION_CHANGE) {
      std::cerr << "Rejecting trajectory for object " << dummy_uuid_str 
                << " due to large direction change: " << direction_change << " rad" << std::endl;
      return false;
    }
  }
  
  // Check path length change
  if (!current_path.path.empty() && !new_path.path.empty()) {
    // Calculate path lengths
    auto calculatePathLength = [](const auto & path) {
      double total_length = 0.0;
      for (size_t i = 1; i < path.path.size(); ++i) {
        const auto & prev_pose = path.path[i - 1];
        const auto & curr_pose = path.path[i];
        
        const double dx = curr_pose.position.x - prev_pose.position.x;
        const double dy = curr_pose.position.y - prev_pose.position.y;
        const double dz = curr_pose.position.z - prev_pose.position.z;
        total_length += std::sqrt(dx * dx + dy * dy + dz * dz);
      }
      return total_length;
    };
    
    const double current_path_length = calculatePathLength(current_path);
    const double new_path_length = calculatePathLength(new_path);
    
    // Only check if both paths have significant length
    if (current_path_length > 0.1 && new_path_length > 0.1) {
      const double length_ratio = std::max(current_path_length / new_path_length, new_path_length / current_path_length);
      if (length_ratio > MAX_PATH_LENGTH_CHANGE_RATIO) {
        std::cerr << "Rejecting trajectory for object " << dummy_uuid_str 
                  << " due to large path length change: " << length_ratio << "x (current: " 
                  << current_path_length << "m, new: " << new_path_length << "m)" << std::endl;
        return false;
      }
    }
  }
  
  return true;
}

}  // namespace autoware::dummy_perception_publisher
