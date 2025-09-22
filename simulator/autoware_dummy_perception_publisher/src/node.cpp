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
: Node("dummy_perception_publisher"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  dummy_predicted_movement_plugin_(this)
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

  // optional ground truth publisher
  if (publish_ground_truth_objects_) {
    ground_truth_objects_pub_ =
      this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "~/output/debug/ground_truth_objects", qos);
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DummyPerceptionPublisherNode::timerCallback, this));

  pluginlib::PredictedObjectMovementPlugin plugin(this);
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
    ObjectInfo obj_info = [&]() {
      // Use straight-line motion (original constructor) for all other actions
      return ObjectInfo(object, current_time);
    }();
    obj_infos.push_back(obj_info);
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
      DummyObject object;
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

}  // namespace autoware::dummy_perception_publisher
