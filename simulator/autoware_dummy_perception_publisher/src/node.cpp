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

#include "autoware/dummy_perception_publisher/predicted_object_movement_plugin.hpp"
#include "autoware/dummy_perception_publisher/straight_line_object_movement_plugin.hpp"
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

#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace autoware::dummy_perception_publisher
{

using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Transform;
using geometry_msgs::msg::TransformStamped;
using tier4_simulation_msgs::msg::DummyObject;

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

  // optional ground truth publisher
  if (publish_ground_truth_objects_) {
    ground_truth_objects_pub_ =
      this->create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
        "~/output/debug/ground_truth_objects", qos);
  }

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&DummyPerceptionPublisherNode::timerCallback, this));

  // Initialize movement plugins directly in the vector
  movement_plugins_.push_back(std::make_shared<pluginlib::StraightLineObjectMovementPlugin>(this));
  movement_plugins_.push_back(std::make_shared<pluginlib::PredictedObjectMovementPlugin>(this));
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
  static std::uniform_real_distribution<> detection_successful_random(0.0, 1.0);

  // merge objects and get object infos
  std::vector<DummyObject> all_objects;
  std::vector<ObjectInfo> obj_infos;

  for (const auto & plugin : movement_plugins_) {
    const auto plugin_objects = plugin->get_objects();
    all_objects.insert(all_objects.end(), plugin_objects.begin(), plugin_objects.end());
    const auto plugin_infos = plugin->move_objects();
    obj_infos.insert(obj_infos.end(), plugin_infos.begin(), plugin_infos.end());
  }

  for (size_t i = 0; i < all_objects.size(); ++i) {
    if (detection_successful_rate_ >= detection_successful_random(random_generator_)) {
      selected_indices.push_back(i);
    }
  }

  // publish ground truth
  // add Tracked Object
  if (publish_ground_truth_objects_) {
    for (size_t i = 0; i < all_objects.size(); ++i) {
      const auto & object = all_objects[i];
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

  if (all_objects.empty()) {
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

    std::vector<unique_identifier_msgs::msg::UUID> delete_uuids;

    for (size_t i = 0; i < selected_indices.size(); ++i) {
      const auto & pointcloud = pointclouds[i];
      const size_t selected_idx = selected_indices[i];
      const auto & object = all_objects.at(selected_idx);
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
        delete_uuids.push_back(object.id);
      }
    }

    // delete
    for (const auto & uuid : delete_uuids) {
      for (auto & plugin : movement_plugins_) {
        plugin->delete_object(uuid);
      }
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
  auto create_dummy_object = [&]() -> std::optional<DummyObject> {
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
      return std::nullopt;
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
        return std::nullopt;
      }
    }
    return object;
  };

  auto get_modified_object_position = [&]() -> std::optional<DummyObject> {
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
      return std::nullopt;
    }
    tf2::fromMsg(msg->initial_state.pose_covariance.pose, tf_input2object_origin);
    tf_map2object_origin = tf_input2map.inverse() * tf_input2object_origin;
    DummyObject modified_object = *msg;
    tf2::toMsg(tf_map2object_origin, modified_object.initial_state.pose_covariance.pose);
    if (!use_base_link_z_) {
      return modified_object;
    }
    TransformStamped ros_map2base_link;
    try {
      ros_map2base_link = tf_buffer_.lookupTransform(
        "map", "base_link", rclcpp::Time(0), rclcpp::Duration::from_seconds(0.5));
      modified_object.initial_state.pose_covariance.pose.position.z =
        ros_map2base_link.transform.translation.z + 0.5 * modified_object.shape.dimensions.z;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "%s", ex.what());
      return std::nullopt;
    }
    return modified_object;
  };

  switch (msg->action) {
    case tier4_simulation_msgs::msg::DummyObject::PREDICT:
    case tier4_simulation_msgs::msg::DummyObject::ADD: {
      auto object = create_dummy_object();
      if (!object) {
        break;
      }
      for (auto & plugin : movement_plugins_) {
        if (plugin->set_dummy_object(*object)) {
          break;
        }
      }
      break;
    }
    case tier4_simulation_msgs::msg::DummyObject::DELETE: {
      for (auto & plugin : movement_plugins_) {
        plugin->delete_object(msg->id);
      }
      break;
    }
    case tier4_simulation_msgs::msg::DummyObject::MODIFY: {
      auto modified_object = get_modified_object_position();
      if (modified_object) {
        for (auto & plugin : movement_plugins_) {
          plugin->modify_object(*modified_object);
        }
      }
      break;
    }
    case tier4_simulation_msgs::msg::DummyObject::DELETEALL: {
      for (auto & plugin : movement_plugins_) {
        plugin->clear_objects();
      }
      break;
    }
  }
}

}  // namespace autoware::dummy_perception_publisher
