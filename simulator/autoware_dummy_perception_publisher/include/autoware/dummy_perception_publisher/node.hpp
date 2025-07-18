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

#ifndef AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
#define AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <tier4_simulation_msgs/msg/dummy_object.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <pcl/common/distances.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <deque>
#include <map>
#include <memory>
#include <random>
#include <vector>

namespace autoware::dummy_perception_publisher
{
struct ObjectInfo
{
  ObjectInfo(
    const tier4_simulation_msgs::msg::DummyObject & object, const rclcpp::Time & current_time);
  ObjectInfo(
    const tier4_simulation_msgs::msg::DummyObject & object,
    const autoware_perception_msgs::msg::PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time);
  double length;
  double width;
  double height;
  double std_dev_x;
  double std_dev_y;
  double std_dev_z;
  double std_dev_yaw;
  tf2::Transform tf_map2moved_object;
  // pose and twist
  geometry_msgs::msg::TwistWithCovariance twist_covariance_;
  geometry_msgs::msg::PoseWithCovariance pose_covariance_;
  // convert to TrackedObject
  // (todo) currently need object input to get id and header information, but it should be removed
  autoware_perception_msgs::msg::TrackedObject toTrackedObject(
    const tier4_simulation_msgs::msg::DummyObject & object) const;
};

class PointCloudCreator
{
public:
  virtual ~PointCloudCreator() {}

  virtual std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const = 0;
};

class ObjectCentricPointCloudCreator : public PointCloudCreator
{
public:
  explicit ObjectCentricPointCloudCreator(bool enable_ray_tracing)
  : enable_ray_tracing_(enable_ray_tracing)
  {
  }

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const override;

private:
  void create_object_pointcloud(
    const ObjectInfo & obj_info, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) const;

  bool enable_ray_tracing_;
};

class EgoCentricPointCloudCreator : public PointCloudCreator
{
public:
  explicit EgoCentricPointCloudCreator(double visible_range) : visible_range_(visible_range) {}
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> create_pointclouds(
    const std::vector<ObjectInfo> & obj_infos, const tf2::Transform & tf_base_link2map,
    std::mt19937 & random_generator,
    pcl::PointCloud<pcl::PointXYZ>::Ptr & merged_pointcloud) const override;

private:
  double visible_range_;
};

class DummyPerceptionPublisherNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr
    detected_object_with_feature_pub_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr
    ground_truth_objects_pub_;
  rclcpp::Subscription<tier4_simulation_msgs::msg::DummyObject>::SharedPtr object_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<tier4_simulation_msgs::msg::DummyObject> objects_;
  std::deque<autoware_perception_msgs::msg::PredictedObjects> predicted_objects_buffer_;
  static constexpr size_t MAX_BUFFER_SIZE = 50;  // Store last 5 seconds at 10Hz
  std::map<std::string, std::string> dummy_to_predicted_uuid_map_;
  std::map<std::string, rclcpp::Time> dummy_mapping_timestamps_;
  std::map<std::string, geometry_msgs::msg::Point> dummy_last_known_positions_;
  std::map<std::string, rclcpp::Time> dummy_creation_timestamps_;
  std::map<std::string, autoware_perception_msgs::msg::PredictedObject> dummy_last_used_predictions_;
  std::map<std::string, rclcpp::Time> dummy_last_used_prediction_times_;
  std::map<std::string, rclcpp::Time> dummy_prediction_update_timestamps_;
  double visible_range_;
  double detection_successful_rate_;
  bool enable_ray_tracing_;
  bool use_object_recognition_;
  bool use_base_link_z_;
  bool publish_ground_truth_objects_;
  std::unique_ptr<PointCloudCreator> pointcloud_creator_;

  double angle_increment_;

  std::mt19937 random_generator_;
  void timerCallback();
  void objectCallback(const tier4_simulation_msgs::msg::DummyObject::ConstSharedPtr msg);
  void predictedObjectsCallback(
    const autoware_perception_msgs::msg::PredictedObjects::ConstSharedPtr msg);
  std::pair<autoware_perception_msgs::msg::PredictedObject, rclcpp::Time>
  findMatchingPredictedObject(
    const unique_identifier_msgs::msg::UUID & object_id, const rclcpp::Time & current_time);
  void updateDummyToPredictedMapping(
    const std::vector<tier4_simulation_msgs::msg::DummyObject> & dummy_objects,
    const autoware_perception_msgs::msg::PredictedObjects & predicted_objects);
  double calculateEuclideanDistance(
    const geometry_msgs::msg::Point & pos1, const geometry_msgs::msg::Point & pos2);
  bool isTrajectoryValid(
    const autoware_perception_msgs::msg::PredictedObject & current_prediction,
    const autoware_perception_msgs::msg::PredictedObject & new_prediction,
    const std::string & dummy_uuid_str);

public:
  DummyPerceptionPublisherNode();
  ~DummyPerceptionPublisherNode() {}
};

}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
