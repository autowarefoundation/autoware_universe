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

#include <optional>
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
#include <set>
#include <string>
#include <utility>
#include <vector>
namespace autoware::dummy_perception_publisher
{
using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseWithCovariance;
using geometry_msgs::msg::TwistWithCovariance;
using tier4_simulation_msgs::msg::DummyObject;
struct CommonParameters
{
  double max_remapping_distance;
  double max_speed_difference_ratio;
  double min_speed_ratio;
  double max_speed_ratio;
  double speed_check_threshold;
  std::string path_selection_strategy;  // "highest_confidence" or "random"
};

struct PredictedDummyObjectInfo
{
  std::string predicted_uuid;
};

struct ObjectInfo
{
  ObjectInfo(const DummyObject & object, const rclcpp::Time & current_time);
  ObjectInfo(
    const DummyObject & object, const PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time,
    const double switch_time_threshold);

  // Position calculation methods
  static Pose calculateStraightLinePosition(
    const DummyObject & object, const rclcpp::Time & current_time);
  static Pose calculateTrajectoryBasedPosition(
    const DummyObject & object, const PredictedObject & predicted_object,
    const rclcpp::Time & predicted_time, const rclcpp::Time & current_time);
  static void stopAtZeroVelocity(double & current_vel, double initial_vel, double initial_acc);
  double length;
  double width;
  double height;
  double std_dev_x;
  double std_dev_y;
  double std_dev_z;
  double std_dev_yaw;
  tf2::Transform tf_map2moved_object;
  // pose and twist
  TwistWithCovariance twist_covariance_;
  PoseWithCovariance pose_covariance_;
  // convert to TrackedObject
  // (todo) currently need object input to get id and header information, but it should be removed
  [[nodiscard]] autoware_perception_msgs::msg::TrackedObject toTrackedObject(
    const DummyObject & object) const;
};

class PointCloudCreator
{
public:
  virtual ~PointCloudCreator() = default;

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
  rclcpp::Subscription<DummyObject>::SharedPtr object_sub_;
  rclcpp::Subscription<PredictedObjects>::SharedPtr predicted_objects_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::vector<DummyObject> objects_;
  std::deque<PredictedObjects> predicted_objects_buffer_;
  static constexpr size_t MAX_BUFFER_SIZE = 50;  // Store last 5 seconds at 10Hz
  std::map<std::string, PredictedDummyObjectInfo> dummy_predicted_info_map_;
  std::map<std::string, rclcpp::Time> dummy_mapping_timestamps_;
  std::map<std::string, geometry_msgs::msg::Point> dummy_last_known_positions_;
  std::map<std::string, rclcpp::Time> dummy_creation_timestamps_;
  std::map<std::string, PredictedObject> dummy_last_used_predictions_;
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
  std::uniform_real_distribution<double> path_selection_dist_;

  // Configuration parameters
  double predicted_path_delay_;
  double min_keep_duration_;
  double switch_time_threshold_;

  // Vehicle parameters
  CommonParameters pedestrian_params_;
  CommonParameters vehicle_params_;
  void timerCallback();
  void objectCallback(const DummyObject::ConstSharedPtr msg);
  void predictedObjectsCallback(const PredictedObjects::ConstSharedPtr msg);
  std::pair<PredictedObject, rclcpp::Time> findMatchingPredictedObject(
    const unique_identifier_msgs::msg::UUID & object_id, const rclcpp::Time & current_time);
  void updateDummyToPredictedMapping(
    const std::vector<DummyObject> & dummy_objects, const PredictedObjects & predicted_objects);
  double calculateEuclideanDistance(
    const geometry_msgs::msg::Point & pos1, const geometry_msgs::msg::Point & pos2);
  bool isValidRemappingCandidate(
    const PredictedObject & candidate_prediction, const std::string & dummy_uuid_str,
    const geometry_msgs::msg::Point & expected_position);
  std::optional<geometry_msgs::msg::Point> calculateExpectedPosition(
    const autoware_perception_msgs::msg::PredictedPath & last_prediction,
    const std::string & dummy_uuid_str);

  // Helper methods for updateDummyToPredictedMapping
  static std::set<std::string> collectAvailablePredictedUUIDs(
    const PredictedObjects & predicted_objects,
    std::map<std::string, geometry_msgs::msg::Point> & predicted_positions);
  std::vector<std::string> findDisappearedPredictedObjectUUIDs(
    std::set<std::string> & available_predicted_uuids);
  std::map<std::string, geometry_msgs::msg::Point> collectDummyObjectPositions(
    const std::vector<DummyObject> & dummy_objects, const rclcpp::Time & current_time,
    std::vector<std::string> & unmapped_dummy_uuids);
  std::optional<std::string> findBestPredictedObjectMatch(
    const std::string & dummy_uuid, const geometry_msgs::msg::Point & dummy_position,
    const std::set<std::string> & available_predicted_uuids,
    const std::map<std::string, geometry_msgs::msg::Point> & predicted_positions,
    const PredictedObjects & predicted_objects);
  void createRemappingsForDisappearedObjects(
    const std::vector<std::string> & dummy_objects_to_remap,
    std::set<std::string> & available_predicted_uuids,
    const std::map<std::string, geometry_msgs::msg::Point> & predicted_positions,
    const std::map<std::string, geometry_msgs::msg::Point> & dummy_positions,
    const PredictedObjects & predicted_objects);

public:
  DummyPerceptionPublisherNode();
};

}  // namespace autoware::dummy_perception_publisher

#endif  // AUTOWARE__DUMMY_PERCEPTION_PUBLISHER__NODE_HPP_
