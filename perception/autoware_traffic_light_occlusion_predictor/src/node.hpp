// Copyright 2023 Tier IV, Inc.
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

#ifndef NODE_HPP_
#define NODE_HPP_

#include "occlusion_predictor.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <perception_utils/prime_synchronizer.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/traffic_light_array.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cstdint>
#include <map>
#include <memory>

namespace autoware::traffic_light
{
class TrafficLightOcclusionPredictorNode : public rclcpp::Node
{
public:
  explicit TrafficLightOcclusionPredictorNode(const rclcpp::NodeOptions & node_options);

private:
  struct Config
  {
    double azimuth_occlusion_resolution_deg;
    double elevation_occlusion_resolution_deg;

    /// Maximum distance of point cloud points considered valid [m]
    double max_valid_pt_dist;

    /// Maximum allowed timestamp difference between image and point cloud [s]
    double max_image_cloud_delay;

    /// Maximum wait time for synchronized messages [s]
    double max_wait_t;

    /// Occlusion ratio threshold above which a signal is treated as UNKNOWN
    int max_occlusion_ratio;
  };

  /**
   * @brief Callback for receiving the Lanelet2 vector map
   *
   * @details
   * Extracts all traffic light geometries from the map and computes their
   * 3D center positions. These positions are later used as reference points
   * for occlusion estimation.
   *
   * @param input_msg Lanelet2 map message
   */
  void mapCallback(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr input_msg);

  enum class TrafficLightIndex : std::uint8_t { Car = 0, Pedestrian = 1, Max };
  static constexpr size_t NUM_TL_TYPES = static_cast<size_t>(TrafficLightIndex::Max);

  static constexpr size_t to_index(TrafficLightIndex traffic_light_type)
  {
    return static_cast<size_t>(traffic_light_type);
  }

  /**
   * @brief Synchronized callback for traffic light occlusion estimation
   *
   * @details
   * This callback is invoked separately for car and pedestrian traffic lights.
   * It performs the following steps:
   *  - Filters ROIs by traffic light type
   *  - Computes occlusion ratios using point cloud projection
   *  - Overrides occluded traffic light signals to UNKNOWN
   *  - Aggregates results across traffic light types
   *
   * Publishing occurs only after both car and pedestrian traffic lights
   * have been processed.
   *
   * @param in_signal_msg Traffic light signal array
   * @param in_roi_msg Traffic light ROI array
   * @param in_cam_info_msg Camera intrinsic parameters
   * @param in_cloud_msg LiDAR point cloud
   * @param traffic_light_type Traffic light category (car or pedestrian)
   */
  void syncCallback(
    const tier4_perception_msgs::msg::TrafficLightArray::ConstSharedPtr in_signal_msg,
    const tier4_perception_msgs::msg::TrafficLightRoiArray::ConstSharedPtr in_roi_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr in_cam_info_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr in_cloud_msg,
    const TrafficLightIndex traffic_light_type);

  rclcpp::Subscription<autoware_map_msgs::msg::LaneletMapBin>::SharedPtr map_sub_;

  /// Publisher for occlusion-filtered traffic light signals
  rclcpp::Publisher<tier4_perception_msgs::msg::TrafficLightArray>::SharedPtr signal_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /**
   * @brief Map of traffic light IDs to their 3D center positions
   *
   * Used during occlusion estimation to associate ROIs with map-based
   * traffic light geometry.
   */
  std::map<lanelet::Id, tf2::Vector3> traffic_light_position_map_;

  Config config_;

  /**
   * @brief Occlusion predictor using point cloud projection
   */
  std::shared_ptr<CloudOcclusionPredictor> cloud_occlusion_predictor_;

  typedef perception_utils::PrimeSynchronizer<
    tier4_perception_msgs::msg::TrafficLightArray, tier4_perception_msgs::msg::TrafficLightRoiArray,
    sensor_msgs::msg::CameraInfo, sensor_msgs::msg::PointCloud2>
    SynchronizerType;

  std::shared_ptr<SynchronizerType> synchronizer_car_;
  std::shared_ptr<SynchronizerType> synchronizer_ped_;

  /**
   * @brief Flags indicating whether each traffic light type has been processed
   *
   * Used to determine when aggregated publishing is possible.
   */
  std::array<bool, NUM_TL_TYPES> subscribed_;

  /**
   * @brief Aggregated output message containing all traffic light signals
   */
  tier4_perception_msgs::msg::TrafficLightArray out_msg_;
};
}  // namespace autoware::traffic_light
#endif  // NODE_HPP_
