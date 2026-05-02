// Copyright 2026 TIER IV, Inc.
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

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR_HPP_

#include "traffic_light_map_based_detector_process.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <rclcpp/time.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tier4_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

struct SetRouteError
{
  std::string message;
};

struct StampedTransform
{
  rclcpp::Time stamp;
  tf2::Transform transform;
};

struct DetectionResult
{
  tier4_perception_msgs::msg::TrafficLightRoiArray rough_rois;
  tier4_perception_msgs::msg::TrafficLightRoiArray expect_rois;
  visualization_msgs::msg::MarkerArray markers;
};

struct TrafficLightMapBasedDetectorConfig
{
  double max_vibration_pitch;
  double max_vibration_yaw;
  double max_vibration_height;
  double max_vibration_width;
  double max_vibration_depth;
  double max_detection_range;
  double car_traffic_light_max_angle_range;
  double pedestrian_traffic_light_max_angle_range;
};

class TrafficLightMapBasedDetector
{
public:
  struct IdLessThan
  {
    bool operator()(
      const lanelet::ConstLineString3d & left, const lanelet::ConstLineString3d & right) const
    {
      return left.id() < right.id();
    }
  };

  using TrafficLightSet = std::set<lanelet::ConstLineString3d, IdLessThan>;

  TrafficLightMapBasedDetector(
    const TrafficLightMapBasedDetectorConfig & config,
    const autoware_map_msgs::msg::LaneletMapBin & map_msg);

  std::optional<SetRouteError> setRoute(
    const autoware_planning_msgs::msg::LaneletRoute & route_msg);

  DetectionResult detect(
    const std::vector<StampedTransform> & tf_map2camera_samples,
    const sensor_msgs::msg::CameraInfo & camera_info) const;

private:
  void setMap(const autoware_map_msgs::msg::LaneletMapBin & map_msg);

  /**
   * @brief Filter traffic lights that are visible from the camera
   *
   * @param all_traffic_lights      all the traffic lights in the route or in the map
   * @param tf_map2camera_samples   the stamped transformation samples from map to camera
   * @param pinhole_camera_model    pinhole model calculated from camera_info
   * @param visible_traffic_lights  the visible traffic lights output
   */
  void getVisibleTrafficLights(
    const TrafficLightSet & all_traffic_lights,
    const std::vector<StampedTransform> & tf_map2camera_samples,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    std::vector<lanelet::ConstLineString3d> & visible_traffic_lights) const;

  /**
   * @brief Compute the traffic light ROI from a single transform
   *
   * @param tf_map2camera         the transformation from map to camera
   * @param pinhole_camera_model  pinhole model calculated from camera_info
   * @param traffic_light         lanelet traffic light object
   * @param config                offset configuration
   * @param roi                   computed result
   * @return true                 the computation succeeded
   * @return false                the computation failed
   */
  bool getTrafficLightRoi(
    const tf2::Transform & tf_map2camera,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    const lanelet::ConstLineString3d traffic_light,
    const TrafficLightMapBasedDetectorConfig & config,
    tier4_perception_msgs::msg::TrafficLightRoi & roi) const;

  /**
   * @brief Compute the bounding ROI from multiple transforms
   *
   * @param tf_map2camera_samples the stamped transformation samples from map to camera
   * @param pinhole_camera_model  pinhole model calculated from camera_info
   * @param traffic_light         lanelet traffic light object
   * @param config                offset configuration
   * @param out_roi               computed result
   * @return true                 the computation succeeded
   * @return false                the computation failed
   */
  bool getTrafficLightRoi(
    const std::vector<StampedTransform> & tf_map2camera_samples,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    const lanelet::ConstLineString3d traffic_light,
    const TrafficLightMapBasedDetectorConfig & config,
    tier4_perception_msgs::msg::TrafficLightRoi & out_roi) const;

  /**
   * @brief Create visualization markers for visible traffic lights
   *
   * @param tf_map2camera           the transformation from map to camera
   * @param camera_header           header of the camera_info message
   * @param visible_traffic_lights  the visible traffic light object vector
   * @return visualization marker array
   */
  static visualization_msgs::msg::MarkerArray createTrafficLightMarkers(
    const tf2::Transform & tf_map2camera, const std_msgs::msg::Header & camera_header,
    const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights);

  TrafficLightMapBasedDetectorConfig config_;
  std::shared_ptr<TrafficLightSet> all_traffic_lights_ptr_;
  std::shared_ptr<TrafficLightSet> route_traffic_lights_ptr_;
  std::set<int64_t> pedestrian_traffic_light_id_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  std::shared_ptr<const lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
};

}  // namespace autoware::traffic_light

#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR_HPP_
