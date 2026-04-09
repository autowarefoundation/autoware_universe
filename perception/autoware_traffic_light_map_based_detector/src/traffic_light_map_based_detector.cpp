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

#include "traffic_light_map_based_detector.hpp"

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <cstdlib>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::traffic_light
{

TrafficLightMapBasedDetector::TrafficLightMapBasedDetector(
  const TrafficLightMapBasedDetectorConfig & config)
: config_(config)
{
  if (config_.max_detection_range <= 0) {
    throw std::invalid_argument("max_detection_range must be positive");
  }
}

void TrafficLightMapBasedDetector::setMap(const autoware_map_msgs::msg::LaneletMapBin & map_msg)
{
  lanelet_map_ptr_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(map_msg));
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  std::vector<lanelet::AutowareTrafficLightConstPtr> all_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  all_traffic_lights_ptr_ = std::make_shared<TrafficLightSet>();
  for (auto traffic_light_iter = all_lanelet_traffic_lights.begin();
       traffic_light_iter != all_lanelet_traffic_lights.end(); ++traffic_light_iter) {
    lanelet::AutowareTrafficLightConstPtr traffic_light_reg_elem = *traffic_light_iter;
    auto lights = traffic_light_reg_elem->trafficLights();
    for (auto light_string_primitive : lights) {
      if (!light_string_primitive.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      all_traffic_lights_ptr_->insert(
        static_cast<lanelet::ConstLineString3d>(light_string_primitive));
    }
  }

  auto crosswalk_lanelets = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  for (const auto & traffic_light_reg_elem :
       lanelet::utils::query::autowareTrafficLights(crosswalk_lanelets)) {
    for (const auto & light_string_primitive : traffic_light_reg_elem->trafficLights()) {
      if (light_string_primitive.isLineString()) {  // traffic lights must be linestrings
        pedestrian_traffic_light_id_.insert(light_string_primitive.id());
      }
    }
  }

  const auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  const auto pedestrian_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Pedestrian);
  lanelet::routing::RoutingGraphConstPtr vehicle_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules);
  lanelet::routing::RoutingGraphConstPtr pedestrian_graph =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *pedestrian_rules);
  lanelet::routing::RoutingGraphContainer overall_graphs({vehicle_graph, pedestrian_graph});
  overall_graphs_ptr_ =
    std::make_shared<const lanelet::routing::RoutingGraphContainer>(overall_graphs);
}

SetRouteResult TrafficLightMapBasedDetector::setRoute(
  const autoware_planning_msgs::msg::LaneletRoute & route_msg)
{
  SetRouteResult result;

  if (lanelet_map_ptr_ == nullptr) {
    result.success = false;
    result.logs.push_back(
      {LogLevel::Warn, "cannot set traffic light in route because don't receive map"});
    return result;
  }

  lanelet::ConstLanelets route_lanelets;
  for (const auto & segment : route_msg.segments) {
    for (const auto & primitive : segment.primitives) {
      try {
        route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(primitive.id));
      } catch (const lanelet::NoSuchPrimitiveError & ex) {
        result.success = false;
        result.logs.push_back({LogLevel::Error, ex.what()});
        return result;
      }
    }
  }

  std::vector<lanelet::AutowareTrafficLightConstPtr> route_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(route_lanelets);
  route_traffic_lights_ptr_ = std::make_shared<TrafficLightSet>();
  for (auto traffic_light_iter = route_lanelet_traffic_lights.begin();
       traffic_light_iter != route_lanelet_traffic_lights.end(); ++traffic_light_iter) {
    lanelet::AutowareTrafficLightConstPtr traffic_light_reg_elem = *traffic_light_iter;
    auto lights = traffic_light_reg_elem->trafficLights();
    for (auto light_string_primitive : lights) {
      if (!light_string_primitive.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      route_traffic_lights_ptr_->insert(
        static_cast<lanelet::ConstLineString3d>(light_string_primitive));
    }
  }

  // crosswalk traffic lights
  lanelet::ConstLanelets conflicting_crosswalks;
  pedestrian_traffic_light_id_.clear();

  for (const auto & route_lanelet : route_lanelets) {
    constexpr int PEDESTRIAN_GRAPH_ID = 1;
    const auto conflict_lls =
      overall_graphs_ptr_->conflictingInGraph(route_lanelet, PEDESTRIAN_GRAPH_ID);
    for (const auto & lanelet : conflict_lls) {
      conflicting_crosswalks.push_back(lanelet);
    }
  }
  std::vector<lanelet::AutowareTrafficLightConstPtr> crosswalk_lanelet_traffic_lights =
    lanelet::utils::query::autowareTrafficLights(conflicting_crosswalks);
  for (auto traffic_light_iter = crosswalk_lanelet_traffic_lights.begin();
       traffic_light_iter != crosswalk_lanelet_traffic_lights.end(); ++traffic_light_iter) {
    lanelet::AutowareTrafficLightConstPtr traffic_light_reg_elem = *traffic_light_iter;
    auto lights = traffic_light_reg_elem->trafficLights();
    for (auto light_string_primitive : lights) {
      if (!light_string_primitive.isLineString()) {  // traffic lights must be linestrings
        continue;
      }
      route_traffic_lights_ptr_->insert(
        static_cast<lanelet::ConstLineString3d>(light_string_primitive));
      pedestrian_traffic_light_id_.insert(light_string_primitive.id());
    }
  }

  return result;
}

bool TrafficLightMapBasedDetector::hasTrafficLights() const
{
  return all_traffic_lights_ptr_ != nullptr || route_traffic_lights_ptr_ != nullptr;
}

DetectionResult TrafficLightMapBasedDetector::detect(
  const std::vector<tf2::Transform> & tf_map2camera_vec, const tf2::Transform & tf_map2camera,
  const sensor_msgs::msg::CameraInfo & camera_info) const
{
  DetectionResult result;
  result.rough_rois.header = camera_info.header;
  result.expect_rois.header = camera_info.header;

  image_geometry::PinholeCameraModel pinhole_camera_model;
  pinhole_camera_model.fromCameraInfo(camera_info);

  // Use route traffic lights if available, otherwise all traffic lights
  std::vector<lanelet::ConstLineString3d> visible_traffic_lights;
  if (route_traffic_lights_ptr_ != nullptr) {
    getVisibleTrafficLights(
      *route_traffic_lights_ptr_, tf_map2camera_vec, pinhole_camera_model, visible_traffic_lights);
  } else if (all_traffic_lights_ptr_ != nullptr) {
    getVisibleTrafficLights(
      *all_traffic_lights_ptr_, tf_map2camera_vec, pinhole_camera_model, visible_traffic_lights);
  } else {
    return result;
  }

  // set all offset to zero when calculating the expect roi
  TrafficLightMapBasedDetectorConfig expect_roi_config = config_;
  expect_roi_config.max_vibration_depth = 0;
  expect_roi_config.max_vibration_height = 0;
  expect_roi_config.max_vibration_width = 0;
  expect_roi_config.max_vibration_yaw = 0;
  expect_roi_config.max_vibration_pitch = 0;

  for (const auto & traffic_light : visible_traffic_lights) {
    tier4_perception_msgs::msg::TrafficLightRoi rough_roi, expect_roi;
    if (!getTrafficLightRoi(
          tf_map2camera, pinhole_camera_model, traffic_light, expect_roi_config, expect_roi)) {
      continue;
    }
    if (!getTrafficLightRoi(
          tf_map2camera_vec, pinhole_camera_model, traffic_light, config_, rough_roi)) {
      continue;
    }
    result.rough_rois.rois.push_back(rough_roi);
    result.expect_rois.rois.push_back(expect_roi);
  }

  result.markers =
    createTrafficLightMarkers(tf_map2camera_vec[0], camera_info.header, visible_traffic_lights);

  return result;
}

void TrafficLightMapBasedDetector::getVisibleTrafficLights(
  const TrafficLightSet & all_traffic_lights, const std::vector<tf2::Transform> & tf_map2camera_vec,
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  std::vector<lanelet::ConstLineString3d> & visible_traffic_lights) const
{
  for (const auto & traffic_light : all_traffic_lights) {
    if (
      traffic_light.hasAttribute("subtype") == false ||
      traffic_light.attribute("subtype").value() == "solid") {
      continue;
    }
    // set different max angle range for ped and car traffic light
    double max_angle_range;
    if (
      pedestrian_traffic_light_id_.find(traffic_light.id()) != pedestrian_traffic_light_id_.end()) {
      max_angle_range = autoware_utils::deg2rad(config_.pedestrian_traffic_light_max_angle_range);
    } else {
      max_angle_range = autoware_utils::deg2rad(config_.car_traffic_light_max_angle_range);
    }

    tf2::Vector3 traffic_light_center = traffic_light_utils::getTrafficLightCenter(traffic_light);
    // for every possible transformation, check if the tl is visible.
    // If under any tf the tl is visible, keep it
    for (const auto & tf_map2camera : tf_map2camera_vec) {
      // check distance range
      if (!utils::isInDistanceRange(
            traffic_light_center, tf_map2camera.getOrigin(), config_.max_detection_range)) {
        continue;
      }

      // check angle range
      // adjust tl_yaw so that perpendicular to ray means 0 angle difference
      double traffic_light_yaw = utils::getTrafficLightYaw(traffic_light);
      traffic_light_yaw =
        autoware_utils::normalize_radian(traffic_light_yaw + M_PI_2);  // adjust to perpendicular
      // get camera yaw
      const double camera_yaw = utils::getCameraYaw(tf_map2camera);
      if (!utils::isInAngleRange(traffic_light_yaw, camera_yaw, max_angle_range)) {
        continue;
      }

      // check within image frame
      tf2::Vector3 top_left_camera_optical =
        tf_map2camera.inverse() * traffic_light_utils::getTrafficLightTopLeft(traffic_light);
      tf2::Vector3 bottom_right_camera_optical =
        tf_map2camera.inverse() * traffic_light_utils::getTrafficLightBottomRight(traffic_light);
      if (
        !utils::isInImageFrame(pinhole_camera_model, top_left_camera_optical) &&
        !utils::isInImageFrame(pinhole_camera_model, bottom_right_camera_optical)) {
        continue;
      }
      visible_traffic_lights.push_back(traffic_light);
      break;
    }
  }
}

bool TrafficLightMapBasedDetector::getTrafficLightRoi(
  const tf2::Transform & tf_map2camera,
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  const lanelet::ConstLineString3d traffic_light, const TrafficLightMapBasedDetectorConfig & config,
  tier4_perception_msgs::msg::TrafficLightRoi & roi) const
{
  roi.traffic_light_id = traffic_light.id();
  if (pedestrian_traffic_light_id_.find(traffic_light.id()) != pedestrian_traffic_light_id_.end()) {
    roi.traffic_light_type = tier4_perception_msgs::msg::TrafficLightRoi::PEDESTRIAN_TRAFFIC_LIGHT;
  } else {
    roi.traffic_light_type = tier4_perception_msgs::msg::TrafficLightRoi::CAR_TRAFFIC_LIGHT;
  }

  // for roi.x_offset and roi.y_offset
  {
    tf2::Vector3 top_left_map = traffic_light_utils::getTrafficLightTopLeft(traffic_light);
    tf2::Vector3 top_left_camera_optical = tf_map2camera.inverse() * top_left_map;
    // max vibration
    tf2::Vector3 margin = utils::getVibrationMargin(
      top_left_camera_optical.z(), config.max_vibration_pitch, config.max_vibration_yaw,
      config.max_vibration_height, config.max_vibration_width, config.max_vibration_depth);
    tf2::Vector3 point3d = top_left_camera_optical - margin;
    if (point3d.z() <= 0.0) {
      return false;
    }
    // enlarged target position in camera coordinate
    {
      cv::Point2d point2d = utils::calcRawImagePointFromPoint3D(pinhole_camera_model, point3d);
      utils::roundInImageFrame(
        pinhole_camera_model.cameraInfo().width, pinhole_camera_model.cameraInfo().height, point2d);
      roi.roi.x_offset = point2d.x;
      roi.roi.y_offset = point2d.y;
    }
  }

  // for roi.width and roi.height
  {
    tf2::Vector3 bottom_right_map = traffic_light_utils::getTrafficLightBottomRight(traffic_light);
    tf2::Vector3 bottom_right_camera_optical = tf_map2camera.inverse() * bottom_right_map;
    // max vibration
    tf2::Vector3 margin = utils::getVibrationMargin(
      bottom_right_camera_optical.z(), config.max_vibration_pitch, config.max_vibration_yaw,
      config.max_vibration_height, config.max_vibration_width, config.max_vibration_depth);
    tf2::Vector3 point3d = bottom_right_camera_optical + margin;
    if (point3d.z() <= 0.0) {
      return false;
    }
    // enlarged target position in camera coordinate
    {
      cv::Point2d point2d = utils::calcRawImagePointFromPoint3D(pinhole_camera_model, point3d);
      utils::roundInImageFrame(
        pinhole_camera_model.cameraInfo().width, pinhole_camera_model.cameraInfo().height, point2d);
      roi.roi.width = point2d.x - roi.roi.x_offset;
      roi.roi.height = point2d.y - roi.roi.y_offset;
    }

    if (roi.roi.width < 1 || roi.roi.height < 1) {
      return false;
    }
  }
  return true;
}

bool TrafficLightMapBasedDetector::getTrafficLightRoi(
  const std::vector<tf2::Transform> & tf_map2camera_vec,
  const image_geometry::PinholeCameraModel & pinhole_camera_model,
  const lanelet::ConstLineString3d traffic_light, const TrafficLightMapBasedDetectorConfig & config,
  tier4_perception_msgs::msg::TrafficLightRoi & out_roi) const
{
  std::vector<tier4_perception_msgs::msg::TrafficLightRoi> rois;
  for (const auto & tf_map2camera : tf_map2camera_vec) {
    tier4_perception_msgs::msg::TrafficLightRoi roi;
    if (getTrafficLightRoi(tf_map2camera, pinhole_camera_model, traffic_light, config, roi)) {
      rois.push_back(roi);
    }
  }
  if (rois.empty()) {
    return false;
  }
  out_roi = rois.front();
  utils::computeBoundingRoi(
    pinhole_camera_model.cameraInfo().width, pinhole_camera_model.cameraInfo().height, rois,
    out_roi);
  return true;
}

visualization_msgs::msg::MarkerArray TrafficLightMapBasedDetector::createTrafficLightMarkers(
  const tf2::Transform & tf_map2camera, const std_msgs::msg::Header & camera_header,
  const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights)
{
  visualization_msgs::msg::MarkerArray output_msg;
  for (const auto & traffic_light : visible_traffic_lights) {
    const int id = traffic_light.id();
    tf2::Vector3 traffic_light_central_point =
      traffic_light_utils::getTrafficLightCenter(traffic_light);
    tf2::Vector3 camera2traffic_light = tf_map2camera.inverse() * traffic_light_central_point;

    visualization_msgs::msg::Marker marker;
    marker.header = camera_header;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.ns = std::string("beam");
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::msg::Marker::MODIFY;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::msg::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    marker.points.push_back(point);
    point.x = camera2traffic_light.x();
    point.y = camera2traffic_light.y();
    point.z = camera2traffic_light.z();
    marker.points.push_back(point);

    builtin_interfaces::msg::Duration lifetime;
    lifetime.sec = 0;
    lifetime.nanosec = 200000000;  // 0.2 seconds
    marker.lifetime = lifetime;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output_msg.markers.push_back(marker);
  }
  return output_msg;
}

}  // namespace autoware::traffic_light
