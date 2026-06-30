// Copyright 2026 TIER IV, inc.
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

#include "autoware/map_based_prediction/priority_predictor/traffic_signal_stop_predictor.hpp"

#include <autoware/traffic_light_utils/traffic_light_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <rclcpp/logging.hpp>
#include <tf2/utils.hpp>

#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Utilities.h>
#include <lanelet2_routing/LaneletPath.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction::priority_predictor
{
namespace
{

using autoware::experimental::lanelet2_utils::LaneletRTree;

bool isRoadLanelet(const lanelet::ConstLanelet & lanelet)
{
  if (!lanelet.hasAttribute(lanelet::AttributeName::Subtype)) {
    return true;
  }
  const auto subtype = lanelet.attribute(lanelet::AttributeName::Subtype).value();
  return subtype != lanelet::AttributeValueString::Crosswalk &&
         subtype != lanelet::AttributeValueString::Walkway;
}

lanelet::ConstLanelets collectRoadLanelets(const lanelet::LaneletMap & lanelet_map)
{
  lanelet::ConstLanelets road_lanelets;
  for (const auto & lanelet : lanelet_map.laneletLayer) {
    if (isRoadLanelet(lanelet)) {
      road_lanelets.push_back(lanelet);
    }
  }
  return road_lanelets;
}

}  // namespace

std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point> stopLineChord(
  const lanelet::ConstLineString3d & stop_line)
{
  geometry_msgs::msg::Point c1;
  c1.x = stop_line.front().x();
  c1.y = stop_line.front().y();
  c1.z = stop_line.front().z();
  geometry_msgs::msg::Point c2;
  c2.x = stop_line.back().x();
  c2.y = stop_line.back().y();
  c2.z = stop_line.back().z();
  return {c1, c2};
}

void clipPathAtStopLine(PredictedPath & path, const lanelet::ConstLineString3d & stop_line)
{
  if (path.path.size() < 2 || stop_line.size() < 2) {
    return;
  }

  const auto [c1, c2] = stopLineChord(stop_line);
  for (size_t i = 1; i < path.path.size(); ++i) {
    const auto crossing_point =
      autoware_utils::intersect(path.path.at(i - 1).position, path.path.at(i).position, c1, c2);
    if (crossing_point) {
      auto crossing = path.path.at(i);
      crossing.position = *crossing_point;
      path.path.resize(i);
      path.path.push_back(crossing);
      return;
    }
  }
}

std::optional<double> arcLengthToStopLine(
  const PosePath & ref_path, const lanelet::ConstLineString3d & stop_line)
{
  if (ref_path.size() < 2 || stop_line.size() < 2) {
    return std::nullopt;
  }

  const auto [c1, c2] = stopLineChord(stop_line);
  double arc_length = 0.0;
  for (size_t i = 1; i < ref_path.size(); ++i) {
    const auto & a = ref_path.at(i - 1).position;
    const auto & b = ref_path.at(i).position;
    if (const auto crossing = autoware_utils::intersect(a, b, c1, c2)) {
      return std::max(arc_length + std::hypot(crossing->x - a.x, crossing->y - a.y), 0.0);
    }
    arc_length += std::hypot(b.x - a.x, b.y - a.y);
  }

  const double center_x = 0.5 * (c1.x + c2.x);
  const double center_y = 0.5 * (c1.y + c2.y);

  double cum_length = 0.0;
  double nearest_arc_length = 0.0;
  size_t nearest_index = 0;
  double nearest_distance_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ref_path.size(); ++i) {
    if (i > 0) {
      cum_length += autoware_utils::calc_distance2d(ref_path.at(i - 1), ref_path.at(i));
    }
    const double dx = ref_path.at(i).position.x - center_x;
    const double dy = ref_path.at(i).position.y - center_y;
    const double distance_sq = dx * dx + dy * dy;
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_arc_length = cum_length;
      nearest_index = i;
    }
  }
  if (nearest_index + 1 < ref_path.size()) {
    return std::nullopt;
  }
  return std::max(nearest_arc_length, 0.0);
}

bool hasStopLineAhead(
  const geometry_msgs::msg::Point & position, const PosePath & ref_path,
  const lanelet::ConstLineString3d & stop_line)
{
  const auto arc_length = arcLengthToStopLine(ref_path, stop_line);
  if (!arc_length) {
    return false;
  }
  const auto & r0 = ref_path.front();
  const double h0 = tf2::getYaw(r0.orientation);
  const double s_obj =
    (position.x - r0.position.x) * std::cos(h0) + (position.y - r0.position.y) * std::sin(h0);
  return *arc_length > s_obj;
}

bool hasTrafficLight(const lanelet::ConstLanelet & way_lanelet)
{
  return !way_lanelet.regulatoryElementsAs<lanelet::TrafficLight>().empty();
}

std::optional<lanelet::ConstLineString3d> getStopLine(const lanelet::ConstLanelet & way_lanelet)
{
  for (const auto & traffic_light : way_lanelet.regulatoryElementsAs<lanelet::TrafficLight>()) {
    if (const auto stop_line = traffic_light->stopLine()) {
      return *stop_line;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> getStopLineOrEntryEdge(
  const lanelet::ConstLanelet & way_lanelet)
{
  if (const auto stop_line = getStopLine(way_lanelet)) {
    return stop_line;
  }
  const auto & left = way_lanelet.leftBound();
  const auto & right = way_lanelet.rightBound();
  if (left.empty() || right.empty()) {
    return std::nullopt;
  }
  const auto lp = left.front();
  const auto rp = right.front();
  return lanelet::ConstLineString3d(
    lanelet::LineString3d(
      lanelet::utils::getId(),
      {lanelet::Point3d(lanelet::utils::getId(), lp.x(), lp.y(), lp.z()),
       lanelet::Point3d(lanelet::utils::getId(), rp.x(), rp.y(), rp.z())}));
}

std::optional<lanelet::Id> getTrafficSignalId(const lanelet::ConstLanelet & way_lanelet)
{
  const auto traffic_light_reg_elems =
    way_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>();
  if (traffic_light_reg_elems.empty()) {
    return std::nullopt;
  }
  if (traffic_light_reg_elems.size() > 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("map_based_prediction"),
      "[Map Based Prediction]: Multiple regulatory elements as TrafficLight are defined to one "
      "lanelet object.");
  }
  return traffic_light_reg_elems.front()->id();
}

std::optional<TrafficLightGroup> getSignalForLanelet(
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & signal_id_map,
  const lanelet::ConstLanelet & way_lanelet)
{
  const auto signal_id = getTrafficSignalId(way_lanelet);
  if (!signal_id) {
    return std::nullopt;
  }
  const auto it = signal_id_map.find(*signal_id);
  if (it == signal_id_map.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool findTrafficLightLaneletOnPath(
  const lanelet::routing::LaneletPath & lanelet_path, lanelet::ConstLanelet & signal_lanelet)
{
  for (const auto & way_lanelet : lanelet_path) {
    if (hasTrafficLight(way_lanelet)) {
      signal_lanelet = way_lanelet;
      return true;
    }
  }
  return false;
}

lanelet::routing::LaneletPath buildLaneletPathFromPredictedPath(
  const PredictedPath & predicted_path, const LaneletRTree & road_lanelet_rtree,
  const double sample_interval_m)
{
  lanelet::ConstLanelets lanelets;
  if (predicted_path.path.empty()) {
    return lanelet::routing::LaneletPath(lanelets);
  }

  std::optional<lanelet::Id> prev_id;
  const auto add_lanelet_at = [&](const geometry_msgs::msg::Pose & pose) {
    const auto lanelet_opt = road_lanelet_rtree.get_closest_lanelet(pose);
    if (!lanelet_opt) {
      return;
    }
    if (prev_id && *prev_id == lanelet_opt->id()) {
      return;
    }
    prev_id = lanelet_opt->id();
    lanelets.push_back(*lanelet_opt);
  };

  add_lanelet_at(predicted_path.path.front());

  double dist = 0.0;
  double next_sample = sample_interval_m;
  for (size_t i = 1; i < predicted_path.path.size(); ++i) {
    dist +=
      autoware_utils::calc_distance2d(predicted_path.path.at(i - 1), predicted_path.path.at(i));
    if (dist >= next_sample) {
      add_lanelet_at(predicted_path.path.at(i));
      next_sample += sample_interval_m;
    }
  }

  if (predicted_path.path.size() > 1) {
    add_lanelet_at(predicted_path.path.back());
  }

  return lanelet::routing::LaneletPath(lanelets);
}

bool findTrafficLightLaneletOnPredictedPath(
  const PredictedPath & predicted_path, const LaneletRTree & road_lanelet_rtree,
  lanelet::ConstLanelet & signal_lanelet)
{
  constexpr double sample_interval_m = 3.0;
  const auto lanelet_path =
    buildLaneletPathFromPredictedPath(predicted_path, road_lanelet_rtree, sample_interval_m);
  return findTrafficLightLaneletOnPath(lanelet_path, signal_lanelet);
}

bool evaluateSignalStopRequirement(
  const lanelet::ConstLanelet & way_lanelet, const std::optional<TrafficLightGroup> & signal)
{
  if (!signal) {
    return false;
  }
  return autoware::traffic_light_utils::isTrafficSignalStop(way_lanelet, *signal);
}

bool shouldAddStopHypothesis(const bool signal_requires_stop, const bool has_stop_line_ahead)
{
  return signal_requires_stop && has_stop_line_ahead;
}

double weakenConfidenceInLaneChange(const Maneuver & maneuver, const double stop_weight)
{
  constexpr double lane_change_penalty = 0.5;
  return maneuver == Maneuver::LANE_FOLLOW ? stop_weight : stop_weight * lane_change_penalty;
}

namespace
{

std::vector<PredictedPath> addTrafficSignalStopHypotheses(
  const ObjectPrediction & prediction,
  const std::unordered_map<lanelet::Id, TrafficLightGroup> & traffic_signal_id_map,
  const LaneletRTree & road_lanelet_rtree, StopHypothesisDebug & debug)
{
  const TrackedObject & object = prediction.object;
  const std::vector<PredictedPath> & predicted_paths = prediction.predicted_paths;

  if (predicted_paths.empty()) {
    return predicted_paths;
  }

  debug.counter.vehicles++;

  std::vector<PredictedPath> result = predicted_paths;

  for (size_t i = 0; i < predicted_paths.size(); ++i) {
    PredictedPath predicted_path = predicted_paths.at(i);

    if (predicted_path.path.size() < 2) {
      continue;
    }

    lanelet::ConstLanelet target_lanelet_signal_object;
    if (!findTrafficLightLaneletOnPredictedPath(
          predicted_path, road_lanelet_rtree, target_lanelet_signal_object)) {
      continue;
    }
    const std::optional<TrafficLightGroup> signal_status =
      getSignalForLanelet(traffic_signal_id_map, target_lanelet_signal_object);
    const std::optional<lanelet::ConstLineString3d> related_stop_line =
      getStopLineOrEntryEdge(target_lanelet_signal_object);

    const bool signal_requires_stop =
      evaluateSignalStopRequirement(target_lanelet_signal_object, signal_status);
    const bool stop_line_ahead =
      related_stop_line && hasStopLineAhead(
                             object.kinematics.pose_with_covariance.pose.position,
                             predicted_path.path, *related_stop_line);
    debug.counter.signal_stop += signal_requires_stop ? 1 : 0;
    debug.counter.stopline_found += stop_line_ahead ? 1 : 0;

    if (!shouldAddStopHypothesis(signal_requires_stop, stop_line_ahead)) {
      continue;
    }

    clipPathAtStopLine(predicted_path, *related_stop_line);

    if (predicted_path.path.size() < 2) {
      continue;
    }

    result.at(i) = predicted_path;

    debug.stop_hypothesis_path_indices[autoware_utils::to_hex_string(object.object_id)].insert(i);
    debug.stop_lines.push_back(*related_stop_line);
    debug.counter.stop_hypothesis_added++;
  }

  return result;
}

}  // namespace

void TrafficSignalStopPredictor::setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  lanelet_map_ptr_ = std::move(lanelet_map_ptr);
  if (!lanelet_map_ptr_) {
    road_lanelet_rtree_.reset();
    return;
  }
  const auto road_lanelets = collectRoadLanelets(*lanelet_map_ptr_);
  road_lanelet_rtree_.emplace(road_lanelets);
}

void TrafficSignalStopPredictor::setTrafficSignal(
  const TrafficLightGroupArray & traffic_signals, const rclcpp::Time & now)
{
  traffic_signal_id_map_.clear();
  for (const auto & group : traffic_signals.traffic_light_groups) {
    traffic_signal_id_map_[group.traffic_light_group_id] = group;
  }

  stabilized_traffic_signal_id_map_ = stabilizeTrafficSignalMap(
    traffic_signal_id_map_, signal_stabilize_state_, now, params_.stop_time_hysteresis,
    params_.go_time_hysteresis, params_.signal_retention_timeout);
  latest_traffic_signal_time_ = now;
}

void TrafficSignalStopPredictor::clearFrameDebug()
{
  debug_.stop_lines.clear();
  debug_.stop_hypothesis_path_indices.clear();
  debug_.used_signal_colors.clear();
}

std::vector<PredictedPath> TrafficSignalStopPredictor::addStopHypotheses(
  const ObjectPrediction & prediction, const rclcpp::Time & now)
{
  if (!road_lanelet_rtree_) {
    return prediction.predicted_paths;
  }

  const bool signal_observation_stale =
    signal_observation_timeout_ > 0.0 &&
    (!latest_traffic_signal_time_ ||
     (now - *latest_traffic_signal_time_).seconds() > signal_observation_timeout_);
  if (signal_observation_stale) {
    signal_stabilize_state_.clear();
    stabilized_traffic_signal_id_map_.clear();
    debug_.used_signal_colors.clear();
    return prediction.predicted_paths;
  }

  debug::populateUsedSignalColors(stabilized_traffic_signal_id_map_, debug_.used_signal_colors);

  return addTrafficSignalStopHypotheses(
    prediction, stabilized_traffic_signal_id_map_, *road_lanelet_rtree_, debug_);
}

}  // namespace autoware::map_based_prediction::priority_predictor
