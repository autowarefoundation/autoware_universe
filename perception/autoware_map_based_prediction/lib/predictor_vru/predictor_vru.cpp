// Copyright 2024 TIER IV, inc.
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

#include "autoware/map_based_prediction/predictor_vru/predictor_vru.hpp"

#include "autoware/map_based_prediction/utils.hpp"

#include <autoware/lanelet2_utils/nn_search.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/uuid_helper.hpp>
#include <autoware_utils/system/time_keeper.hpp>
#include <tf2/utils.hpp>

#include <boost/geometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace autoware::map_based_prediction
{
using autoware_utils::ScopedTimeTrack;

namespace
{
std::optional<CrosswalkEdgePoints> isReachableCrosswalkEdgePoints(
  const TrackedObject & object, const Eigen::Vector2d & p1, const Eigen::Vector2d & p2,
  const lanelet::ConstLanelets & surrounding_lanelets,
  const lanelet::ConstLanelets & surrounding_crosswalks)
{
  using Point = boost::geometry::model::d2::point_xy<double>;

  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;

  CrosswalkEdgePoints ret{p1, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(),
                          p2, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero()};
  auto distance_pedestrian_to_p1 = std::hypot(p1.x() - obj_pos.x, p1.y() - obj_pos.y);
  auto distance_pedestrian_to_p2 = std::hypot(p2.x() - obj_pos.x, p2.y() - obj_pos.y);

  if (distance_pedestrian_to_p2 < distance_pedestrian_to_p1) {
    ret.swap();
    std::swap(distance_pedestrian_to_p1, distance_pedestrian_to_p2);
  }

  const auto isAcrossAnyRoad = [&surrounding_lanelets, &surrounding_crosswalks](
                                 const Point & p_src, const Point & p_dst) {
    const auto withinAnyCrosswalk = [&surrounding_crosswalks](const Point & p) {
      for (const auto & crosswalk : surrounding_crosswalks) {
        if (boost::geometry::within(p, crosswalk.polygon2d().basicPolygon())) {
          return true;
        }
      }
      return false;
    };

    const auto isExist = [](const Point & p, const std::vector<Point> & points) {
      for (const auto & existingPoint : points) {
        if (boost::geometry::distance(p, existingPoint) < 1e-1) {
          return true;
        }
      }
      return false;
    };

    std::vector<Point> points_of_intersect;
    const boost::geometry::model::linestring<Point> line{p_src, p_dst};
    for (const auto & lanelet : surrounding_lanelets) {
      const lanelet::Attribute attr = lanelet.attribute(lanelet::AttributeName::Subtype);
      if (attr.value() != lanelet::AttributeValueString::Road) {
        continue;
      }

      std::vector<Point> tmp_intersects;
      boost::geometry::intersection(line, lanelet.polygon2d().basicPolygon(), tmp_intersects);
      for (const auto & p : tmp_intersects) {
        if (isExist(p, points_of_intersect) || withinAnyCrosswalk(p)) {
          continue;
        }
        points_of_intersect.push_back(p);
        if (points_of_intersect.size() >= 2) {
          return true;
        }
      }
    }
    return false;
  };

  const bool first_intersects_road = isAcrossAnyRoad(
    {obj_pos.x, obj_pos.y}, {ret.front_center_point.x(), ret.front_center_point.y()});
  const bool second_intersects_road =
    isAcrossAnyRoad({obj_pos.x, obj_pos.y}, {ret.back_center_point.x(), ret.back_center_point.y()});

  if (first_intersects_road && second_intersects_road) {
    return {};
  }

  if (first_intersects_road && !second_intersects_road) {
    ret.swap();
  }

  return ret;
}

CrosswalkEdgePoints getCrosswalkEdgePoints(const lanelet::ConstLanelet & crosswalk)
{
  const Eigen::Vector2d r_p_front = crosswalk.rightBound().front().basicPoint2d();
  const Eigen::Vector2d l_p_front = crosswalk.leftBound().front().basicPoint2d();
  const Eigen::Vector2d front_center_point = (r_p_front + l_p_front) / 2.0;

  const Eigen::Vector2d r_p_back = crosswalk.rightBound().back().basicPoint2d();
  const Eigen::Vector2d l_p_back = crosswalk.leftBound().back().basicPoint2d();
  const Eigen::Vector2d back_center_point = (r_p_back + l_p_back) / 2.0;

  return CrosswalkEdgePoints{front_center_point, r_p_front, l_p_front,
                             back_center_point,  r_p_back,  l_p_back};
}
}  // namespace

void PredictorVru::setLaneletMap(std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  lanelet_map_ptr_ = lanelet_map_ptr;

  const auto all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  const auto crosswalks = lanelet::utils::query::crosswalkLanelets(all_lanelets);
  const auto walkways = lanelet::utils::query::walkwayLanelets(all_lanelets);
  crosswalks_.clear();
  crosswalks_.insert(crosswalks_.end(), crosswalks.begin(), crosswalks.end());
  crosswalks_.insert(crosswalks_.end(), walkways.begin(), walkways.end());

  fence_module_.buildFromMap(lanelet_map_ptr_);
}

void PredictorVru::loadCurrentCrosswalkUsers(const TrackedObjects & objects)
{
  if (!lanelet_map_ptr_) {
    return;
  }
  traffic_signal_module_.removeDisappearedObjects(objects);
  history_manager_.initialize();
  history_manager_.loadCurrentUsers(objects, lanelet_map_ptr_);
}

void PredictorVru::removeOldKnownMatches(const double current_time, const double buffer_time)
{
  history_manager_.removeOldHistory(current_time, buffer_time);
}

PredictedObject PredictorVru::predict(
  const std_msgs::msg::Header & header, const TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::string object_id = autoware_utils::to_hex_string(object.object_id);
  object_id = history_manager_.tryMatchToDisappeared(object_id);
  history_manager_.predictedIds().insert(object_id);
  history_manager_.updateHistory(header, object, object_id);
  return getPredictedObjectAsCrosswalkUser(object);
}

PredictedObjects PredictorVru::retrieveUndetectedObjects()
{
  PredictedObjects output;
  for (const auto & [id, crosswalk_user] : history_manager_.history()) {
    if (history_manager_.predictedIds().count(id) == 0) {
      const auto predicted_object =
        getPredictedObjectAsCrosswalkUser(crosswalk_user.back().tracked_object);
      output.objects.push_back(predicted_object);
    }
  }
  return output;
}

PredictedObject PredictorVru::getPredictedObjectAsCrosswalkUser(const TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Create a mutable copy of the object
  TrackedObject mutable_object = object;

  // flip the object if the object has negative velocity
  {
    switch (object.kinematics.orientation_availability) {
      case autoware_perception_msgs::msg::TrackedObjectKinematics::SIGN_UNKNOWN: {
        const double & vx = object.kinematics.twist_with_covariance.twist.linear.x;
        if (vx < 0) {
          const auto original_yaw =
            tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
          mutable_object.kinematics.pose_with_covariance.pose.orientation =
            autoware_utils::create_quaternion_from_yaw(autoware_utils::pi + original_yaw);
          mutable_object.kinematics.twist_with_covariance.twist.linear.x *= -1.0;
          mutable_object.kinematics.twist_with_covariance.twist.linear.y *= -1.0;
        }
        break;
      }
      case autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE: {
        const auto & object_twist = object.kinematics.twist_with_covariance.twist;
        constexpr double VELOCITY_THRESHOLD = 1e-2;  // 0.01 m/s
        const double object_vel = std::hypot(object_twist.linear.x, object_twist.linear.y);
        if (object_vel > VELOCITY_THRESHOLD) {
          const auto object_vel_yaw = std::atan2(object_twist.linear.y, object_twist.linear.x);
          const auto object_orientation =
            tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
          mutable_object.kinematics.pose_with_covariance.pose.orientation =
            autoware_utils::create_quaternion_from_yaw(object_vel_yaw + object_orientation);
          mutable_object.kinematics.twist_with_covariance.twist.linear.x = object_vel;
          mutable_object.kinematics.twist_with_covariance.twist.linear.y = 0.0;
        }
        break;
      }
      default: {
        break;
      }
    }
  }

  auto predicted_object = utils::convertToPredictedObject(mutable_object);
  {
    PredictedPath predicted_path = path_generator_->generatePathForNonVehicleObject(
      mutable_object, params_.prediction_time_horizon);
    predicted_path.confidence = 1.0;

    predicted_object.kinematics.predicted_paths.push_back(
      fence_module_.cutPathBeforeFences(predicted_path));
  }

  boost::optional<lanelet::ConstLanelet> crossing_crosswalk{boost::none};
  const auto & obj_pos = mutable_object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = mutable_object.kinematics.twist_with_covariance.twist.linear;
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto velocity = std::max(params_.min_crosswalk_user_velocity, estimated_velocity);
  const auto surrounding_lanelets_with_dist = lanelet::geometry::findWithin2d(
    lanelet_map_ptr_->laneletLayer, lanelet::BasicPoint2d{obj_pos.x, obj_pos.y},
    params_.prediction_time_horizon * velocity);
  lanelet::ConstLanelets surrounding_lanelets;
  lanelet::ConstLanelets surrounding_crosswalks;
  for (const auto & [dist, lanelet] : surrounding_lanelets_with_dist) {
    surrounding_lanelets.push_back(lanelet);
    const auto attr = lanelet.attribute(lanelet::AttributeName::Subtype);
    if (
      attr.value() == lanelet::AttributeValueString::Crosswalk ||
      attr.value() == lanelet::AttributeValueString::Walkway) {
      const auto & crosswalk = lanelet;
      surrounding_crosswalks.push_back(crosswalk);
      if (std::abs(dist) < 1e-5) {
        crossing_crosswalk = crosswalk;
      }
    }
  }

  const auto within_road = utils::withinRoadLanelet(mutable_object, surrounding_lanelets_with_dist);
  const auto within_minimum_distance =
    [&](const geometry_msgs::msg::Point & object, const lanelet::ConstLanelet & ll) {
      return utils::lateral_distance_to_lanelet_bounds(ll, object) <=
             params_.max_crosswalk_user_on_road_distance;
    };

  if (crossing_crosswalk) {
    const auto edge_points = getCrosswalkEdgePoints(crossing_crosswalk.get());

    if (history_manager_.hasPotentialToReachWithHistory(
          mutable_object, edge_points.front_center_point, edge_points.front_right_point,
          edge_points.front_left_point, std::numeric_limits<double>::max(),
          params_.min_crosswalk_user_velocity,
          params_.max_crosswalk_user_delta_yaw_threshold_for_lanelet, true)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(mutable_object, edge_points.front_center_point);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

    if (history_manager_.hasPotentialToReachWithHistory(
          mutable_object, edge_points.back_center_point, edge_points.back_right_point,
          edge_points.back_left_point, std::numeric_limits<double>::max(),
          params_.min_crosswalk_user_velocity,
          params_.max_crosswalk_user_delta_yaw_threshold_for_lanelet, true)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(mutable_object, edge_points.back_center_point);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

  } else if (within_road) {
    const auto & obj_pose = mutable_object.kinematics.pose_with_covariance.pose;
    const auto closest_crosswalk_opt =
      experimental::lanelet2_utils::get_closest_lanelet(crosswalks_, obj_pose);
    if (
      closest_crosswalk_opt &&
      within_minimum_distance(obj_pose.position, closest_crosswalk_opt.value())) {
      const auto edge_points = getCrosswalkEdgePoints(closest_crosswalk_opt.value());
      if (history_manager_.hasPotentialToReachWithHistory(
            mutable_object, edge_points.front_center_point, edge_points.front_right_point,
            edge_points.front_left_point, params_.prediction_time_horizon * 2.0,
            params_.min_crosswalk_user_velocity,
            params_.max_crosswalk_user_delta_yaw_threshold_for_lanelet, true)) {
        PredictedPath predicted_path = path_generator_->generatePathToTargetPoint(
          mutable_object, edge_points.front_center_point);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }

      if (history_manager_.hasPotentialToReachWithHistory(
            mutable_object, edge_points.back_center_point, edge_points.back_right_point,
            edge_points.back_left_point, params_.prediction_time_horizon * 2.0,
            params_.min_crosswalk_user_velocity,
            params_.max_crosswalk_user_delta_yaw_threshold_for_lanelet, true)) {
        PredictedPath predicted_path =
          path_generator_->generatePathToTargetPoint(mutable_object, edge_points.back_center_point);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }
    }
  }

  for (const auto & crosswalk : surrounding_crosswalks) {
    if (crossing_crosswalk && crossing_crosswalk.get() == crosswalk) {
      continue;
    }
    const auto crosswalk_signal_id_opt = traffic_signal_module_.getSignalId(crosswalk);
    if (crosswalk_signal_id_opt.has_value() && params_.use_crosswalk_signal) {
      if (!traffic_signal_module_.calcIntentionToCross(
            mutable_object, crosswalk, crosswalk_signal_id_opt.value())) {
        continue;
      }
    }
    if (within_road && !within_minimum_distance(obj_pos, crosswalk)) {
      continue;
    }

    const auto edge_points = getCrosswalkEdgePoints(crosswalk);

    const auto reachable_first = history_manager_.hasPotentialToReachWithHistory(
      mutable_object, edge_points.front_center_point, edge_points.front_right_point,
      edge_points.front_left_point, params_.prediction_time_horizon,
      params_.min_crosswalk_user_velocity,
      params_.max_crosswalk_user_delta_yaw_threshold_for_lanelet, false);
    const auto reachable_second = history_manager_.hasPotentialToReachWithHistory(
      mutable_object, edge_points.back_center_point, edge_points.back_right_point,
      edge_points.back_left_point, params_.prediction_time_horizon,
      params_.min_crosswalk_user_velocity,
      params_.max_crosswalk_user_delta_yaw_threshold_for_lanelet, false);

    if (!reachable_first && !reachable_second) {
      continue;
    }

    const auto reachable_crosswalk = isReachableCrosswalkEdgePoints(
      mutable_object, edge_points.front_center_point, edge_points.back_center_point,
      surrounding_lanelets, surrounding_crosswalks);

    if (!reachable_crosswalk.has_value()) {
      continue;
    }

    auto predicted_path = path_generator_->generatePathForCrosswalkUser(
      mutable_object, reachable_crosswalk.value(), params_.prediction_time_horizon);
    predicted_path.confidence = 1.0;

    if (predicted_path.path.empty()) {
      continue;
    }
    if (fence_module_.doesPathCrossAnyFenceBeforeCrosswalk(predicted_path)) {
      continue;
    }
    predicted_object.kinematics.predicted_paths.push_back(predicted_path);
  }

  const auto n_path = predicted_object.kinematics.predicted_paths.size();
  for (auto & predicted_path : predicted_object.kinematics.predicted_paths) {
    predicted_path.confidence = 1.0 / n_path;
  }

  return predicted_object;
}

}  // namespace autoware::map_based_prediction
