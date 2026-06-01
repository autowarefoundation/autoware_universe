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

#include <boost/geometry.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <algorithm>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
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

bool hasPotentialToReach(
  const TrackedObject & object, const Eigen::Vector2d & center_point,
  const Eigen::Vector2d & right_point, const Eigen::Vector2d & left_point,
  const double time_horizon, const double min_object_vel,
  const double max_crosswalk_user_delta_yaw_threshold_for_lanelet)
{
  const auto & obj_pos = object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = object.kinematics.twist_with_covariance.twist.linear;
  const auto yaw = autoware_utils::get_rpy(object.kinematics.pose_with_covariance.pose).z;

  constexpr double stop_velocity_th = 0.14;  // [m/s]
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto is_stop_object = estimated_velocity < stop_velocity_th;
  const auto velocity = std::max(min_object_vel, estimated_velocity);

  const double pedestrian_to_crosswalk_center_direction =
    std::atan2(center_point.y() - obj_pos.y, center_point.x() - obj_pos.x);

  const auto
    [pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction] =
      [&]() {
        const double pedestrian_to_crosswalk_right_direction =
          std::atan2(right_point.y() - obj_pos.y, right_point.x() - obj_pos.x);
        const double pedestrian_to_crosswalk_left_direction =
          std::atan2(left_point.y() - obj_pos.y, left_point.x() - obj_pos.x);
        return std::make_pair(
          autoware_utils::normalize_radian(
            pedestrian_to_crosswalk_right_direction - pedestrian_to_crosswalk_center_direction),
          autoware_utils::normalize_radian(
            pedestrian_to_crosswalk_left_direction - pedestrian_to_crosswalk_center_direction));
      }();

  const double pedestrian_heading_rel_direction = [&]() {
    const double pedestrian_heading_direction =
      std::atan2(obj_vel.x * std::sin(yaw), obj_vel.x * std::cos(yaw));
    return autoware_utils::normalize_radian(
      pedestrian_heading_direction - pedestrian_to_crosswalk_center_direction);
  }();

  const double pedestrian_to_crosswalk_min_rel_direction = std::min(
    pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction);
  const double pedestrian_to_crosswalk_max_rel_direction = std::max(
    pedestrian_to_crosswalk_right_rel_direction, pedestrian_to_crosswalk_left_rel_direction);
  const double pedestrian_vel_angle_against_crosswalk = [&]() {
    if (pedestrian_heading_rel_direction < pedestrian_to_crosswalk_min_rel_direction) {
      return pedestrian_to_crosswalk_min_rel_direction - pedestrian_heading_rel_direction;
    }
    if (pedestrian_to_crosswalk_max_rel_direction < pedestrian_heading_rel_direction) {
      return pedestrian_to_crosswalk_max_rel_direction - pedestrian_heading_rel_direction;
    }
    return 0.0;
  }();
  const auto heading_for_crosswalk = std::abs(pedestrian_vel_angle_against_crosswalk) <
                                     max_crosswalk_user_delta_yaw_threshold_for_lanelet;
  const auto reachable = std::hypot(center_point.x() - obj_pos.x, center_point.y() - obj_pos.y) <
                         velocity * time_horizon;

  if (reachable && (heading_for_crosswalk || is_stop_object)) {
    return true;
  }

  return false;
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

void PredictorVru::initialize()
{
  current_crosswalk_users_.clear();
  predicted_crosswalk_users_ids_.clear();
}

PredictedObject PredictorVru::predict(
  const std_msgs::msg::Header & header, const TrackedObject & object)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::string object_id = autoware_utils::to_hex_string(object.object_id);
  if (match_lost_and_appeared_crosswalk_users_) {
    object_id = tryMatchNewObjectToDisappeared(object_id, current_crosswalk_users_);
  }
  predicted_crosswalk_users_ids_.insert(object_id);
  updateCrosswalkUserHistory(header, object, object_id);
  return getPredictedObjectAsCrosswalkUser(object);
}

PredictedObjects PredictorVru::retrieveUndetectedObjects()
{
  PredictedObjects output;
  for (const auto & [id, crosswalk_user] : crosswalk_users_history_) {
    // get a predicted path for crosswalk users in history who didn't get path yet using latest
    // message
    if (predicted_crosswalk_users_ids_.count(id) == 0) {
      const auto predicted_object =
        getPredictedObjectAsCrosswalkUser(crosswalk_user.back().tracked_object);
      output.objects.push_back(predicted_object);
    }
  }
  return output;
}

bool PredictorVru::hasPotentialToReachWithHistory(
  const TrackedObject & object, const Eigen::Vector2d & center_point,
  const Eigen::Vector2d & right_point, const Eigen::Vector2d & left_point,
  const double time_horizon, const double min_object_vel,
  const double max_crosswalk_user_delta_yaw_threshold_for_lanelet, const bool is_crossing)
{
  // Current frame's intention estimation based on geometry and velocity
  const auto has_crossing_intention = hasPotentialToReach(
    object, center_point, right_point, left_point, time_horizon, min_object_vel,
    max_crosswalk_user_delta_yaw_threshold_for_lanelet);

  const auto object_id = autoware_utils::to_hex_string(object.object_id);

  // If the object has no history, use the current estimation as-is and initialize its history
  if (crosswalk_users_history_.count(object_id) == 0) {
    return has_crossing_intention;
  }

  auto & last_object_data = crosswalk_users_history_.at(object_id).back();
  const auto now = node_.get_clock()->now();

  // Reset the pedestrian crossing intention estimation
  // when the pedestrian starts or finishes crossing the crosswalk.
  if (last_object_data.is_crossing != is_crossing) {
    last_object_data.intention_history.clear();
  }
  last_object_data.is_crossing = is_crossing;

  // Find historical record corresponding to the same crossing point (center_point)
  const auto itr = std::find_if(
    last_object_data.intention_history.begin(), last_object_data.intention_history.end(),
    [&center_point](const auto & intention) {
      return std::hypot(
               intention.point.x() - center_point.x(), intention.point.y() - center_point.y()) <
             1e-3;
    });

  // If this is the first time observing this crossing point, initialize its intention state
  if (itr == last_object_data.intention_history.end()) {
    last_object_data.intention_history.push_back(
      CrosswalkUser::Intention{rclcpp::Time(0, 0, RCL_ROS_TIME), now, center_point});
    return has_crossing_intention;
  }

  if (has_crossing_intention) {
    // If current estimation is "intends to cross"
    // Consider it a valid crossing intention only if it has been sustained long enough
    if ((now - itr->last_no_crossing_intention_time).seconds() > crossing_intention_duration_) {
      itr->last_crossing_intention_time = now;
      return true;
    } else {
      return false;  // Not enough time has passed since intention appeared.
    }
  } else {
    // If current estimation is "no crossing intention"
    // Confirm the lack of intention only if it has persisted long enough
    if ((now - itr->last_crossing_intention_time).seconds() > no_crossing_intention_duration_) {
      itr->last_no_crossing_intention_time = now;
      return false;
    } else {
      return true;  // Not enough time has passed since intention disappeared.
    }
  }

  return false;  // should not be reached
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
          // flip the object orientation and velocity
          const auto original_yaw =
            tf2::getYaw(object.kinematics.pose_with_covariance.pose.orientation);
          mutable_object.kinematics.pose_with_covariance.pose.orientation =
            autoware_utils::create_quaternion_from_yaw(autoware_utils::pi + original_yaw);
          mutable_object.kinematics.twist_with_covariance.twist.linear.x *= -1.0;
          mutable_object.kinematics.twist_with_covariance.twist.linear.y *= -1.0;
        }
        // if the object is moving forward, use the orientation from the object
        break;
      }
      case autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE: {
        const auto & object_twist = object.kinematics.twist_with_covariance.twist;
        // if the velocity is not too small, calculate the yaw from the velocity
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
        // if the velocity is too small, use the orientation from the object, as is
        break;
      }
      default: {
        // full orientation is available, do nothing
        break;
      }
    }
  }

  auto predicted_object = utils::convertToPredictedObject(mutable_object);
  {
    PredictedPath predicted_path =
      path_generator_->generatePathForNonVehicleObject(mutable_object, prediction_time_horizon_);
    predicted_path.confidence = 1.0;

    predicted_object.kinematics.predicted_paths.push_back(cutPathBeforeFences(predicted_path));
  }

  boost::optional<lanelet::ConstLanelet> crossing_crosswalk{boost::none};
  const auto & obj_pos = mutable_object.kinematics.pose_with_covariance.pose.position;
  const auto & obj_vel = mutable_object.kinematics.twist_with_covariance.twist.linear;
  const auto estimated_velocity = std::hypot(obj_vel.x, obj_vel.y);
  const auto velocity = std::max(min_crosswalk_user_velocity_, estimated_velocity);
  const auto surrounding_lanelets_with_dist = lanelet::geometry::findWithin2d(
    lanelet_map_ptr_->laneletLayer, lanelet::BasicPoint2d{obj_pos.x, obj_pos.y},
    prediction_time_horizon_ * velocity);
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
             max_crosswalk_user_on_road_distance_;
    };

  // If the object is in the crosswalk, generate path to the crosswalk edge
  if (crossing_crosswalk) {
    const auto edge_points = getCrosswalkEdgePoints(crossing_crosswalk.get());

    if (hasPotentialToReachWithHistory(
          mutable_object, edge_points.front_center_point, edge_points.front_right_point,
          edge_points.front_left_point, std::numeric_limits<double>::max(),
          min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_,
          true)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(mutable_object, edge_points.front_center_point);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

    if (hasPotentialToReachWithHistory(
          mutable_object, edge_points.back_center_point, edge_points.back_right_point,
          edge_points.back_left_point, std::numeric_limits<double>::max(),
          min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_,
          true)) {
      PredictedPath predicted_path =
        path_generator_->generatePathToTargetPoint(mutable_object, edge_points.back_center_point);
      predicted_path.confidence = 1.0;
      predicted_object.kinematics.predicted_paths.push_back(predicted_path);
    }

    // If the object is not crossing the crosswalk, in the road lanelets, try to find the closest
    // crosswalk and generate path to the crosswalk edge
  } else if (within_road) {
    const auto & obj_pose = mutable_object.kinematics.pose_with_covariance.pose;
    const auto closest_crosswalk_opt =
      experimental::lanelet2_utils::get_closest_lanelet(crosswalks_, obj_pose);
    if (
      closest_crosswalk_opt &&
      within_minimum_distance(obj_pose.position, closest_crosswalk_opt.value())) {
      const auto edge_points = getCrosswalkEdgePoints(closest_crosswalk_opt.value());
      if (hasPotentialToReachWithHistory(
            mutable_object, edge_points.front_center_point, edge_points.front_right_point,
            edge_points.front_left_point, prediction_time_horizon_ * 2.0,
            min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_,
            true)) {
        PredictedPath predicted_path = path_generator_->generatePathToTargetPoint(
          mutable_object, edge_points.front_center_point);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }

      if (hasPotentialToReachWithHistory(
            mutable_object, edge_points.back_center_point, edge_points.back_right_point,
            edge_points.back_left_point, prediction_time_horizon_ * 2.0,
            min_crosswalk_user_velocity_, max_crosswalk_user_delta_yaw_threshold_for_lanelet_,
            true)) {
        PredictedPath predicted_path =
          path_generator_->generatePathToTargetPoint(mutable_object, edge_points.back_center_point);
        predicted_path.confidence = 1.0;
        predicted_object.kinematics.predicted_paths.push_back(predicted_path);
      }
    }
  }

  // try to find the edge points for other surrounding crosswalks and generate path to the crosswalk
  // edge
  for (const auto & crosswalk : surrounding_crosswalks) {
    if (crossing_crosswalk && crossing_crosswalk.get() == crosswalk) {
      continue;
    }
    const auto crosswalk_signal_id_opt = getTrafficSignalId(crosswalk);
    if (crosswalk_signal_id_opt.has_value() && use_crosswalk_signal_) {
      if (!calcIntentionToCrossWithTrafficSignal(
            mutable_object, crosswalk, crosswalk_signal_id_opt.value())) {
        continue;
      }
    }
    if (within_road && !within_minimum_distance(obj_pos, crosswalk)) {
      continue;
    }

    const auto edge_points = getCrosswalkEdgePoints(crosswalk);

    const auto reachable_first = hasPotentialToReachWithHistory(
      mutable_object, edge_points.front_center_point, edge_points.front_right_point,
      edge_points.front_left_point, prediction_time_horizon_, min_crosswalk_user_velocity_,
      max_crosswalk_user_delta_yaw_threshold_for_lanelet_, false);
    const auto reachable_second = hasPotentialToReachWithHistory(
      mutable_object, edge_points.back_center_point, edge_points.back_right_point,
      edge_points.back_left_point, prediction_time_horizon_, min_crosswalk_user_velocity_,
      max_crosswalk_user_delta_yaw_threshold_for_lanelet_, false);

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
      mutable_object, reachable_crosswalk.value(), prediction_time_horizon_);
    predicted_path.confidence = 1.0;

    if (predicted_path.path.empty()) {
      continue;
    }
    // If the predicted path to the crosswalk is crossing the fence, don't use it
    if (doesPathCrossAnyFenceBeforeCrosswalk(predicted_path)) {
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
