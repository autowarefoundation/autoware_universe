// Copyright 2025 TIER IV, Inc.
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

#include "object_manager.hpp"

#include "autoware/behavior_velocity_intersection_module/util.hpp"

#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>  // for toPolygon2d

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>

#include <memory>
#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
namespace bg = boost::geometry;

ObjectInfo::ObjectInfo(const unique_identifier_msgs::msg::UUID & uuid)
: uuid_str(util::to_string(uuid))
{
}

void ObjectInfo::initialize(
  const autoware_perception_msgs::msg::PredictedObject & object,
  std::optional<lanelet::ConstLanelet> attention_lanelet_opt)
{
  predicted_object_ = object;
  attention_lanelet_opt_ = attention_lanelet_opt;
  unsafe_interval_ = std::nullopt;
}

void ObjectInfo::update_safety(
  const std::optional<CollisionInterval> & unsafe_interval,
  const std::optional<CollisionInterval> & safe_interval)
{
  unsafe_interval_ = unsafe_interval;
  safe_interval_ = safe_interval;
}

std::optional<geometry_msgs::msg::Point> ObjectInfo::estimated_past_position(
  const double past_duration) const
{
  if (!attention_lanelet_opt_) {
    return std::nullopt;
  }
  const auto attention_lanelet = attention_lanelet_opt_.value();
  const auto current_arc_coords = lanelet::utils::getArcCoordinates(
    {attention_lanelet}, predicted_object_.kinematics.initial_pose_with_covariance.pose);
  const auto distance = current_arc_coords.distance;
  const auto past_length =
    current_arc_coords.length -
    predicted_object_.kinematics.initial_twist_with_covariance.twist.linear.x * past_duration;
  const auto past_point = lanelet::geometry::fromArcCoordinates(
    attention_lanelet.centerline2d(), lanelet::ArcCoordinates{past_length, distance});
  geometry_msgs::msg::Point past_position;
  past_position.x = past_point.x();
  past_position.y = past_point.y();
  return std::make_optional(past_position);
}

std::shared_ptr<ObjectInfo> ObjectInfoManager::registerObject(
  const unique_identifier_msgs::msg::UUID & uuid, const bool belong_attention_area)
{
  if (objects_info_.count(uuid) == 0) {
    auto object = std::make_shared<ObjectInfo>(uuid);
    objects_info_[uuid] = object;
  }
  auto object = objects_info_[uuid];
  if (belong_attention_area) {
    attention_area_objects_.push_back(object);
  }
  return object;
}

void ObjectInfoManager::registerExistingObject(
  const unique_identifier_msgs::msg::UUID & uuid, const bool belong_attention_area,
  std::shared_ptr<ObjectInfo> object)
{
  objects_info_[uuid] = object;
  if (belong_attention_area) {
    attention_area_objects_.push_back(object);
  }
}

void ObjectInfoManager::clearObjects()
{
  objects_info_.clear();
  attention_area_objects_.clear();
};

std::vector<std::shared_ptr<ObjectInfo>> ObjectInfoManager::allObjects() const
{
  return attention_area_objects_;
}

std::optional<CollisionInterval> findPassageInterval(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path,
  const autoware_perception_msgs::msg::Shape & shape, const lanelet::BasicPolygon2d & ego_lane_poly,
  const std::optional<lanelet::ConstLanelet> & first_attention_lane_opt)
{
  const auto first_itr = std::adjacent_find(
    predicted_path.path.cbegin(), predicted_path.path.cend(), [&](const auto & a, const auto & b) {
      return bg::intersects(ego_lane_poly, util::createOneStepPolygon(a, b, shape));
    });
  if (first_itr == predicted_path.path.cend()) {
    // even the predicted path end does not collide with the beginning of ego_lane_poly
    return std::nullopt;
  }
  const auto last_itr = std::adjacent_find(
    predicted_path.path.crbegin(), predicted_path.path.crend(),
    [&](const auto & a, const auto & b) {
      return bg::intersects(ego_lane_poly, util::createOneStepPolygon(a, b, shape));
    });
  if (last_itr == predicted_path.path.crend()) {
    // even the predicted path start does not collide with the end of ego_lane_poly
    return std::nullopt;
  }

  const size_t enter_idx = static_cast<size_t>(first_itr - predicted_path.path.begin());
  const double object_enter_time =
    static_cast<double>(enter_idx) * rclcpp::Duration(predicted_path.time_step).seconds();
  const size_t exit_idx = std::distance(predicted_path.path.begin(), last_itr.base()) - 1;
  const double object_exit_time =
    static_cast<double>(exit_idx) * rclcpp::Duration(predicted_path.time_step).seconds();
  const auto lane_position = [&]() {
    if (first_attention_lane_opt) {
      if (lanelet::geometry::inside(
            first_attention_lane_opt.value(),
            lanelet::BasicPoint2d(first_itr->position.x, first_itr->position.y))) {
        return CollisionInterval::LanePosition::FIRST;
      }
    }
    return CollisionInterval::LanePosition::ELSE;
  }();

  std::vector<geometry_msgs::msg::Pose> path;
  for (const auto & pose : predicted_path.path) {
    path.push_back(pose);
  }
  return CollisionInterval{
    lane_position, path, {enter_idx, exit_idx}, {object_enter_time, object_exit_time}};
}

}  // namespace autoware::behavior_velocity_planner
