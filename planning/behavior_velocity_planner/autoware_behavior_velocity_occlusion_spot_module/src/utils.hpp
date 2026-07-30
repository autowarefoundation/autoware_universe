// Copyright 2021 Tier IV, Inc.
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

#ifndef UTILS_HPP_
#define UTILS_HPP_

#include "grid_utils.hpp"

#include <autoware/trajectory/path_point_with_lane_id.hpp>

#include <string>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using autoware_perception_msgs::msg::PredictedObject;
using autoware_planning_msgs::msg::PathPoint;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using Trajectory = autoware::experimental::trajectory::Trajectory<PathPointWithLaneId>;

namespace utils
{
enum DETECTION_METHOD { OCCUPANCY_GRID, PREDICTED_OBJECT };
enum PASS_JUDGE { SMOOTH_VELOCITY, CURRENT_VELOCITY };

struct DetectionArea
{
  double min_longitudinal_offset;  // [m] detection area safety buffer from front bumper
  double max_lateral_distance;     // [m] distance to care about occlusion spot
  double slice_length;             // [m] size of each slice
  double min_occlusion_spot_size;  // [m] minumum size to care about the occlusion spot
};
struct Velocity
{
  double safety_ratio;          // [-] safety margin for planning error
  double max_stop_jerk;         // [m/s^3] emergency braking system jerk
  double max_stop_accel;        // [m/s^2] emergency braking system deceleration
  double max_slow_down_jerk;    // [m/s^3] maximum allowed slowdown jerk
  double max_slow_down_accel;   // [m/s^2] maximum allowed deceleration
  double non_effective_jerk;    // [m/s^3] too weak jerk for velocity planning.
  double non_effective_accel;   // [m/s^2] too weak deceleration for velocity planning.
  double min_allowed_velocity;  // [m/s]   minimum allowed velocity not to stop
  double a_ego;                 // [m/s^2] current ego acceleration
  double v_ego;                 // [m/s]   current ego velocity
  double delay_time;            // [s] safety time buffer for delay response
  double safe_margin;           // [m] maximum safety distance for any error
};

struct PlannerParam
{
  DETECTION_METHOD detection_method;
  PASS_JUDGE pass_judge;
  bool is_show_occlusion;           // [-]
  bool is_show_cv_window;           // [-]
  bool is_show_processing_time;     // [-]
  bool use_object_info;             // [-]
  bool use_moving_object_ray_cast;  // [-]
  bool use_partition_lanelet;       // [-]
  // parameters in yaml
  double detection_area_offset;      // [m]
  double detection_area_length;      // [m]
  double detection_area_max_length;  // [m]
  double stuck_vehicle_vel;          // [m/s]
  double lateral_distance_thr;       // [m] lateral distance threshold to consider
  double pedestrian_vel;             // [m/s]
  double pedestrian_radius;          // [m]

  double dist_thr;   // [m]
  double angle_thr;  // [rad]

  // vehicle info
  double baselink_to_front;  // [m]  wheel_base + front_overhang
  double wheel_tread;        // [m]  wheel_tread from vehicle info
  double right_overhang;     // [m]  right_overhang from vehicle info
  double left_overhang;      // [m]  left_overhang from vehicle info

  Velocity v;
  DetectionArea detection_area;
  grid_utils::GridParam grid;
};

struct SafeMotion
{
  double stop_dist;
  double safe_velocity;
};

struct ObstacleInfo
{
  SafeMotion safe_motion;  // safe motion of velocity and stop point
  Point position;
  double max_velocity;  // [m/s] Maximum velocity of the possible obstacle
  double ttv;           // [s] time to vehicle for pedestrian
};

/**
 * @brief representation of a possible collision between ego and some obstacle
 *                                      ^
 *                                      |
 * Ego ---------collision----------intersection-------> path
 *                                      |
 *             ------------------       |
 *            |     Vehicle      |   obstacle
 *             ------------------
 */
struct PossibleCollisionInfo
{
  ObstacleInfo obstacle_info;                          // For hidden obstacle
  PathPoint collision_with_margin;                     // For baselink at collision point
  Pose collision_pose;                                 // only use this for debugging
  Pose intersection_pose;                              // For egp path and hidden obstacle
  lanelet::ArcCoordinates arc_lane_dist_at_collision;  // For ego distance to obstacle in s-d
  PossibleCollisionInfo()
  : obstacle_info(),
    collision_with_margin(),
    collision_pose(),
    intersection_pose(),
    arc_lane_dist_at_collision()
  {
  }
  PossibleCollisionInfo(
    const ObstacleInfo & obstacle_info, const PathPoint & collision_with_margin,
    const Pose & intersection_pose, const lanelet::ArcCoordinates & arc_lane_dist_to_occlusion)
  : obstacle_info(obstacle_info),
    collision_with_margin(collision_with_margin),
    intersection_pose(intersection_pose),
    arc_lane_dist_at_collision(arc_lane_dist_to_occlusion)
  {
  }
};

struct DebugData
{
  double z;
  double baselink_to_front;
  std::string road_type = "";
  std::string detection_type = "";
  Polygons2d detection_area_polygons;
  std::vector<lanelet::BasicPolygon2d> close_partition;
  std::vector<Point> parked_vehicle_point;
  std::vector<PossibleCollisionInfo> possible_collisions;
  std::vector<Point> occlusion_points;
  std::vector<Pose> debug_poses;
  void resetData()
  {
    debug_poses.clear();
    close_partition.clear();
    detection_area_polygons.clear();
    parked_vehicle_point.clear();
    possible_collisions.clear();
    occlusion_points.clear();
  }
};

/**
 * @param: v: ego velocity config
 * @param: ttv: time to vehicle
 * @return safe motion
 **/
SafeMotion calculateSafeMotion(const Velocity & v, const double ttv);
/**
 * @brief apply current velocity to path
 */
void applyVelocityToPath(Trajectory & path, const double velocity);
/**
 * @brief wrapper for detection area polygon generation
 */
bool buildDetectionAreaPolygons(
  Polygons2d & polygons, const Trajectory & path, const double s_ego, const PlannerParam & param);
void applySafeVelocityConsideringPossibleCollision(
  Trajectory & path, std::vector<PossibleCollisionInfo> & possible_collisions,
  std::vector<geometry_msgs::msg::Pose> & debug_poses, const PlannerParam & param);
void handleCollisionOffset(
  std::vector<PossibleCollisionInfo> & possible_collisions, const double offset);
bool isStuckVehicle(const PredictedObject & obj, const double min_vel);
bool isMovingVehicle(const PredictedObject & obj, const double min_vel);
std::vector<PredictedObject> extractVehicles(
  const PredictedObjects::ConstSharedPtr objects_ptr, const Point & ego_position,
  const double distance);
std::vector<PredictedObject> filterVehiclesByDetectionArea(
  const std::vector<PredictedObject> & objs, const Polygons2d & polys);
bool isVehicle(const PredictedObject & obj);
void categorizeVehicles(
  const std::vector<PredictedObject> & vehicles, Polygons2d & stuck_vehicle_foot_prints,
  Polygons2d & moving_vehicle_foot_prints, const double stuck_vehicle_vel);
bool generatePossibleCollisionsFromObjects(
  std::vector<PossibleCollisionInfo> & possible_collisions, const Trajectory & path,
  const PlannerParam & param, const double offset_from_start_to_ego,
  const std::vector<PredictedObject> & dyn_objects);
void calcSlowDownPointsForPossibleCollision(
  const Trajectory & path, const double offset,
  std::vector<PossibleCollisionInfo> & possible_collisions);
/**
 * @brief generate possible collisions coming from occlusion spots on the side of the path
 */
bool generatePossibleCollisionsFromGridMap(
  std::vector<PossibleCollisionInfo> & possible_collisions, const grid_map::GridMap & grid,
  const Trajectory & path, const double offset_from_start_to_ego, const PlannerParam & param,
  DebugData & debug_data);

}  // namespace utils
}  // namespace autoware::behavior_velocity_planner

#endif  // UTILS_HPP_
