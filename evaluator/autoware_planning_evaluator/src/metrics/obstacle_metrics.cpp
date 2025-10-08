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

#include "autoware/planning_evaluator/metrics/obstacle_metrics.hpp"
#include "autoware/planning_evaluator/metrics/metrics_utils.hpp"

#include "autoware_utils/geometry/geometry.hpp"
#include <autoware_utils/geometry/boost_polygon_utils.hpp>

#include <Eigen/Core>
#include <boost/geometry.hpp>

#include "autoware_planning_msgs/msg/trajectory_point.hpp"

#include <algorithm>
#include <limits>

namespace planning_diagnostics
{
namespace metrics
{
using autoware_planning_msgs::msg::TrajectoryPoint;
using autoware_utils::calc_distance2d;
namespace bg = boost::geometry;

Accumulator<double> calcDistanceToObstacle(
  const PredictedObjects & obstacles, const Trajectory & traj, const VehicleInfo & vehicle_info)
{
  Accumulator<double> stat;
  
  // Get local ego footprint once
  const autoware_utils::LinearRing2d local_ego_footprint = vehicle_info.createFootprint();
  
  for (const TrajectoryPoint & p : traj.points) {
    double min_dist = std::numeric_limits<double>::max();
    for (const auto & object : obstacles.objects) {
      // Use footprint-based distance calculation
      const auto dist = utils::calc_ego_object_distance(local_ego_footprint, p.pose, object);
      min_dist = std::min(min_dist, dist);
    }
    stat.add(min_dist);
  }
  return stat;
}

Accumulator<double> calcTimeToCollision(
  const PredictedObjects & obstacles, const Trajectory & traj, const VehicleInfo & vehicle_info, const double distance_threshold)
{
  Accumulator<double> stat;

  if (traj.points.empty() || obstacles.objects.empty()) {
    return stat;
  }

  // Get local ego footprint once
  const autoware_utils::LinearRing2d local_ego_footprint = vehicle_info.createFootprint();

  // Dynamic obstacle state
  struct DynamicObstacle {
    autoware_perception_msgs::msg::PredictedObject object;
    size_t path_index = 1;  // Next target point index in path
    geometry_msgs::msg::Vector3 direction;
    double velocity;
    
    explicit DynamicObstacle(const autoware_perception_msgs::msg::PredictedObject & obj) : object(obj) {
      // Calculate velocity
      const auto & twist = obj.kinematics.initial_twist_with_covariance.twist.linear;
      velocity = std::sqrt(twist.x * twist.x + twist.y * twist.y);
      
      // Set default direction from object's orientation (fallback for no path)
      const auto & quat = obj.kinematics.initial_pose_with_covariance.pose.orientation;
      direction.x = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
      direction.y = 2.0 * (quat.x * quat.y + quat.w * quat.z);
    }
    
    void updatePosition(double dt) {
      if (velocity < 1e-3) return;
      
      auto & pose = object.kinematics.initial_pose_with_covariance.pose;
      double remaining_dist = velocity * dt;
      
      // Use a more conservative threshold to handle floating-point precision issues
      constexpr double DISTANCE_THRESHOLD = 1e-3;
      
      // Move along path segments, handling multiple points if dt is large
      if (!object.kinematics.predicted_paths.empty()) {
        const auto & path = object.kinematics.predicted_paths[0].path;
        
        while (remaining_dist > DISTANCE_THRESHOLD && path_index < path.size()) {
          const auto & target = path[path_index];
          const double seg_dist = calc_distance2d(pose, target);
          
          // Handle case where path points are too close together
          if (seg_dist < DISTANCE_THRESHOLD) {
            pose = target;
            path_index++;
            continue;
          }
          
          if (seg_dist <= remaining_dist) {
            // Move to this path point and continue to next
            pose = target;
            remaining_dist -= seg_dist;
            path_index++;
            
            // Update direction for next segment
            if (path_index < path.size()) {
              const auto & next_target = path[path_index];
              const double dx = next_target.position.x - pose.position.x;
              const double dy = next_target.position.y - pose.position.y;
              const double norm = std::sqrt(dx * dx + dy * dy);
              if (norm > 1e-6) {
                direction.x = dx / norm;
                direction.y = dy / norm;
              }
            }
          } else {
            // Move partway to this path point
            const double ratio = remaining_dist / seg_dist;
            const double dx = target.position.x - pose.position.x;
            const double dy = target.position.y - pose.position.y;
            pose.position.x += ratio * dx;
            pose.position.y += ratio * dy;
            // Calculate orientation from movement direction
            const double half_yaw = std::atan2(dy, dx) * 0.5;
            pose.orientation = geometry_msgs::msg::Quaternion{};
            pose.orientation.z = std::sin(half_yaw);
            pose.orientation.w = std::cos(half_yaw);
            remaining_dist = 0.0;
          }
        }
      }
      
      // If path exhausted but still have distance, continue in last direction
      if (remaining_dist > DISTANCE_THRESHOLD) {
        pose.position.x += remaining_dist * direction.x;
        pose.position.y += remaining_dist * direction.y;
        pose.orientation = geometry_msgs::msg::Quaternion{};
        // Update orientation to match movement direction
        const double half_yaw = std::atan2(direction.y, direction.x) * 0.5;
        pose.orientation = geometry_msgs::msg::Quaternion{};
        pose.orientation.z = std::sin(half_yaw);
        pose.orientation.w = std::cos(half_yaw);
      }
    }
  };
  
  // Initialize dynamic obstacles
  std::vector<DynamicObstacle> dynamic_obstacles(obstacles.objects.begin(), obstacles.objects.end());

  TrajectoryPoint p0 = traj.points.front();
  double t = 0.0;
  constexpr double MIN_VELOCITY_THRESHOLD = 1e-3;
  
  for (const TrajectoryPoint & p : traj.points) {
    if (std::abs(p0.longitudinal_velocity_mps) < MIN_VELOCITY_THRESHOLD) {
      p0 = p;
      continue;
    }
    
    const double dt = calc_distance2d(p0, p) / std::abs(p0.longitudinal_velocity_mps);
    t += dt;
    
    // Update obstacle positions and check collisions
    for (auto & obstacle : dynamic_obstacles) {
      obstacle.updatePosition(dt);

      const auto distance = utils::calc_ego_object_distance(local_ego_footprint, p.pose, obstacle.object);
      if (distance <= distance_threshold) {
        stat.add(t);
        return stat;
      }
    }
    p0 = p;
  }
  return stat;
}

}  // namespace metrics
}  // namespace planning_diagnostics

// --- Trajectory ---
// // autoware_planning_msgs::msg::Trajectory
// import { Header } from "../base/Header";
// import { Time } from "@foxglove/schemas";
// import { Pose } from "../base/Pose";

// export type Trajectory = {
//   header: Header;
//   points: {
//     time_from_start: Time;
//     pose: Pose;
//     longitudinal_velocity_mps: number;
//     lateral_velocity_mps: number;
//     acceleration_mps2: number;
//     heading_rate_rps: number;
//     front_wheel_angle_rad: number;
//     rear_wheel_angle_rad: number;
//   }[];
// };

// --- Example message ---
// header:
//   stamp:
//     sec: 1759910394
//     nanosec: 996479141
//   frame_id: map
// points:
// - time_from_start:
//     sec: 0
//     nanosec: 0
//   pose:
//     position:
//       x: 89319.93690911525
//       y: 42814.39848351471
//       z: 6.732216302351307
//     orientation:
//       x: 0.00046032805402114495
//       y: -0.0015642847145964024
//       z: 0.2823038467161438
//       w: 0.9593236571880642
//   longitudinal_velocity_mps: 0.2500198483467102
//   lateral_velocity_mps: 0.0
//   acceleration_mps2: 0.5
//   heading_rate_rps: 0.0
//   front_wheel_angle_rad: 7.309041393455118e-06
//   rear_wheel_angle_rad: 0.0
// - time_from_start:
//     sec: 0
//     nanosec: 0
//   pose:
//     position:
//       x: 89320.0209699805
//       y: 42814.45264781047
//       z: 6.7318901790742975
//     orientation:
//       x: 0.00046032846401448214
//       y: -0.0015642845947016508
//       z: 0.28230409802588946
//       w: 0.9593235832341458
//   longitudinal_velocity_mps: 0.2500198483467102
//   lateral_velocity_mps: 0.0
//   acceleration_mps2: 0.5
//   heading_rate_rps: 0.0
//   front_wheel_angle_rad: 7.309041393455118e-06
//   rear_wheel_angle_rad: 0.0
// - time_from_start:
//     sec: 0
//     nanosec: 0
//   pose:
//     position:
//       x: 89320.10503081736
//       y: 42814.506812150255
//       z: 6.731564055797191
//     orientation:
//       x: 0.00046032825889766007
//       y: -0.0015642846541778998
//       z: 0.2823039723814531
//       w: 0.9593236202080435
//   longitudinal_velocity_mps: 0.2500198483467102
//   lateral_velocity_mps: 0.0
//   acceleration_mps2: 0.5
//   heading_rate_rps: 0.0
//   front_wheel_angle_rad: -7.3081660048046615e-06
//   rear_wheel_angle_rad: 0.0




// --- PredictedObjects ---
// // autoware_perception_msgs::msg::PredictedObjects
// import { Header } from "../base/Header";
// import { ClassificationLabel } from "../perception/ClassificationLabel";
// import { Point2D } from "../base/Point";
// import { Pose } from "../base/Pose";
// import { Twist } from "../base/Twist";
// import { Time } from "@foxglove/schemas";
// import { Dimensions } from "../base/Dimensions";

// export type PredictedObjects = {
//   header: Header;
//   objects: {
//     object_id: {
//       uuid: Uint8Array;
//     };
//     existence_probability: number;
//     classification: ClassificationLabel[];
//     kinematics: {
//       initial_pose_with_covariance: {
//         pose: Pose;
//         covariance: Float64Array;
//       };
//       initial_twist_with_covariance: {
//         twist: Twist;
//         covariance: Float64Array;
//       };
//       initial_acceleration_with_covariance: {
//         accel: Twist;
//         covariance: Float64Array;
//       };
//       predicted_paths: {
//         path: Pose[];
//         time_step: Time;
//         confidence: number;
//       }[];
//     };
//     shape: {
//       type: number;
//       footprint: {
//         points: Point2D[];
//       };
//       dimensions: Dimensions;
//     };
//   }[];
// };

// --- Example message ---
// header:
//   stamp:
//     sec: 1759910236
//     nanosec: 105867535
//   frame_id: map
// objects:
// - object_id:
//     uuid:
//     - 82
//     - 241
//     - 216
//     - 124
//     - 21
//     - 121
//     - 25
//     - 56
//     - 126
//     - 8
//     - 151
//     - 98
//     - 40
//     - 80
//     - 246
//     - 29
//   existence_probability: 0.9990000128746033
//   classification:
//   - label: 7
//     probability: 0.9990000128746033
//   kinematics:
//     initial_pose_with_covariance:
//       pose:
//         position:
//           x: 89351.45989577392
//           y: 42867.57572734029
//           z: 8.005940890021085
//         orientation:
//           x: -0.0
//           y: 0.0
//           z: 0.3266845556626807
//           w: -0.9451334303110206
//       covariance:
//       - 0.029165124607459847
//       - -0.013196207076906023
//       - -0.0005808238758571875
//       - 0.0
//       - 0.0
//       - 0.0
//       - -0.013196207076906021
//       - 0.028328188630868553
//       - -0.00037855005882831994
//       - 0.0
//       - 0.0
//       - 0.0
//       - -0.0005808238758571875
//       - -0.0003785500588283199
//       - 0.010000000000000002
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.010000000000000002
//       - 2.381403078491891e-06
//       - 0.001000855835054969
//       - 0.0
//       - 0.0
//       - 0.0
//       - 2.3814030784918912e-06
//       - 0.010000000000000002
//       - 0.0006523045125849525
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.001000855835054969
//       - 0.0006523045125849526
//       - 139.18012725130058
//     initial_twist_with_covariance:
//       twist:
//         linear:
//           x: 0.008870860754915291
//           y: -0.0
//           z: 0.0
//         angular:
//           x: 0.0
//           y: 0.0
//           z: 0.5147100481644954
//       covariance:
//       - 0.45698721673762
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - -0.002773302535145298
//       - 0.0
//       - 0.010000000000000002
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.010000000000000002
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.010000000000000002
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.010000000000000002
//       - 0.0
//       - -0.0027733025351452906
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.6009651966338189
//     initial_acceleration_with_covariance:
//       accel:
//         linear:
//           x: 0.0
//           y: 0.0
//           z: 0.0
//         angular:
//           x: 0.0
//           y: 0.0
//           z: 0.0
//       covariance:
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//       - 0.0
//     predicted_paths:
//     - path:
//       - position:
//           x: 89351.45989577392
//           y: 42867.57572734029
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.4633844812
//           y: 42867.572988368935
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.4668731885
//           y: 42867.570249397584
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.47036189579
//           y: 42867.56751042623
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.47385060308
//           y: 42867.56477145487
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.47733931037
//           y: 42867.56203248352
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.48082801765
//           y: 42867.55929351216
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.48431672495
//           y: 42867.5565545408
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.48780543223
//           y: 42867.55381556945
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.49129413952
//           y: 42867.551076598094
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.49478284681
//           y: 42867.54833762674
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.4982715541
//           y: 42867.545598655386
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.5017602614
//           y: 42867.54285968403
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.50524896868
//           y: 42867.54012071267
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.50873767596
//           y: 42867.53738174132
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.51222638326
//           y: 42867.53464276996
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.51571509054
//           y: 42867.531903798605
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.51920379784
//           y: 42867.529164827254
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.52269250512
//           y: 42867.5264258559
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.52618121241
//           y: 42867.52368688454
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       - position:
//           x: 89351.5296699197
//           y: 42867.52094791319
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.32668455566268073
//           w: 0.9451334303110206
//       time_step:
//         sec: 0
//         nanosec: 500000000
//       confidence: 0.3333333432674408
//     - path:
//       - position:
//           x: 89351.45989577392
//           y: 42867.57572734029
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.52995118884
//           y: 42866.88426711813
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.60000660377
//           y: 42866.19280689595
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.67006201869
//           y: 42865.50134667379
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.74011743361
//           y: 42864.80988645162
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.81017284852
//           y: 42864.118426229456
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.88022826344
//           y: 42863.42696600728
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89351.95028367836
//           y: 42862.73550578512
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.02033909329
//           y: 42862.04404556295
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.09039450821
//           y: 42861.352585340785
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.16044992313
//           y: 42860.66112511861
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.23050533806
//           y: 42859.969664896445
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.30056075298
//           y: 42859.27820467428
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.37061616789
//           y: 42858.58674445211
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.44067158281
//           y: 42857.89528422994
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.51072699773
//           y: 42857.203824007775
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.58078241265
//           y: 42856.51236378561
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.65083782758
//           y: 42855.82090356344
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.7208932425
//           y: 42855.12944334127
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.79094865742
//           y: 42854.4379831191
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.86100407234
//           y: 42853.74652289694
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89352.93105948727
//           y: 42853.05506267477
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89353.00111490217
//           y: 42852.3636024526
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       - position:
//           x: 89353.0711703171
//           y: 42851.67214223043
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.6705224984445002
//           w: 0.7418891959583622
//       time_step:
//         sec: 0
//         nanosec: 500000000
//       confidence: 0.3333333432674408
//     - path:
//       - position:
//           x: 89351.45989577392
//           y: 42867.57572734029
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89352.03972741148
//           y: 42867.19255600513
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89352.61955904904
//           y: 42866.80938466997
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89353.19939068658
//           y: 42866.42621333481
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89353.77922232414
//           y: 42866.04304199964
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89354.3590539617
//           y: 42865.65987066448
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89354.93888559926
//           y: 42865.27669932932
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89355.5187172368
//           y: 42864.89352799416
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89356.09854887436
//           y: 42864.510356659
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89356.67838051192
//           y: 42864.127185323836
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89357.25821214948
//           y: 42863.74401398867
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89357.83804378704
//           y: 42863.36084265351
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89358.41787542458
//           y: 42862.977671318346
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89358.99770706214
//           y: 42862.594499983185
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89359.5775386997
//           y: 42862.21132864802
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89360.15737033726
//           y: 42861.82815731286
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89360.7372019748
//           y: 42861.444985977694
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89361.31703361236
//           y: 42861.06181464253
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89361.89686524992
//           y: 42860.67864330737
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89362.47669688748
//           y: 42860.29547197221
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89363.05652852503
//           y: 42859.91230063705
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89363.63636016258
//           y: 42859.52912930189
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       - position:
//           x: 89364.21619180014
//           y: 42859.14595796673
//           z: 8.005940890021085
//         orientation:
//           x: 0.0
//           y: 0.0
//           z: -0.28784533522464295
//           w: 0.9576769094999695
//       time_step:
//         sec: 0
//         nanosec: 500000000
//       confidence: 0.3333333432674408
//   shape:
//     type: 1
//     footprint:
//       points: []
//     dimensions:
//       x: 0.7248825240973595
//       y: 0.7248825240973595
//       z: 1.094065077793848
// ---
