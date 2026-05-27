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

#include "trajectory_utils.hpp"

#include <Eigen/Geometry>
#include <autoware/interpolation/linear_interpolation.hpp>

#include <boost/geometry.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance
{
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double start_time, double end_time, double time_resolution)
{
  const double initial_velocity = std::hypot(initial_twist.linear.x, initial_twist.linear.y);
  if (initial_velocity <= 0.0 || start_time >= end_time) {
    return {{start_time}, {0.0}};
  }

  struct StopProfile
  {
    double stop_time;
    double stop_position;
  };

  const auto stop_profile = [&]() -> std::optional<StopProfile> {
    if (assumed_acceleration >= 0.0) return std::nullopt;
    const double time_to_stop = initial_velocity / -assumed_acceleration;
    const double stop_time = braking_lag + time_to_stop;
    const double stop_position = initial_velocity * (braking_lag + 0.5 * time_to_stop);
    return StopProfile{stop_time, stop_position};
  }();

  TimeTrajectory times;
  TravelDistanceTrajectory distances;
  times.reserve(static_cast<size_t>((end_time - start_time) / time_resolution) + 4U);
  distances.reserve(times.capacity());

  auto distance = [&](double t) {
    if (t < braking_lag) {
      return initial_velocity * t;
    }
    if (stop_profile.has_value() && t >= stop_profile->stop_time) {
      return stop_profile->stop_position;
    }
    const double time_after_lag = t - braking_lag;
    return initial_velocity * t + (0.5 * assumed_acceleration * time_after_lag * time_after_lag);
  };

  constexpr double epsilon = 1e-3;
  auto append_sample = [&](const double t) {
    if (t < start_time || t > end_time) return;
    if (!times.empty() && t < times.back() + epsilon) return;
    times.push_back(t);
    distances.push_back(distance(t));
  };

  append_sample(start_time);
  for (int64_t tick = static_cast<int64_t>(std::floor(start_time / time_resolution)) + 1;; ++tick) {
    const double tick_time = static_cast<double>(tick) * time_resolution;
    if (
      stop_profile.has_value() && times.back() < stop_profile->stop_time &&
      tick_time > stop_profile->stop_time) {
      append_sample(stop_profile->stop_time);
      break;
    }

    if (tick_time >= end_time) {
      break;
    }
    append_sample(tick_time);
  }
  append_sample(end_time);

  return {times, distances};
}
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance

namespace autoware::trajectory_validator::plugin::safety::trajectory::pose
{
geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  const double ratio)
{
  geometry_msgs::msg::Pose interpolated_pose;
  interpolated_pose.position.x =
    interpolation::lerp(start_pose.position.x, end_pose.position.x, ratio);
  interpolated_pose.position.y =
    interpolation::lerp(start_pose.position.y, end_pose.position.y, ratio);
  interpolated_pose.position.z =
    interpolation::lerp(start_pose.position.z, end_pose.position.z, ratio);

  tf2::Quaternion start_q;
  tf2::Quaternion end_q;
  tf2::fromMsg(start_pose.orientation, start_q);
  tf2::fromMsg(end_pose.orientation, end_q);
  interpolated_pose.orientation = tf2::toMsg(start_q.slerp(end_q, ratio));

  return interpolated_pose;
}

namespace constant_curvature_predictor
{
namespace detail
{
Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry2d iso = Eigen::Isometry2d::Identity();
  iso.translation() << pose.position.x, pose.position.y;
  iso.linear() = Eigen::Rotation2Dd(tf2::getYaw(pose.orientation)).toRotationMatrix();
  return iso;
}

TwistPerDistance compute_twist_per_distance(const geometry_msgs::msg::Twist & twist)
{
  const Eigen::Vector2d vel(twist.linear.x, twist.linear.y);
  const double linear_speed = vel.norm();
  const double inv_speed = (linear_speed > 1e-6) ? (1.0 / linear_speed) : 0.0;

  return {vel * inv_speed, twist.angular.z * inv_speed};
}

Eigen::Isometry2d compute_delta_isometry(const TwistPerDistance & twist_per_dist, double distance)
{
  Eigen::Isometry2d delta_iso = Eigen::Isometry2d::Identity();
  const double theta = twist_per_dist.angular * distance;

  delta_iso.rotate(Eigen::Rotation2Dd(theta));

  double a;
  double b;
  if (std::abs(theta) < 1e-4) {
    const double theta_sq = theta * theta;
    a = 1.0 - theta_sq / 6.0;
    b = theta / 2.0 - (theta_sq * theta) / 24.0;
  } else {
    a = std::sin(theta) / theta;
    b = (1.0 - std::cos(theta)) / theta;
  }

  Eigen::Matrix2d v;
  v << a, -b, b, a;
  delta_iso.translation() = v * (twist_per_dist.linear * distance);

  return delta_iso;
}

geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry2d & iso, double initial_z)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = iso.translation().x();
  pose.position.y = iso.translation().y();
  pose.position.z = initial_z;

  const double yaw = Eigen::Rotation2Dd(iso.linear()).angle();
  pose.orientation = autoware::universe_utils::createQuaternionFromYaw(yaw);
  return pose;
}
}  // namespace detail

PoseTrajectory compute(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const TravelDistanceTrajectory & distance_trajectory)
{
  const Eigen::Isometry2d start_iso = detail::pose_to_isometry(initial_pose);
  const TwistPerDistance twist_per_dist = detail::compute_twist_per_distance(initial_twist);

  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());

  for (const auto & distance : distance_trajectory) {
    const Eigen::Isometry2d delta_iso = detail::compute_delta_isometry(twist_per_dist, distance);
    const Eigen::Isometry2d target_iso = start_iso * delta_iso;
    pose_trajectory.push_back(detail::isometry_to_pose(target_iso, initial_pose.position.z));
  }

  return pose_trajectory;
}
}  // namespace constant_curvature_predictor

PoseTrajectory compute_pose_trajectory_from_time(
  const TrajectoryPoints & traj_points, const TimeTrajectory & time_trajectory)
{
  if (traj_points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  std::vector<double> point_times;
  point_times.reserve(traj_points.size());
  for (const auto & point : traj_points) {
    point_times.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }

  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(time_trajectory.size());

  for (const auto target_time : time_trajectory) {
    size_t lower_idx = 0;
    size_t upper_idx = 0;
    if (traj_points.size() == 1) {
      pose_trajectory.push_back(traj_points.front().pose);
      continue;
    }
    if (target_time <= point_times.front()) {
      lower_idx = 0;
      upper_idx = 1;
    } else if (target_time >= point_times.back()) {
      lower_idx = traj_points.size() - 2;
      upper_idx = traj_points.size() - 1;
    } else {
      const auto upper_it = std::lower_bound(point_times.begin(), point_times.end(), target_time);
      upper_idx = static_cast<size_t>(std::distance(point_times.begin(), upper_it));
      lower_idx = upper_idx - 1;
    }

    const double lower_time = point_times.at(lower_idx);
    const double upper_time = point_times.at(upper_idx);
    const double denom = upper_time - lower_time;
    const double ratio = denom > 1e-6 ? (target_time - lower_time) / denom : 0.0;

    pose_trajectory.push_back(
      interpolate_pose(traj_points.at(lower_idx).pose, traj_points.at(upper_idx).pose, ratio));
  }

  return pose_trajectory;
}
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::pose

namespace autoware::trajectory_validator::plugin::safety::geometry
{
Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
  transform.linear() = Eigen::Rotation2Dd(tf2::getYaw(pose.orientation)).toRotationMatrix();
  transform.translation() = Eigen::Vector2d{pose.position.x, pose.position.y};
  return transform;
}

Point2d transform_point(const Eigen::Isometry2d & transform, const Point2d & point)
{
  const Eigen::Vector2d transformed = transform * Eigen::Vector2d{point.x(), point.y()};
  return Point2d{transformed.x(), transformed.y()};
}

std::vector<Point2d> create_base_polygon(const autoware_perception_msgs::msg::Shape & shape)
{
  if (shape.type == autoware_perception_msgs::msg::Shape::BOUNDING_BOX) {
    const double half_x = shape.dimensions.x / 2.0;
    const double half_y = shape.dimensions.y / 2.0;

    return {
      Point2d{half_x, half_y}, Point2d{half_x, -half_y}, Point2d{-half_x, -half_y},
      Point2d{-half_x, half_y}};
  } else if (shape.type == autoware_perception_msgs::msg::Shape::CYLINDER) {
    const double radius = shape.dimensions.x / 2.0;

    return {
      Point2d{radius, radius}, Point2d{radius, -radius}, Point2d{-radius, -radius},
      Point2d{-radius, radius}};
  } else if (shape.type == autoware_perception_msgs::msg::Shape::POLYGON) {
    std::vector<Point2d> polygon;
    polygon.reserve(shape.footprint.points.size());
    for (const auto & point : shape.footprint.points) {
      polygon.push_back(Point2d{point.x, point.y});
    }

    return polygon;
  }

  throw std::logic_error("The shape type is not supported in autoware_utils.");
}

std::vector<Point2d> create_base_polygon(const VehicleInfo & vehicle_info)
{
  const double half_width = vehicle_info.vehicle_width_m / 2.0;
  return {
    Point2d{vehicle_info.max_longitudinal_offset_m, half_width},
    Point2d{vehicle_info.max_longitudinal_offset_m, -half_width},
    Point2d{vehicle_info.min_longitudinal_offset_m, -half_width},
    Point2d{vehicle_info.min_longitudinal_offset_m, half_width}};
}

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape)
{
  const auto iso = pose_to_isometry(pose);
  const auto points = create_base_polygon(shape);

  Polygon2d polygon;
  polygon.outer().reserve(points.size() + 1U);
  for (const auto & point : points) {
    polygon.outer().push_back(transform_point(iso, point));
  }
  if (!polygon.outer().empty()) {
    polygon.outer().push_back(polygon.outer().front());
  }
  return polygon;
}
}  // namespace autoware::trajectory_validator::plugin::safety::geometry

namespace autoware::trajectory_validator::plugin::safety::trajectory
{
namespace footprint
{
FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const std::vector<Point2d> & base_poly)
{
  if (base_poly.size() == 4) {
    QuadTrajectory trajectory;
    trajectory.reserve(pose_trajectory.size());

    for (const auto & pose : pose_trajectory) {
      const auto iso = geometry::pose_to_isometry(pose);

      trajectory.push_back(
        std::array<Point2d, 4>{
          geometry::transform_point(iso, base_poly[0]),
          geometry::transform_point(iso, base_poly[1]),
          geometry::transform_point(iso, base_poly[2]),
          geometry::transform_point(iso, base_poly[3])});
    }
    return trajectory;
  } else {
    NgonTrajectory trajectory;
    trajectory.reserve(pose_trajectory.size());

    for (const auto & pose : pose_trajectory) {
      const auto iso = geometry::pose_to_isometry(pose);

      std::vector<Point2d> transformed_poly;
      transformed_poly.reserve(base_poly.size());

      for (const auto & pt : base_poly) {
        transformed_poly.push_back(geometry::transform_point(iso, pt));
      }

      trajectory.push_back(std::move(transformed_poly));
    }
    return trajectory;
  }
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const autoware_perception_msgs::msg::Shape & object_shape)
{
  return compute_footprint_trajectory(pose_trajectory, geometry::create_base_polygon(object_shape));
}

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const VehicleInfo & vehicle_info)
{
  return compute_footprint_trajectory(pose_trajectory, geometry::create_base_polygon(vehicle_info));
}
}  // namespace footprint

namespace detail
{
double to_seconds(const builtin_interfaces::msg::Duration & duration)
{
  return rclcpp::Duration(duration).seconds();
}

double project_current_pose_on_trajectory(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & current_pose)
{
  if (traj_points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  if (traj_points.size() == 1) {
    return to_seconds(traj_points.front().time_from_start);
  }

  const auto project_on_segment = [&](const size_t start_idx, const size_t end_idx) {
    const auto & start_pose = traj_points.at(start_idx).pose;
    const auto & end_pose = traj_points.at(end_idx).pose;
    const double current_x = current_pose.position.x;
    const double current_y = current_pose.position.y;

    const double dx = end_pose.position.x - start_pose.position.x;
    const double dy = end_pose.position.y - start_pose.position.y;
    const double segment_length_sq = dx * dx + dy * dy;

    double ratio = 0.0;
    if (segment_length_sq > 1e-6) {
      ratio =
        ((current_x - start_pose.position.x) * dx + (current_y - start_pose.position.y) * dy) /
        segment_length_sq;
    }

    return interpolation::lerp(
      to_seconds(traj_points.at(start_idx).time_from_start),
      to_seconds(traj_points.at(end_idx).time_from_start), ratio);
  };

  const size_t nearest_segment_idx =
    autoware::motion_utils::findNearestSegmentIndex(traj_points, current_pose.position);
  return project_on_segment(nearest_segment_idx, nearest_segment_idx + 1);
}

TravelDistanceTrajectory compute_cumulative_distances(const PoseTrajectory & pose_trajectory)
{
  TravelDistanceTrajectory distances;
  distances.reserve(pose_trajectory.size());

  double cumulative_distance = 0.0;
  for (size_t i = 0; i < pose_trajectory.size(); ++i) {
    if (i > 0) {
      cumulative_distance += autoware_utils_geometry::calc_distance2d(
        pose_trajectory.at(i - 1).position, pose_trajectory.at(i).position);
    }
    distances.push_back(cumulative_distance);
  }

  return distances;
}

TimeTrajectory compute_sample_times(double start_time, double end_time, double time_resolution)
{
  TimeTrajectory times;
  times.reserve(static_cast<size_t>((end_time - start_time) / time_resolution) + 2U);

  constexpr double epsilon = 1e-3;
  auto append_sample = [&](const double t) {
    if (t < start_time || t > end_time) return;
    if (!times.empty() && t < times.back() + epsilon) return;
    times.push_back(t);
  };

  append_sample(start_time);
  for (int64_t tick = static_cast<int64_t>(std::floor(start_time / time_resolution)) + 1;; ++tick) {
    const double tick_time = static_cast<double>(tick) * time_resolution;
    if (tick_time >= end_time) {
      break;
    }
    append_sample(tick_time);
  }
  append_sample(end_time);

  return times;
}

geometry_msgs::msg::Pose interpolate_predicted_path_pose(
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double query_time,
  double path_start_time)
{
  if (predicted_path.path.empty()) {
    throw std::invalid_argument("predicted path must not be empty");
  }

  const double path_time_step = rclcpp::Duration(predicted_path.time_step).seconds();
  if (predicted_path.path.size() == 1 || path_time_step <= 0.0) {
    return predicted_path.path.front();
  }

  const double clamped_query_time = std::clamp(
    query_time, path_start_time,
    path_start_time + path_time_step * static_cast<double>(predicted_path.path.size() - 1));
  const double shifted_query_time = clamped_query_time - path_start_time;
  const size_t index = static_cast<size_t>(std::floor(shifted_query_time / path_time_step));
  const size_t next_index = std::min(index + 1, predicted_path.path.size() - 1);
  const double ratio =
    (shifted_query_time - static_cast<double>(index) * path_time_step) / path_time_step;
  return autoware::universe_utils::calcInterpolatedPose(
    predicted_path.path.at(index), predicted_path.path.at(next_index), ratio, false);
}
}  // namespace detail

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double max_time, double time_resolution, const TrajectoryPoints & traj_points,
  VehicleInfo & vehicle_info)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    initial_twist, braking_lag, assumed_acceleration, 0.0, max_time, time_resolution);

  double distance_offset =
    autoware::motion_utils::calcSignedArcLength(traj_points, traj_points.front().pose.position, 0);
  for (double & val : distances) {
    val += distance_offset;
  }
  auto poses = pose::compute_pose_trajectory(traj_points, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    TrajectoryIdentification{"EGO"}, std::move(times), std::move(distances), std::move(poses),
    std::move(footprints));
}

TrajectoryData generate_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, double max_time,
  double time_resolution, VehicleInfo & vehicle_info)
{
  if (traj_points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }

  const double start_time =
    detail::project_current_pose_on_trajectory(traj_points, context.odometry->pose.pose);
  const double end_time =
    std::min(detail::to_seconds(traj_points.back().time_from_start), start_time + max_time);

  TimeTrajectory relative_times{0.0};
  TimeTrajectory absolute_times{start_time};
  for (double sample_time = time_resolution; start_time + sample_time < end_time;
       sample_time =
         std::floor((sample_time + time_resolution + 1e-6) / time_resolution) * time_resolution) {
    relative_times.push_back(sample_time);
    absolute_times.push_back(start_time + sample_time);
  }

  auto poses = pose::compute_pose_trajectory_from_time(traj_points, absolute_times);
  auto distances = detail::compute_cumulative_distances(poses);
  auto footprints = footprint::compute_footprint_trajectory(poses, vehicle_info);

  return TrajectoryData(
    TrajectoryIdentification{"EGO"}, std::move(relative_times), std::move(distances),
    std::move(poses), std::move(footprints));
}

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution)
{
  const auto most_confident_path_it = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(),
    std::min(
      max_time, most_confident_path_it->path.size() *
                  rclcpp::Duration(most_confident_path_it->time_step).seconds()),
    time_resolution);

  auto poses = pose::compute_pose_trajectory(most_confident_path_it->path, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{
      predicted_object, stamp, "map_based_predicted_path", assumed_acceleration},
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_diffusion_based_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  rclcpp::Duration start_time, double max_time, const builtin_interfaces::msg::Time & stamp,
  double time_resolution)
{
  const auto most_confident_path_it = std::max_element(
    predicted_object.kinematics.predicted_paths.begin(),
    predicted_object.kinematics.predicted_paths.end(),
    [](const auto & a, const auto & b) { return a.confidence < b.confidence; });
  const auto & predicted_path = *most_confident_path_it;
  const double prediction_horizon =
    start_time.seconds() + static_cast<double>(predicted_path.path.size() - 1) *
                             rclcpp::Duration(predicted_path.time_step).seconds();
  auto times = detail::compute_sample_times(
    start_time.seconds(), std::min(max_time, prediction_horizon), time_resolution);
  PoseTrajectory poses;
  poses.reserve(times.size());
  for (const auto & time : times) {
    poses.push_back(
      detail::interpolate_predicted_path_pose(predicted_path, time, start_time.seconds()));
  }

  TravelDistanceTrajectory distances;
  distances.reserve(poses.size());
  distances.push_back(0.0);
  for (size_t i = 1; i < poses.size(); ++i) {
    distances.push_back(
      distances.back() +
      autoware_utils_geometry::calc_distance2d(poses.at(i - 1).position, poses.at(i).position));
  }

  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{predicted_object, stamp, "diffusion_based_trajectory", 0.0},
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_constant_curvature_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution)
{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(), max_time, time_resolution);

  auto poses = pose::constant_curvature_predictor::compute(
    predicted_object.kinematics.initial_pose_with_covariance.pose,
    predicted_object.kinematics.initial_twist_with_covariance.twist, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{
      predicted_object, stamp, "constant_curvature_path", assumed_acceleration},
    std::move(times), std::move(distances), std::move(poses), std::move(footprints));
}

TrajectoryData generate_object_trajectory(
  const FilterContext & context, const unique_identifier_msgs::msg::UUID object_id,
  const std::string & traj_type_str, const double acc, const double time_resolution,
  const double time_horizon)
{
  const auto find_predicted_object =
    [&object_id](
      const auto & predicted_objects) -> const autoware_perception_msgs::msg::PredictedObject & {
    auto it = std::find_if(
      predicted_objects.begin(), predicted_objects.end(),
      [&object_id](const auto & object) { return object.object_id == object_id; });
    assert(it != predicted_objects.end());
    return *it;
  };

  if (traj_type_str.find("map_based_predicted_path") != std::string::npos) {
    assert(context.predicted_objects);
    const auto & predicted_object = find_predicted_object(context.predicted_objects->objects);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    return generate_predicted_path_trajectory(
      predicted_object, 0.0, acc, objects_reference_time, time_horizon,
      context.predicted_objects->header.stamp, time_resolution);
  }

  if (traj_type_str.find("constant_curvature_path") != std::string::npos) {
    assert(context.predicted_objects);
    const auto & predicted_object = find_predicted_object(context.predicted_objects->objects);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    return generate_constant_curvature_trajectory(
      predicted_object, 0.0, acc, objects_reference_time, time_horizon,
      context.predicted_objects->header.stamp, time_resolution);
  }

  if (traj_type_str.find("diffusion_based") != std::string::npos) {
    assert(context.neural_network_predicted_objects);
    const auto & predicted_object =
      find_predicted_object(context.neural_network_predicted_objects->objects);
    const rclcpp::Duration objects_reference_time =
      rclcpp::Time(context.neural_network_predicted_objects->header.stamp) -
      rclcpp::Time(context.odometry->header.stamp);
    return generate_predicted_path_trajectory(
      predicted_object, 0.0, acc, objects_reference_time, time_horizon,
      context.neural_network_predicted_objects->header.stamp, time_resolution);
  }

  throw std::logic_error("Unsupported trajectory type in DRAC assessment: " + traj_type_str);
}
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory
