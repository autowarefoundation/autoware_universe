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
#include <tuple>
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

std::vector<Point2d> create_base_polygon(
  const trajectory::footprint::EgoDimensions & ego_dimensions)
{
  const double half_width = ego_dimensions.vehicle_width / 2.0;
  return {
    Point2d{ego_dimensions.front_offset, half_width},
    Point2d{ego_dimensions.front_offset, -half_width},
    Point2d{-ego_dimensions.rear_overhang, -half_width},
    Point2d{-ego_dimensions.rear_overhang, half_width}};
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
  const PoseTrajectory & pose_trajectory, const EgoDimensions & ego_dimensions)
{
  return compute_footprint_trajectory(
    pose_trajectory, geometry::create_base_polygon(ego_dimensions));
}
}  // namespace footprint

std::tuple<size_t, size_t, double> resolve_interpolation(
  const std::vector<double> & values, const double target_value)
{
  size_t lower_idx = 0;
  size_t upper_idx = 1;
  double ratio = 0.0;

  if (target_value <= values.front()) {
    lower_idx = 0;
    upper_idx = 1;
  } else if (target_value >= values.back()) {
    lower_idx = values.size() - 2;
    upper_idx = values.size() - 1;
  } else {
    const auto upper_it = std::lower_bound(values.begin(), values.end(), target_value);
    upper_idx = static_cast<size_t>(std::distance(values.begin(), upper_it));
    lower_idx = upper_idx - 1;
  }

  const double denom = values.at(upper_idx) - values.at(lower_idx);
  ratio = denom > 1e-6 ? (target_value - values.at(lower_idx)) / denom : 0.0;

  return {lower_idx, upper_idx, ratio};
}

namespace detail
{

struct BrakingProfile
{
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  InterpolatedState start_state;
};

std::optional<BrakingProfile> compute_braking_profile(
  const TrajectoryInterpolator & trajectory_interpolator, const rclcpp::Time & current_time,
  const EgoTrajectoryGenerationParams & params)
{
  if (params.assumed_acceleration >= 0.0) {
    return std::nullopt;
  }

  const rclcpp::Time braking_start_time =
    current_time + rclcpp::Duration::from_seconds(params.braking_lag);
  const auto braking_start_state =
    trajectory_interpolator.interpolate_state_from_time(braking_start_time);

  const double braking_initial_velocity = std::max(0.0, braking_start_state.longitudinal_velocity);
  const rclcpp::Duration braking_time_duration =
    rclcpp::Duration::from_seconds(braking_initial_velocity / -params.assumed_acceleration);
  const rclcpp::Time braking_end_time = braking_start_time + braking_time_duration;

  return BrakingProfile{braking_start_time, braking_end_time, braking_start_state};
}

}  // namespace detail

InterpolatedState TrajectoryInterpolator::interpolate_state_from_time(
  const rclcpp::Time & target_time) const
{
  if (trajectory_points_.size() == 1) {
    const auto & point = trajectory_points_.front();
    return InterpolatedState{0.0, point.pose, point.longitudinal_velocity_mps};
  }

  if (time_from_refs_.empty()) {
    throw std::invalid_argument(
      "reference_time_ must be set before calling interpolate_state_from_time");
  }

  const double target_time_from_ref = (target_time - reference_time_).seconds();
  const auto [lower_idx, upper_idx, ratio] =
    resolve_interpolation(time_from_refs_, target_time_from_ref);

  const auto pose = pose::interpolate_pose(
    trajectory_points_.at(lower_idx).pose, trajectory_points_.at(upper_idx).pose, ratio);

  return InterpolatedState{
    interpolation::lerp(
      static_cast<double>(dist_from_fronts_.at(lower_idx)),
      static_cast<double>(dist_from_fronts_.at(upper_idx)), ratio),
    pose,
    interpolation::lerp(
      static_cast<double>(trajectory_points_.at(lower_idx).longitudinal_velocity_mps),
      static_cast<double>(trajectory_points_.at(upper_idx).longitudinal_velocity_mps), ratio)};
}

InterpolatedState TrajectoryInterpolator::interpolate_state_from_dist(
  const double target_dist) const
{
  if (trajectory_points_.size() == 1) {
    const auto & point = trajectory_points_.front();
    return InterpolatedState{0.0, point.pose, point.longitudinal_velocity_mps};
  }

  if (dist_from_fronts_.empty()) {
    throw std::invalid_argument(
      "dist_from_fronts_ must be set before calling interpolate_state_from_dist");
  }

  const auto [lower_idx, upper_idx, ratio] = resolve_interpolation(dist_from_fronts_, target_dist);

  const auto pose = pose::interpolate_pose(
    trajectory_points_.at(lower_idx).pose, trajectory_points_.at(upper_idx).pose, ratio);

  return InterpolatedState{
    interpolation::lerp(
      static_cast<double>(dist_from_fronts_.at(lower_idx)),
      static_cast<double>(dist_from_fronts_.at(upper_idx)), ratio),
    pose,
    interpolation::lerp(
      static_cast<double>(trajectory_points_.at(lower_idx).longitudinal_velocity_mps),
      static_cast<double>(trajectory_points_.at(upper_idx).longitudinal_velocity_mps), ratio)};
}

std::vector<double> serialize_times(const CandidateTrajectory & candidate_trajectory)
{
  std::vector<double> times;
  times.reserve(candidate_trajectory.points.size());
  for (const auto & point : candidate_trajectory.points) {
    times.push_back(rclcpp::Duration(point.time_from_start).seconds());
  }
  return times;
}

std::vector<double> serialize_distances(const CandidateTrajectory & candidate_trajectory)
{
  const auto & points = candidate_trajectory.points;
  std::vector<double> distances;
  distances.reserve(points.size());
  double cumulative_distance = 0.0;
  for (size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      cumulative_distance += autoware_utils_geometry::calc_distance2d(
        points.at(i - 1).pose.position, points.at(i).pose.position);
    }
    distances.push_back(cumulative_distance);
  }

  return distances;
}

TrajectoryData generate_ego_trajectory(
  const TrajectoryInterpolator & trajectory_interpolator,
  const rclcpp::Time & sampling_reference_time, const rclcpp::Time & current_time,
  const double time_resolution, const VehicleInfo & vehicle_info,
  const EgoTrajectoryGenerationParams & params)
{
  const rclcpp::Time trajectory_start_time =
    trajectory_interpolator.reference_time_ +
    rclcpp::Duration::from_seconds(trajectory_interpolator.time_from_refs_.front());
  const rclcpp::Time trajectory_end_time =
    trajectory_interpolator.reference_time_ +
    rclcpp::Duration::from_seconds(trajectory_interpolator.time_from_refs_.back());

  const auto braking_profile =
    detail::compute_braking_profile(trajectory_interpolator, current_time, params);

  TimeTrajectory times;
  TravelDistanceTrajectory distances;
  PoseTrajectory poses;

  const auto append_sample = [&](
                               const double time_from_reference, const InterpolatedState & sample) {
    times.push_back(time_from_reference);
    distances.push_back(sample.distance);
    poses.push_back(sample.pose);
  };

  for (uint64_t n = 0;; ++n) {
    const auto time_from_sampling_reference =
      rclcpp::Duration::from_seconds(static_cast<double>(n) * time_resolution);
    const auto sampling_time = sampling_reference_time + time_from_sampling_reference;

    if (!braking_profile.has_value() || sampling_time < braking_profile->start_time) {
      auto sample_state = trajectory_interpolator.interpolate_state_from_time(sampling_time);
      append_sample(time_from_sampling_reference.seconds(), sample_state);
      if (sampling_time > trajectory_end_time) {
        break;
      }
    } else {
      const double sample_distance = [&]() {
        const double elapsed_time = (sampling_time - braking_profile->start_time).seconds();
        if (elapsed_time <= 0.0) {
          return 0.0;
        }
        return braking_profile->start_state.distance +
               braking_profile->start_state.longitudinal_velocity * elapsed_time +
               0.5 * params.assumed_acceleration * elapsed_time * elapsed_time;
      }();
      auto sample_state = trajectory_interpolator.interpolate_state_from_dist(sample_distance);
      append_sample(time_from_sampling_reference.seconds(), sample_state);
      if (sampling_time > braking_profile->end_time && sampling_time > trajectory_end_time) {
        break;
      }
    }
  }

  if (times.empty()) {
    // append_sample() is executed before break.
    throw std::invalid_argument("no samples are available for the requested time range");
  }

  const footprint::EgoDimensions ego_dimensions{
    vehicle_info.max_longitudinal_offset_m + params.ego_footprint_margin.front,
    -vehicle_info.min_longitudinal_offset_m + params.ego_footprint_margin.rear,
    vehicle_info.vehicle_width_m + 2.0 * params.ego_footprint_margin.lateral};
  auto footprints = footprint::compute_footprint_trajectory(poses, ego_dimensions);

  return TrajectoryData(
    TrajectoryIdentification{"EGO"}, std::move(times), std::move(distances), std::move(poses),
    std::move(footprints));
}

EgoTrajectoryCache::EgoTrajectoryCache(
  const CandidateTrajectory & candidate_traj, const rclcpp::Time & sampling_reference_time,
  const rclcpp::Time & current_time, const double time_resolution, const VehicleInfo & vehicle_info)
: trajectory_interpolator_(candidate_traj),
  vehicle_info_(vehicle_info),
  sampling_reference_time_(sampling_reference_time),
  current_time_(current_time),
  time_resolution_(time_resolution)
{
  if (candidate_traj.points.empty()) {
    throw std::invalid_argument("points must not be empty");
  }
  if (time_resolution <= 0.0) {
    throw std::invalid_argument("time_resolution must be positive");
  }
}

const TrajectoryData & EgoTrajectoryCache::get_or_compute_trajectory_data(
  const EgoTrajectoryGenerationParams & params) const
{
  const auto it = trajectory_data_cache_.find(params);
  if (it != trajectory_data_cache_.end()) {
    return it->second;
  }

  const auto [inserted_it, inserted] = trajectory_data_cache_.emplace(
    params, generate_ego_trajectory(
              trajectory_interpolator_, sampling_reference_time_, current_time_, time_resolution_,
              vehicle_info_, params));
  static_cast<void>(inserted);
  return inserted_it->second;
}

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object,
  const autoware_perception_msgs::msg::PredictedPath & predicted_path, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution)

{
  auto [times, distances] = time_distance::compute_motion_profile_1d(
    predicted_object.kinematics.initial_twist_with_covariance.twist, braking_lag,
    assumed_acceleration, start_time.seconds(),
    std::min(
      max_time, predicted_path.path.size() * rclcpp::Duration(predicted_path.time_step).seconds()),
    time_resolution);

  auto poses = pose::compute_pose_trajectory(predicted_path.path, distances);
  auto footprints = footprint::compute_footprint_trajectory(poses, predicted_object.shape);

  return TrajectoryData(
    TrajectoryIdentification{
      predicted_object, stamp, "map_based_predicted_path", assumed_acceleration},
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

}  // namespace autoware::trajectory_validator::plugin::safety::trajectory
