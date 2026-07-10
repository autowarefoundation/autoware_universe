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

#ifndef FILTERS__SAFETY__COLLISION_CHECK_FILTER__TRAJECTORY_UTILS_HPP_
#define FILTERS__SAFETY__COLLISION_CHECK_FILTER__TRAJECTORY_UTILS_HPP_

#include "autoware/trajectory_validator/validator_interface.hpp"
#include "types.hpp"

#include <autoware/motion_utils/trajectory/interpolation.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <rclcpp/duration.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace autoware::trajectory_validator::plugin::safety
{

using IndexRange = std::pair<size_t, size_t>;
using TimeRange = std::pair<double, double>;

using TimeTrajectory = std::vector<double>;
using TravelDistanceTrajectory = std::vector<double>;
using PoseTrajectory = std::vector<geometry_msgs::msg::Pose>;
using QuadTrajectory = std::vector<std::array<Point2d, 4>>;
using NgonTrajectory = std::vector<std::vector<Point2d>>;
using FootprintTrajectory = std::variant<QuadTrajectory, NgonTrajectory>;

class TrajectoryData
{
private:
  TrajectoryIdentification identification_;
  TimeTrajectory times_;
  TravelDistanceTrajectory distances_;  // todo(takagi): obsolete
  PoseTrajectory poses_;
  FootprintTrajectory footprints_;
  mutable std::map<IndexRange, Box2d> envelope_cache_;
  mutable std::map<IndexRange, Polygon2d> convex_cache_;

  size_t get_same_or_earlier_time_index(const double t) const
  {
    const auto it = std::upper_bound(times_.begin(), times_.end(), t + TIME_INDEX_EPSILON);
    if (it == times_.begin()) return 0;
    return std::distance(times_.begin(), it - 1);
  }

  size_t get_same_or_later_time_index(const double t) const
  {
    const auto it = std::lower_bound(times_.begin(), times_.end(), t - TIME_INDEX_EPSILON);
    if (it == times_.end()) return times_.size() - 1;
    return std::distance(times_.begin(), it);
  }

  IndexRange resolve_covering_index_range(const TimeRange & key_time) const
  {
    assert(key_time.first <= key_time.second);

    auto start_index = get_same_or_earlier_time_index(key_time.first);
    auto end_index = get_same_or_later_time_index(key_time.second);
    return {start_index, end_index};
  }

  Box2d compute_envelope(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    Box2d box;
    boost::geometry::assign_inverse(box);
    std::visit(
      [&](const auto & polygons) {
        for (size_t i = key.first; i <= key.second; ++i) {
          for (const auto & pt : polygons[i]) {
            boost::geometry::expand(box, pt);
          }
        }
      },
      footprints_);
    return box;
  }

  Polygon2d compute_convex(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    MultiPoint2d all_points;
    std::visit(
      [&](const auto & polygons) {
        size_t total_point_count = (key.second - key.first + 1U) * polygons[key.first].size();
        all_points.reserve(total_point_count);
        for (size_t i = key.first; i <= key.second; ++i) {
          for (const auto & pt : polygons[i]) {
            all_points.push_back(pt);
          }
        }
      },
      footprints_);

    Polygon2d hull;
    hull.outer().reserve(all_points.size());
    boost::geometry::convex_hull(all_points, hull);
    return hull;
  }

public:
  TrajectoryData(
    TrajectoryIdentification trajectory_identification, TimeTrajectory times,
    TravelDistanceTrajectory distances, PoseTrajectory poses, FootprintTrajectory footprints)
  : identification_(std::move(trajectory_identification)),
    times_(std::move(times)),
    distances_(std::move(distances)),
    poses_(std::move(poses)),
    footprints_(std::move(footprints))
  {
    if (times_.empty()) {
      throw std::invalid_argument(
        "Trajectory must not be empty classification: " + identification_.classification);
    }
    if (times_.size() != distances_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs distances) classification: " +
        identification_.classification);
    }
    if (times_.size() != poses_.size()) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs poses) classification: " +
        identification_.classification);
    }
    const auto footprint_size =
      std::visit([](const auto & polygons) { return polygons.size(); }, footprints_);
    if (times_.size() != footprint_size) {
      throw std::invalid_argument(
        "Trajectory sizes mismatch (times vs footprints) classification: " +
        identification_.classification);
    }
  }

  TrajectoryData() = delete;

  const TrajectoryIdentification & getObjectIdentification() const { return identification_; }
  const TimeTrajectory & getTimes() const { return times_; }
  const TravelDistanceTrajectory & getDistances() const { return distances_; }
  const PoseTrajectory & getPoses() const { return poses_; }
  const FootprintTrajectory & getFootprints() const { return footprints_; }

  size_t size() const { return times_.size(); }

  const Box2d & get_or_compute_envelope(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    auto [it, inserted] = envelope_cache_.try_emplace(key);
    if (inserted) {
      it->second = compute_envelope(key);
    }
    return it->second;
  }

  const Box2d & get_or_compute_envelope(const TimeRange & key_time) const
  {
    return get_or_compute_envelope(resolve_covering_index_range(key_time));
  }

  const Box2d & get_or_compute_overall_envelope() const
  {
    return get_or_compute_envelope(IndexRange{0U, times_.size() - 1});
  }

  const Polygon2d & get_or_compute_convex(const IndexRange & key) const
  {
    assert(key.first <= key.second);

    auto [it, inserted] = convex_cache_.try_emplace(key);
    if (inserted) {
      it->second = compute_convex(key);
    }
    return it->second;
  }

  const Polygon2d & get_or_compute_convex(const TimeRange & key_time) const
  {
    return get_or_compute_convex(resolve_covering_index_range(key_time));
  }
};
}  // namespace autoware::trajectory_validator::plugin::safety

namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance
{
std::pair<TimeTrajectory, TravelDistanceTrajectory> compute_motion_profile_1d(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double start_time, double end_time, double time_resolution);
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::time_distance

namespace autoware::trajectory_validator::plugin::safety::trajectory::pose
{
geometry_msgs::msg::Pose interpolate_pose(
  const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & end_pose,
  double ratio);

namespace constant_curvature_predictor
{
struct TwistPerDistance
{
  Eigen::Vector2d linear;
  double angular;
};

namespace detail
{
Eigen::Isometry2d pose_to_isometry(const geometry_msgs::msg::Pose & pose);
TwistPerDistance compute_twist_per_distance(const geometry_msgs::msg::Twist & twist);
Eigen::Isometry2d compute_delta_isometry(const TwistPerDistance & twist_per_dist, double distance);
geometry_msgs::msg::Pose isometry_to_pose(const Eigen::Isometry2d & iso, double initial_z);
}  // namespace detail

PoseTrajectory compute(
  const geometry_msgs::msg::Pose & initial_pose, const geometry_msgs::msg::Twist & initial_twist,
  const TravelDistanceTrajectory & distance_trajectory);
}  // namespace constant_curvature_predictor

template <class T>
PoseTrajectory compute_pose_trajectory(
  const T & traj_points, const TravelDistanceTrajectory & distance_trajectory)
{
  PoseTrajectory pose_trajectory;
  pose_trajectory.reserve(distance_trajectory.size());
  for (const auto & distance : distance_trajectory) {
    pose_trajectory.push_back(
      autoware::motion_utils::calcInterpolatedPose(traj_points, distance, false));
  }
  return pose_trajectory;
}

PoseTrajectory compute_pose_trajectory_from_time(
  const TrajectoryPoints & traj_points, const TimeTrajectory & time_trajectory);
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory::pose

namespace autoware::trajectory_validator::plugin::safety::geometry
{
namespace detail
{
template <typename Points>
std::pair<double, double> project_points(const Points & ring, double axis_x, double axis_y)
{
  if (ring.empty()) {
    return {0.0, 0.0};
  }

  const auto project = [&](const auto & point) {
    return boost::geometry::get<0>(point) * axis_x + boost::geometry::get<1>(point) * axis_y;
  };

  auto point_it = ring.begin();
  double min_projection = project(*point_it);
  double max_projection = min_projection;
  for (++point_it; point_it != ring.end(); ++point_it) {
    const double projection = project(*point_it);
    min_projection = std::min(min_projection, projection);
    max_projection = std::max(max_projection, projection);
  }

  return {min_projection, max_projection};
}

template <typename ClosedRing>
bool has_separating_axis(
  const ClosedRing & candidate_axes, const ClosedRing & ring_a, const ClosedRing & ring_b)
{
  auto previous_it = candidate_axes.begin();
  for (auto current_it = std::next(candidate_axes.begin()); current_it != candidate_axes.end();
       ++current_it) {
    const double edge_x =
      boost::geometry::get<0>(*current_it) - boost::geometry::get<0>(*previous_it);
    const double edge_y =
      boost::geometry::get<1>(*current_it) - boost::geometry::get<1>(*previous_it);
    const auto [min_a, max_a] = project_points(ring_a, -edge_y, edge_x);
    const auto [min_b, max_b] = project_points(ring_b, -edge_y, edge_x);
    if (max_a < min_b || max_b < min_a) {
      return true;
    }
    previous_it = current_it;
  }

  return false;
}
}  // namespace detail

template <typename ConvexPolygon>
bool intersects_sat(const ConvexPolygon & poly_a, const ConvexPolygon & poly_b)
{
  const auto & ring_a = poly_a.outer();
  const auto & ring_b = poly_b.outer();

  constexpr size_t minimum_closed_convex_ring_size = 3U;
  if (
    ring_a.size() < minimum_closed_convex_ring_size ||
    ring_b.size() < minimum_closed_convex_ring_size) {
    return false;
  }

  return !detail::has_separating_axis(ring_a, ring_a, ring_b) &&
         !detail::has_separating_axis(ring_b, ring_a, ring_b);
}

Polygon2d to_polygon2d(
  const geometry_msgs::msg::Pose & pose, const autoware_perception_msgs::msg::Shape & shape);
}  // namespace autoware::trajectory_validator::plugin::safety::geometry

namespace autoware::trajectory_validator::plugin::safety::trajectory
{

TimeTrajectory serialize_times(const CandidateTrajectory & candidate_trajectory);
TravelDistanceTrajectory serialize_distances(const CandidateTrajectory & candidate_trajectory);

namespace footprint
{
struct EgoDimensions
{
  double front_offset{0.0};
  double rear_overhang{0.0};
  double vehicle_width{0.0};
};

EgoDimensions make_ego_dimensions(
  const VehicleInfo & vehicle_info, const EgoFootprintMargin & ego_footprint_margin);

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory,
  const autoware_perception_msgs::msg::Shape & object_shape);

FootprintTrajectory compute_footprint_trajectory(
  const PoseTrajectory & pose_trajectory, const EgoDimensions & ego_dimensions);
}  // namespace footprint

struct EgoTrajectoryGenerationParams
{
  double braking_lag{0.0};
  double assumed_acceleration{0.0};
  footprint::EgoDimensions ego_dimensions{};

  bool operator<(const EgoTrajectoryGenerationParams & rhs) const
  {
    return std::tie(
             braking_lag, assumed_acceleration, ego_dimensions.front_offset,
             ego_dimensions.rear_overhang, ego_dimensions.vehicle_width) <
           std::tie(
             rhs.braking_lag, rhs.assumed_acceleration, rhs.ego_dimensions.front_offset,
             rhs.ego_dimensions.rear_overhang, rhs.ego_dimensions.vehicle_width);
  }
};

struct InterpolatedState
{
  // double time_from_start;
  double distance;
  geometry_msgs::msg::Pose pose;
  double longitudinal_velocity;
};

class TrajectoryInterpolator
{
public:
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> trajectory_points_{};
  const rclcpp::Time reference_time_{};
  const std::vector<double> time_from_refs_{};
  const std::vector<double> dist_from_fronts_{};

  explicit TrajectoryInterpolator(const CandidateTrajectory & candidate_traj)
  : trajectory_points_(candidate_traj.points),
    reference_time_(rclcpp::Time(candidate_traj.header.stamp)),
    time_from_refs_(serialize_times(candidate_traj)),
    dist_from_fronts_(serialize_distances(candidate_traj))
  {
    if (trajectory_points_.empty()) {
      throw std::invalid_argument("points must not be empty");
    }
  };
  InterpolatedState interpolate_state_from_time(const rclcpp::Time & target_time) const;
  InterpolatedState interpolate_state_from_dist(const double target_dist) const;
};

namespace detail
{
double project_current_pose_on_trajectory(
  const TrajectoryPoints & traj_points, const geometry_msgs::msg::Pose & current_pose);

TravelDistanceTrajectory compute_cumulative_distances(const PoseTrajectory & pose_trajectory);
}  // namespace detail

// todo(takagi): reuse for object trajectory generation
class EgoTrajectoryCache
{
private:
  const TrajectoryInterpolator trajectory_interpolator_;
  rclcpp::Time sampling_reference_time_;
  rclcpp::Time current_time_;
  double time_resolution_;
  mutable std::map<EgoTrajectoryGenerationParams, TrajectoryData> trajectory_data_cache_;

public:
  EgoTrajectoryCache(
    const CandidateTrajectory & candidate_traj, const rclcpp::Time & sampling_reference_time,
    const rclcpp::Time & current_time, double time_resolution);

  const TrajectoryData & get_or_compute_trajectory_data(
    const EgoTrajectoryGenerationParams & params) const;
};

TrajectoryData generate_ego_trajectory(
  const CandidateTrajectory & candidate_traj, const rclcpp::Time & sampling_reference_time,
  const rclcpp::Time & current_time, double time_resolution,
  const EgoTrajectoryGenerationParams & params);

TrajectoryData generate_ego_trajectory(
  const geometry_msgs::msg::Twist & initial_twist, double braking_lag, double assumed_acceleration,
  double max_time, double time_resolution, const TrajectoryPoints & traj_points,
  const footprint::EgoDimensions & ego_dimensions);

TrajectoryData generate_ego_trajectory(
  const TrajectoryPoints & traj_points, const FilterContext & context, double max_time,
  double time_resolution, const footprint::EgoDimensions & ego_dimensions);

TrajectoryData generate_predicted_path_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution);

TrajectoryData generate_constant_curvature_trajectory(
  const autoware_perception_msgs::msg::PredictedObject & predicted_object, double braking_lag,
  double assumed_acceleration, rclcpp::Duration start_time, double max_time,
  const builtin_interfaces::msg::Time & stamp, double time_resolution);

TrajectoryData generate_object_trajectory(
  const FilterContext & context, unique_identifier_msgs::msg::UUID object_id,
  const std::string & traj_type_str, double acc, double time_resolution, double time_horizon);
}  // namespace autoware::trajectory_validator::plugin::safety::trajectory

#endif  // FILTERS__SAFETY__COLLISION_CHECK_FILTER__TRAJECTORY_UTILS_HPP_
