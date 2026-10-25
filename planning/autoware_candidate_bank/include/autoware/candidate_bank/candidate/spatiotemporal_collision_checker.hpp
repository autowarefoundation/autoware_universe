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

#ifndef AUTOWARE__CANDIDATE_BANK__CANDIDATE__SPATIOTEMPORAL_COLLISION_CHECKER_HPP_
#define AUTOWARE__CANDIDATE_BANK__CANDIDATE__SPATIOTEMPORAL_COLLISION_CHECKER_HPP_
//
// This file intentionally contains the complete geometry kernel used by the
// Candidate Bank cross-scenario evaluator.  Keeping the mathematics in one
// place makes it possible to benchmark/review the collision model without
// chasing helpers through the planner implementation.
//
// There are two related, but deliberately different, notions below.
//
//  (1) same-time physical collision
//      E(t) is compared with O(t).  Each body is approximated by a row of
//      circles.  Between two message samples the circle centres are linearly
//      swept, so fast motion cannot tunnel through the sampling instants.
//
//  (2) scaled-XYT spatiotemporal conflict
//      The trajectory is lifted to
//
//          q = ( x / D, y / D, t / T )
//
//      where D [m] is the spatial scale and T [s] is the temporal scale.  Two
//      points that visit almost the same map position at slightly different
//      times can therefore still be close in q-space.  This is useful for a
//      predicted trajectory whose timing is uncertain.  It is NOT a claim that
//      the two physical bodies occupy the same place at the same instant; the
//      same-time collision flag is kept separately for that reason.
//
// The XYT search uses a hierarchy of lower bounds:
//
//      whole trajectory XY AABB
//           -> time blocks in scaled XYT AABB
//           -> interval XYT AABB
//           -> circle-segment XYT AABB
//           -> exact 3-D segment/segment closest points
//
// A branch is rejected only when its AABB lower bound is already larger than
// the current relevant rho.  Thus the hierarchy removes work; it does not
// intentionally change the centre-trajectory XYT distance result.
//
// Vehicle circle approximation
// ----------------------------
// For a bounding box, circle radius is nominally W/2 and the circle centres are
// placed from the physical rear end to the physical front end.  This keeps the
// lateral extent close to the real vehicle width and allows the circular ends
// to extend longitudinally.  Between adjacent circles the rectangle side can
// be under-covered by at most `lateral_undercoverage_m` (unless the maximum
// circle-count cap is reached).  Hard same-time collision compensates for this
// known approximation error by adding the requested under-coverage allowance
// for both bodies to the collision threshold.  XYT clearance uses the nominal
// W/2 radii so parallel-lane clearance is not needlessly inflated.
//
// Historical paired predictions
// -----------------------------
// PredictedPath time is originally relative to the candidate generation time.
// `prepare_scenario()` subtracts scenario_age_sec and clips the elapsed prefix,
// so all prepared trajectories are expressed in CURRENT-relative time.  The
// age itself must therefore never be added again to the XYT time coordinate.

#include <builtin_interfaces/msg/duration.hpp>

#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::candidate_bank::spatiotemporal_collision
{

using autoware_perception_msgs::msg::PredictedObject;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_perception_msgs::msg::PredictedPath;
using autoware_perception_msgs::msg::Shape;
using autoware_planning_msgs::msg::Trajectory;
using geometry_msgs::msg::Pose;

constexpr double kEpsilon = 1.0e-9;
constexpr double kPi = 3.14159265358979323846;

struct Config
{
  bool enabled{false};
  bool compute_toi{true};

  // q=(x/D,y/D,t/T).  rho <= conflict_rho is reported as an XYT conflict.
  double spatial_scale_m{1.0};
  double temporal_scale_sec{0.5};
  double conflict_rho{1.0};
  double block_duration_sec{1.0};

  // Side-tight adaptive multi-circle approximation.
  double lateral_undercoverage_m{0.10};
  std::size_t max_circles_per_body{10};

  // Extra physical same-time margin.  The known circle under-coverage is added
  // independently by the checker; do not include it again here.
  double hard_collision_margin_m{0.0};

  // Optional additional spatial margin used only in the XYT clearance metric.
  double xyt_spatial_margin_m{0.0};
  // Soft front-clearance risk used by the existing E_i x O_j cross aggregation.
  // This is independent of legacy candidate_bank.safety.dynamic.
  bool following_enabled{false};
  double following_preferred_clearance_m{3.0};
  double following_lateral_margin_m{0.5};
  double following_risk_weight{1.0};

  // Reuse Candidate Bank's normal confidence threshold for paired paths.
  double min_prediction_confidence{0.05};

  // Ego dimensions are filled from VehicleSpec by DiffusionPlannerCore.
  double ego_length_m{4.8};
  double ego_width_m{1.8};
  double ego_base_link_to_center_m{1.4};
};

struct Statistics
{
  std::uint64_t path_tests{0};
  std::uint64_t path_rejects{0};
  std::uint64_t block_tests{0};
  std::uint64_t block_rejects{0};
  std::uint64_t interval_tests{0};
  std::uint64_t interval_rejects{0};
  std::uint64_t circle_aabb_tests{0};
  std::uint64_t circle_aabb_rejects{0};
  std::uint64_t xyt_segment_tests{0};
  std::uint64_t same_time_interval_tests{0};
  std::uint64_t same_time_circle_tests{0};
};

struct Result
{
  bool valid{false};
  bool same_time_collision{false};
  bool xyt_conflict{false};
  bool trajectory_conflict{false};
  // Earliest same-time continuous swept-circle contact in CURRENT-relative time.
  // Infinity means no physical collision was found.  The contact threshold uses
  // the same hard margin and adaptive-circle under-coverage compensation as the
  // same_time_collision decision, so the TOI and hard flag have identical geometry semantics.
  double first_collision_time_sec{std::numeric_limits<double>::infinity()};
  std::string first_collision_object_id;
  int first_collision_path_index{-1};
  double first_collision_path_confidence{0.0};
  int first_collision_ego_circle_index{-1};
  int first_collision_object_circle_index{-1};

  // Minimum nominal circle clearance among trajectory intervals that survive
  // the configured collision/proximity relevance bounds.  Far-away pairs may
  // therefore leave this as infinity.  The hard collision decision additionally
  // uses the known per-body under-coverage compensation described above.
  double min_same_time_clearance_m{std::numeric_limits<double>::infinity()};
  double min_forward_clearance_m{std::numeric_limits<double>::infinity()};
  double following_risk{0.0};

  // Minimum body-aware XYT score FOUND inside conflict_rho.  If no relevant
  // branch survives the lower bounds, this remains infinity.
  double min_xyt_rho{std::numeric_limits<double>::infinity()};
  double xyt_spatial_distance_m{std::numeric_limits<double>::infinity()};
  double xyt_spatial_clearance_m{std::numeric_limits<double>::infinity()};
  double xyt_ego_time_sec{0.0};
  double xyt_object_time_sec{0.0};
  double xyt_delta_time_sec{0.0};

  std::string object_id;
  int path_index{-1};
  double path_confidence{0.0};
  int ego_circle_index{-1};
  int object_circle_index{-1};

  std::size_t valid_path_count{0};
  Statistics stats;
};

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

inline Vec2 add(const Vec2 a, const Vec2 b)
{
  return {a.x + b.x, a.y + b.y};
}
inline Vec2 sub(const Vec2 a, const Vec2 b)
{
  return {a.x - b.x, a.y - b.y};
}
inline Vec2 mul(const Vec2 a, const double s)
{
  return {a.x * s, a.y * s};
}
inline double dot(const Vec2 a, const Vec2 b)
{
  return a.x * b.x + a.y * b.y;
}
inline double norm2(const Vec2 a)
{
  return dot(a, a);
}
inline Vec2 lerp(const Vec2 a, const Vec2 b, const double u)
{
  return add(a, mul(sub(b, a), u));
}

inline Vec3 add(const Vec3 a, const Vec3 b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3 sub(const Vec3 a, const Vec3 b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3 mul(const Vec3 a, const double s)
{
  return {a.x * s, a.y * s, a.z * s};
}
inline double dot(const Vec3 a, const Vec3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline double norm2(const Vec3 a)
{
  return dot(a, a);
}

struct Aabb2
{
  double min_x{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
};

struct Aabb3
{
  double min_x{std::numeric_limits<double>::infinity()};
  double max_x{-std::numeric_limits<double>::infinity()};
  double min_y{std::numeric_limits<double>::infinity()};
  double max_y{-std::numeric_limits<double>::infinity()};
  double min_z{std::numeric_limits<double>::infinity()};
  double max_z{-std::numeric_limits<double>::infinity()};
};

inline bool valid(const Aabb2 & b)
{
  return b.min_x <= b.max_x && b.min_y <= b.max_y;
}

inline bool valid(const Aabb3 & b)
{
  return b.min_x <= b.max_x && b.min_y <= b.max_y && b.min_z <= b.max_z;
}

inline void expand(Aabb2 & b, const Vec2 p, const double radius = 0.0)
{
  b.min_x = std::min(b.min_x, p.x - radius);
  b.max_x = std::max(b.max_x, p.x + radius);
  b.min_y = std::min(b.min_y, p.y - radius);
  b.max_y = std::max(b.max_y, p.y + radius);
}

inline void expand(Aabb3 & b, const Vec3 p)
{
  b.min_x = std::min(b.min_x, p.x);
  b.max_x = std::max(b.max_x, p.x);
  b.min_y = std::min(b.min_y, p.y);
  b.max_y = std::max(b.max_y, p.y);
  b.min_z = std::min(b.min_z, p.z);
  b.max_z = std::max(b.max_z, p.z);
}

inline void merge(Aabb3 & a, const Aabb3 & b)
{
  if (!valid(b)) return;
  a.min_x = std::min(a.min_x, b.min_x);
  a.max_x = std::max(a.max_x, b.max_x);
  a.min_y = std::min(a.min_y, b.min_y);
  a.max_y = std::max(a.max_y, b.max_y);
  a.min_z = std::min(a.min_z, b.min_z);
  a.max_z = std::max(a.max_z, b.max_z);
}

inline Aabb3 spatially_expanded(const Aabb3 & b, const double normalized_radius)
{
  Aabb3 out = b;
  if (!valid(out)) return out;
  out.min_x -= normalized_radius;
  out.max_x += normalized_radius;
  out.min_y -= normalized_radius;
  out.max_y += normalized_radius;
  return out;
}

inline double interval_gap(
  const double a_min, const double a_max, const double b_min, const double b_max)
{
  if (a_max < b_min) return b_min - a_max;
  if (b_max < a_min) return a_min - b_max;
  return 0.0;
}

inline double aabb2_distance(const Aabb2 & a, const Aabb2 & b)
{
  if (!valid(a) || !valid(b)) return std::numeric_limits<double>::infinity();
  const double dx = interval_gap(a.min_x, a.max_x, b.min_x, b.max_x);
  const double dy = interval_gap(a.min_y, a.max_y, b.min_y, b.max_y);
  return std::hypot(dx, dy);
}

inline double aabb3_distance2(const Aabb3 & a, const Aabb3 & b)
{
  if (!valid(a) || !valid(b)) return std::numeric_limits<double>::infinity();
  const double dx = interval_gap(a.min_x, a.max_x, b.min_x, b.max_x);
  const double dy = interval_gap(a.min_y, a.max_y, b.min_y, b.max_y);
  const double dz = interval_gap(a.min_z, a.max_z, b.min_z, b.max_z);
  return dx * dx + dy * dy + dz * dz;
}

inline double yaw_from_pose(const Pose & pose)
{
  const auto & q = pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

inline geometry_msgs::msg::Quaternion quaternion_from_yaw(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

inline double normalize_angle(double angle)
{
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

inline Pose interpolate_pose(const Pose & a, const Pose & b, const double u)
{
  Pose out;
  out.position.x = a.position.x + u * (b.position.x - a.position.x);
  out.position.y = a.position.y + u * (b.position.y - a.position.y);
  out.position.z = a.position.z + u * (b.position.z - a.position.z);
  const double yaw_a = yaw_from_pose(a);
  const double yaw_b = yaw_from_pose(b);
  out.orientation = quaternion_from_yaw(yaw_a + u * normalize_angle(yaw_b - yaw_a));
  return out;
}

inline double duration_sec(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1.0e-9 * static_cast<double>(d.nanosec);
}

inline std::string uuid_string(const unique_identifier_msgs::msg::UUID & id)
{
  static constexpr char kHex[] = "0123456789abcdef";
  std::string out;
  out.reserve(id.uuid.size() * 2U);
  for (const auto byte : id.uuid) {
    const auto value = static_cast<unsigned int>(byte);
    out.push_back(kHex[(value >> 4U) & 0x0FU]);
    out.push_back(kHex[value & 0x0FU]);
  }
  return out;
}

struct TimedPose
{
  Pose pose;
  double time_sec{0.0};
};

struct CircleTemplate
{
  double local_x{0.0};
  double local_y{0.0};
  double radius_m{0.0};
  double undercoverage_m{0.0};
};

inline std::vector<CircleTemplate> make_side_tight_circle_templates(
  const double length_m, const double width_m, const double local_center_x,
  const double local_center_y, const Config & config)
{
  std::vector<CircleTemplate> circles;
  const double length = std::max(0.0, length_m);
  const double width = std::max(0.0, width_m);
  const double radius = 0.5 * width;
  if (radius <= kEpsilon) {
    circles.push_back({local_center_x, local_center_y, 0.5 * length, 0.0});
    return circles;
  }

  // If two equal-radius circles are s metres apart, the maximum side gap at
  // their midpoint is
  //
  //   delta = r - sqrt(r^2 - (s/2)^2).
  //
  // Solving for s gives the largest spacing that respects a requested delta:
  //
  //   s_max = 2 * sqrt(2*r*delta - delta^2).
  //
  // This is why long vehicles naturally receive more circles without relying
  // on semantic labels such as CAR/BUS/TRUCK.
  const double requested_delta =
    std::clamp(config.lateral_undercoverage_m, 1.0e-3, std::max(1.0e-3, 0.95 * radius));
  const double spacing_limit =
    2.0 *
    std::sqrt(std::max(0.0, 2.0 * radius * requested_delta - requested_delta * requested_delta));

  std::size_t count = 1U;
  if (length > kEpsilon && spacing_limit > kEpsilon) {
    count = static_cast<std::size_t>(std::ceil(length / spacing_limit)) + 1U;
    count = std::max<std::size_t>(2U, count);
  }
  count = std::min(std::max<std::size_t>(1U, config.max_circles_per_body), count);

  double actual_undercoverage = 0.0;
  if (count > 1U) {
    const double spacing = length / static_cast<double>(count - 1U);
    const double half_spacing = 0.5 * spacing;
    if (half_spacing < radius) {
      actual_undercoverage =
        radius - std::sqrt(std::max(0.0, radius * radius - half_spacing * half_spacing));
    } else {
      // The max-circle cap may force a spacing wider than one diameter.  In
      // that unusual case the nominal row has a real gap; expose the full
      // radius as the hard-collision compensation rather than hiding it.
      actual_undercoverage = radius;
    }
  }

  circles.reserve(count);
  if (count == 1U) {
    circles.push_back({local_center_x, local_center_y, radius, 0.0});
    return circles;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const double u = static_cast<double>(i) / static_cast<double>(count - 1U);
    circles.push_back(
      {local_center_x - 0.5 * length + u * length, local_center_y, radius, actual_undercoverage});
  }
  return circles;
}

inline std::vector<CircleTemplate> make_object_circle_templates(
  const Shape & shape, const Config & config)
{
  if (shape.type == Shape::CYLINDER) {
    return {{0.0, 0.0, std::max(0.0, 0.5 * static_cast<double>(shape.dimensions.x)), 0.0}};
  }

  if (shape.type == Shape::POLYGON && !shape.footprint.points.empty()) {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto & p : shape.footprint.points) {
      min_x = std::min(min_x, static_cast<double>(p.x));
      max_x = std::max(max_x, static_cast<double>(p.x));
      min_y = std::min(min_y, static_cast<double>(p.y));
      max_y = std::max(max_y, static_cast<double>(p.y));
    }
    return make_side_tight_circle_templates(
      std::max(0.0, max_x - min_x), std::max(0.0, max_y - min_y), 0.5 * (min_x + max_x),
      0.5 * (min_y + max_y), config);
  }

  return make_side_tight_circle_templates(
    std::max(0.0, static_cast<double>(shape.dimensions.x)),
    std::max(0.0, static_cast<double>(shape.dimensions.y)), 0.0, 0.0, config);
}

inline Vec2 circle_center(const Pose & pose, const CircleTemplate & circle)
{
  const double yaw = yaw_from_pose(pose);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {
    pose.position.x + c * circle.local_x - s * circle.local_y,
    pose.position.y + s * circle.local_x + c * circle.local_y};
}

inline Vec3 to_q(const Vec2 xy, const double time_sec, const Config & config)
{
  return {
    xy.x / config.spatial_scale_m, xy.y / config.spatial_scale_m,
    time_sec / config.temporal_scale_sec};
}

struct CircleInterval
{
  Vec2 xy0;
  Vec2 xy1;
  Vec3 q0;
  Vec3 q1;
  double t0{0.0};
  double t1{0.0};
  double radius_m{0.0};
  double undercoverage_m{0.0};
  Aabb2 xy_outer_aabb;
  Aabb3 center_aabb;
  Aabb3 expanded_aabb;
};

struct CircleBlock
{
  std::size_t begin_interval{0};
  std::size_t end_interval{0};  // exclusive
  Aabb3 expanded_aabb;
};

struct CircleTrack
{
  double radius_m{0.0};
  double undercoverage_m{0.0};
  Aabb2 whole_xy_outer_aabb;
  Aabb3 whole_xyt_expanded_aabb;
  std::vector<CircleInterval> intervals;
  std::vector<CircleBlock> blocks;
};

struct PreparedBodyTrajectory
{
  bool valid{false};
  Aabb2 whole_xy_outer_aabb;
  std::vector<CircleTrack> circles;
};

struct PreparedScenarioPath
{
  std::string object_id;
  int path_index{-1};
  double confidence{0.0};
  PreparedBodyTrajectory body;
};

struct PreparedScenario
{
  bool valid{false};
  std::vector<PreparedScenarioPath> paths;
};

inline PreparedBodyTrajectory prepare_body(
  const std::vector<TimedPose> & poses, const std::vector<CircleTemplate> & templates,
  const Config & config)
{
  PreparedBodyTrajectory out;
  if (poses.size() < 2U || templates.empty()) return out;

  out.circles.reserve(templates.size());
  for (const auto & circle_template : templates) {
    CircleTrack track;
    track.radius_m = circle_template.radius_m;
    track.undercoverage_m = circle_template.undercoverage_m;
    track.intervals.reserve(poses.size() - 1U);

    for (std::size_t i = 0; i + 1U < poses.size(); ++i) {
      const auto & p0 = poses[i];
      const auto & p1 = poses[i + 1U];
      if (!(p1.time_sec > p0.time_sec + kEpsilon)) continue;

      CircleInterval interval;
      interval.t0 = p0.time_sec;
      interval.t1 = p1.time_sec;
      interval.xy0 = circle_center(p0.pose, circle_template);
      interval.xy1 = circle_center(p1.pose, circle_template);
      interval.q0 = to_q(interval.xy0, interval.t0, config);
      interval.q1 = to_q(interval.xy1, interval.t1, config);
      interval.radius_m = circle_template.radius_m;
      interval.undercoverage_m = circle_template.undercoverage_m;
      expand(interval.xy_outer_aabb, interval.xy0, circle_template.radius_m);
      expand(interval.xy_outer_aabb, interval.xy1, circle_template.radius_m);
      expand(interval.center_aabb, interval.q0);
      expand(interval.center_aabb, interval.q1);
      interval.expanded_aabb = spatially_expanded(
        interval.center_aabb,
        (circle_template.radius_m + config.xyt_spatial_margin_m) / config.spatial_scale_m);
      track.intervals.push_back(interval);
      merge(track.whole_xyt_expanded_aabb, interval.expanded_aabb);
      expand(track.whole_xy_outer_aabb, interval.xy0, circle_template.radius_m);
      expand(track.whole_xy_outer_aabb, interval.xy1, circle_template.radius_m);
      expand(out.whole_xy_outer_aabb, interval.xy0, circle_template.radius_m);
      expand(out.whole_xy_outer_aabb, interval.xy1, circle_template.radius_m);
    }

    if (track.intervals.empty()) continue;

    // Fixed-duration time blocks are a lightweight BVH specialised for ordered
    // trajectory data.  Unlike a general BVH, no spatial tree construction is
    // required: neighbouring time intervals are already the natural children.
    std::size_t begin = 0U;
    while (begin < track.intervals.size()) {
      const double block_start = track.intervals[begin].t0;
      std::size_t end = begin;
      CircleBlock block;
      block.begin_interval = begin;
      while (end < track.intervals.size() &&
             track.intervals[end].t0 < block_start + config.block_duration_sec - kEpsilon) {
        merge(block.expanded_aabb, track.intervals[end].expanded_aabb);
        ++end;
      }
      if (end == begin) {
        merge(block.expanded_aabb, track.intervals[begin].expanded_aabb);
        ++end;
      }
      block.end_interval = end;
      track.blocks.push_back(block);
      begin = end;
    }
    out.circles.push_back(std::move(track));
  }

  out.valid = !out.circles.empty() && valid(out.whole_xy_outer_aabb);
  return out;
}

inline std::vector<TimedPose> ego_timed_poses(
  const Trajectory & trajectory, const std::size_t live_start_index)
{
  std::vector<TimedPose> out;
  if (trajectory.points.size() < 2U || live_start_index >= trajectory.points.size()) return out;
  const double t0 = duration_sec(trajectory.points[live_start_index].time_from_start);
  out.reserve(trajectory.points.size() - live_start_index);
  double last_t = -std::numeric_limits<double>::infinity();
  for (std::size_t i = live_start_index; i < trajectory.points.size(); ++i) {
    const double t = std::max(0.0, duration_sec(trajectory.points[i].time_from_start) - t0);
    if (t <= last_t + kEpsilon) continue;
    out.push_back({trajectory.points[i].pose, t});
    last_t = t;
  }
  return out;
}

inline double predicted_path_dt_sec(const PredictedPath & path)
{
  return duration_sec(path.time_step);
}

inline std::vector<TimedPose> live_predicted_path_poses(
  const PredictedPath & path, const double scenario_age_sec)
{
  std::vector<TimedPose> out;
  const double dt = predicted_path_dt_sec(path);
  if (path.path.size() < 2U || dt <= kEpsilon || scenario_age_sec < -kEpsilon) return out;
  const double horizon = dt * static_cast<double>(path.path.size() - 1U);
  if (scenario_age_sec >= horizon - kEpsilon) return out;

  const double age = std::max(0.0, scenario_age_sec);
  const double index_f = age / dt;
  const std::size_t i0 =
    std::min(static_cast<std::size_t>(std::floor(index_f)), path.path.size() - 2U);
  const std::size_t i1 = i0 + 1U;
  const double original_t0 = dt * static_cast<double>(i0);
  const double u = std::clamp((age - original_t0) / dt, 0.0, 1.0);

  // Interpolate an exact t=current starting pose if the bank entry is older
  // than an integer number of PredictedPath samples.  This avoids a hidden
  // +/- one-sample timing bias in historical cross evaluation.
  out.push_back({interpolate_pose(path.path[i0], path.path[i1], u), 0.0});
  for (std::size_t i = i1; i < path.path.size(); ++i) {
    const double current_relative_t = dt * static_cast<double>(i) - age;
    if (current_relative_t <= kEpsilon) continue;
    out.push_back({path.path[i], current_relative_t});
  }
  return out;
}

inline PreparedBodyTrajectory prepare_ego(
  const Trajectory & trajectory, const std::size_t live_start_index, const Config & config)
{
  const auto poses = ego_timed_poses(trajectory, live_start_index);
  const auto circles = make_side_tight_circle_templates(
    config.ego_length_m, config.ego_width_m, config.ego_base_link_to_center_m, 0.0, config);
  return prepare_body(poses, circles, config);
}

inline PreparedScenario prepare_scenario(
  const PredictedObjects & predicted_objects, const double scenario_age_sec, const Config & config)
{
  PreparedScenario out;
  for (const auto & object : predicted_objects.objects) {
    const auto circle_templates = make_object_circle_templates(object.shape, config);
    for (std::size_t path_index = 0; path_index < object.kinematics.predicted_paths.size();
         ++path_index) {
      const auto & path = object.kinematics.predicted_paths[path_index];
      if (path.confidence < config.min_prediction_confidence) continue;
      const auto poses = live_predicted_path_poses(path, scenario_age_sec);
      PreparedBodyTrajectory body = prepare_body(poses, circle_templates, config);
      if (!body.valid) continue;
      PreparedScenarioPath prepared;
      prepared.object_id = uuid_string(object.object_id);
      prepared.path_index = static_cast<int>(path_index);
      prepared.confidence = path.confidence;
      prepared.body = std::move(body);
      out.paths.push_back(std::move(prepared));
    }
  }
  out.valid = !out.paths.empty();
  return out;
}

struct SegmentClosest3
{
  double u{0.0};
  double v{0.0};
  double distance2{0.0};
};

// Closest points between two finite 3-D line segments.  This is the standard
// segment/segment formulation used in real-time collision detection.  Here the
// third coordinate is scaled time, not physical z.
inline SegmentClosest3 segment_segment_closest(
  const Vec3 p1, const Vec3 q1, const Vec3 p2, const Vec3 q2)
{
  const Vec3 d1 = sub(q1, p1);
  const Vec3 d2 = sub(q2, p2);
  const Vec3 r = sub(p1, p2);
  const double a = dot(d1, d1);
  const double e = dot(d2, d2);
  const double f = dot(d2, r);

  double s = 0.0;
  double t = 0.0;
  if (a <= kEpsilon && e <= kEpsilon) {
    return {0.0, 0.0, norm2(r)};
  }
  if (a <= kEpsilon) {
    t = std::clamp(f / std::max(e, kEpsilon), 0.0, 1.0);
  } else {
    const double c = dot(d1, r);
    if (e <= kEpsilon) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = dot(d1, d2);
      const double denom = a * e - b * b;
      if (std::abs(denom) > kEpsilon) {
        s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
      }
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  const Vec3 c1 = add(p1, mul(d1, s));
  const Vec3 c2 = add(p2, mul(d2, t));
  return {s, t, norm2(sub(c1, c2))};
}

inline Vec2 point_at_time(const CircleInterval & interval, const double time_sec)
{
  const double duration = interval.t1 - interval.t0;
  const double u =
    duration <= kEpsilon ? 0.0 : std::clamp((time_sec - interval.t0) / duration, 0.0, 1.0);
  return lerp(interval.xy0, interval.xy1, u);
}

inline double swept_circle_clearance(
  const Vec2 e0, const Vec2 e1, const double ego_radius, const Vec2 o0, const Vec2 o1,
  const double object_radius)
{
  const Vec2 r0 = sub(e0, o0);
  const Vec2 r1 = sub(e1, o1);
  const Vec2 v = sub(r1, r0);
  const double vv = norm2(v);
  double alpha = 0.0;
  if (vv > kEpsilon) {
    alpha = std::clamp(-dot(r0, v) / vv, 0.0, 1.0);
  }
  return std::sqrt(norm2(add(r0, mul(v, alpha)))) - ego_radius - object_radius;
}
// Earliest contact parameter for two circles whose centres move linearly over
// the SAME physical time interval.  This solve is called only after minimum
// clearance has already proved that a collision exists, keeping the added work
// conditional on an actual contact rather than on every narrow-phase pair.
inline std::optional<double> swept_circle_toi_alpha(
  const Vec2 e0, const Vec2 e1, const Vec2 o0, const Vec2 o1,
  const double contact_center_distance_m)
{
  const Vec2 r0 = sub(e0, o0);
  const Vec2 r1 = sub(e1, o1);
  const Vec2 v = sub(r1, r0);
  const double radius2 = contact_center_distance_m * contact_center_distance_m;
  const double c = norm2(r0) - radius2;
  if (c <= kEpsilon) return 0.0;  // already touching/overlapping at interval start

  const double a = norm2(v);
  if (a <= kEpsilon) return std::nullopt;
  const double b = 2.0 * dot(r0, v);
  double discriminant = b * b - 4.0 * a * c;
  if (discriminant < -kEpsilon) return std::nullopt;
  discriminant = std::max(0.0, discriminant);

  const double root = (-b - std::sqrt(discriminant)) / (2.0 * a);
  if (root < -kEpsilon || root > 1.0 + kEpsilon) return std::nullopt;
  return std::clamp(root, 0.0, 1.0);
}

inline void evaluate_same_time_circle_pair(
  const CircleTrack & ego, const CircleTrack & object, const int ego_circle_index,
  const int object_circle_index, const std::string & object_id, const int path_index,
  const double path_confidence, const Config & config, Result & result)
{
  // First reject two complete circle tracks when their swept XY bounding boxes
  // cannot approach within the compensated hard margin.  This is a pure lower
  // bound: rejecting here cannot remove a same-time circle collision.
  const double hard_track_margin =
    config.hard_collision_margin_m + ego.undercoverage_m + object.undercoverage_m;
  const double relevance_margin = std::max(
    hard_track_margin, config.spatial_scale_m * config.conflict_rho + config.xyt_spatial_margin_m);
  if (aabb2_distance(ego.whole_xy_outer_aabb, object.whole_xy_outer_aabb) > relevance_margin) {
    return;
  }

  std::size_t ei = 0U;
  std::size_t oi = 0U;
  while (ei < ego.intervals.size() && oi < object.intervals.size()) {
    const auto & e = ego.intervals[ei];
    const auto & o = object.intervals[oi];
    const double overlap_begin = std::max(e.t0, o.t0);
    const double overlap_end = std::min(e.t1, o.t1);
    if (overlap_end > overlap_begin + kEpsilon) {
      ++result.stats.same_time_interval_tests;
      const double compensated_margin =
        config.hard_collision_margin_m + e.undercoverage_m + o.undercoverage_m;
      const double interval_relevance_margin = std::max(
        compensated_margin,
        config.spatial_scale_m * config.conflict_rho + config.xyt_spatial_margin_m);
      // The stored interval AABB encloses each nominal swept circle.  If even
      // these boxes are farther apart than the configured collision/proximity
      // relevance range, neither hard collision nor a useful near-clearance
      // measurement can result from this interval pair.
      if (aabb2_distance(e.xy_outer_aabb, o.xy_outer_aabb) > interval_relevance_margin) {
        if (e.t1 < o.t1 - kEpsilon) {
          ++ei;
        } else if (o.t1 < e.t1 - kEpsilon) {
          ++oi;
        } else {
          ++ei;
          ++oi;
        }
        continue;
      }
      const Vec2 e0 = point_at_time(e, overlap_begin);
      const Vec2 e1 = point_at_time(e, overlap_end);
      const Vec2 o0 = point_at_time(o, overlap_begin);
      const Vec2 o1 = point_at_time(o, overlap_end);
      ++result.stats.same_time_circle_tests;
      const double clearance = swept_circle_clearance(e0, e1, e.radius_m, o0, o1, o.radius_m);
      result.min_same_time_clearance_m = std::min(result.min_same_time_clearance_m, clearance);

      // Nominal side-tight circles can under-cover each rectangle side by a
      // known amount.  Compensate only the hard collision decision, not the
      // reported nominal clearance used for parallel-lane analysis.
      if (clearance <= compensated_margin) {
        result.same_time_collision = true;
        if (config.compute_toi) {
          const double contact_center_distance_m = e.radius_m + o.radius_m + compensated_margin;
          const auto alpha = swept_circle_toi_alpha(e0, e1, o0, o1, contact_center_distance_m);
          if (alpha) {
            const double toi_sec = overlap_begin + (*alpha) * (overlap_end - overlap_begin);
            if (toi_sec < result.first_collision_time_sec) {
              result.first_collision_time_sec = toi_sec;
              result.first_collision_object_id = object_id;
              result.first_collision_path_index = path_index;
              result.first_collision_path_confidence = path_confidence;
              result.first_collision_ego_circle_index = ego_circle_index;
              result.first_collision_object_circle_index = object_circle_index;
            }
          }
        }
      }
    }

    if (e.t1 < o.t1 - kEpsilon) {
      ++ei;
    } else if (o.t1 < e.t1 - kEpsilon) {
      ++oi;
    } else {
      ++ei;
      ++oi;
    }
  }
}

inline double time_interval_lower_bound_rho(
  const CircleInterval & e, const CircleInterval & o, const Config & config)
{
  return interval_gap(e.t0, e.t1, o.t0, o.t1) / config.temporal_scale_sec;
}

inline void evaluate_xyt_interval_pair(
  const CircleInterval & e, const CircleInterval & o, const int ego_circle_index,
  const int object_circle_index, const std::string & object_id, const int path_index,
  const double path_confidence, const Config & config, Result & result, double & best_rho)
{
  ++result.stats.interval_tests;
  if (time_interval_lower_bound_rho(e, o, config) > best_rho) {
    ++result.stats.interval_rejects;
    return;
  }

  ++result.stats.circle_aabb_tests;
  if (aabb3_distance2(e.expanded_aabb, o.expanded_aabb) > best_rho * best_rho) {
    ++result.stats.circle_aabb_rejects;
    return;
  }

  ++result.stats.xyt_segment_tests;
  const auto closest = segment_segment_closest(e.q0, e.q1, o.q0, o.q1);
  const Vec2 ego_xy = lerp(e.xy0, e.xy1, closest.u);
  const Vec2 object_xy = lerp(o.xy0, o.xy1, closest.v);
  const double spatial_distance = std::sqrt(norm2(sub(ego_xy, object_xy)));
  const double spatial_clearance =
    spatial_distance - e.radius_m - o.radius_m - config.xyt_spatial_margin_m;
  const double ego_time = e.t0 + closest.u * (e.t1 - e.t0);
  const double object_time = o.t0 + closest.v * (o.t1 - o.t0);
  const double delta_time = ego_time - object_time;

  // The exact segment/segment solve above finds the closest CENTRE trajectories
  // in scaled XYT.  We then convert the spatial component to body clearance.
  // This body-aware rho is an intentionally simple first implementation; hard
  // same-time collision remains independently exact under the circle/linear-
  // motion model.  Keeping both quantities in the log lets later experiments
  // decide whether a more expensive surface-tube optimizer is justified.
  const double normalized_clearance = std::max(0.0, spatial_clearance) / config.spatial_scale_m;
  const double normalized_time = delta_time / config.temporal_scale_sec;
  const double body_rho = std::hypot(normalized_clearance, normalized_time);
  if (body_rho > best_rho) return;

  best_rho = body_rho;
  result.min_xyt_rho = body_rho;
  result.xyt_spatial_distance_m = spatial_distance;
  result.xyt_spatial_clearance_m = spatial_clearance;
  result.xyt_ego_time_sec = ego_time;
  result.xyt_object_time_sec = object_time;
  result.xyt_delta_time_sec = delta_time;
  result.object_id = object_id;
  result.path_index = path_index;
  result.path_confidence = path_confidence;
  result.ego_circle_index = ego_circle_index;
  result.object_circle_index = object_circle_index;
}

inline void evaluate_xyt_circle_pair(
  const CircleTrack & ego, const CircleTrack & object, const int ego_circle_index,
  const int object_circle_index, const PreparedScenarioPath & scenario_path, const Config & config,
  Result & result, double & best_rho)
{
  // Circle-track AABB lower bound before descending into time blocks.
  // These whole-track boxes are built once in prepare_body(), not rebuilt for
  // every E_i x O_j comparison.
  ++result.stats.circle_aabb_tests;
  if (
    aabb3_distance2(ego.whole_xyt_expanded_aabb, object.whole_xyt_expanded_aabb) >
    best_rho * best_rho) {
    ++result.stats.circle_aabb_rejects;
    return;
  }

  for (const auto & eb : ego.blocks) {
    for (const auto & ob : object.blocks) {
      ++result.stats.block_tests;
      if (aabb3_distance2(eb.expanded_aabb, ob.expanded_aabb) > best_rho * best_rho) {
        ++result.stats.block_rejects;
        continue;
      }
      for (std::size_t ei = eb.begin_interval; ei < eb.end_interval; ++ei) {
        for (std::size_t oi = ob.begin_interval; oi < ob.end_interval; ++oi) {
          evaluate_xyt_interval_pair(
            ego.intervals[ei], object.intervals[oi], ego_circle_index, object_circle_index,
            scenario_path.object_id, scenario_path.path_index, scenario_path.confidence, config,
            result, best_rho);
        }
      }
    }
  }
}
// Evaluate only the longitudinal front corridor. Geometry already prepared for cross collision is
// reused: first/last ego circles define the vehicle longitudinal axis, and object circle tracks are
// already age-aligned to current-relative time.
//
// Complexity is O(C_obj * (I_ego + I_obj)) per predicted path; there is no extra E_i x O_j pass.
inline void evaluate_following_path(
  const PreparedBodyTrajectory & ego, const PreparedBodyTrajectory & object,
  const double path_confidence, const Config & config, Result & result)
{
  if (
    !config.following_enabled || ego.circles.size() < 2U || object.circles.empty() ||
    config.following_preferred_clearance_m <= kEpsilon) {
    return;
  }

  // Circle templates are ordered rear -> front by make_side_tight_circle_templates().
  const CircleTrack & ego_rear = ego.circles.front();
  const CircleTrack & ego_front = ego.circles.back();
  if (ego_rear.intervals.empty() || ego_front.intervals.empty()) return;

  const std::size_t ego_interval_count =
    std::min(ego_rear.intervals.size(), ego_front.intervals.size());
  if (ego_interval_count == 0U) return;

  const double ego_half_length = 0.5 * std::max(0.0, config.ego_length_m);
  const double ego_half_width = 0.5 * std::max(0.0, config.ego_width_m);
  const bool object_is_single_circle = object.circles.size() == 1U;

  double path_min_gap = std::numeric_limits<double>::infinity();

  for (const auto & object_circle : object.circles) {
    std::size_t ei = 0U;
    std::size_t oi = 0U;
    while (ei < ego_interval_count && oi < object_circle.intervals.size()) {
      const auto & rear_interval = ego_rear.intervals[ei];
      const auto & front_interval = ego_front.intervals[ei];
      const auto & object_interval = object_circle.intervals[oi];

      const double ego_t0 = std::max(rear_interval.t0, front_interval.t0);
      const double ego_t1 = std::min(rear_interval.t1, front_interval.t1);
      const double overlap_begin = std::max(ego_t0, object_interval.t0);
      const double overlap_end = std::min(ego_t1, object_interval.t1);

      const auto evaluate_sample = [&](const double t) {
        const Vec2 rear_xy = point_at_time(rear_interval, t);
        const Vec2 front_xy = point_at_time(front_interval, t);
        const Vec2 center_xy = mul(add(rear_xy, front_xy), 0.5);
        const Vec2 axis = sub(front_xy, rear_xy);
        const double axis_norm = std::sqrt(norm2(axis));
        if (axis_norm <= kEpsilon) return;

        const Vec2 forward = mul(axis, 1.0 / axis_norm);
        const Vec2 left{-forward.y, forward.x};
        const Vec2 object_xy = point_at_time(object_interval, t);
        const Vec2 rel = sub(object_xy, center_xy);
        const double longitudinal = dot(rel, forward);
        const double lateral = std::abs(dot(rel, left));

        // Side-by-side / adjacent-lane traffic is excluded by the body-aware lateral corridor.
        const double lateral_limit =
          ego_half_width + object_circle.radius_m + config.following_lateral_margin_m;
        if (longitudinal <= 0.0 || lateral > lateral_limit) return;

        // Multi-circle templates place end circle centres at longitudinal body ends. For a single
        // circle (e.g. cylinder), subtract its radius to estimate the near longitudinal surface.
        const double object_rear_longitudinal =
          object_is_single_circle ? longitudinal - object_circle.radius_m : longitudinal;
        const double gap = object_rear_longitudinal - ego_half_length;
        path_min_gap = std::min(path_min_gap, gap);
      };

      if (overlap_end > overlap_begin + kEpsilon) {
        evaluate_sample(overlap_begin);
        evaluate_sample(0.5 * (overlap_begin + overlap_end));
        evaluate_sample(overlap_end);
      }

      const double ego_end = std::min(rear_interval.t1, front_interval.t1);
      if (ego_end < object_interval.t1 - kEpsilon) {
        ++ei;
      } else if (object_interval.t1 < ego_end - kEpsilon) {
        ++oi;
      } else {
        ++ei;
        ++oi;
      }
    }
  }

  if (!std::isfinite(path_min_gap)) return;

  result.min_forward_clearance_m = std::min(result.min_forward_clearance_m, path_min_gap);
  const double deficit = std::max(0.0, config.following_preferred_clearance_m - path_min_gap);
  const double normalized = deficit / std::max(config.following_preferred_clearance_m, kEpsilon);
  const double confidence = std::clamp(path_confidence, 0.0, 1.0);
  const double risk = std::clamp(
    std::max(0.0, config.following_risk_weight) * confidence * normalized * normalized, 0.0, 1.0);
  result.following_risk = std::max(result.following_risk, risk);
}

inline Result evaluate(
  const PreparedBodyTrajectory & ego, const PreparedScenario & scenario, const Config & config)
{
  Result result;
  if (!config.enabled || !ego.valid || !scenario.valid) return result;
  result.valid = true;

  double best_rho = config.conflict_rho;
  for (const auto & scenario_path : scenario.paths) {
    ++result.valid_path_count;
    ++result.stats.path_tests;

    // Hard same-time collision is independent of XYT pruning.  In particular,
    // the side-tight circle row can under-cover the rectangle by a known small
    // amount, and the hard decision compensates that amount.  Therefore a path
    // must never be discarded by a nominal XYT AABB before the hard check.
    // The circle-pair kernel itself is still cheap and scans only overlapping
    // physical time intervals.
    for (std::size_t ec = 0; ec < ego.circles.size(); ++ec) {
      for (std::size_t oc = 0; oc < scenario_path.body.circles.size(); ++oc) {
        evaluate_same_time_circle_pair(
          ego.circles[ec], scenario_path.body.circles[oc], static_cast<int>(ec),
          static_cast<int>(oc), scenario_path.object_id, scenario_path.path_index,
          scenario_path.confidence, config, result);
      }
    }
    evaluate_following_path(ego, scenario_path.body, scenario_path.confidence, config, result);

    // This AABB contains nominal circle radii.  If the two complete spatial
    // sweeps are farther apart than D*rho, this path cannot affect the configured
    // XYT proximity threshold at ANY relative time.  This rejection applies only
    // to the XYT metric; hard same-time collision was already evaluated above.
    const double whole_xy_lower_bound =
      aabb2_distance(ego.whole_xy_outer_aabb, scenario_path.body.whole_xy_outer_aabb);
    if (whole_xy_lower_bound / config.spatial_scale_m > best_rho) {
      ++result.stats.path_rejects;
      continue;
    }

    for (std::size_t ec = 0; ec < ego.circles.size(); ++ec) {
      for (std::size_t oc = 0; oc < scenario_path.body.circles.size(); ++oc) {
        evaluate_xyt_circle_pair(
          ego.circles[ec], scenario_path.body.circles[oc], static_cast<int>(ec),
          static_cast<int>(oc), scenario_path, config, result, best_rho);
      }
    }
  }

  result.xyt_conflict =
    std::isfinite(result.min_xyt_rho) && result.min_xyt_rho <= config.conflict_rho + kEpsilon;
  result.trajectory_conflict = result.same_time_collision || result.xyt_conflict;
  return result;
}

}  // namespace autoware::candidate_bank::spatiotemporal_collision

#endif  // AUTOWARE__CANDIDATE_BANK__CANDIDATE__SPATIOTEMPORAL_COLLISION_CHECKER_HPP_
