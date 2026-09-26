// Copyright 2026 Xinchen Lin
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

#include "autoware/diffusion_planner/camp_atom_materializer.hpp"

#include "autoware/diffusion_planner/dimensions.hpp"

#include <Eigen/LU>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::diffusion_planner
{
namespace
{

namespace bg = boost::geometry;

constexpr double kDt = 0.1;
constexpr double kTtcThresholdSeconds = 0.95;
constexpr double kTtcProjectionHorizonSeconds = 3.0;
constexpr double kTtcStoppedSpeedMps = 5.0e-3;
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kPoseDimensions = static_cast<std::size_t>(POSE_DIM);
constexpr std::size_t kPointsPerLaneSegment = static_cast<std::size_t>(POINTS_PER_SEGMENT);
constexpr std::size_t kLanePointDimensions = static_cast<std::size_t>(SEGMENT_POINT_DIM);
constexpr std::size_t kLaneSegmentCount = static_cast<std::size_t>(NUM_SEGMENTS_IN_LANE);
constexpr std::size_t kRouteSegmentCount = static_cast<std::size_t>(NUM_SEGMENTS_IN_ROUTE);

constexpr double kLongitudinalAccelerationPositive = 2.40;
constexpr double kLongitudinalAccelerationNegative = 4.05;
constexpr double kLateralAcceleration = 4.89;
constexpr double kYawRate = 0.95;
constexpr double kYawAcceleration = 1.93;
constexpr double kLongitudinalJerk = 4.13;
constexpr double kJerkMagnitude = 8.37;

struct Vector2
{
  double x{0.0};
  double y{0.0};
};

Vector2 operator+(const Vector2 & lhs, const Vector2 & rhs)
{
  return {lhs.x + rhs.x, lhs.y + rhs.y};
}

Vector2 operator-(const Vector2 & lhs, const Vector2 & rhs)
{
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

Vector2 operator*(const Vector2 & value, const double scalar)
{
  return {value.x * scalar, value.y * scalar};
}

Vector2 operator/(const Vector2 & value, const double scalar)
{
  return {value.x / scalar, value.y / scalar};
}

double dot(const Vector2 & lhs, const Vector2 & rhs)
{
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

double norm(const Vector2 & value)
{
  return std::hypot(value.x, value.y);
}

Vector2 normalized(const Vector2 & value)
{
  const double length = norm(value);
  return length > 1.0e-12 ? value / length : Vector2{};
}

double wrap_angle(const double value)
{
  return std::atan2(std::sin(value), std::cos(value));
}

struct Pose2
{
  Vector2 position;
  double yaw{0.0};
};

using Trajectory = std::array<Pose2, kCampHorizonSteps>;
using Velocity = std::array<Vector2, kCampHorizonSteps>;
using Corners = std::array<Vector2, 4>;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;
using BgBox = bg::model::box<BgPoint>;

class PredictionView
{
public:
  explicit PredictionView(const CampAtomMaterializationInput & input) : input_(input)
  {
    const auto required = static_cast<std::size_t>(
      input_.batch_size * input_.agent_count * static_cast<std::int64_t>(kCampHorizonSteps) *
      POSE_DIM);
    if (
      input_.batch_size < 1 || input_.agent_count < 1 ||
      input_.denormalized_predictions.size() != required) {
      throw std::invalid_argument("CAMP predictions have an incompatible shape");
    }
  }

  double at(
    const std::size_t batch, const std::size_t agent, const std::size_t time,
    const std::size_t component) const
  {
    const auto index =
      (((batch * static_cast<std::size_t>(input_.agent_count) + agent) * kCampHorizonSteps + time) *
       kPoseDimensions) +
      component;
    return static_cast<double>(input_.denormalized_predictions.at(index));
  }

private:
  const CampAtomMaterializationInput & input_;
};

Corners obb_corners(
  const Pose2 & pose, const double length, const double width, const double wheelbase = 0.0)
{
  const Vector2 direction{std::cos(pose.yaw), std::sin(pose.yaw)};
  const Vector2 lateral{-direction.y, direction.x};
  const Vector2 center = pose.position + direction * (0.5 * wheelbase);
  const Vector2 longitudinal = direction * (0.5 * length);
  const Vector2 lateral_offset = lateral * (0.5 * width);
  return {
    center - longitudinal - lateral_offset, center + longitudinal - lateral_offset,
    center + longitudinal + lateral_offset, center - longitudinal + lateral_offset};
}

std::pair<double, double> projection_bounds(const Corners & corners, const Vector2 & axis)
{
  double minimum = dot(corners.front(), axis);
  double maximum = minimum;
  for (std::size_t i = 1; i < corners.size(); ++i) {
    const double value = dot(corners.at(i), axis);
    minimum = std::min(minimum, value);
    maximum = std::max(maximum, value);
  }
  return {minimum, maximum};
}

bool obb_collides(const Corners & first, const Corners & second)
{
  for (const Corners * corners : {&first, &second}) {
    for (std::size_t i = 0; i < corners->size(); ++i) {
      const Vector2 edge = corners->at((i + 1) % corners->size()) - corners->at(i);
      const Vector2 axis = normalized(Vector2{-edge.y, edge.x});
      const auto [first_minimum, first_maximum] = projection_bounds(first, axis);
      const auto [second_minimum, second_maximum] = projection_bounds(second, axis);
      if (first_maximum < second_minimum || second_maximum < first_minimum) return false;
    }
  }
  return true;
}

double point_segment_distance(const Vector2 & point, const Vector2 & start, const Vector2 & end)
{
  const Vector2 segment = end - start;
  const double denominator = dot(segment, segment);
  const double fraction =
    denominator > 1.0e-12 ? std::clamp(dot(point - start, segment) / denominator, 0.0, 1.0) : 0.0;
  return norm(point - (start + segment * fraction));
}

double obb_distance(const Corners & first, const Corners & second)
{
  if (obb_collides(first, second)) return 0.0;
  double result = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < first.size(); ++i) {
    for (std::size_t j = 0; j < second.size(); ++j) {
      result = std::min(
        result,
        point_segment_distance(first.at(i), second.at(j), second.at((j + 1) % second.size())));
      result = std::min(
        result,
        point_segment_distance(second.at(i), first.at(j), first.at((j + 1) % first.size())));
    }
  }
  return result;
}

BgPolygon polygon_from_corners(const Corners & corners)
{
  BgPolygon polygon;
  for (const auto & corner : corners) {
    polygon.outer().emplace_back(corner.x, corner.y);
  }
  bg::correct(polygon);
  return polygon;
}

bool tensor_row_is_valid(
  const std::vector<float> & tensor, const std::size_t segment, const std::size_t point)
{
  const std::size_t row = (segment * kPointsPerLaneSegment + point) * kLanePointDimensions;
  for (std::size_t feature = 0; feature < 8; ++feature) {
    if (std::abs(static_cast<double>(tensor.at(row + feature))) > 1.0e-8) return true;
  }
  return false;
}

double tensor_value(
  const std::vector<float> & tensor, const std::size_t segment, const std::size_t point,
  const std::size_t feature)
{
  const std::size_t row = (segment * kPointsPerLaneSegment + point) * kLanePointDimensions;
  return static_cast<double>(tensor.at(row + feature));
}

std::optional<BgPolygon> lane_polygon(const std::vector<float> & tensor, const std::size_t segment)
{
  BgPolygon polygon;
  std::vector<BgPoint> right;
  for (std::size_t point = 0; point < kPointsPerLaneSegment; ++point) {
    if (!tensor_row_is_valid(tensor, segment, point)) continue;
    const double x = tensor_value(tensor, segment, point, X);
    const double y = tensor_value(tensor, segment, point, Y);
    polygon.outer().emplace_back(
      x + tensor_value(tensor, segment, point, LB_X),
      y + tensor_value(tensor, segment, point, LB_Y));
    right.emplace_back(
      x + tensor_value(tensor, segment, point, RB_X),
      y + tensor_value(tensor, segment, point, RB_Y));
  }
  if (polygon.outer().size() < 2 || right.size() < 2) return std::nullopt;
  polygon.outer().insert(polygon.outer().end(), right.rbegin(), right.rend());
  bg::correct(polygon);
  if (!bg::is_valid(polygon) || std::abs(bg::area(polygon)) <= 1.0e-9) return std::nullopt;
  return polygon;
}

std::optional<BgPolygon> lane_segment_polygon(
  const LaneSegment & segment, const Eigen::Matrix4d & map_to_ego)
{
  if (segment.left_boundary.size() < 2 || segment.right_boundary.size() < 2) {
    return std::nullopt;
  }
  const auto transform = [&map_to_ego](const LanePoint & point) {
    const Eigen::Vector4d local =
      map_to_ego * Eigen::Vector4d(point.x(), point.y(), point.z(), 1.0);
    return BgPoint{local.x(), local.y()};
  };

  BgPolygon polygon;
  for (const auto & point : segment.left_boundary) polygon.outer().push_back(transform(point));
  for (auto point = segment.right_boundary.rbegin(); point != segment.right_boundary.rend();
       ++point) {
    polygon.outer().push_back(transform(*point));
  }
  bg::correct(polygon);
  if (!bg::is_valid(polygon) || std::abs(bg::area(polygon)) <= 1.0e-9) return std::nullopt;
  return polygon;
}

std::vector<Trajectory> candidate_trajectories(
  const PredictionView & prediction, const std::size_t candidate_count)
{
  std::vector<Trajectory> result(candidate_count);
  for (std::size_t candidate = 0; candidate < candidate_count; ++candidate) {
    double previous_yaw = 0.0;
    for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
      const double cosine = prediction.at(candidate, 0, time, 2);
      const double sine = prediction.at(candidate, 0, time, 3);
      if (std::hypot(cosine, sine) < 1.0e-6) {
        throw std::invalid_argument("CAMP ego prediction has an invalid heading");
      }
      const double raw_yaw = std::atan2(sine, cosine);
      const double yaw = time == 0 ? raw_yaw : previous_yaw + wrap_angle(raw_yaw - previous_yaw);
      result.at(candidate).at(time) = {
        {prediction.at(candidate, 0, time, 0), prediction.at(candidate, 0, time, 1)}, yaw};
      previous_yaw = yaw;
    }
  }
  return result;
}

Velocity trajectory_velocity(const Trajectory & trajectory)
{
  Velocity velocity{};
  for (std::size_t time = 0; time + 1 < kCampHorizonSteps; ++time) {
    velocity.at(time) = (trajectory.at(time + 1).position - trajectory.at(time).position) / kDt;
  }
  velocity.back() = velocity.at(kCampHorizonSteps - 2);
  return velocity;
}

Pose2 actor_pose(
  const PredictionView & prediction, const std::size_t candidate, const std::size_t actor,
  const std::size_t time)
{
  const double cosine = prediction.at(candidate, actor + 1, time, 2);
  const double sine = prediction.at(candidate, actor + 1, time, 3);
  if (std::hypot(cosine, sine) < 1.0e-6) {
    throw std::invalid_argument("CAMP actor prediction has an invalid heading");
  }
  return {
    {prediction.at(candidate, actor + 1, time, 0), prediction.at(candidate, actor + 1, time, 1)},
    std::atan2(sine, cosine)};
}

Vector2 actor_velocity(
  const PredictionView & prediction, const std::size_t candidate, const std::size_t actor,
  const std::size_t time)
{
  const std::size_t next = std::min(time + 1, kCampHorizonSteps - 1);
  const std::size_t previous = next == time ? time - 1 : time;
  return {
    (prediction.at(candidate, actor + 1, next, 0) -
     prediction.at(candidate, actor + 1, previous, 0)) /
      kDt,
    (prediction.at(candidate, actor + 1, next, 1) -
     prediction.at(candidate, actor + 1, previous, 1)) /
      kDt};
}

struct RouteSegment
{
  Vector2 start;
  Vector2 direction;
  double length{0.0};
  Vector2 left_start;
  Vector2 left_end;
  Vector2 right_start;
  Vector2 right_end;
  double speed_start{std::numeric_limits<double>::quiet_NaN()};
  double speed_end{std::numeric_limits<double>::quiet_NaN()};
  double arc_start{0.0};
};

struct RouteProjection
{
  std::vector<std::array<double, kCampHorizonSteps>> arc;
  std::vector<std::array<double, kCampHorizonSteps>> speed_limit;
  std::vector<BgPolygon> route_objects;
  bool speed_observed{true};
};

RouteProjection project_to_route(
  const std::vector<Trajectory> & trajectories, const CampTensorContext & context)
{
  const std::size_t expected_route =
    kRouteSegmentCount * kPointsPerLaneSegment * kLanePointDimensions;
  if (
    context.route_lanes.size() != expected_route ||
    context.route_speed_limits.size() != kRouteSegmentCount) {
    throw std::invalid_argument("CAMP route tensor shape is incompatible");
  }

  std::vector<Vector2> centers;
  std::vector<Vector2> left;
  std::vector<Vector2> right;
  std::vector<double> speeds;
  RouteProjection output;
  for (std::size_t slot = 0; slot < kRouteSegmentCount; ++slot) {
    const double slot_speed = static_cast<double>(context.route_speed_limits.at(slot));
    const bool speed_available = std::isfinite(slot_speed) && slot_speed > 0.0;
    const auto polygon = lane_polygon(context.route_lanes, slot);
    if (polygon) output.route_objects.push_back(*polygon);
    for (std::size_t point = 0; point < kPointsPerLaneSegment; ++point) {
      if (!tensor_row_is_valid(context.route_lanes, slot, point)) continue;
      const double x = tensor_value(context.route_lanes, slot, point, X);
      const double y = tensor_value(context.route_lanes, slot, point, Y);
      centers.push_back({x, y});
      left.push_back(
        {tensor_value(context.route_lanes, slot, point, LB_X),
         tensor_value(context.route_lanes, slot, point, LB_Y)});
      right.push_back(
        {tensor_value(context.route_lanes, slot, point, RB_X),
         tensor_value(context.route_lanes, slot, point, RB_Y)});
      speeds.push_back(speed_available ? slot_speed : std::numeric_limits<double>::quiet_NaN());
    }
  }
  if (centers.size() < 2 || output.route_objects.empty()) {
    throw std::invalid_argument("CAMP route tensor has no usable geometry");
  }

  std::vector<RouteSegment> segments;
  double arc = 0.0;
  for (std::size_t index = 0; index + 1 < centers.size(); ++index) {
    const Vector2 delta = centers.at(index + 1) - centers.at(index);
    const double length = norm(delta);
    if (length <= 1.0e-6) continue;
    segments.push_back(
      {centers.at(index), delta / length, length, left.at(index), left.at(index + 1),
       right.at(index), right.at(index + 1), speeds.at(index), speeds.at(index + 1), arc});
    arc += length;
  }
  if (segments.empty()) throw std::invalid_argument("CAMP route has no nonzero segment");

  output.arc.resize(trajectories.size());
  output.speed_limit.resize(trajectories.size());
  for (std::size_t candidate = 0; candidate < trajectories.size(); ++candidate) {
    for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
      const Vector2 point = trajectories.at(candidate).at(time).position;
      std::size_t best = 0;
      double best_distance = std::numeric_limits<double>::infinity();
      double best_along = 0.0;
      for (std::size_t index = 0; index < segments.size(); ++index) {
        const auto & segment = segments.at(index);
        const double along =
          std::clamp(dot(point - segment.start, segment.direction), 0.0, segment.length);
        const double distance = norm(point - (segment.start + segment.direction * along));
        if (distance < best_distance) {
          best = index;
          best_distance = distance;
          best_along = along;
        }
      }
      const auto & segment = segments.at(best);
      const double fraction = best_along / segment.length;
      const Vector2 normal{-segment.direction.y, segment.direction.x};
      const Vector2 left_offset =
        segment.left_start + (segment.left_end - segment.left_start) * fraction;
      const Vector2 right_offset =
        segment.right_start + (segment.right_end - segment.right_start) * fraction;
      if (dot(left_offset, normal) <= 0.0 || -dot(right_offset, normal) <= 0.0) {
        throw std::invalid_argument("CAMP projected route boundaries are invalid");
      }
      output.arc.at(candidate).at(time) = segment.arc_start + best_along;
      output.speed_limit.at(candidate).at(time) =
        segment.speed_start + fraction * (segment.speed_end - segment.speed_start);
      output.speed_observed = output.speed_observed &&
                              std::isfinite(output.speed_limit.at(candidate).at(time)) &&
                              output.speed_limit.at(candidate).at(time) > 0.0;
    }
  }
  return output;
}

std::optional<BgMultiPolygon> build_drivable_area(
  const CampAtomMaterializationInput & input, const std::vector<Trajectory> & trajectories)
{
  double minimum_x = std::numeric_limits<double>::infinity();
  double minimum_y = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double maximum_y = -std::numeric_limits<double>::infinity();
  for (const auto & trajectory : trajectories) {
    for (const auto & pose : trajectory) {
      minimum_x = std::min(minimum_x, pose.position.x);
      minimum_y = std::min(minimum_y, pose.position.y);
      maximum_x = std::max(maximum_x, pose.position.x);
      maximum_y = std::max(maximum_y, pose.position.y);
    }
  }
  const BgBox envelope{
    BgPoint{minimum_x - 8.0, minimum_y - 8.0}, BgPoint{maximum_x + 8.0, maximum_y + 8.0}};

  BgMultiPolygon drivable;
  const auto add_polygon = [&drivable, &envelope](const std::optional<BgPolygon> & polygon) {
    if (!polygon || !bg::intersects(envelope, *polygon)) return true;
    if (drivable.empty()) {
      drivable.push_back(*polygon);
      return true;
    }
    BgMultiPolygon merged;
    bg::union_(drivable, *polygon, merged);
    if (merged.empty()) return false;
    drivable = std::move(merged);
    return true;
  };
  try {
    if (input.lanelet_map != nullptr) {
      const Eigen::Matrix4d map_to_ego = input.ego_to_map.inverse();
      for (const auto & segment : input.lanelet_map->lane_segments) {
        if (!add_polygon(lane_segment_polygon(segment, map_to_ego))) return std::nullopt;
      }
    } else {
      const auto & lanes = input.tensor_context.lanes;
      const std::size_t expected_lanes =
        kLaneSegmentCount * kPointsPerLaneSegment * kLanePointDimensions;
      if (lanes.size() != expected_lanes) return std::nullopt;
      for (std::size_t slot = 0; slot < kLaneSegmentCount; ++slot) {
        if (!add_polygon(lane_polygon(lanes, slot))) return std::nullopt;
      }
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }
  return drivable.empty() ? std::nullopt : std::make_optional(std::move(drivable));
}

double intersection_area(const BgPolygon & footprint, const BgMultiPolygon & area)
{
  std::vector<BgPolygon> intersections;
  bg::intersection(footprint, area, intersections);
  double result = 0.0;
  for (const auto & polygon : intersections) result += std::abs(bg::area(polygon));
  return result;
}

bool footprint_is_covered(const BgPolygon & footprint, const std::vector<BgPolygon> & objects)
{
  const double footprint_area = std::abs(bg::area(footprint));
  for (const auto & object : objects) {
    std::vector<BgPolygon> intersections;
    bg::intersection(footprint, object, intersections);
    double covered = 0.0;
    for (const auto & polygon : intersections) covered += std::abs(bg::area(polygon));
    if (footprint_area - covered <= 1.0e-9) return true;
  }
  return false;
}

void materialize_actor_atoms(
  const PredictionView & prediction, const CampAtomMaterializationInput & input,
  const std::vector<Trajectory> & trajectories, const RouteProjection & route,
  std::vector<trajectory_ranker::CampAtomVector> & atoms)
{
  for (std::size_t candidate = 0; candidate < trajectories.size(); ++candidate) {
    const Velocity ego_velocity = trajectory_velocity(trajectories.at(candidate));
    double collision_frames = 0.0;
    double ttc_sum = 0.0;
    double clearance_sum = 0.0;
    for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
      const Pose2 & ego_pose = trajectories.at(candidate).at(time);
      const Vector2 ego_direction{std::cos(ego_pose.yaw), std::sin(ego_pose.yaw)};
      const double ego_speed = norm(ego_velocity.at(time));
      const Vector2 ego_motion = ego_direction * ego_speed;
      const Vector2 ego_center = ego_pose.position + ego_direction * (0.5 * input.ego_wheelbase_m);
      const Corners ego_corners =
        obb_corners(ego_pose, input.ego_length_m, input.ego_width_m, input.ego_wheelbase_m);
      const BgPolygon footprint = polygon_from_corners(ego_corners);
      const bool lateral_relevance = !footprint_is_covered(footprint, route.route_objects);
      bool collision = false;
      double minimum_ttc = std::numeric_limits<double>::infinity();
      double maximum_clearance_deficit = 0.0;

      for (std::size_t actor = 0; actor < kCampActorCount; ++actor) {
        const auto & shape = input.actor_shapes.at(actor);
        if (!shape.observed) continue;
        const Pose2 obstacle = actor_pose(prediction, candidate, actor, time);
        const Vector2 obstacle_velocity = actor_velocity(prediction, candidate, actor, time);
        const Corners obstacle_corners = obb_corners(obstacle, shape.length_m, shape.width_m);
        const double distance = obb_distance(ego_corners, obstacle_corners);
        collision = collision || distance <= 1.0e-12;

        const Vector2 relative = obstacle.position - ego_center;
        const double center_distance = norm(relative);
        const Vector2 relative_unit = normalized(relative);
        const double closing =
          center_distance <= 1.0e-12
            ? 0.0
            : std::max(dot(ego_velocity.at(time) - obstacle_velocity, relative_unit), 0.0);
        maximum_clearance_deficit = std::max(
          maximum_clearance_deficit, std::max(kTtcThresholdSeconds * closing - distance, 0.0));

        const double cosine = center_distance <= 1.0e-12
                                ? 1.0
                                : std::clamp(dot(ego_direction, relative_unit), -1.0, 1.0);
        const double angle = std::acos(cosine);
        const bool ahead = angle < kPi / 6.0;
        const bool behind = angle > 5.0 * kPi / 6.0;
        if (ego_speed <= kTtcStoppedSpeedMps || (!ahead && (!lateral_relevance || behind))) {
          continue;
        }

        const double actor_speed = norm(obstacle_velocity);
        const Vector2 actor_direction{std::cos(obstacle.yaw), std::sin(obstacle.yaw)};
        const Vector2 actor_motion = actor_direction * actor_speed;
        const Pose2 elongated_ego{
          ego_center + ego_motion * (0.5 * kTtcProjectionHorizonSeconds), ego_pose.yaw};
        const Pose2 elongated_actor{
          obstacle.position + actor_motion * (0.5 * kTtcProjectionHorizonSeconds), obstacle.yaw};
        const Corners elongated_ego_corners = obb_corners(
          elongated_ego, input.ego_length_m + kTtcProjectionHorizonSeconds * ego_speed,
          input.ego_width_m);
        const Corners elongated_actor_corners = obb_corners(
          elongated_actor, shape.length_m + kTtcProjectionHorizonSeconds * actor_speed,
          shape.width_m);
        if (obb_distance(elongated_ego_corners, elongated_actor_corners) > 1.0e-12) continue;
        if (distance <= 1.0e-12) {
          minimum_ttc = 0.0;
          continue;
        }
        for (double projection = kDt; projection < kTtcProjectionHorizonSeconds;
             projection += kDt) {
          const Pose2 projected_ego{ego_center + ego_motion * projection, ego_pose.yaw};
          const Pose2 projected_actor{obstacle.position + actor_motion * projection, obstacle.yaw};
          if (
            obb_distance(
              obb_corners(projected_ego, input.ego_length_m, input.ego_width_m),
              obb_corners(projected_actor, shape.length_m, shape.width_m)) <= 1.0e-12) {
            minimum_ttc = std::min(minimum_ttc, projection);
            break;
          }
        }
      }
      collision_frames += collision ? 1.0 : 0.0;
      const double ttc_deficit = std::max(kTtcThresholdSeconds - minimum_ttc, 0.0);
      ttc_sum += kDt * ttc_deficit * ttc_deficit;
      clearance_sum += kDt * maximum_clearance_deficit * maximum_clearance_deficit;
    }
    atoms.at(candidate).at(0) = collision_frames / static_cast<double>(kCampHorizonSteps);
    atoms.at(candidate).at(1) = ttc_sum;
    atoms.at(candidate).at(2) = clearance_sum;
  }
}

void materialize_route_atoms(
  const std::vector<Trajectory> & trajectories, const RouteProjection & route,
  std::vector<trajectory_ranker::CampAtomVector> & atoms)
{
  std::vector<double> progress(trajectories.size(), 0.0);
  for (std::size_t candidate = 0; candidate < trajectories.size(); ++candidate) {
    double overspeed = 0.0;
    double reverse = 0.0;
    double running_progress = route.arc.at(candidate).front();
    for (std::size_t time = 1; time < kCampHorizonSteps; ++time) {
      const double speed = norm(
                             trajectories.at(candidate).at(time).position -
                             trajectories.at(candidate).at(time - 1).position) /
                           kDt;
      if (route.speed_observed) {
        const double excess = std::max(speed - route.speed_limit.at(candidate).at(time), 0.0);
        overspeed += kDt * excess * excess;
      }
      running_progress = std::max(running_progress, route.arc.at(candidate).at(time));
    }
    for (std::size_t time = 0; time + 10 < kCampHorizonSteps; ++time) {
      reverse +=
        std::max(-(route.arc.at(candidate).at(time + 10) - route.arc.at(candidate).at(time)), 0.0);
    }
    atoms.at(candidate).at(3) = route.speed_observed ? overspeed : 0.0;
    atoms.at(candidate).at(5) = reverse / static_cast<double>(kCampHorizonSteps - 10);
    progress.at(candidate) = running_progress;
  }
  const double reference = *std::max_element(progress.begin(), progress.end());
  for (std::size_t candidate = 0; candidate < trajectories.size(); ++candidate) {
    atoms.at(candidate).at(8) = std::max(reference - progress.at(candidate), 0.0);
  }
}

void materialize_road_atom(
  const std::vector<Trajectory> & trajectories, const CampAtomMaterializationInput & input,
  const std::optional<BgMultiPolygon> & drivable,
  std::vector<trajectory_ranker::CampAtomVector> & atoms)
{
  if (!drivable) return;
  for (std::size_t candidate = 0; candidate < trajectories.size(); ++candidate) {
    double severity = 0.0;
    for (const auto & pose : trajectories.at(candidate)) {
      const BgPolygon footprint = polygon_from_corners(
        obb_corners(pose, input.ego_length_m, input.ego_width_m, input.ego_wheelbase_m));
      const double area = std::abs(bg::area(footprint));
      const double inside = intersection_area(footprint, *drivable);
      severity += kDt * std::clamp(1.0 - inside / area, 0.0, 1.0);
    }
    atoms.at(candidate).at(4) = severity;
  }
}

void materialize_comfort_atoms(
  const std::vector<Trajectory> & trajectories,
  std::vector<trajectory_ranker::CampAtomVector> & atoms)
{
  for (std::size_t candidate = 0; candidate < trajectories.size(); ++candidate) {
    const auto & trajectory = trajectories.at(candidate);
    std::array<Vector2, kCampHorizonSteps - 1> velocity{};
    std::array<Vector2, kCampHorizonSteps - 2> acceleration{};
    std::array<Vector2, kCampHorizonSteps - 3> jerk{};
    std::array<double, kCampHorizonSteps - 1> yaw_rate{};
    std::array<double, kCampHorizonSteps - 2> yaw_acceleration{};
    for (std::size_t time = 0; time < velocity.size(); ++time) {
      velocity.at(time) = (trajectory.at(time + 1).position - trajectory.at(time).position) / kDt;
      yaw_rate.at(time) = (trajectory.at(time + 1).yaw - trajectory.at(time).yaw) / kDt;
    }
    for (std::size_t time = 0; time < acceleration.size(); ++time) {
      acceleration.at(time) = (velocity.at(time + 1) - velocity.at(time)) / kDt;
      yaw_acceleration.at(time) = (yaw_rate.at(time + 1) - yaw_rate.at(time)) / kDt;
    }
    for (std::size_t time = 0; time < jerk.size(); ++time) {
      jerk.at(time) = (acceleration.at(time + 1) - acceleration.at(time)) / kDt;
    }

    double longitudinal_acceleration_energy = 0.0;
    double lateral_acceleration_energy = 0.0;
    double yaw_rate_energy = 0.0;
    double yaw_acceleration_energy = 0.0;
    double longitudinal_jerk_energy = 0.0;
    double jerk_magnitude_energy = 0.0;
    for (std::size_t time = 0; time < acceleration.size(); ++time) {
      const double heading = trajectory.at(time + 2).yaw;
      const Vector2 forward{std::cos(heading), std::sin(heading)};
      const Vector2 left{-forward.y, forward.x};
      const double longitudinal = dot(acceleration.at(time), forward);
      const double lateral = dot(acceleration.at(time), left);
      const double longitudinal_scaled = longitudinal >= 0.0
                                           ? longitudinal / kLongitudinalAccelerationPositive
                                           : longitudinal / kLongitudinalAccelerationNegative;
      longitudinal_acceleration_energy += kDt * longitudinal_scaled * longitudinal_scaled;
      lateral_acceleration_energy += kDt * std::pow(lateral / kLateralAcceleration, 2.0);
      yaw_acceleration_energy += kDt * std::pow(yaw_acceleration.at(time) / kYawAcceleration, 2.0);
    }
    for (const double value : yaw_rate) {
      yaw_rate_energy += kDt * std::pow(value / kYawRate, 2.0);
    }
    for (std::size_t time = 0; time < jerk.size(); ++time) {
      const double heading = trajectory.at(time + 3).yaw;
      const Vector2 forward{std::cos(heading), std::sin(heading)};
      longitudinal_jerk_energy +=
        kDt * std::pow(dot(jerk.at(time), forward) / kLongitudinalJerk, 2.0);
      jerk_magnitude_energy += kDt * std::pow(norm(jerk.at(time)) / kJerkMagnitude, 2.0);
    }
    atoms.at(candidate).at(9) = longitudinal_acceleration_energy;
    atoms.at(candidate).at(10) = lateral_acceleration_energy;
    atoms.at(candidate).at(11) = yaw_rate_energy;
    atoms.at(candidate).at(12) = yaw_acceleration_energy;
    atoms.at(candidate).at(13) = longitudinal_jerk_energy;
    atoms.at(candidate).at(14) = jerk_magnitude_energy;
  }
}

CampWorldPlan world_plan(const Trajectory & trajectory, const Eigen::Matrix4d & ego_to_map)
{
  CampWorldPlan result{};
  const double yaw_offset = std::atan2(ego_to_map(1, 0), ego_to_map(0, 0));
  Vector2 previous{};
  for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
    const auto & pose = trajectory.at(time);
    const Eigen::Vector4d local(pose.position.x, pose.position.y, 0.0, 1.0);
    const Eigen::Vector4d mapped = ego_to_map * local;
    const Vector2 displacement = pose.position - previous;
    result.at(time) = {
      mapped.x(), mapped.y(), pose.yaw + yaw_offset,
      dot(displacement, Vector2{std::cos(pose.yaw), std::sin(pose.yaw)}) / kDt};
    previous = pose.position;
  }
  return result;
}

CampPlanState interpolate_plan(
  const CampWorldPlan & plan, const double origin_seconds, const double target_seconds)
{
  const double sample = (target_seconds - origin_seconds) / kDt - 1.0;
  const double bounded = std::clamp(sample, 0.0, static_cast<double>(kCampHorizonSteps - 1));
  const std::size_t lower = static_cast<std::size_t>(std::floor(bounded));
  const std::size_t upper = std::min(lower + 1, kCampHorizonSteps - 1);
  const double fraction = bounded - static_cast<double>(lower);
  const auto & first = plan.at(lower);
  const auto & second = plan.at(upper);
  return {
    first.x_m + fraction * (second.x_m - first.x_m),
    first.y_m + fraction * (second.y_m - first.y_m),
    first.yaw_rad + fraction * (second.yaw_rad - first.yaw_rad),
    first.longitudinal_velocity_mps +
      fraction * (second.longitudinal_velocity_mps - first.longitudinal_velocity_mps)};
}

bool materialize_transition_atom(
  const CampAtomMaterializationInput & input, const std::vector<CampWorldPlan> & plans,
  const std::array<double, 3> & scales, std::vector<trajectory_ranker::CampAtomVector> & atoms)
{
  if (!input.previous_plan) return false;
  const double previous_start = input.previous_plan->origin_seconds + kDt;
  const double previous_end =
    input.previous_plan->origin_seconds + kDt * static_cast<double>(kCampHorizonSteps);
  std::vector<double> common;
  for (std::size_t time = 0; time < kCampHorizonSteps; ++time) {
    const double stamp = input.origin_seconds + kDt * static_cast<double>(time + 1);
    if (stamp >= previous_start - 1.0e-9 && stamp <= previous_end + 1.0e-9) {
      common.push_back(stamp);
    }
  }
  if (common.empty()) return false;

  for (std::size_t candidate = 0; candidate < plans.size(); ++candidate) {
    double position_sum = 0.0;
    double yaw_sum = 0.0;
    double velocity_sum = 0.0;
    for (const double stamp : common) {
      const CampPlanState current =
        interpolate_plan(plans.at(candidate), input.origin_seconds, stamp);
      const CampPlanState previous =
        interpolate_plan(input.previous_plan->states, input.previous_plan->origin_seconds, stamp);
      position_sum +=
        std::pow(current.x_m - previous.x_m, 2.0) + std::pow(current.y_m - previous.y_m, 2.0);
      yaw_sum += std::pow(wrap_angle(current.yaw_rad - previous.yaw_rad), 2.0);
      velocity_sum +=
        std::pow(current.longitudinal_velocity_mps - previous.longitudinal_velocity_mps, 2.0);
    }
    const double count = static_cast<double>(common.size());
    const double position = std::sqrt(position_sum / count) / scales.at(0);
    const double yaw = std::sqrt(yaw_sum / count) / scales.at(1);
    const double velocity = std::sqrt(velocity_sum / count) / scales.at(2);
    atoms.at(candidate).at(15) =
      std::sqrt((position * position + yaw * yaw + velocity * velocity) / 3.0);
  }
  return true;
}

}  // namespace

CampAtomMaterializationResult materialize_camp_atoms(
  const CampAtomMaterializationInput & input,
  const std::array<double, 3> & transition_component_scales)
{
  if (
    input.batch_size < 1 || input.agent_count <= static_cast<std::int64_t>(kCampActorCount) ||
    !std::isfinite(input.ego_wheelbase_m) || input.ego_wheelbase_m <= 0.0 ||
    !std::isfinite(input.ego_length_m) || input.ego_length_m <= 0.0 ||
    !std::isfinite(input.ego_width_m) || input.ego_width_m <= 0.0) {
    throw std::invalid_argument("CAMP materialization input is incompatible");
  }
  for (const double scale : transition_component_scales) {
    if (!std::isfinite(scale) || scale <= 0.0) {
      throw std::invalid_argument("CAMP transition scales must be finite and positive");
    }
  }
  for (const auto & actor : input.actor_shapes) {
    if (
      actor.observed && (!std::isfinite(actor.length_m) || actor.length_m <= 0.0 ||
                         !std::isfinite(actor.width_m) || actor.width_m <= 0.0)) {
      throw std::invalid_argument("CAMP actor shapes must be finite and positive");
    }
  }

  const auto candidate_count = static_cast<std::size_t>(input.batch_size);
  const PredictionView prediction(input);
  const std::vector<Trajectory> trajectories = candidate_trajectories(prediction, candidate_count);
  const RouteProjection route = project_to_route(trajectories, input.tensor_context);
  const auto drivable = build_drivable_area(input, trajectories);

  CampAtomMaterializationResult output;
  output.status.fill(trajectory_ranker::CampAtomStatus::Observed);
  output.raw_atoms.assign(candidate_count, trajectory_ranker::CampAtomVector{});
  output.candidate_world_plans.reserve(candidate_count);

  materialize_actor_atoms(prediction, input, trajectories, route, output.raw_atoms);
  materialize_route_atoms(trajectories, route, output.raw_atoms);
  materialize_road_atom(trajectories, input, drivable, output.raw_atoms);
  materialize_comfort_atoms(trajectories, output.raw_atoms);

  if (!route.speed_observed) {
    output.status.at(3) = trajectory_ranker::CampAtomStatus::TypedMissing;
  }
  if (!drivable) {
    output.status.at(4) = trajectory_ranker::CampAtomStatus::TypedMissing;
  }
  output.status.at(6) = input.tensor_context.route_has_traffic_light
                          ? trajectory_ranker::CampAtomStatus::TypedMissing
                          : trajectory_ranker::CampAtomStatus::NotApplicable;
  output.status.at(7) = output.status.at(6);

  for (const auto & trajectory : trajectories) {
    output.candidate_world_plans.push_back(world_plan(trajectory, input.ego_to_map));
  }
  if (!materialize_transition_atom(
        input, output.candidate_world_plans, transition_component_scales, output.raw_atoms)) {
    output.status.at(15) = trajectory_ranker::CampAtomStatus::NotApplicable;
  }

  for (auto & candidate : output.raw_atoms) {
    for (std::size_t atom = 0; atom < candidate.size(); ++atom) {
      if (output.status.at(atom) != trajectory_ranker::CampAtomStatus::Observed) {
        candidate.at(atom) = std::numeric_limits<double>::quiet_NaN();
      } else if (!std::isfinite(candidate.at(atom)) || candidate.at(atom) < 0.0) {
        throw std::runtime_error("CAMP produced an invalid observed atom");
      }
    }
  }
  return output;
}

}  // namespace autoware::diffusion_planner
