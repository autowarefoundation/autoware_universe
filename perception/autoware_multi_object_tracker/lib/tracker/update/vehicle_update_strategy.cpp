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

#include "autoware/multi_object_tracker/tracker/update/vehicle_update_strategy.hpp"

#include <autoware_utils_geometry/msg/covariance.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <cmath>

namespace autoware::multi_object_tracker
{

namespace
{

constexpr double ALIGNMENT_RATIO_THRESHOLD = 0.2;     // 20% of the larger object's length
constexpr double ALIGNMENT_ABSOLUTE_THRESHOLD = 1.0;  // [m] minimum tolerance for small objects

struct EdgePositions
{
  double front_x, front_y;
  double rear_x, rear_y;
};

enum class Edge { FRONT, REAR };

struct EdgeAlignment
{
  double min_alignment_distance;
  Edge aligned_pred_edge;
  Edge aligned_meas_edge;
};

EdgePositions calculateEdgeCenters(const types::DynamicObject & obj)
{
  const double yaw = tf2::getYaw(obj.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double half_length = obj.shape.dimensions.x * 0.5;

  return {
    obj.pose.position.x + half_length * cos_yaw,  // front_x
    obj.pose.position.y + half_length * sin_yaw,  // front_y
    obj.pose.position.x - half_length * cos_yaw,  // rear_x
    obj.pose.position.y - half_length * sin_yaw   // rear_y
  };
}

EdgeAlignment findAlignedEdges(
  const EdgePositions & meas_edges, const types::DynamicObject & prediction)
{
  const double pred_yaw = tf2::getYaw(prediction.pose.orientation);
  const double pred_cos_yaw = std::cos(pred_yaw);
  const double pred_sin_yaw = std::sin(pred_yaw);

  const auto project_to_axis = [pred_cos_yaw, pred_sin_yaw](double x, double y) {
    return x * pred_cos_yaw + y * pred_sin_yaw;
  };

  const double meas_front_axis = project_to_axis(meas_edges.front_x, meas_edges.front_y);
  const double meas_rear_axis = project_to_axis(meas_edges.rear_x, meas_edges.rear_y);

  const double pred_center_axis =
    prediction.pose.position.x * pred_cos_yaw + prediction.pose.position.y * pred_sin_yaw;
  const double predicted_half_length = prediction.shape.dimensions.x * 0.5;
  const double pred_front_axis = pred_center_axis + predicted_half_length;
  const double pred_rear_axis = pred_center_axis - predicted_half_length;

  struct Candidate
  {
    double distance;
    Edge pred_edge;
    Edge meas_edge;
  };
  const std::array<Candidate, 4> candidates = {
    {{std::abs(meas_front_axis - pred_front_axis), Edge::FRONT, Edge::FRONT},
     {std::abs(meas_rear_axis - pred_front_axis), Edge::FRONT, Edge::REAR},
     {std::abs(meas_front_axis - pred_rear_axis), Edge::REAR, Edge::FRONT},
     {std::abs(meas_rear_axis - pred_rear_axis), Edge::REAR, Edge::REAR}}};

  const auto best = std::min_element(
    candidates.begin(), candidates.end(),
    [](const Candidate & a, const Candidate & b) { return a.distance < b.distance; });

  return {best->distance, best->pred_edge, best->meas_edge};
}

geometry_msgs::msg::Point calculateAnchorPoint(
  const EdgeAlignment & alignment, const types::DynamicObject & measurement)
{
  geometry_msgs::msg::Point anchor_point;

  const double meas_yaw = tf2::getYaw(measurement.pose.orientation);
  const double meas_cos_yaw = std::cos(meas_yaw);
  const double meas_sin_yaw = std::sin(meas_yaw);
  const double meas_half_length = measurement.shape.dimensions.x * 0.5;

  if (alignment.aligned_meas_edge == Edge::FRONT) {
    anchor_point.x = measurement.pose.position.x + meas_half_length * meas_cos_yaw;
    anchor_point.y = measurement.pose.position.y + meas_half_length * meas_sin_yaw;
  } else {
    anchor_point.x = measurement.pose.position.x - meas_half_length * meas_cos_yaw;
    anchor_point.y = measurement.pose.position.y - meas_half_length * meas_sin_yaw;
  }

  return anchor_point;
}

}  // namespace

UpdateStrategy determineUpdateStrategy(
  const types::DynamicObject & measurement, const types::DynamicObject & prediction)
{
  UpdateStrategy strategy;

  const EdgePositions meas_edges = calculateEdgeCenters(measurement);
  const EdgeAlignment alignment = findAlignedEdges(meas_edges, prediction);

  const double predicted_length = prediction.shape.dimensions.x;
  const double measured_length = measurement.shape.dimensions.x;
  const double max_length = std::max(predicted_length, measured_length);
  const double alignment_threshold =
    std::max(ALIGNMENT_RATIO_THRESHOLD * max_length, ALIGNMENT_ABSOLUTE_THRESHOLD);
  const bool is_edge_aligned = alignment.min_alignment_distance < alignment_threshold;

  if (!is_edge_aligned) {
    strategy.type = UpdateStrategyType::WEAK_UPDATE;
    return strategy;
  }

  strategy.type = (alignment.aligned_pred_edge == Edge::FRONT)
                    ? UpdateStrategyType::FRONT_WHEEL_UPDATE
                    : UpdateStrategyType::REAR_WHEEL_UPDATE;
  strategy.anchor_point = calculateAnchorPoint(alignment, measurement);

  return strategy;
}

types::DynamicObject createPseudoMeasurement(
  const types::DynamicObject & meas, const types::DynamicObject & prediction,
  const bool enlarge_covariance)
{
  types::DynamicObject pred = prediction;

  // Apply linear fall‑off weight on dist square
  const double dx = meas.pose.position.x - pred.pose.position.x;
  const double dy = meas.pose.position.y - pred.pose.position.y;
  const double dist2 = dx * dx + dy * dy;
  constexpr double d_max_square_inv = 1 / 2.0;  // saturate when distance overs 1.414 m
  constexpr double min_w = 0.0;
  const double w_pose = std::clamp(1.0 - dist2 * d_max_square_inv, min_w, 1.0);

  // Blend position (x, y, z)
  pred.pose.position.x = pred.pose.position.x * (1 - w_pose) + meas.pose.position.x * w_pose;
  pred.pose.position.y = pred.pose.position.y * (1 - w_pose) + meas.pose.position.y * w_pose;
  pred.pose.position.z = pred.pose.position.z * (1 - w_pose) + meas.pose.position.z * w_pose;

  // Refresh the area from pred's (tracker) shape
  pred.area = types::getArea(pred.shape);

  // Blend orientation
  if (meas.kinematics.orientation_availability != types::OrientationAvailability::UNAVAILABLE) {
    double yaw_pred = tf2::getYaw(pred.pose.orientation);
    double yaw_meas = tf2::getYaw(meas.pose.orientation);

    double yaw_diff = autoware_utils_math::normalize_radian(yaw_meas - yaw_pred);
    // Handle SIGN_UNKNOWN: limit yaw difference to [-90°, 90°]
    if (meas.kinematics.orientation_availability == types::OrientationAvailability::SIGN_UNKNOWN) {
      if (yaw_diff > M_PI_2) {
        yaw_diff -= M_PI;
      } else if (yaw_diff < -M_PI_2) {
        yaw_diff += M_PI;
      }
    }
    double yaw_fused = yaw_pred + yaw_diff * w_pose;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_fused);
    pred.pose.orientation = tf2::toMsg(q);
  }

  // Enlarge covariance if requested (for weak updates)
  if (enlarge_covariance) {
    using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;
    constexpr double additional_position_cov = 9.0;     // [m^2] additional variance
    constexpr double additional_orientation_cov = 0.5;  // [rad^2] additional variance
    constexpr double additional_velocity_cov = 25.0;    // [m^2/s^2] additional variance

    pred.pose_covariance[XYZRPY_COV_IDX::X_X] += additional_position_cov;
    pred.pose_covariance[XYZRPY_COV_IDX::Y_Y] += additional_position_cov;
    pred.pose_covariance[XYZRPY_COV_IDX::YAW_YAW] += additional_orientation_cov;

    // Enlarge velocity covariance if available
    if (pred.kinematics.has_twist_covariance) {
      pred.twist_covariance[XYZRPY_COV_IDX::X_X] += additional_velocity_cov;
      pred.twist_covariance[XYZRPY_COV_IDX::Y_Y] += additional_velocity_cov;
    }
  }

  return pred;
}

double correctWheelAnchorLateral(
  const double anchor_lateral, const double tracker_half, const double polygon_half,
  double & var_lat)
{
  // The true center lies within +/-half_dead_zone of the anchor; outside it, the closer polygon
  // edge is taken as a real vehicle edge.
  const double half_dead_zone = std::abs(polygon_half - tracker_half);
  const double low = anchor_lateral - half_dead_zone;
  const double high = anchor_lateral + half_dead_zone;

  // Project the tracker center (0) into the dead-zone.
  const double target = std::clamp(0.0, low, high);

  // Variance = (dead-zone half-width)^2: the std equals the half-width by which the center may
  // move.
  var_lat = half_dead_zone * half_dead_zone;

  return target - anchor_lateral;
}

geometry_msgs::msg::Point correctWheelAnchor(
  const types::DynamicObject & prediction, const double polygon_width,
  const geometry_msgs::msg::Point & anchor, std::array<double, 36> & pose_cov)
{
  using autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

  const double yaw = tf2::getYaw(prediction.pose.orientation);
  const double tracker_width = prediction.shape.dimensions.y;
  const geometry_msgs::msg::Point & tracker_center = prediction.pose.position;

  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  // Project the observed anchor offset from the tracker center onto the body lateral axis
  // n = (-sin yaw, cos yaw). The longitudinal component is orthogonal to n and drops.
  const double lateral_offset =
    -(anchor.x - tracker_center.x) * sin_yaw + (anchor.y - tracker_center.y) * cos_yaw;

  const double tracker_half = tracker_width * 0.5;
  const double polygon_half = polygon_width * 0.5;
  double var_lat = 0.0;
  const double lateral_move =
    correctWheelAnchorLateral(lateral_offset, tracker_half, polygon_half, var_lat);

  // Apply the scalar lateral move back along n to get the corrected anchor point.
  geometry_msgs::msg::Point corrected = anchor;
  corrected.x = anchor.x - lateral_move * sin_yaw;
  corrected.y = anchor.y + lateral_move * cos_yaw;

  // add var_lat * n * n^T to the x/y block
  pose_cov[XYZRPY_COV_IDX::X_X] += var_lat * sin_yaw * sin_yaw;
  pose_cov[XYZRPY_COV_IDX::X_Y] += -var_lat * sin_yaw * cos_yaw;
  pose_cov[XYZRPY_COV_IDX::Y_X] += -var_lat * sin_yaw * cos_yaw;
  pose_cov[XYZRPY_COV_IDX::Y_Y] += var_lat * cos_yaw * cos_yaw;

  return corrected;
}

}  // namespace autoware::multi_object_tracker
