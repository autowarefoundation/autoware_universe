// Copyright 2020 Tier IV, Inc.
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

#include "autoware/multi_object_tracker/association/association.hpp"

#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
#include "autoware/multi_object_tracker/object_model/shapes.hpp"
#include "autoware/multi_object_tracker/object_model/types.hpp"

#include <autoware/object_recognition_utils/object_recognition_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <algorithm>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

namespace
{
double getMahalanobisDistance(
  const geometry_msgs::msg::Point & measurement, const geometry_msgs::msg::Point & tracker,
  const Eigen::Matrix2d & covariance)
{
  Eigen::Vector2d measurement_point;
  measurement_point << measurement.x, measurement.y;
  Eigen::Vector2d tracker_point;
  tracker_point << tracker.x, tracker.y;
  Eigen::MatrixXd mahalanobis_squared = (measurement_point - tracker_point).transpose() *
                                        covariance.inverse() * (measurement_point - tracker_point);
  return std::sqrt(mahalanobis_squared(0));
}

Eigen::Matrix2d getXYCovariance(const std::array<double, 36> & pose_covariance)
{
  Eigen::Matrix2d covariance;
  covariance << pose_covariance[0], pose_covariance[1], pose_covariance[6], pose_covariance[7];
  return covariance;
}

double getFormedYawAngle(
  const geometry_msgs::msg::Quaternion & measurement_quat,
  const geometry_msgs::msg::Quaternion & tracker_quat, const bool distinguish_front_or_back = true)
{
  const double measurement_yaw = autoware_utils::normalize_radian(tf2::getYaw(measurement_quat));
  const double tracker_yaw = autoware_utils::normalize_radian(tf2::getYaw(tracker_quat));
  const double angle_range = distinguish_front_or_back ? M_PI : M_PI_2;
  const double angle_step = distinguish_front_or_back ? 2.0 * M_PI : M_PI;
  // Fixed measurement_yaw to be in the range of +-90 or 180 degrees of X_t(IDX::YAW)
  double measurement_fixed_yaw = measurement_yaw;
  while (angle_range <= tracker_yaw - measurement_fixed_yaw) {
    measurement_fixed_yaw = measurement_fixed_yaw + angle_step;
  }
  while (angle_range <= measurement_fixed_yaw - tracker_yaw) {
    measurement_fixed_yaw = measurement_fixed_yaw - angle_step;
  }
  return std::fabs(measurement_fixed_yaw - tracker_yaw);
}
}  // namespace

namespace autoware::multi_object_tracker
{

DataAssociation::DataAssociation(const AssociatorConfig & config)
: config_(config), score_threshold_(0.01)
{
  // Initialize the GNN solver
  gnn_solver_ptr_ = std::make_unique<gnn_solver::MuSSP>();
}

void DataAssociation::assign(
  const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment)
{
  std::vector<std::vector<double>> score(src.rows());
  for (int row = 0; row < src.rows(); ++row) {
    score.at(row).resize(src.cols());
    for (int col = 0; col < src.cols(); ++col) {
      score.at(row).at(col) = src(row, col);
    }
  }
  // Solve
  gnn_solver_ptr_->maximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  for (auto itr = direct_assignment.begin(); itr != direct_assignment.end();) {
    if (src(itr->first, itr->second) < score_threshold_) {
      itr = direct_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
  for (auto itr = reverse_assignment.begin(); itr != reverse_assignment.end();) {
    if (src(itr->second, itr->first) < score_threshold_) {
      itr = reverse_assignment.erase(itr);
      continue;
    } else {
      ++itr;
    }
  }
}

Eigen::MatrixXd DataAssociation::calcScoreMatrix(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  // Ensure that the detected_objects and list_tracker are not empty
  if (measurements.objects.empty() || trackers.empty()) {
    return Eigen::MatrixXd();
  }
  // Initialize the score matrix
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(trackers.size(), measurements.objects.size());

  size_t tracker_idx = 0;
  for (auto tracker_itr = trackers.begin(); tracker_itr != trackers.end();
       ++tracker_itr, ++tracker_idx) {
    types::DynamicObject tracked_object;
    (*tracker_itr)->getTrackedObject(measurements.header.stamp, tracked_object);
    const std::uint8_t tracker_label = (*tracker_itr)->getHighestProbLabel();

    for (size_t measurement_idx = 0; measurement_idx < measurements.objects.size();
         ++measurement_idx) {
      const types::DynamicObject & measurement_object = measurements.objects.at(measurement_idx);
      const std::uint8_t measurement_label =
        autoware::object_recognition_utils::getHighestProbLabel(measurement_object.classification);

      double score =
        calculateScore(tracked_object, tracker_label, measurement_object, measurement_label);

      score_matrix(tracker_idx, measurement_idx) = score;
    }
  }

  return score_matrix;
}

double DataAssociation::calculateScore(
  const types::DynamicObject & tracked_object, const std::uint8_t tracker_label,
  const types::DynamicObject & measurement_object, const std::uint8_t measurement_label) const
{
  if (!config_.can_assign_matrix(tracker_label, measurement_label)) {
    return 0.0;
  }

  const double max_dist = config_.max_dist_matrix(tracker_label, measurement_label);
  const double dist =
    autoware_utils::calc_distance2d(measurement_object.pose.position, tracked_object.pose.position);

  // dist gate
  if (max_dist < dist) return 0.0;

  // area gate
  const double max_area = config_.max_area_matrix(tracker_label, measurement_label);
  const double min_area = config_.min_area_matrix(tracker_label, measurement_label);
  const double area = autoware_utils::get_area(measurement_object.shape);
  if (area < min_area || max_area < area) return 0.0;

  // angle gate
  const double max_rad = config_.max_rad_matrix(tracker_label, measurement_label);
  const double angle =
    getFormedYawAngle(measurement_object.pose.orientation, tracked_object.pose.orientation, false);
  if (std::fabs(max_rad) < M_PI && std::fabs(max_rad) < std::fabs(angle)) {
    return 0.0;
  }

  // mahalanobis dist gate
  const double mahalanobis_dist = getMahalanobisDistance(
    measurement_object.pose.position, tracked_object.pose.position,
    getXYCovariance(tracked_object.pose_covariance));
  constexpr double mahalanobis_dist_threshold =
    3.717;  // 99.99% confidence level for 2 degrees of freedom, square root of chi-square critical
            // value of 13.816
  if (mahalanobis_dist_threshold <= mahalanobis_dist) return 0.0;

  // 2d iou gate
  const double min_iou = config_.min_iou_matrix(tracker_label, measurement_label);
  const double min_union_iou_area = 1e-2;
  const double iou = shapes::get2dIoU(measurement_object, tracked_object, min_union_iou_area);
  if (iou < min_iou) return 0.0;

  // all gate is passed
  double score = (max_dist - std::min(dist, max_dist)) / max_dist;
  if (score < score_threshold_) score = 0.0;
  return score;
}

}  // namespace autoware::multi_object_tracker
