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
#include <utility>
#include <vector>

namespace
{
constexpr double INVALID_SCORE = -1.0;

double getMahalanobisDistance(
  const geometry_msgs::msg::Point & measurement, const geometry_msgs::msg::Point & tracker,
  const Eigen::Matrix2d & covariance)
{
  // Compute difference directly without intermediate vectors
  const double dx = measurement.x - tracker.x;
  const double dy = measurement.y - tracker.y;

  // Pre-compute inverse elements (covariance is 2x2)
  const double det = covariance(0, 0) * covariance(1, 1) - covariance(0, 1) * covariance(1, 0);
  const double inv_det = 1.0 / det;
  const double inv00 = covariance(1, 1) * inv_det;
  const double inv01 = -covariance(0, 1) * inv_det;
  const double inv11 = covariance(0, 0) * inv_det;

  // Direct computation of (dx,dy) * inv_covariance * (dx,dy)^T
  return dx * (inv00 * dx + inv01 * dy) + dy * (inv01 * dx + inv11 * dy);
}

Eigen::Matrix2d getXYCovariance(const std::array<double, 36> & pose_covariance)
{
  Eigen::Matrix2d covariance;
  covariance << pose_covariance[0], pose_covariance[1], pose_covariance[6], pose_covariance[7];
  return covariance;
}
}  // namespace

namespace autoware::multi_object_tracker
{
using autoware_utils::ScopedTimeTrack;

DataAssociation::DataAssociation(const AssociatorConfig & config)
: config_(config), score_threshold_(0.0)
{
  // Initialize the GNN solver
  gnn_solver_ptr_ = std::make_unique<gnn_solver::MuSSP>();
  updateMaxSearchDistances();
}

void DataAssociation::setTimeKeeper(std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
}

void DataAssociation::updateMaxSearchDistances()
{
  const int num_classes = config_.max_dist_matrix.cols();
  max_squared_dist_per_class_.resize(num_classes);
  squared_distance_matrix_ = config_.max_dist_matrix;  // These are already squared distances

  // For each measurement class (column), find maximum squared distance with any tracker class
  for (int measurement_class = 0; measurement_class < num_classes; ++measurement_class) {
    double max_squared_dist = 0.0;
    for (int tracker_class = 0; tracker_class < config_.max_dist_matrix.rows(); ++tracker_class) {
      max_squared_dist =
        std::max(max_squared_dist, config_.max_dist_matrix(tracker_class, measurement_class));
    }
    max_squared_dist_per_class_[measurement_class] = max_squared_dist;
  }
}

void DataAssociation::assign(
  const Eigen::MatrixXd & src, std::unordered_map<int, int> & direct_assignment,
  std::unordered_map<int, int> & reverse_assignment)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

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
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Ensure that the detected_objects and list_tracker are not empty
  if (measurements.objects.empty() || trackers.empty()) {
    return Eigen::MatrixXd();
  }

  // Initialize the score matrix
  Eigen::MatrixXd score_matrix =
    Eigen::MatrixXd::Zero(trackers.size(), measurements.objects.size());

  // Clear previous tracker/measurement pair that shape significantly changed
  significant_shape_change_set_.clear();

  // Pre-allocate vectors to avoid reallocations
  rtree_.clear();
  std::vector<ValueType> nearby_trackers;
  nearby_trackers.reserve(std::min(size_t{100}, trackers.size()));  // Reasonable initial capacity

  std::vector<types::DynamicObject> tracked_objects;
  std::vector<std::uint8_t> tracker_labels;
  tracked_objects.reserve(trackers.size());
  tracker_labels.reserve(trackers.size());

  // Build R-tree and store tracker data
  {
    size_t tracker_idx = 0;
    std::vector<ValueType> rtree_points;
    rtree_points.reserve(trackers.size());
    for (const auto & tracker : trackers) {
      types::DynamicObject tracked_object;
      tracker->getTrackedObject(measurements.header.stamp, tracked_object);
      tracked_objects.push_back(tracked_object);
      tracker_labels.push_back(tracker->getHighestProbLabel());

      Point p(tracked_object.pose.position.x, tracked_object.pose.position.y);
      rtree_points.push_back(std::make_pair(p, tracker_idx));
      ++tracker_idx;
    }
    rtree_.insert(rtree_points.begin(), rtree_points.end());
  }

  // For each measurement, find nearby trackers using R-tree
  for (size_t measurement_idx = 0; measurement_idx < measurements.objects.size();
       ++measurement_idx) {
    const auto & measurement_object = measurements.objects.at(measurement_idx);
    const auto measurement_label =
      autoware::object_recognition_utils::getHighestProbLabel(measurement_object.classification);

    // Get pre-computed maximum squared distance for this measurement class
    const double max_squared_dist = max_squared_dist_per_class_[measurement_label];

    // Use circle query instead of box for more precise filtering
    Point measurement_point(measurement_object.pose.position.x, measurement_object.pose.position.y);
    nearby_trackers.clear();  // Reuse vector
    rtree_.query(
      bgi::satisfies([&](const ValueType & v) {
        const double dx = bg::get<0>(v.first) - measurement_object.pose.position.x;
        const double dy = bg::get<1>(v.first) - measurement_object.pose.position.y;
        return dx * dx + dy * dy <= max_squared_dist;
      }),
      std::back_inserter(nearby_trackers));

    // Process nearby trackers
    for (const auto & tracker_value : nearby_trackers) {
      const size_t tracker_idx = tracker_value.second;
      const auto & tracked_object = tracked_objects[tracker_idx];
      const auto tracker_label = tracker_labels[tracker_idx];

      bool significant_shape_change = false;

      // The actual distance check was already done in the R-tree query
      double score = calculateScore(
        tracked_object, tracker_label, measurement_object, measurement_label,
        significant_shape_change);
      score_matrix(tracker_idx, measurement_idx) = score;

      if (significant_shape_change) {
        significant_shape_change_set_.insert((uint64_t(tracker_idx) << 32) | measurement_idx);
      }
    }
  }

  return score_matrix;
}

double DataAssociation::calculateScore(
  const types::DynamicObject & tracked_object, const std::uint8_t tracker_label,
  const types::DynamicObject & measurement_object, const std::uint8_t measurement_label,
  bool & significant_shape_change) const
{
  if (!config_.can_assign_matrix(tracker_label, measurement_label)) {
    return INVALID_SCORE;
  }

  const double max_dist_sq = config_.max_dist_matrix(tracker_label, measurement_label);
  const double dx = measurement_object.pose.position.x - tracked_object.pose.position.x;
  const double dy = measurement_object.pose.position.y - tracked_object.pose.position.y;
  const double dist_sq = dx * dx + dy * dy;

  // dist gate
  if (dist_sq > max_dist_sq) return INVALID_SCORE;

  // area gate
  const double max_area = config_.max_area_matrix(tracker_label, measurement_label);
  const double min_area = config_.min_area_matrix(tracker_label, measurement_label);
  const double & area = measurement_object.area;
  if (area < min_area || area > max_area) return INVALID_SCORE;

  // mahalanobis dist gate
  const double mahalanobis_dist = getMahalanobisDistance(
    measurement_object.pose.position, tracked_object.pose.position,
    getXYCovariance(tracked_object.pose_covariance));
  constexpr double mahalanobis_dist_threshold =
    13.816;  // 99.99% confidence level for 2 degrees of freedom, chi-square critical value
  if (mahalanobis_dist >= mahalanobis_dist_threshold) return INVALID_SCORE;

  // return giou value over threshold as similarity score
  const double min_giou = config_.min_giou_matrix(tracker_label, measurement_label);
  double giou = shapes::get3dGeneralizedIoU(measurement_object, tracked_object);
  if (giou < min_giou) return INVALID_SCORE;

  // check if shape changes too much for vehicle labels
  constexpr double CheckGiouThreshold = 0.3;
  constexpr double AreaRatioThreshold = 1.5;
  constexpr uint8_t VehicleLabelMin = 1;  // CAR
  constexpr uint8_t VehicleLabelMax = 4;  // TRAILER

  if (
    giou < CheckGiouThreshold && tracker_label >= VehicleLabelMin &&
    tracker_label <= VehicleLabelMax) {
    // BEV‑area ratio
    const double area_trk = tracked_object.area;
    const double area_meas = measurement_object.area;
    const double area_ratio = std::max(area_trk, area_meas) / std::min(area_trk, area_meas);

    if (area_ratio > AreaRatioThreshold) {
      significant_shape_change = true;
    }
  }

  return giou;
}

bool DataAssociation::hasSignificantShapeChange(size_t tracker_idx, size_t measurement_idx) const
{
  uint64_t key = (uint64_t(tracker_idx) << 32) | measurement_idx;
  return significant_shape_change_set_.count(key);
}

}  // namespace autoware::multi_object_tracker
