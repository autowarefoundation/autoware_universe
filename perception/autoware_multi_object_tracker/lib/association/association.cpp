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

#include <algorithm>
#include <array>
#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
constexpr double INVALID_SCORE = 0.0;
}  // namespace

namespace autoware::multi_object_tracker
{
using autoware_utils_debug::ScopedTimeTrack;
using Label = autoware_perception_msgs::msg::ObjectClassification;

DataAssociation::DataAssociation(const AssociatorConfig & config)
: config_(config), score_threshold_(0.01)
{
  // Initialize the GNN solver
  gnn_solver_ptr_ = std::make_unique<gnn_solver::MuSSP>();
  updateMaxSearchDistances();
}

void DataAssociation::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
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

inline double getMahalanobisDistanceFast(double dx, double dy, const InverseCovariance2D & inv_cov)
{
  return dx * dx * inv_cov.inv00 + 2.0 * dx * dy * inv_cov.inv01 + dy * dy * inv_cov.inv11;
}

// Directly computes inverse covariance from pose_covariance array
inline InverseCovariance2D precomputeInverseCovarianceFromPose(
  const std::array<double, 36> & pose_covariance)
{
  // Step 1: Extract a, b, d directly from pose_covariance (no temporary Matrix2d)
  constexpr double minimum_cov = 0.25;  // 0.5 m to avoid too large mahalanobis distance
  const double a = std::max(pose_covariance[0], minimum_cov);  // cov(0,0)
  const double b = pose_covariance[1];  // cov(0,1) == pose_covariance[6] (symmetry)
  const double d = std::max(pose_covariance[7], minimum_cov);  // cov(1,1)

  // Step 2: Compute determinant and inverse components in one pass
  const double det = a * d - b * b;
  const double inv_det = 1.0 / det;

  InverseCovariance2D result;
  result.inv00 = d * inv_det;   // d / det
  result.inv01 = -b * inv_det;  // -b / det
  result.inv11 = a * inv_det;   // a / det
  return result;
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
  significant_shape_change_checker_.clear();

  // Pre-allocate vectors to avoid reallocations
  std::vector<types::DynamicObject> tracked_objects;
  std::vector<std::uint8_t> tracker_labels;
  std::vector<TrackerType> tracker_types;
  tracked_objects.reserve(trackers.size());
  tracker_labels.reserve(trackers.size());
  tracker_types.reserve(trackers.size());
  // Build R-tree and store tracker data
  {
    size_t tracker_idx = 0;
    std::vector<ValueType> rtree_points;
    rtree_.clear();
    rtree_points.reserve(trackers.size());
    for (const auto & tracker : trackers) {
      types::DynamicObject tracked_object;
      tracker->getTrackedObject(measurements.header.stamp, tracked_object);
      tracked_objects.push_back(tracked_object);
      tracker_labels.push_back(tracker->getHighestProbLabel());
      tracker_types.push_back(tracker->getTrackerType());

      Point p(tracked_object.pose.position.x, tracked_object.pose.position.y);
      rtree_points.push_back(std::make_pair(p, tracker_idx));
      ++tracker_idx;
    }
    rtree_.insert(rtree_points.begin(), rtree_points.end());
  }

  // Pre-compute inverse covariance for each tracker
  std::vector<InverseCovariance2D> tracker_inverse_covariances;
  tracker_inverse_covariances.reserve(tracked_objects.size());
  for (const auto & tracked_object : tracked_objects) {
    tracker_inverse_covariances.push_back(
      precomputeInverseCovarianceFromPose(tracked_object.pose_covariance));
  }

  // For each measurement, find nearby trackers using R-tree

  for (size_t measurement_idx = 0; measurement_idx < measurements.objects.size();
       ++measurement_idx) {
    const auto & measurement_object = measurements.objects[measurement_idx];
    const auto measurement_label =
      autoware::object_recognition_utils::getHighestProbLabel(measurement_object.classification);
    if (measurement_label >= types::NUM_LABELS) {
      RCLCPP_WARN(
        rclcpp::get_logger("DataAssociation"),
        "Measurement label %d is out of range. Skipping association.",
        static_cast<int>(measurement_label));
      continue;
    }

    // Get pre-computed maximum squared distance for this measurement class
    const double max_squared_dist = max_squared_dist_per_class_[measurement_label];

    // Use circle query instead of box for more precise filtering
    Point measurement_point(measurement_object.pose.position.x, measurement_object.pose.position.y);

    std::vector<ValueType> nearby_trackers;
    nearby_trackers.reserve(std::min(size_t{100}, trackers.size()));  // Reasonable initial capacity

    // Compute search bounding box (square that contains the circle)
    const double max_dist = std::sqrt(max_squared_dist);
    const Box query_box(
      Point(measurement_point.get<0>() - max_dist, measurement_point.get<1>() - max_dist),
      Point(measurement_point.get<0>() + max_dist, measurement_point.get<1>() + max_dist));
    // Initial R-tree box query
    rtree_.query(bgi::within(query_box), std::back_inserter(nearby_trackers));

    // Process nearby trackers
    for (const auto & tracker_value : nearby_trackers) {
      const size_t tracker_idx = tracker_value.second;
      const auto tracker_type = tracker_types[tracker_idx];

      // Check if this tracker can be assigned to the measurement
      bool can_assign =
        config_.can_assign_map.at(tracker_type)[static_cast<int>(measurement_label)];
      if (!can_assign) continue;

      // Calculate score for this tracker-measurement pair
      const auto & tracked_object = tracked_objects[tracker_idx];
      const auto tracker_label = tracker_labels[tracker_idx];

      bool has_significant_shape_change = false;
      double score = calculateScore(
        tracked_object, tracker_label, measurement_object, measurement_label,
        tracker_inverse_covariances[tracker_idx], has_significant_shape_change);
      score_matrix(tracker_idx, measurement_idx) = score;

      if (has_significant_shape_change) {
        significant_shape_change_checker_.addPair(tracker_idx, measurement_idx);
      }
    }
  }

  return score_matrix;
}

double DataAssociation::calculateScore(
  const types::DynamicObject & tracked_object, const std::uint8_t tracker_label,
  const types::DynamicObject & measurement_object, const std::uint8_t measurement_label,
  const InverseCovariance2D & inv_cov, bool & has_significant_shape_change) const
{
  // when the tracker and measurements are unknown, use generalized IoU
  if (tracker_label == Label::UNKNOWN && measurement_label == Label::UNKNOWN) {
    const double & generalized_iou_threshold = config_.unknown_association_giou_threshold;
    const double generalized_iou = shapes::get2dGeneralizedIoU(tracked_object, measurement_object);
    if (generalized_iou < generalized_iou_threshold) {
      return INVALID_SCORE;
    }
    // rescale score to [0, 1]
    return (generalized_iou - generalized_iou_threshold) / (1.0 - generalized_iou_threshold);
  }

  const double max_dist_sq = config_.max_dist_matrix(tracker_label, measurement_label);
  const double dx = measurement_object.pose.position.x - tracked_object.pose.position.x;
  const double dy = measurement_object.pose.position.y - tracked_object.pose.position.y;
  const double dist_sq = dx * dx + dy * dy;

  // dist gate
  if (dist_sq > max_dist_sq) return INVALID_SCORE;

  // gates for non-vehicle objects
  const double area_meas = measurement_object.area;
  const bool is_vehicle_tracker = tracker_label == Label::CAR || tracker_label == Label::BUS ||
                                  tracker_label == Label::TRUCK || tracker_label == Label::TRAILER;
  if (!is_vehicle_tracker) {
    // area gate
    const double max_area = config_.max_area_matrix(tracker_label, measurement_label);
    const double min_area = config_.min_area_matrix(tracker_label, measurement_label);
    if (area_meas < min_area || area_meas > max_area) return INVALID_SCORE;

    // mahalanobis dist gate
    const double mahalanobis_dist = getMahalanobisDistanceFast(dx, dy, inv_cov);

    constexpr double mahalanobis_dist_threshold =
      11.62;  // This is an empirical value corresponding to the 99.6% confidence level
              // for a chi-square distribution with 2 degrees of freedom (critical value).

    if (mahalanobis_dist >= mahalanobis_dist_threshold) return INVALID_SCORE;
  }

  const double min_iou = config_.min_iou_matrix(tracker_label, measurement_label);

  // use 1d iou for pedestrian, 3d giou for other objects if both extensions are trustable
  // otherwise use 2d giou
  const bool use_1d_iou = (tracker_label == Label::PEDESTRIAN);
  const bool use_3d_iou = (tracked_object.trust_extension) && (measurement_object.trust_extension);

  double iou_score;
  if (use_1d_iou) {
    iou_score = shapes::get1dIoU(measurement_object, tracked_object);
  } else if (use_3d_iou) {
    iou_score = shapes::get3dGeneralizedIoU(measurement_object, tracked_object);
  } else {
    iou_score = shapes::get2dGeneralizedIoU(measurement_object, tracked_object);
  }
  if (iou_score < min_iou) return INVALID_SCORE;

  // check if shape changes too much for vehicle labels
  if (iou_score < CHECK_GIOU_THRESHOLD && is_vehicle_tracker) {
    // BEV‑area ratio
    const double area_trk = tracked_object.area;
    const double area_ratio = std::max(area_trk, area_meas) / std::min(area_trk, area_meas);

    if (area_ratio > AREA_RATIO_THRESHOLD) {
      has_significant_shape_change = true;
    }
  }

  // rescale score to [0, 1]
  return (iou_score - min_iou) / (1.0 - min_iou);
}

}  // namespace autoware::multi_object_tracker
