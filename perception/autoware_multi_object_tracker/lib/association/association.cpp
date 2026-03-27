// Copyright 2020 TIER IV, Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
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

struct MeasurementWithIndex
{
  const types::DynamicObject & object;
  size_t index;

  MeasurementWithIndex(const types::DynamicObject & obj, size_t idx) : object(obj), index(idx) {}
};
using autoware_utils_debug::ScopedTimeTrack;

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
  max_squared_dist_per_class_.clear();
  for (const auto measurement_label : classes::trackedLabels()) {
    double max_squared_dist = 0.0;
    const auto tracker_params_map_opt =
      get_map_value_if_exists(config_.association_params_map, measurement_label);
    if (!tracker_params_map_opt) {
      continue;
    }
    const auto & tracker_params_map = tracker_params_map_opt->get();
    for (const auto & [tracker_type, association_params] : tracker_params_map) {
      static_cast<void>(tracker_type);
      max_squared_dist = std::max(max_squared_dist, association_params.max_dist_sq);
    }
    max_squared_dist_per_class_.insert_or_assign(measurement_label, max_squared_dist);
  }
}

void DataAssociation::assign(
  const types::AssociationData & data, types::AssociationResult & association_result)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;

  std::vector<std::vector<double>> score = formatScoreMatrix(data);

  // Solve
  gnn_solver_ptr_->maximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  // Build a map from (tracker_idx, measurement_idx) to entry info for efficient shape change lookup
  std::unordered_map<int, std::unordered_map<int, const types::AssociationEntry *>> entry_map;
  for (const auto & entry : data.entries) {
    if (entry.has_significant_shape_change && entry.score >= score_threshold_) {
      entry_map[static_cast<int>(entry.tracker_idx)][static_cast<int>(entry.measurement_idx)] =
        &entry;
    }
  }

  // Pre-allocate capacity for unassigned vectors
  association_result.unassigned_trackers.reserve(data.tracker_uuids.size());
  association_result.unassigned_measurements.reserve(data.measurement_uuids.size());

  // Process assignments and shape changes in a single loop
  for (const auto & [tracker_idx, measurement_idx] : direct_assignment) {
    if (score[tracker_idx][measurement_idx] >= score_threshold_) {
      association_result.add(
        data.tracker_uuids[tracker_idx], data.measurement_uuids[measurement_idx]);

      // Check for shape change using the pre-built entry map
      auto tracker_it = entry_map.find(tracker_idx);
      if (tracker_it != entry_map.end()) {
        auto measurement_it = tracker_it->second.find(measurement_idx);
        if (measurement_it != tracker_it->second.end()) {
          association_result.trackers_with_shape_change.insert(data.tracker_uuids[tracker_idx]);
        }
      }
    }
  }

  // Fill unassigned trackers using direct_assignment map (faster than UUID map lookup)
  for (size_t i = 0; i < data.tracker_uuids.size(); ++i) {
    auto it = direct_assignment.find(static_cast<int>(i));
    if (
      it == direct_assignment.end() || score[static_cast<int>(i)][it->second] < score_threshold_) {
      association_result.unassigned_trackers.emplace_back(data.tracker_uuids[i]);
    }
  }

  // Fill unassigned measurements using reverse_assignment map (faster than UUID map lookup)
  for (size_t i = 0; i < data.measurement_uuids.size(); ++i) {
    auto it = reverse_assignment.find(static_cast<int>(i));
    if (
      it == reverse_assignment.end() || score[it->second][static_cast<int>(i)] < score_threshold_) {
      association_result.unassigned_measurements.emplace_back(data.measurement_uuids[i]);
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
  InverseCovariance2D result;

  // Guard against invalid / non-PSD covariance (or extreme off-diagonal terms).
  // In that case, fall back to a diagonal inverse (still bounded by minimum_cov).
  constexpr double min_det = 1e-12;
  if (!(std::isfinite(det)) || det <= min_det) {
    result.inv00 = 1.0 / a;
    result.inv01 = 0.0;
    result.inv11 = 1.0 / d;
    return result;
  }

  const double inv_det = 1.0 / det;
  result.inv00 = d * inv_det;   // d / det
  result.inv01 = -b * inv_det;  // -b / det
  result.inv11 = a * inv_det;   // a / det
  return result;
}

PreparationData DataAssociation::prepareAssociationData(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  PreparationData prep_data;

  // Pre-allocate vectors to avoid reallocations
  prep_data.tracked_objects.reserve(trackers.size());
  prep_data.tracker_labels.reserve(trackers.size());
  prep_data.tracker_types.reserve(trackers.size());

  // Build R-tree and store tracker data
  {
    size_t tracker_idx = 0;
    std::vector<ValueType> rtree_points;
    rtree_.clear();
    rtree_points.reserve(trackers.size());
    for (const auto & tracker : trackers) {
      types::DynamicObject tracked_object;
      tracker->getTrackedObject(measurements.header.stamp, tracked_object);

      Point p(tracked_object.pose.position.x, tracked_object.pose.position.y);
      rtree_points.emplace_back(p, tracker_idx);

      prep_data.tracked_objects.emplace_back(std::move(tracked_object));
      prep_data.tracker_labels.emplace_back(tracker->getHighestProbLabel());
      prep_data.tracker_types.emplace_back(tracker->getTrackerType());
      ++tracker_idx;
    }
    rtree_.insert(rtree_points.begin(), rtree_points.end());
  }

  // Pre-compute inverse covariance for each tracker
  prep_data.tracker_inverse_covariances.reserve(prep_data.tracked_objects.size());
  for (const auto & tracked_object : prep_data.tracked_objects) {
    prep_data.tracker_inverse_covariances.emplace_back(
      precomputeInverseCovarianceFromPose(tracked_object.pose_covariance));
  }

  return prep_data;
}

void DataAssociation::processMeasurement(
  const types::DynamicObject & measurement_object, size_t measurement_idx,
  const classes::Label measurement_label, const PreparationData & prep_data,
  types::AssociationData & association_data)
{
  const auto tracker_params_map_opt =
    get_map_value_if_exists(config_.association_params_map, measurement_label);
  if (!tracker_params_map_opt) {
    return;
  }
  const auto & tracker_params_map = tracker_params_map_opt->get();

  // Get pre-computed maximum squared distance for this measurement class
  const auto max_squared_dist_opt =
    get_map_value_if_exists(max_squared_dist_per_class_, measurement_label);
  const double max_squared_dist = max_squared_dist_opt ? max_squared_dist_opt->get() : 0.0;

  // Use circle query instead of box for more precise filtering
  Point measurement_point(measurement_object.pose.position.x, measurement_object.pose.position.y);

  std::vector<ValueType> nearby_trackers;
  nearby_trackers.reserve(
    std::min(size_t{100}, prep_data.tracked_objects.size()));  // Reasonable initial capacity

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
    const auto tracker_type = prep_data.tracker_types[tracker_idx];

    const auto association_params_opt = get_map_value_if_exists(tracker_params_map, tracker_type);
    if (!association_params_opt) continue;

    // Calculate score for this tracker-measurement pair
    const auto & tracked_object = prep_data.tracked_objects[tracker_idx];
    const auto tracker_label = prep_data.tracker_labels[tracker_idx];

    bool has_significant_shape_change = false;
    double score = calculateScore(
      tracked_object, tracker_label, tracker_type, association_params_opt->get(),
      measurement_object, measurement_label, prep_data.tracker_inverse_covariances[tracker_idx],
      has_significant_shape_change);

    if (score > INVALID_SCORE) {
      association_data.entries.emplace_back(
        types::AssociationEntry{tracker_idx, measurement_idx, score, has_significant_shape_change});
    }
  }
}

types::AssociationData DataAssociation::calcAssociationData(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  // Ensure that the detected_objects and list_tracker are not empty
  if (measurements.objects.empty() || trackers.empty()) {
    return types::AssociationData{};
  }

  // Preparation stage: build R-tree and pre-compute data
  auto prep_data = prepareAssociationData(measurements, trackers);

  // Initialize association data with UUIDs
  types::AssociationData association_data;
  association_data.tracker_uuids.reserve(trackers.size());
  association_data.measurement_uuids.reserve(measurements.objects.size());

  for (const auto & object : measurements.objects) {
    association_data.measurement_uuids.emplace_back(object.uuid);
  }

  for (const auto & tracker : trackers) {
    association_data.tracker_uuids.emplace_back(tracker->getUUID());
  }

  // Processing stage: process each measurement
  for (auto it = measurements.objects.begin(); it != measurements.objects.end(); ++it) {
    const size_t measurement_idx = std::distance(measurements.objects.begin(), it);
    const MeasurementWithIndex measurement_with_idx(*it, measurement_idx);

    const auto measurement_label =
      classes::getHighestProbLabel(measurement_with_idx.object.classification);

    processMeasurement(
      measurement_with_idx.object, measurement_with_idx.index, measurement_label, prep_data,
      association_data);
  }

  return association_data;
}

std::vector<std::vector<double>> DataAssociation::formatScoreMatrix(
  const types::AssociationData & data) const
{
  std::vector<std::vector<double>> score_matrix(
    data.tracker_uuids.size(), std::vector<double>(data.measurement_uuids.size(), 0.0));
  for (const auto & entry : data.entries) {
    score_matrix[entry.tracker_idx][entry.measurement_idx] = entry.score;
  }
  return score_matrix;
}

double DataAssociation::calculateScore(
  const types::DynamicObject & tracked_object, const classes::Label tracker_label,
  const types::TrackerType tracker_type,
  const AssociatorConfig::TrackerAssociationParameters & association_params,
  const types::DynamicObject & measurement_object, const classes::Label measurement_label,
  const InverseCovariance2D & inv_cov, bool & has_significant_shape_change) const
{
  // when the tracker and measurements are unknown, use generalized IoU
  if (tracker_label == classes::Label::UNKNOWN && measurement_label == classes::Label::UNKNOWN) {
    const double & generalized_iou_threshold = config_.unknown_association_giou_threshold;
    const double generalized_iou = shapes::get2dGeneralizedIoU(tracked_object, measurement_object);
    if (generalized_iou < generalized_iou_threshold) {
      return INVALID_SCORE;
    }
    // rescale score to [0, 1]
    return (generalized_iou - generalized_iou_threshold) / (1.0 - generalized_iou_threshold);
  }

  const double max_dist_sq = association_params.max_dist_sq;
  const double dx = measurement_object.pose.position.x - tracked_object.pose.position.x;
  const double dy = measurement_object.pose.position.y - tracked_object.pose.position.y;
  const double dist_sq = dx * dx + dy * dy;

  // dist gate
  if (dist_sq > max_dist_sq) return INVALID_SCORE;

  // gates for non-vehicle objects
  const double area_meas = measurement_object.area;
  const bool is_vehicle_tracker = isVehicleTrackerType(tracker_type);
  if (!is_vehicle_tracker) {
    // area gate
    const double max_area = association_params.max_area;
    const double min_area = association_params.min_area;
    if (area_meas < min_area || area_meas > max_area) return INVALID_SCORE;

    // mahalanobis dist gate
    const double mahalanobis_dist = getMahalanobisDistanceFast(dx, dy, inv_cov);

    constexpr double mahalanobis_dist_threshold =
      11.62;  // This is an empirical value corresponding to the 99.6% confidence level
              // for a chi-square distribution with 2 degrees of freedom (critical value).

    if (mahalanobis_dist >= mahalanobis_dist_threshold) return INVALID_SCORE;
  }

  const double min_iou = association_params.min_iou;

  // use 1d iou for pedestrian, 3d giou for other objects if both extensions are trustable
  // otherwise use 2d giou
  const bool use_1d_iou = (tracker_label == classes::Label::PEDESTRIAN);
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
