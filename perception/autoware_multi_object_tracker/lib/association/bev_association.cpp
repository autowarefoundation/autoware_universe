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

#include "autoware/multi_object_tracker/association/bev_association.hpp"

#include "autoware/multi_object_tracker/association/scoring/bev_assignment_scoring.hpp"
#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <list>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace
{
constexpr double INVALID_SCORE = 0.0;
}  // namespace

using autoware_utils_debug::ScopedTimeTrack;

struct MeasurementWithIndex
{
  const types::DynamicObject & object;
  size_t index;

  MeasurementWithIndex(const types::DynamicObject & obj, size_t idx) : object(obj), index(idx) {}
};

BevAssociation::BevAssociation(const TrackerAssociationConfig & config)
: config_(config), score_threshold_(config.score_threshold)
{
  gnn_solver_ptr_ = std::make_unique<gnn_solver::MuSSP>();
}

void BevAssociation::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
}

types::AssociationResult BevAssociation::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  const types::AssociationData data = calcAssociationData(measurements, trackers);
  types::AssociationResult result;
  assign(data, result);
  return result;
}

void BevAssociation::assign(
  const types::AssociationData & data, types::AssociationResult & association_result)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;

  std::vector<std::vector<double>> score = formatScoreMatrix(data);

  // Solve the linear assignment problem
  gnn_solver_ptr_->maximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  // Build a lookup map for entries with significant shape change
  std::unordered_map<int, std::unordered_map<int, const types::AssociationEntry *>> entry_map;
  for (const auto & entry : data.entries) {
    if (entry.has_significant_shape_change && entry.score >= score_threshold_) {
      entry_map[static_cast<int>(entry.tracker_idx)][static_cast<int>(entry.measurement_idx)] =
        &entry;
    }
  }

  association_result.unassigned_trackers.reserve(data.tracker_uuids.size());
  association_result.unassigned_measurements.reserve(data.measurement_uuids.size());

  for (const auto & [tracker_idx, measurement_idx] : direct_assignment) {
    if (score[tracker_idx][measurement_idx] >= score_threshold_) {
      association_result.add(
        data.tracker_uuids[tracker_idx], data.measurement_uuids[measurement_idx]);

      auto tracker_it = entry_map.find(tracker_idx);
      if (tracker_it != entry_map.end()) {
        auto measurement_it = tracker_it->second.find(measurement_idx);
        if (measurement_it != tracker_it->second.end()) {
          association_result.trackers_with_shape_change.insert(data.tracker_uuids[tracker_idx]);
        }
      }
    }
  }

  for (size_t i = 0; i < data.tracker_uuids.size(); ++i) {
    auto it = direct_assignment.find(static_cast<int>(i));
    if (
      it == direct_assignment.end() || score[static_cast<int>(i)][it->second] < score_threshold_) {
      association_result.unassigned_trackers.emplace_back(data.tracker_uuids[i]);
    }
  }

  for (size_t i = 0; i < data.measurement_uuids.size(); ++i) {
    auto it = reverse_assignment.find(static_cast<int>(i));
    if (
      it == reverse_assignment.end() || score[it->second][static_cast<int>(i)] < score_threshold_) {
      association_result.unassigned_measurements.emplace_back(data.measurement_uuids[i]);
    }
  }
}

// Directly computes inverse covariance from pose_covariance array
inline InverseCovariance2D precomputeInverseCovarianceFromPose(
  const std::array<double, 36> & pose_covariance)
{
  constexpr double minimum_cov = 0.25;
  const double a = std::max(pose_covariance[0], minimum_cov);  // cov(0,0)
  const double b = pose_covariance[1];                         // cov(0,1)
  const double d = std::max(pose_covariance[7], minimum_cov);  // cov(1,1)

  const double det = a * d - b * b;
  InverseCovariance2D result;

  constexpr double min_det = 1e-12;
  if (!(std::isfinite(det)) || det <= min_det) {
    result.inv00 = 1.0 / a;
    result.inv01 = 0.0;
    result.inv11 = 1.0 / d;
    return result;
  }

  const double inv_det = 1.0 / det;
  result.inv00 = d * inv_det;
  result.inv01 = -b * inv_det;
  result.inv11 = a * inv_det;
  return result;
}

PreparationData BevAssociation::prepareAssociationData(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  PreparationData prep_data;
  prep_data.trackers.reserve(trackers.size());

  std::vector<ValueType> rtree_points;
  rtree_.clear();
  rtree_points.reserve(trackers.size());

  size_t tracker_idx = 0;
  for (const auto & tracker : trackers) {
    TrackerBevEntry entry;
    tracker->getTrackedObject(measurements.header.stamp, entry.object);
    entry.label = tracker->getHighestProbLabel();
    entry.type = tracker->getTrackerType();
    entry.inv_cov = precomputeInverseCovarianceFromPose(entry.object.pose_covariance);

    Point p(entry.object.pose.position.x, entry.object.pose.position.y);
    rtree_points.emplace_back(p, tracker_idx);

    prep_data.trackers.emplace_back(std::move(entry));
    ++tracker_idx;
  }
  rtree_.insert(rtree_points.begin(), rtree_points.end());

  return prep_data;
}

void BevAssociation::processMeasurement(
  const types::DynamicObject & measurement_object, const size_t measurement_idx,
  const classes::Label measurement_label, const PreparationData & prep_data,
  types::AssociationData & association_data)
{
  const ShapeLabelKey shape_label_key{
    types::toShapeType(measurement_object.shape.type), measurement_label};
  const auto tracker_params_map_opt =
    get_map_value_if_exists(config_.association_params_map, shape_label_key);
  if (!tracker_params_map_opt) {
    return;
  }
  const auto & tracker_params_map = tracker_params_map_opt->get();

  const auto max_squared_dist_opt =
    get_map_value_if_exists(config_.max_dist_sq_per_label, measurement_label);
  const double max_squared_dist = max_squared_dist_opt ? max_squared_dist_opt->get() : 0.0;

  Point measurement_point(measurement_object.pose.position.x, measurement_object.pose.position.y);

  std::vector<ValueType> nearby_trackers;
  nearby_trackers.reserve(std::min(size_t{100}, prep_data.trackers.size()));

  const double max_dist = std::sqrt(max_squared_dist);
  const Box query_box(
    Point(measurement_point.get<0>() - max_dist, measurement_point.get<1>() - max_dist),
    Point(measurement_point.get<0>() + max_dist, measurement_point.get<1>() + max_dist));
  rtree_.query(bgi::within(query_box), std::back_inserter(nearby_trackers));

  for (const auto & tracker_value : nearby_trackers) {
    const size_t tracker_idx = tracker_value.second;
    const auto & tracker_entry = prep_data.trackers[tracker_idx];

    const auto association_params_opt =
      get_map_value_if_exists(tracker_params_map, tracker_entry.type);
    if (!association_params_opt) continue;

    const auto result = calculateBevAssignmentScore(
      tracker_entry.object, tracker_entry.label, tracker_entry.type, association_params_opt->get(),
      measurement_object, measurement_label, tracker_entry.inv_cov,
      config_.unknown_association_giou_threshold);

    if (result.score > INVALID_SCORE) {
      association_data.entries.emplace_back(
        types::AssociationEntry{
          tracker_idx, measurement_idx, result.score, result.has_significant_shape_change});
    }
  }
}

types::AssociationData BevAssociation::calcAssociationData(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (measurements.objects.empty() || trackers.empty()) {
    return types::AssociationData{};
  }

  auto prep_data = prepareAssociationData(measurements, trackers);

  types::AssociationData association_data;
  association_data.tracker_uuids.reserve(trackers.size());
  association_data.measurement_uuids.reserve(measurements.objects.size());

  for (const auto & object : measurements.objects) {
    association_data.measurement_uuids.emplace_back(object.uuid);
  }
  for (const auto & tracker : trackers) {
    association_data.tracker_uuids.emplace_back(tracker->getUUID());
  }

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

std::vector<std::vector<double>> BevAssociation::formatScoreMatrix(
  const types::AssociationData & data) const
{
  std::vector<std::vector<double>> score_matrix(
    data.tracker_uuids.size(), std::vector<double>(data.measurement_uuids.size(), 0.0));
  for (const auto & entry : data.entries) {
    score_matrix[entry.tracker_idx][entry.measurement_idx] = entry.score;
  }
  return score_matrix;
}

}  // namespace autoware::multi_object_tracker
