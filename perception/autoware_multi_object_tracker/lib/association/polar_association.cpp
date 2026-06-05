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

#include "autoware/multi_object_tracker/association/polar_association.hpp"

#include "autoware/multi_object_tracker/association/scoring/polar_assignment_scoring.hpp"
#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
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

//// Construction & configuration

PolarAssociation::PolarAssociation(const TrackerAssociationConfig & config)
: config_(config), score_threshold_(config.score_threshold)
{
  gnn_solver_ptr_ = std::make_unique<gnn_solver::MuSSP>();
}

void PolarAssociation::setEgoPose(const std::optional<geometry_msgs::msg::Pose> & ego_pose)
{
  ego_pose_ = ego_pose;
}

void PolarAssociation::setTimeKeeper(
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr)
{
  time_keeper_ = std::move(time_keeper_ptr);
}

//// Top-level associate

types::AssociationResult PolarAssociation::associate(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  const types::AssociationData data = calcAssociationData(measurements, trackers);
  types::AssociationResult result;
  assign(data, result);
  return result;
}

//// Data preparation

std::vector<PolarAssociation::TrackerPolarEntry> PolarAssociation::prepareAssociationData(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers, const EgoContext & ego)
{
  std::vector<TrackerPolarEntry> tracker_entries;
  tracker_entries.reserve(trackers.size());

  size_t tracker_idx = 0;
  azimuth_bin_index_.clear();

  for (const auto & tracker : trackers) {
    TrackerPolarEntry entry;
    tracker->getTrackedObject(measurements.header.stamp, entry.object);
    entry.label = tracker->getHighestProbLabel();
    entry.type = tracker->getTrackerType();
    entry.footprint =
      polar_scoring::computePolarFootprint(entry.object, ego.x, ego.y, ego.z, ego.yaw);

    azimuth_bin_index_.add(entry.footprint.azimuth, tracker_idx, entry.footprint.r_min);
    tracker_entries.emplace_back(std::move(entry));
    ++tracker_idx;
  }

  return tracker_entries;
}

//// Per-measurement scoring

void PolarAssociation::processMeasurement(
  const types::DynamicObject & measurement_object, const size_t measurement_idx,
  const classes::Label measurement_label, const std::vector<TrackerPolarEntry> & tracker_entries,
  const EgoContext & ego, types::AssociationData & association_data)
{
  const ShapeLabelKey shape_label_key{
    types::toShapeType(measurement_object.shape.type), measurement_label};
  const auto tracker_params_map_opt =
    get_map_value_if_exists(config_.association_params_map, shape_label_key);
  if (!tracker_params_map_opt) return;
  const auto & tracker_params_map = tracker_params_map_opt->get();

  // Compute measurement polar footprint and query azimuth bins for candidate trackers
  const auto meas_fp =
    polar_scoring::computePolarFootprint(measurement_object, ego.x, ego.y, ego.z, ego.yaw);
  const auto candidate_indices = azimuth_bin_index_.find(meas_fp.azimuth, meas_fp.r_min);

  for (const size_t tracker_idx : candidate_indices) {
    const auto & tracker_entry = tracker_entries[tracker_idx];

    const auto association_params_opt =
      get_map_value_if_exists(tracker_params_map, tracker_entry.type);
    if (!association_params_opt) continue;
    const auto & association_params = association_params_opt->get();

    // Depth gate (Step 2): LiDAR can only return the nearest visible surface of a solid object,
    // so the cluster's closest 3D corner must be near the tracker's closest 3D corner.
    //
    // The gate scales with the tracker's footprint area to tolerate nearest-corner geometry
    // shifts that occur when a large vehicle's visible face changes (e.g., a truck turning 15°
    // so the front face replaces the side as the nearest surface):
    //
    //   gate [m] = kDepthGateBase + kDepthGateAreaRate * sqrt(tracker_area_m2)
    //
    // Object-size assumptions and resulting gate values (10 Hz, good CV prediction):
    //   Pedestrian  area ~0.25 m²  →  gate = 2.0 + 0.25*0.50 = 2.1 m
    //   Car         area ~9 m²     →  gate = 2.0 + 0.25*3.00 = 2.75 m
    //   Truck       area ~20 m²    →  gate = 2.0 + 0.25*4.47 = 3.1 m
    //   Bus         area ~30 m²    →  gate = 2.0 + 0.25*5.48 = 3.4 m
    //
    // Dominant error source is prediction inaccuracy, not raw displacement: at 10 Hz a good
    // constant-velocity model keeps position error below 0.5–1.5 m for nominal maneuvers.
    // The size scaling absorbs post-occlusion coasting drift for large vehicles without
    // inflating the gate for small objects where false associations are most damaging.
    constexpr double kDepthGateBase = 2.0;       // [m] minimum gate regardless of object size
    constexpr double kDepthGateAreaRate = 0.25;  // [m/sqrt(m²)] size-scaling coefficient
    const double depth_gate =
      kDepthGateBase + kDepthGateAreaRate * std::sqrt(std::max(0.0, tracker_entry.object.area));
    const double depth_gap = std::abs(meas_fp.r_min_3d - tracker_entry.footprint.r_min_3d);
    if (depth_gap > depth_gate) continue;

    const auto result = polar_scoring::calculatePolarAssignmentScore(
      meas_fp, tracker_entry.footprint, measurement_object, tracker_entry.object,
      tracker_entry.type, association_params.min_iou);

    if (result.score > INVALID_SCORE) {
      association_data.entries.emplace_back(
        types::AssociationEntry{
          tracker_idx, measurement_idx, result.score, result.has_significant_shape_change});
    }
  }
}

//// Full association data computation

types::AssociationData PolarAssociation::calcAssociationData(
  const types::DynamicObjectList & measurements,
  const std::list<std::shared_ptr<Tracker>> & trackers)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (measurements.objects.empty() || trackers.empty() || !ego_pose_) {
    if (!ego_pose_) {
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("polar_association"), steady_clock_, 5000,
        "PolarAssociation: ego_pose not set, returning empty association");
    }
    return types::AssociationData{};
  }

  const EgoContext ego{
    ego_pose_->position.x, ego_pose_->position.y, ego_pose_->position.z,
    tf2::getYaw(ego_pose_->orientation)};

  const auto tracker_entries = prepareAssociationData(measurements, trackers, ego);

  types::AssociationData association_data;
  association_data.tracker_uuids.reserve(trackers.size());
  association_data.measurement_uuids.reserve(measurements.objects.size());

  for (const auto & object : measurements.objects) {
    association_data.measurement_uuids.emplace_back(object.uuid);
  }
  for (const auto & tracker : trackers) {
    association_data.tracker_uuids.emplace_back(tracker->getUUID());
  }

  for (size_t measurement_idx = 0; measurement_idx < measurements.objects.size();
       ++measurement_idx) {
    const auto & obj = measurements.objects[measurement_idx];
    const auto measurement_label = classes::getHighestProbLabel(obj.classification);
    processMeasurement(
      obj, measurement_idx, measurement_label, tracker_entries, ego, association_data);
  }

  return association_data;
}

//// Assignment (GNN solver)

void PolarAssociation::assign(
  const types::AssociationData & data, types::AssociationResult & association_result)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  std::unordered_map<int, int> direct_assignment;
  std::unordered_map<int, int> reverse_assignment;

  std::vector<std::vector<double>> score = formatScoreMatrix(data);

  // Solve the linear assignment problem
  gnn_solver_ptr_->maximizeLinearAssignment(score, &direct_assignment, &reverse_assignment);

  association_result.unassigned_trackers.reserve(data.tracker_uuids.size());
  association_result.unassigned_measurements.reserve(data.measurement_uuids.size());

  for (const auto & [tracker_idx, measurement_idx] : direct_assignment) {
    if (score[tracker_idx][measurement_idx] >= score_threshold_) {
      association_result.add(
        data.tracker_uuids[tracker_idx], data.measurement_uuids[measurement_idx]);

      for (const auto & entry : data.entries) {
        if (
          static_cast<int>(entry.tracker_idx) == tracker_idx &&
          static_cast<int>(entry.measurement_idx) == measurement_idx &&
          entry.has_significant_shape_change && entry.score >= score_threshold_) {
          association_result.trackers_with_shape_change.insert(data.tracker_uuids[tracker_idx]);
          break;
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

std::vector<std::vector<double>> PolarAssociation::formatScoreMatrix(
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
