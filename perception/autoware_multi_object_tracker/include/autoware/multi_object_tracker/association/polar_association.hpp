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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__POLAR_ASSOCIATION_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__POLAR_ASSOCIATION_HPP_

#include "autoware/multi_object_tracker/association/association_base.hpp"
#include "autoware/multi_object_tracker/association/azimuth_bin_index.hpp"
#include "autoware/multi_object_tracker/association/scoring/polar_assignment_scoring.hpp"
#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"
#include "autoware/multi_object_tracker/types.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <rclcpp/clock.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <list>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

/// Polar-coordinate association algorithm.
/// Projects bounding boxes to ego-centric polar coordinates and scores pairs using 2D perspective
/// IoU in (azimuth × height) space, hard-gated by a min-IoU threshold and blended with a graded
/// nearest-surface depth proximity term.

class PolarAssociation : public AssociationBase
{
public:
  explicit PolarAssociation(const AssociatorConfig & config);
  ~PolarAssociation() override = default;

  /// AssociationBase implementation.
  types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers) override;

  /// Set ego pose for polar coordinate computation. Must be called before associate().
  void setEgoPose(const std::optional<geometry_msgs::msg::Pose> & ego_pose);

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);

private:
  // Per-tracker entry bundling all precomputed data for one tracker
  struct TrackerPolarEntry
  {
    types::DynamicObject object;
    classes::Label label;
    types::TrackerType type;
    polar_scoring::PolarFootprint footprint;
  };

  struct EgoContext
  {
    double x, y, z, yaw;
  };

  AssociatorConfig config_;
  const double score_threshold_;
  std::unique_ptr<gnn_solver::GnnSolverInterface> gnn_solver_ptr_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;
  std::optional<geometry_msgs::msg::Pose> ego_pose_;
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  AzimuthBinIndex azimuth_bin_index_;

  std::vector<TrackerPolarEntry> prepareAssociationData(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers, const EgoContext & ego);

  void processMeasurement(
    const types::DynamicObject & measurement_object, size_t measurement_idx,
    classes::Label measurement_label, const std::vector<TrackerPolarEntry> & tracker_entries,
    const EgoContext & ego, types::AssociationData & association_data);

  types::AssociationData calcAssociationData(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  void assign(const types::AssociationData & data, types::AssociationResult & result);

  std::vector<std::vector<double>> formatScoreMatrix(const types::AssociationData & data) const;
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__POLAR_ASSOCIATION_HPP_
