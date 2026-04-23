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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__BEV_ASSOCIATION_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__BEV_ASSOCIATION_HPP_

#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/association/association_base.hpp"
#include "autoware/multi_object_tracker/association/scoring/bev_assignment_scoring.hpp"
#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
#include "autoware/multi_object_tracker/configurations.hpp"
#include "autoware/multi_object_tracker/tracker/tracker.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <autoware_utils_debug/time_keeper.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <list>
#include <memory>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Spatial index types for R-tree tracker lookup
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::box<Point> Box;
typedef std::pair<Point, size_t> ValueType;  // (position, tracker index)

// Per-tracker precomputed data for a single association round
struct PreparationData
{
  std::vector<types::DynamicObject> tracked_objects;
  std::vector<classes::Label> tracker_labels;
  std::vector<types::TrackerType> tracker_types;
  std::vector<InverseCovariance2D> tracker_inverse_covariances;
};

class BevAssociation : public AssociationBase
{
private:
  AssociatorConfig config_;
  const double score_threshold_;
  std::unique_ptr<gnn_solver::GnnSolverInterface> gnn_solver_ptr_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  // R-tree for spatial indexing of trackers
  bgi::rtree<ValueType, bgi::quadratic<16>> rtree_;
  // Maximum squared search distance per measurement class (precomputed from config)
  AssociatorConfig::LabelDoubleMap max_squared_dist_per_class_;

  void updateMaxSearchDistances();

  PreparationData prepareAssociationData(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  void processMeasurement(
    const types::DynamicObject & measurement_object, size_t measurement_idx,
    const classes::Label measurement_label, const PreparationData & prep_data,
    types::AssociationData & association_data);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit BevAssociation(const AssociatorConfig & config);
  ~BevAssociation() override = default;

  /// AssociationBase implementation: full pipeline (calcAssociationData + assign).
  types::AssociationResult associate(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers) override;

  // Lower-level access (for diagnostics / debug):
  void assign(const types::AssociationData & data, types::AssociationResult & association_result);

  types::AssociationData calcAssociationData(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  std::vector<std::vector<double>> formatScoreMatrix(const types::AssociationData & data) const;

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__BEV_ASSOCIATION_HPP_
