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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_HPP_

#define EIGEN_MPL2_ONLY

#include "autoware/multi_object_tracker/association/solver/gnn_solver.hpp"
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
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

// Define point and box types for R-tree
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef bg::model::box<Point> Box;
typedef std::pair<Point, size_t> ValueType;  // Point and tracker index

struct AssociatorConfig
{
  std::unordered_map<TrackerType, std::array<bool, types::NUM_LABELS>> can_assign_map;
  Eigen::MatrixXd max_dist_matrix;
  Eigen::MatrixXd max_area_matrix;
  Eigen::MatrixXd min_area_matrix;
  Eigen::MatrixXd min_iou_matrix;
  double unknown_association_giou_threshold;
};

struct InverseCovariance2D
{
  double inv00;  // (d / det)
  double inv01;  // (-b / det)
  double inv11;  // (a / det)
};

struct PreparationData
{
  std::vector<types::DynamicObject> tracked_objects;
  std::vector<std::uint8_t> tracker_labels;
  std::vector<TrackerType> tracker_types;
  std::vector<InverseCovariance2D> tracker_inverse_covariances;
};

class DataAssociation
{
private:
  AssociatorConfig config_;
  const double score_threshold_;
  std::unique_ptr<gnn_solver::GnnSolverInterface> gnn_solver_ptr_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  // R-tree for spatial indexing of trackers
  bgi::rtree<ValueType, bgi::quadratic<16>> rtree_;
  // Cache of maximum squared distances per measurement class
  // For each measurement class, stores the maximum squared distance it could match with any tracker
  // class
  std::vector<double> max_squared_dist_per_class_;

  // Helper to compute max search distances from config
  void updateMaxSearchDistances();

  // Preparation and processing stages for association
  PreparationData prepareAssociationData(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);
  void processMeasurement(
    const types::DynamicObject & measurement_object, size_t measurement_idx,
    const std::uint8_t measurement_label, const PreparationData & prep_data,
    types::AssociationData & association_data);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit DataAssociation(const AssociatorConfig & config);
  virtual ~DataAssociation() {}

  void assign(const types::AssociationData & data, types::AssociationResult & association_result);

  double calculateScore(
    const types::DynamicObject & tracked_object, const std::uint8_t tracker_label,
    const types::DynamicObject & measurement_object, const std::uint8_t measurement_label,
    const InverseCovariance2D & inv_cov, bool & has_significant_shape_change) const;

  types::AssociationData calcAssociationData(
    const types::DynamicObjectList & measurements,
    const std::list<std::shared_ptr<Tracker>> & trackers);

  std::vector<std::vector<double>> formatScoreMatrix(const types::AssociationData & data) const;

  const double CHECK_GIOU_THRESHOLD = 0.7;
  const double AREA_RATIO_THRESHOLD = 1.3;

  void setTimeKeeper(std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_ptr);
};

}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__ASSOCIATION__ASSOCIATION_HPP_
