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

#ifndef MRM_ROAD_BORDER_STOP_PLANNER_HPP_
#define MRM_ROAD_BORDER_STOP_PLANNER_HPP_

#include "type_alias.hpp"

#include <autoware/planning_factor_interface/planning_factor_interface.hpp>
#include <autoware_utils_geometry/boost_geometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/geometry/index/rtree.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::in_lane_mrm_planner
{

/// Spatial index of lanelet2 linestring segments whose `type` attribute matches one of the
/// configured boundary types (e.g. `road_border`). Built once per map instance.
class BoundarySegmentIndex
{
public:
  struct Entry
  {
    autoware_utils_geometry::Segment2d segment;
    double z_min{0.0};
    double z_max{0.0};
    lanelet::Id linestring_id{lanelet::InvalId};
  };

  void build(const lanelet::LaneletMapPtr & map, const std::vector<std::string> & boundary_types);
  bool is_built_for(
    const lanelet::LaneletMapPtr & map, const std::vector<std::string> & boundary_types) const;
  void clear();

  std::vector<const Entry *> query(const autoware_utils_geometry::Box2d & box) const;
  bool empty() const { return entries_.empty(); }
  size_t size() const { return entries_.size(); }

private:
  using Value = std::pair<autoware_utils_geometry::Box2d, size_t>;
  using RTree = boost::geometry::index::rtree<Value, boost::geometry::index::rstar<16>>;

  std::vector<Entry> entries_;
  RTree rtree_;
  const lanelet::LaneletMap * map_identity_{nullptr};
  std::vector<std::string> boundary_types_;
};

struct RoadBorderContact
{
  double contact_arc_length{0.0};  //!< refined base_link arc length (from trajectory start) [m]
  geometry_msgs::msg::Pose contact_pose;  //!< base_link pose where the footprint first touches
  double ego_arc_length{0.0};  //!< arc length of the ego position projected on the trajectory [m]
  lanelet::Id linestring_id{lanelet::InvalId};
  autoware_utils_geometry::Segment2d segment;
  geometry_msgs::msg::Point contact_point;
  std::optional<size_t> stop_index;  //!< index of the inserted stop point (if inserted)
  double stop_arc_length{0.0};
};

/// Phase2 "Road Border Stop": sweeps the vehicle footprint along the candidate trajectory,
/// finds the first interference with a map road border and inserts a stop point
/// `stop_margin` before it. Deceleration feasibility is left to MrmStopVelocityPlanner.
class MrmRoadBorderStopPlanner
{
public:
  /// `node` may be nullptr (unit tests); publishers and planning factors are then disabled.
  void initialize(rclcpp::Node * node, const VehicleInfo & vehicle_info, const Params & params);
  void update_params(const Params & params);
  void set_lanelet_map(const lanelet::LaneletMapPtr & lanelet_map_ptr);

  /// Inserts a stop point when the footprint swept along `points` interferes with a boundary.
  /// Returns the contact information (with `stop_index` set when a stop point was inserted).
  std::optional<RoadBorderContact> apply(TrajectoryPoints & points, const Odometry & odom);

  void publish_planning_factor();

  /// Forward sweep from `start_idx`. `ego_arc_length` is the arc length of the ego position
  /// (from the trajectory start) used as the origin of `max_check_length` and as the lower
  /// bound of the stop point. Exposed for unit tests.
  /// Sweep the footprint starting at the ego pose, then along the trajectory points ahead of the
  /// ego (index > ego_segment_idx), and return the first contact with a border (if any).
  std::optional<RoadBorderContact> find_first_contact(
    const TrajectoryPoints & points, const geometry_msgs::msg::Pose & ego_pose,
    const size_t ego_segment_idx, const double ego_arc_length) const;

  const BoundarySegmentIndex & boundary_index() const { return boundary_index_; }

private:
  using RoadBorderStopParams = Params::RoadBorderStop;

  autoware_utils_geometry::Polygon2d create_footprint(const geometry_msgs::msg::Pose & pose) const;
  std::optional<const BoundarySegmentIndex::Entry *> find_intersecting_segment(
    const autoware_utils_geometry::Polygon2d & footprint, const double pose_z) const;
  bool is_within_height(const BoundarySegmentIndex::Entry & entry, const double pose_z) const;
  /// Bisection between a non-interfering and an interfering pose. Returns the refined arc length
  /// and the corresponding (interpolated) pose.
  std::pair<double, geometry_msgs::msg::Pose> refine_contact(
    const geometry_msgs::msg::Pose & pose_prev, const geometry_msgs::msg::Pose & pose_contact,
    const double arc_prev, const double arc_contact) const;
  void set_stop_point(
    TrajectoryPoints & points, RoadBorderContact & contact, const Odometry & odom);
  void publish_debug_markers(const TrajectoryPoints & points, const Odometry & odom) const;

  rclcpp::Node * node_{nullptr};
  VehicleInfo vehicle_info_;
  RoadBorderStopParams params_;
  autoware_utils_geometry::LinearRing2d base_footprint_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  BoundarySegmentIndex boundary_index_;

  std::optional<RoadBorderContact> last_contact_;
  autoware_internal_planning_msgs::msg::SafetyFactorArray safety_factors_;

  std::unique_ptr<autoware::planning_factor_interface::PlanningFactorInterface>
    planning_factor_interface_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
};

}  // namespace autoware::in_lane_mrm_planner

#endif  // MRM_ROAD_BORDER_STOP_PLANNER_HPP_
