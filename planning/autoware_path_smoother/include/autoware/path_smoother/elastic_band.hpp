// Copyright 2023 TIER IV, Inc.
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

#ifndef AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_HPP_
#define AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_HPP_

#include "autoware/osqp_interface/osqp_interface.hpp"
#include "autoware/path_smoother/common_structs.hpp"
#include "autoware/path_smoother/type_alias.hpp"

#include <Eigen/Core>
#include <autoware_utils_debug/debug_publisher.hpp>

#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace autoware::path_smoother
{
// Node-independent parameter set for the elastic band smoother.
// Declaration of the parameters lives in the node layer (see declare_eb_param()),
// so this struct itself does not depend on any node type.
struct EBParam
{
  // qp
  struct QPParam
  {
    int max_iteration;
    double eps_abs;
    double eps_rel;
  };

  EBParam() = default;
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  // option
  bool enable_warm_start;
  bool enable_optimization_validation;

  // common
  double delta_arc_length;
  int num_points;

  // clearance
  int num_joint_points;
  double clearance_for_fix;
  double clearance_for_joint;
  double clearance_for_smooth;

  // weight
  double smooth_weight;
  double lat_error_weight;

  // qp
  QPParam qp_param;

  // validation
  double max_validation_error;
};

// Node-layer factory: declares the "elastic_band.*" parameters on the given node
// and returns a Node-independent EBParam value. Templated on the node type so it
// works with both rclcpp::Node and autoware::agnocast_wrapper::Node.
template <typename NodeT>
EBParam declare_eb_param(NodeT * node);

// Node-independent elastic band smoother.
//
// The smoothing logic itself does not depend on any node type; the only place a
// node type appears is the debug publisher, which is injected as a
// autoware_utils_debug::BasicDebugPublisher<NodeT>. NodeT defaults to rclcpp::Node
// so existing callers keep working unchanged (see the EBPathSmoother alias below);
// the agnocast path instantiates it with autoware::agnocast_wrapper::Node.
//
// Parameters, logger and clock are passed by value and are node-type-agnostic.
template <typename NodeT = rclcpp::Node>
class BasicEBPathSmoother
{
public:
  BasicEBPathSmoother(
    const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const CommonParam & common_param, const EBParam & eb_param, rclcpp::Logger logger,
    const rclcpp::Clock & clock,
    const std::shared_ptr<autoware_utils_debug::BasicDebugPublisher<NodeT>> & debug_publisher,
    const std::shared_ptr<TimeKeeper> time_keeper_ptr);

  std::vector<TrajectoryPoint> smoothTrajectory(
    const std::vector<TrajectoryPoint> & traj_points, const geometry_msgs::msg::Pose & ego_pose);

  void initialize(const bool enable_debug_info, const CommonParam & common_param);
  void resetPreviousData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

private:
  struct Constraint2d
  {
    struct Constraint
    {
      Eigen::Vector2d coef;
      double upper_bound;
      double lower_bound;
    };

    Constraint lon;
    Constraint lat;
  };

  // arguments
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  CommonParam common_param_;
  EBParam eb_param_;
  mutable std::shared_ptr<TimeKeeper> time_keeper_ptr_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;

  // debug publisher (injected by the node layer; may be nullptr to disable debug output)
  std::shared_ptr<autoware_utils_debug::BasicDebugPublisher<NodeT>> debug_publisher_;

  std::unique_ptr<autoware::osqp_interface::OSQPInterface> osqp_solver_ptr_;
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_eb_traj_points_ptr_{nullptr};

  std::vector<TrajectoryPoint> insertFixedPoint(
    const std::vector<TrajectoryPoint> & traj_point) const;

  std::tuple<std::vector<TrajectoryPoint>, size_t> getPaddedTrajectoryPoints(
    const std::vector<TrajectoryPoint> & traj_points) const;

  void updateConstraint(
    const std::vector<TrajectoryPoint> & traj_points, const bool is_goal_contained,
    const int pad_start_idx);

  std::optional<std::vector<double>> calcSmoothedTrajectory();

  std::optional<std::vector<TrajectoryPoint>> convertOptimizedPointsToTrajectory(
    const std::vector<double> & optimized_points, const std::vector<TrajectoryPoint> & traj_points,
    const int pad_start_idx) const;
};

// Backward-compatible alias: existing callers using a plain rclcpp::Node keep using
// EBPathSmoother without any change.
using EBPathSmoother = BasicEBPathSmoother<rclcpp::Node>;
}  // namespace autoware::path_smoother

#endif  // AUTOWARE__PATH_SMOOTHER__ELASTIC_BAND_HPP_
