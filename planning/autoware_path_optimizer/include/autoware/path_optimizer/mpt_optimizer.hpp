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

#ifndef AUTOWARE__PATH_OPTIMIZER__MPT_OPTIMIZER_HPP_
#define AUTOWARE__PATH_OPTIMIZER__MPT_OPTIMIZER_HPP_

#include "autoware/interpolation/linear_interpolation.hpp"
#include "autoware/interpolation/spline_interpolation_points_2d.hpp"
#include "autoware/osqp_interface/osqp_interface.hpp"
#include "autoware/path_optimizer/common_structs.hpp"
#include "autoware/path_optimizer/state_equation_generator.hpp"
#include "autoware/path_optimizer/type_alias.hpp"
#include "autoware/path_optimizer/utils/conditional_timer.hpp"
#include "autoware_utils/geometry/geometry.hpp"
#include "autoware_utils/system/time_keeper.hpp"
#include "autoware_vehicle_info_utils/vehicle_info_utils.hpp"
#include "gtest/gtest.h"

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace autoware::path_optimizer
{
struct Bounds
{
  double lower_bound;
  double upper_bound;

  static Bounds lerp(Bounds prev_bounds, Bounds next_bounds, double ratio)
  {
    const double lower_bound =
      autoware::interpolation::lerp(prev_bounds.lower_bound, next_bounds.lower_bound, ratio);
    const double upper_bound =
      autoware::interpolation::lerp(prev_bounds.upper_bound, next_bounds.upper_bound, ratio);

    return Bounds{lower_bound, upper_bound};
  }

  void translate(const double offset)
  {
    lower_bound -= offset;
    upper_bound -= offset;
  }

  friend std::ostream & operator<<(std::ostream & os, const Bounds & bounds)
  {
    os << "Bounds: {\n";
    os << "\tlower_bound: " << bounds.lower_bound << ",\n";
    os << "\tupper_bound: " << bounds.upper_bound << "\n";
    os << "}\n";
    return os;
  }
};

struct KinematicState
{
  double lat{0.0};
  double yaw{0.0};

  Eigen::Vector2d toEigenVector() const { return Eigen::Vector2d{lat, yaw}; }

  friend std::ostream & operator<<(std::ostream & os, const KinematicState & state)
  {
    os << "KinematicState: {\n";
    os << "\tlat: " << state.lat << ",\n";
    os << "\tyaw: " << state.yaw << "\n";
    os << "}\n";
    return os;
  }
};

struct ReferencePoint
{
  geometry_msgs::msg::Pose pose;
  double longitudinal_velocity_mps;

  // additional information
  double curvature{0.0};
  double delta_arc_length{0.0};
  double alpha{0.0};           // for minimizing lateral error
  Bounds bounds{};             // bounds on `pose`
  std::vector<double> beta{};  // for collision-free constraint
  double normalized_avoidance_cost{0.0};

  // bounds and its local pose on each collision-free constraint
  std::vector<Bounds> bounds_on_constraints{};
  std::vector<geometry_msgs::msg::Pose> pose_on_constraints{};

  // optimization result
  std::optional<KinematicState> fixed_kinematic_state{std::nullopt};
  KinematicState optimized_kinematic_state{};
  double optimized_input{};
  std::optional<std::vector<double>> slack_variables{std::nullopt};

  double getYaw() const { return tf2::getYaw(pose.orientation); }

  friend std::ostream & operator<<(std::ostream & os, const ReferencePoint & ref_point)
  {
    os << "ReferencePoint: {\n";
    os << "\tpose: " << ref_point.pose.position.x << ", " << ref_point.pose.position.y << ",\n";
    os << "\tlongitudinal_velocity_mps: " << ref_point.longitudinal_velocity_mps << ",\n";
    os << "\tcurvature: " << ref_point.curvature << ",\n";
    os << "\tdelta_arc_length: " << ref_point.delta_arc_length << ",\n";
    os << "\talpha: " << ref_point.alpha << ",\n";
    os << "\tbounds: " << ref_point.bounds << ",\n";
    os << "\tbeta: [";
    for (const auto & b : ref_point.beta) {
      os << b << ", ";
    }
    os << "],\n";
    os << "\tnormalized_avoidance_cost: " << ref_point.normalized_avoidance_cost << ",\n";
    os << "\tbounds_on_constraints: [";
    for (const auto & b : ref_point.bounds_on_constraints) {
      os << b << ", ";
    }
    os << "],\n";
    os << "\tpose_on_constraints: [";
    for (const auto & p : ref_point.pose_on_constraints) {
      os << "(" << p.position.x << ", " << p.position.y << ") , ";
    }
    os << "],\n";
    os << "\tfixed_kinematic_state: ";
    if (ref_point.fixed_kinematic_state) {
      os << *ref_point.fixed_kinematic_state;
    } else {
      os << "nullopt";
    }
    os << ",\n";
    os << "\toptimized_kinematic_state: " << ref_point.optimized_kinematic_state << ",\n";
    os << "\toptimized_input: " << ref_point.optimized_input << ",\n";
    os << "\tslack_variables: ";
    if (ref_point.slack_variables) {
      os << "[";
      for (const auto & s : *ref_point.slack_variables) {
        os << s << ", ";
      }
      os << "]";
    } else {
      os << "nullopt";
    }
    os << "\n";
    os << "}\n";
    return os;
  }

  geometry_msgs::msg::Pose offsetDeviation(const double lat_dev, const double yaw_dev) const
  {
    auto pose_with_deviation = autoware_utils::calc_offset_pose(pose, 0.0, lat_dev, 0.0);
    pose_with_deviation.orientation =
      autoware_utils::create_quaternion_from_yaw(getYaw() + yaw_dev);
    return pose_with_deviation;
  }
};

class MPTOptimizer
{
public:
  MPTOptimizer(
    rclcpp::Node * node, const bool enable_debug_info, const EgoNearestParam ego_nearest_param,
    const autoware::vehicle_info_utils::VehicleInfo & vehicle_info,
    const TrajectoryParam & traj_param, const std::shared_ptr<DebugData> debug_data_ptr,
    const std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_);

  std::optional<std::vector<TrajectoryPoint>> optimizeTrajectory(const PlannerData & planner_data);
  std::optional<std::vector<TrajectoryPoint>> getPrevOptimizedTrajectoryPoints() const;

  void initialize(const bool enable_debug_info, const TrajectoryParam & traj_param);
  void resetPreviousData();
  void onParam(const std::vector<rclcpp::Parameter> & parameters);

  double getTrajectoryLength() const;
  double getDeltaArcLength() const;
  int getNumberOfPoints() const;

private:
  struct ValueMatrix
  {
    Eigen::SparseMatrix<double> Q;
    Eigen::SparseMatrix<double> R;

    friend std::ostream & operator<<(std::ostream & os, const ValueMatrix & matrix)
    {
      os << "ValueMatrix: {\n";
      os << "\tQ: (Sparse Matrix):" << matrix.Q;
      os << "\tR: (Sparse Matrix):" << matrix.R;
      os << "}\n";
      return os;
    }
  };

  struct ObjectiveMatrix
  {
    Eigen::MatrixXd hessian;
    Eigen::VectorXd gradient;

    friend std::ostream & operator<<(std::ostream & os, const ObjectiveMatrix & matrix)
    {
      os << "ObjectiveMatrix: {\n";
      os << "\thessian:\n" << matrix.hessian << "\n";
      os << "\tgradient:\n" << matrix.gradient << "\n";
      os << "}\n";
      return os;
    }
  };

  struct ConstraintMatrix
  {
    Eigen::MatrixXd linear;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;

    friend std::ostream & operator<<(std::ostream & os, const ConstraintMatrix & matrix)
    {
      os << "ConstraintMatrix: {\n";
      os << "\tlinear:\n" << matrix.linear << "\n";
      os << "\tlower_bound:\n" << matrix.lower_bound << "\n";
      os << "\tupper_bound:\n" << matrix.upper_bound << "\n";
      os << "}\n";
      return os;
    }
  };

  struct MPTParam
  {
    explicit MPTParam(
      rclcpp::Node * node, const autoware::vehicle_info_utils::VehicleInfo & vehicle_info);
    MPTParam() = default;
    void onParam(const std::vector<rclcpp::Parameter> & parameters);

    // option
    bool enable_warm_start;
    bool enable_manual_warm_start;
    bool enable_optimization_validation;
    bool steer_limit_constraint;
    int mpt_visualize_sampling_num;  // for debug

    // common
    double delta_arc_length;
    int num_points;

    // kinematics
    double optimization_center_offset;
    double max_steer_rad;

    // clearance
    double hard_clearance_from_road;
    double soft_clearance_from_road;
    double soft_collision_free_weight;

    // weight
    double lat_error_weight;
    double yaw_error_weight;
    double yaw_error_rate_weight;

    double terminal_lat_error_weight;
    double terminal_yaw_error_weight;
    double goal_lat_error_weight;
    double goal_yaw_error_weight;

    double steer_input_weight;
    double steer_rate_weight;

    // avoidance
    double max_bound_fixing_time;
    double max_longitudinal_margin_for_bound_violation;
    double max_avoidance_cost;
    double avoidance_cost_margin;
    double avoidance_cost_band_length;
    double avoidance_cost_decrease_rate;
    double min_drivable_width;
    double avoidance_lat_error_weight;
    double avoidance_yaw_error_weight;
    double avoidance_steer_input_weight;

    // constraint type
    bool soft_constraint;
    bool hard_constraint;
    bool l_inf_norm;

    // vehicle circles
    std::string vehicle_circles_method;
    int vehicle_circles_uniform_circle_num;
    double vehicle_circles_uniform_circle_radius_ratio;
    int vehicle_circles_bicycle_model_num;
    double vehicle_circles_bicycle_model_rear_radius_ratio;
    double vehicle_circles_bicycle_model_front_radius_ratio;
    int vehicle_circles_fitting_uniform_circle_num;

    // validation
    double max_validation_lat_error;
    double max_validation_yaw_error;
  };

  // publisher
  rclcpp::Publisher<Trajectory>::SharedPtr debug_fixed_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_ref_traj_pub_;
  rclcpp::Publisher<Trajectory>::SharedPtr debug_mpt_traj_pub_;

  // argument
  bool enable_debug_info_;
  EgoNearestParam ego_nearest_param_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;
  TrajectoryParam traj_param_;
  mutable std::shared_ptr<DebugData> debug_data_ptr_;
  mutable std::shared_ptr<autoware_utils::TimeKeeper> time_keeper_;
  rclcpp::Logger logger_;
  MPTParam mpt_param_;

  StateEquationGenerator state_equation_generator_;
  std::unique_ptr<autoware::osqp_interface::OSQPInterface> osqp_solver_ptr_;

  const double osqp_epsilon_ = 1.0e-3;

  // vehicle circles
  std::vector<double> vehicle_circle_longitudinal_offsets_;  // from base_link
  std::vector<double> vehicle_circle_radiuses_;

  // previous data
  int prev_mat_n_ = 0;
  int prev_mat_m_ = 0;
  int prev_solution_status_ = 0;
  std::shared_ptr<std::vector<ReferencePoint>> prev_ref_points_ptr_{nullptr};
  std::shared_ptr<std::vector<TrajectoryPoint>> prev_optimized_traj_points_ptr_{nullptr};

  void updateVehicleCircles();

  std::vector<ReferencePoint> calcReferencePoints(
    const PlannerData & planner_data, const std::vector<TrajectoryPoint> & smoothed_points) const;
  void updateCurvature(
    std::vector<ReferencePoint> & ref_points,
    const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const;
  void updateOrientation(
    std::vector<ReferencePoint> & ref_points,
    const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const;
  void updateBounds(
    std::vector<ReferencePoint> & ref_points,
    const std::vector<geometry_msgs::msg::Point> & left_bound,
    const std::vector<geometry_msgs::msg::Point> & right_bound,
    const geometry_msgs::msg::Pose & ego_pose, const double ego_vel) const;
  void keepMinimumBoundsWidth(std::vector<ReferencePoint> & ref_points) const;
  std::vector<ReferencePoint> extendViolatedBounds(
    const std::vector<ReferencePoint> & ref_points) const;
  void avoidSuddenSteering(
    std::vector<ReferencePoint> & ref_points, const geometry_msgs::msg::Pose & ego_pose,
    const double ego_vel) const;
  void updateVehicleBounds(
    std::vector<ReferencePoint> & ref_points,
    const autoware::interpolation::SplineInterpolationPoints2d & ref_points_spline) const;
  void updateFixedPoint(std::vector<ReferencePoint> & ref_points) const;
  void updateDeltaArcLength(std::vector<ReferencePoint> & ref_points) const;
  void updateExtraPoints(std::vector<ReferencePoint> & ref_points) const;

  ValueMatrix calcValueMatrix(
    const std::vector<ReferencePoint> & reference_points,
    const std::vector<TrajectoryPoint> & traj_points) const;

  ObjectiveMatrix calcObjectiveMatrix(
    const StateEquationGenerator::Matrix & mpt_mat, const ValueMatrix & obj_mat,
    const std::vector<ReferencePoint> & ref_points) const;

  ConstraintMatrix calcConstraintMatrix(
    const StateEquationGenerator::Matrix & mpt_mat,
    const std::vector<ReferencePoint> & ref_points) const;

  std::optional<Eigen::VectorXd> calcOptimizedSteerAngles(
    const std::vector<ReferencePoint> & ref_points, const ObjectiveMatrix & obj_mat,
    const ConstraintMatrix & const_mat);
  Eigen::VectorXd calcInitialSolutionForManualWarmStart(
    const std::vector<ReferencePoint> & ref_points,
    const std::vector<ReferencePoint> & prev_ref_points) const;
  std::pair<ObjectiveMatrix, ConstraintMatrix> updateMatrixForManualWarmStart(
    const ObjectiveMatrix & obj_mat, const ConstraintMatrix & const_mat,
    const std::optional<Eigen::VectorXd> & u0) const;

  void addSteerWeightR(
    std::vector<Eigen::Triplet<double>> & R_triplet_vec,
    const std::vector<ReferencePoint> & ref_points) const;

  std::optional<std::vector<TrajectoryPoint>> calcMPTPoints(
    std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & optimized_variables,
    const StateEquationGenerator::Matrix & mpt_matrix) const;

  void publishDebugTrajectories(
    const std_msgs::msg::Header & header, const std::vector<ReferencePoint> & ref_points,
    const std::vector<TrajectoryPoint> & mpt_traj_points) const;
  std::vector<TrajectoryPoint> extractFixedPoints(
    const std::vector<ReferencePoint> & ref_points) const;

  size_t getNumberOfSlackVariables() const;
  std::optional<double> calcNormalizedAvoidanceCost(const ReferencePoint & ref_point) const;
};
}  // namespace autoware::path_optimizer
#endif  // AUTOWARE__PATH_OPTIMIZER__MPT_OPTIMIZER_HPP_
