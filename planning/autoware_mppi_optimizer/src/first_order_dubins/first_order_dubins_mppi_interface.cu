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

#include "autoware/mppi_optimizer/first_order_dubins_mppi_cost_params.hpp"
#include "autoware/mppi_optimizer/first_order_dubins_mppi_interface.hpp"
#include "autoware/mppi_optimizer/mppi_debug_trajectory_logger.hpp"
#include "autoware/mppi_optimizer/tracked_objects_obstacles.hpp"

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost_bridge.hpp>
#include <mppi/cost_functions/moving_car_obstacles.hpp>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>
#include <mppi/feedback_controllers/zero_feedback.cuh>
#include <mppi/path/path2d.hpp>
#include <mppi/path/path_projection.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/utils/gpu_err_chk.cuh>
#include <rclcpp/logging.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace autoware::mppi_optimizer
{
namespace
{
constexpr int kMppiHorizon = 80;
constexpr int kRefHorizon = kMppiHorizon;
constexpr float kDt = 0.1F;
constexpr int kNumRollouts = 32 * 1024;
constexpr float kLambda = 1500.0F;
constexpr float kInitArcLength = 1.5F;
constexpr size_t kTrackingIndexResetThreshold = 15U;
// constexpr int kMaxVizRollouts = 200;  // rollout viz disabled
constexpr char kLoggerName[] = "first_order_dubins_mppi";

rclcpp::Logger mppiLogger()
{
  return rclcpp::get_logger(kLoggerName);
}

using DYN = FirstOrderDubinsBicycle;
using COST = FirstOrderDubinsBicycleCost<kRefHorizon>;
using FB = ZeroFeedback<DYN, kMppiHorizon>;
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

void applyUserCostParams(
  FirstOrderDubinsBicycleCostParams<kRefHorizon> & cost_params,
  const FirstOrderDubinsMppiCostParams & user)
{
  cost_params.desired_speed = user.desired_speed;
  cost_params.speed_coeff = user.speed_coeff;
  cost_params.track_coeff = user.track_coeff;
  cost_params.heading_coeff = user.heading_coeff;
  cost_params.crash_coeff = user.crash_coeff;
  cost_params.boundary_threshold = user.boundary_threshold;
  cost_params.boundary_threshold_left = user.boundary_threshold_left;
  cost_params.boundary_threshold_right = user.boundary_threshold_right;
  cost_params.accel_cmd_coeff = user.accel_cmd_coeff;
  cost_params.steer_cmd_coeff = user.steer_cmd_coeff;
  cost_params.lateral_acceleration_coeff = user.lateral_acceleration_coeff;
  cost_params.lateral_jerk_coeff = user.lateral_jerk_coeff;
  cost_params.longitudinal_jerk_coeff = user.longitudinal_jerk_coeff;
  cost_params.obstacle_collision_margin = user.obstacle_collision_margin;
  cost_params.goal_pos_coeff = user.goal_pos_coeff;
  cost_params.goal_speed_coeff = user.goal_speed_coeff;
  cost_params.goal_yaw_coeff = user.goal_yaw_coeff;
  cost_params.goal_terminal_scale = user.goal_terminal_scale;
}

FirstOrderDubinsMppiState toHostState(const DYN::state_array & x)
{
  FirstOrderDubinsMppiState state;
  state.x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
  state.y = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
  state.yaw = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
  state.vel_x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
  return state;
}

void fromHostState(DYN::state_array & x, const FirstOrderDubinsMppiState & state)
{
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)) = state.x;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)) = state.y;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)) = state.yaw;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)) = state.vel_x;
}

mppi::path::Path2D trajectoryToPath2D(const Trajectory & trajectory)
{
  std::vector<std::pair<float, float>> control_xy;
  control_xy.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    control_xy.emplace_back(
      static_cast<float>(point.pose.position.x), static_cast<float>(point.pose.position.y));
  }
  if (control_xy.size() < 2U) {
    throw std::runtime_error("Trajectory must contain at least two points for MPPI tracking");
  }
  return mppi::path::Path2D::catmullRom(control_xy, false, 8);
}

float yawFromOdometry(const Odometry & odometry)
{
  return static_cast<float>(tf2::getYaw(odometry.pose.pose.orientation));
}

float longitudinalAccelerationMps2(
  const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration)
{
  if (!acceleration.has_value()) {
    return 0.0F;
  }
  return static_cast<float>(acceleration->accel.accel.linear.x);
}

float steeringTireAngleRad(
  const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status)
{
  if (!steering_status.has_value()) {
    return 0.0F;
  }
  return steering_status->steering_tire_angle;
}

geometry_msgs::msg::Quaternion quaternionFromYaw(const float yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

size_t findTrackingStartIndex(const Trajectory & trajectory, const Odometry & odometry)
{
  const float ego_x = static_cast<float>(odometry.pose.pose.position.x);
  const float ego_y = static_cast<float>(odometry.pose.pose.position.y);

  const auto dist_sq_to_ego = [&](const auto & point) {
    const float dx = static_cast<float>(point.pose.position.x) - ego_x;
    const float dy = static_cast<float>(point.pose.position.y) - ego_y;
    return dx * dx + dy * dy;
  };

  const auto nearest = std::min_element(
    trajectory.points.begin(), trajectory.points.end(),
    [&](const auto & a, const auto & b) { return dist_sq_to_ego(a) < dist_sq_to_ego(b); });
  return static_cast<size_t>(std::distance(trajectory.points.begin(), nearest));
}

std::vector<mppi::path::PathReferenceSample> buildDiffusionReferenceHorizon(
  const Trajectory & trajectory, const size_t start_idx, const mppi::path::Path2D & path)
{
  std::vector<mppi::path::PathReferenceSample> ref(static_cast<size_t>(kRefHorizon));
  float arc_hint = kInitArcLength;

  size_t k = 0;
  for (auto & sample : ref) {
    const size_t idx = std::min(start_idx + k, trajectory.points.size() - 1U);
    const auto & point = trajectory.points[idx];

    sample.t = static_cast<float>(k) * kDt;
    sample.x = static_cast<float>(point.pose.position.x);
    sample.y = static_cast<float>(point.pose.position.y);
    sample.yaw = static_cast<float>(tf2::getYaw(point.pose.orientation));
    sample.v = point.longitudinal_velocity_mps;

    const mppi::path::PathProjection proj =
      mppi::path::projectPoseOntoPath(path, sample.x, sample.y, arc_hint);
    sample.arc_length_s = proj.arc_length_s;
    arc_hint = proj.arc_length_s;
    ++k;
  }

  return ref;
}

#if 0  // rollout visualization disabled (~80ms CPU replay of top-K samples)
void replayRolloutPoints(
  DYN & model, const DYN::state_array & x0, const float * controls, const int horizon,
  const float dt, std::vector<std::pair<float, float>> & points)
{
  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);

  points.clear();
  points.reserve(static_cast<size_t>(horizon) + 1U);
  points.emplace_back(x0(pos_x_idx), x0(pos_y_idx));

  DYN::state_array x = x0;
  DYN::state_array x_next = model.getZeroState();
  DYN::state_array xdot = model.getZeroState();
  DYN::output_array y = DYN::output_array::Zero();
  DYN::control_array u = DYN::control_array::Zero();

  for (int t = 0; t < horizon; ++t) {
    for (int d = 0; d < DYN::CONTROL_DIM; ++d) {
      u(d) = controls
        [(static_cast<size_t>(t) * static_cast<size_t>(DYN::CONTROL_DIM)) + static_cast<size_t>(d)];
    }
    model.enforceConstraints(x, u);
    model.step(x, x_next, xdot, u, y, static_cast<float>(t), dt);
    points.emplace_back(x_next(pos_x_idx), x_next(pos_y_idx));
    x = x_next;
  }
}
#endif

void fillOptimalHorizonPoints(
  const Mppi::state_trajectory & state_trajectory, std::vector<std::pair<float, float>> & points)
{
  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);
  const int horizon = static_cast<int>(state_trajectory.cols());

  points.clear();
  points.reserve(static_cast<size_t>(horizon));
  for (int col = 0; col < horizon; ++col) {
    points.emplace_back(state_trajectory(pos_x_idx, col), state_trajectory(pos_y_idx, col));
  }
}

#if 0  // rollout visualization disabled
void copySamplerControlsToHost(
  SAMPLER & sampler, const int horizon, const std::vector<int> & rollout_indices,
  std::vector<float> & host_controls)
{
  const size_t rollout_stride =
    static_cast<size_t>(horizon) * static_cast<size_t>(DYN::CONTROL_DIM) * sizeof(float);
  host_controls.assign(
    rollout_indices.size() * static_cast<size_t>(horizon) * static_cast<size_t>(DYN::CONTROL_DIM),
    0.0F);

  size_t out_idx = 0;
  for (const int rollout : rollout_indices) {
    float * device_controls = sampler.getControlSample(rollout, 0, 0);
    HANDLE_ERROR(cudaMemcpy(
      host_controls.data() +
        out_idx * static_cast<size_t>(horizon) * static_cast<size_t>(DYN::CONTROL_DIM),
      device_controls, rollout_stride, cudaMemcpyDeviceToHost));
    ++out_idx;
  }
}

void selectTopRolloutIndices(
  const std::vector<float> & normalized_weights, const std::vector<float> & raw_costs,
  const int top_n, std::vector<int> & rollout_indices, std::vector<float> & selected_costs)
{
  const int num_rollouts = static_cast<int>(normalized_weights.size());
  rollout_indices.resize(static_cast<size_t>(num_rollouts));
  selected_costs = raw_costs;
  std::iota(rollout_indices.begin(), rollout_indices.end(), 0);

  if (top_n <= 0 || top_n >= num_rollouts) {
    return;
  }

  std::partial_sort(
    rollout_indices.begin(), rollout_indices.begin() + top_n, rollout_indices.end(),
    [&](const int a, const int b) {
      return normalized_weights[static_cast<size_t>(a)] >
             normalized_weights[static_cast<size_t>(b)];
    });
  rollout_indices.resize(static_cast<size_t>(top_n));
  selected_costs.resize(static_cast<size_t>(top_n));
  std::transform(
    rollout_indices.begin(), rollout_indices.end(), selected_costs.begin(),
    [&](const int rollout) { return raw_costs[static_cast<size_t>(rollout)]; });
}

void buildRolloutVisualization(
  Mppi & controller, SAMPLER & sampler, DYN & model, const DYN::state_array & x_at_optimization,
  FirstOrderDubinsMppiDebug & debug)
{
  const Mppi::state_trajectory state_trajectory = controller.getActualStateSeq();
  fillOptimalHorizonPoints(state_trajectory, debug.optimal_horizon);
  debug.baseline_cost = controller.getBaselineCost();

  const auto & importance = controller.getSampledCostSeq();
  const float baseline = controller.getBaselineCost();
  const int num_rollouts = static_cast<int>(importance.size());

  std::vector<float> raw_costs(static_cast<size_t>(num_rollouts));
  std::vector<float> normalized_weights(static_cast<size_t>(num_rollouts));
  const float normalizer = controller.getNormalizerCost();
  for (size_t i = 0; i < normalized_weights.size(); ++i) {
    const float w = static_cast<float>(importance(static_cast<int>(i)));
    normalized_weights[i] = (normalizer > 0.0F) ? w / normalizer : 0.0F;
    raw_costs[i] = (w > 0.0F) ? (baseline - kLambda * std::log(w)) : (baseline + 1.0e30F);
  }

  std::vector<int> rollout_indices;
  std::vector<float> selected_costs;
  selectTopRolloutIndices(
    normalized_weights, raw_costs, kMaxVizRollouts, rollout_indices, selected_costs);

  std::vector<float> host_controls;
  copySamplerControlsToHost(sampler, kMppiHorizon, rollout_indices, host_controls);

  debug.rollouts.clear();
  debug.rollouts.reserve(rollout_indices.size());
  size_t out_idx = 0;
  for (const float cost : selected_costs) {
    FirstOrderDubinsMppiRollout rollout;
    rollout.cost = cost;
    const float * controls = host_controls.data() + out_idx * static_cast<size_t>(kMppiHorizon) *
                                                      static_cast<size_t>(DYN::CONTROL_DIM);
    replayRolloutPoints(model, x_at_optimization, controls, kMppiHorizon, kDt, rollout.points);
    debug.rollouts.push_back(std::move(rollout));
    ++out_idx;
  }
}
#endif

}  // namespace

struct FirstOrderDubinsMppiInterface::Impl
{
  Trajectory diffusion_reference;
  TrackedObjects tracked_objects;
  mppi::path::Path2D path;
  std::vector<mppi::cost::MovingCarObstacle> obstacles;

  DYN model;
  FirstOrderDubinsBicycleParams dyn;
  FirstOrderDubinsMppiVehicleParams vehicle_params{};
  FirstOrderDubinsMppiCostParams user_cost_params_{};
  MppiDebugTrajectoryLogger debug_trajectory_logger;
  COST cost;
  FirstOrderDubinsBicycleCostParams<kRefHorizon> cost_params{};
  SAMPLER sampler;
  FB feedback;
  std::unique_ptr<Mppi> controller;
  Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
  Mppi::control_trajectory u_opt = Mppi::control_trajectory::Zero();
  DYN::state_array x = DYN::state_array::Zero();

  std::vector<float> obs_traj_x;
  std::vector<float> obs_traj_y;
  std::vector<float> obs_traj_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;

  bool initialized{false};
  int step_count{0};
  size_t tracking_start_idx{0U};
  float arc_length{kInitArcLength};
  float sim_time{0.0F};
  bool ignore_obstacles{false};
  bool ignore_drivable_area{false};
  bool force_cold_start_each_step{false};

  Impl() : feedback(&model, kDt), sampler(SAMPLER::SAMPLING_PARAMS_T{}) {}

  void setup()
  {
    dyn = FirstOrderDubinsBicycleParams{};
    dyn.wheel_base = vehicle_params.wheel_base;
    dyn.max_steer_angle = vehicle_params.max_steer_angle;
    dyn.accel_time_constant = vehicle_params.acc_time_constant;
    dyn.steer_time_constant = vehicle_params.steer_time_constant;
    dyn.max_steer_rate = vehicle_params.steer_rate_lim;
    dyn.min_accel = vehicle_params.min_accel();
    dyn.max_accel = vehicle_params.max_accel();
    model.setParams(dyn);

    if (vehicle_params.acc_time_delay > 0.0F || vehicle_params.steer_time_delay > 0.0F) {
      RCLCPP_WARN_ONCE(
        mppiLogger(),
        "MPPI FirstOrderDubinsBicycle ignores acc_time_delay=%.3f and steer_time_delay=%.3f "
        "(no dead-time state in dynamics model)",
        vehicle_params.acc_time_delay, vehicle_params.steer_time_delay);
    }

    cost.GPUSetup();

    cost_params = FirstOrderDubinsBicycleCostParams<kRefHorizon>{};
    applyUserCostParams(cost_params, user_cost_params_);
    mppi::cost::fillFirstOrderDubinsBicycleCostGeometry<kRefHorizon>(cost_params, dyn);

    cost_params.ego_length = vehicle_params.ego_length;
    cost_params.ego_width = vehicle_params.ego_width;
    cost_params.ego_axle_to_box_center = vehicle_params.ego_axle_to_box_center;
    cost.setParams(cost_params);

    const float kMaxSteer = dyn.max_steer_angle;
    std::array<float2, DYN::CONTROL_DIM> u_rng{};
    u_rng[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] = {
      dyn.min_accel, dyn.max_accel};
    u_rng[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] = {
      -kMaxSteer, kMaxSteer};
    model.setControlRanges(u_rng);

    SAMPLER::SAMPLING_PARAMS_T sp{};
    sp.std_dev[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] =
      0.35F;
    // Steer exploration scales with wheelbase: larger vehicles need larger delta-steer to change
    // yaw.
    constexpr float kReferenceWheelBase = 0.32F;
    constexpr float kReferenceSteerStd = 0.03F;
    const float steer_std = std::clamp(
      kReferenceSteerStd * (vehicle_params.wheel_base / kReferenceWheelBase), kReferenceSteerStd,
      0.12F);

    sp.std_dev[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] =
      steer_std;
    sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
    sampler = SAMPLER(sp);

    controller = std::make_unique<Mppi>(
      &model, &cost, &feedback, &sampler, kDt, 1, kLambda, 0.0F, kMppiHorizon, u_nom);
    auto cp = controller->getParams();
    cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
    cp.cost_rollout_dim_ = dim3(32, 2, 1);
    cp.seed_ = 1U;
    controller->setParams(cp);
    controller->setPercentageSampledControlTrajectories(128.0F / static_cast<float>(kNumRollouts));

    model.GPUSetup();

    initialized = true;
    step_count = 0;
    tracking_start_idx = 0U;
    arc_length = kInitArcLength;
    sim_time = 0.0F;

    RCLCPP_INFO(
      mppiLogger(),
      "MPPI GPU initialized (horizon=%d, rollouts=%d, dt=%.2f, lambda=%.1f, "
      "wheel_base=%.2f, max_steer=%.2f, steer_std=%.3f, acc_tau=%.2f, steer_tau=%.2f, "
      "steer_rate_lim=%.2f, vel_rate_lim=%.2f, ego=%.2fx%.2f, axle_to_center=%.2f, "
      "desired_speed=%.2f, boundary_threshold=%.2f, obs_margin=%.2f)",
      kMppiHorizon, kNumRollouts, kDt, kLambda, vehicle_params.wheel_base,
      vehicle_params.max_steer_angle, steer_std, vehicle_params.acc_time_constant,
      vehicle_params.steer_time_constant, vehicle_params.steer_rate_lim,
      vehicle_params.vel_rate_lim, vehicle_params.ego_length, vehicle_params.ego_width,
      vehicle_params.ego_axle_to_box_center, cost_params.desired_speed,
      cost_params.boundary_threshold, cost_params.obstacle_collision_margin);
  }

  void resetTrackingState()
  {
    step_count = 0;
    u_opt.setZero();
    arc_length = kInitArcLength;
    sim_time = 0.0F;
  }

  void seedNominalControlFromDiffusionReference(
    const Trajectory & reference, const size_t start_idx)
  {
    if (reference.points.empty()) {
      u_nom.setZero();
      return;
    }

    const int accel_idx =
      static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
    const int steer_idx = static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);
    const float min_accel = vehicle_params.min_accel();
    const float max_accel = vehicle_params.max_accel();
    const float max_steer = vehicle_params.max_steer_angle;
    const float wheel_base = vehicle_params.wheel_base;

    for (int t = 0; t < kMppiHorizon; ++t) {
      const size_t idx = std::min(start_idx + static_cast<size_t>(t), reference.points.size() - 1U);
      const auto & point = reference.points[idx];
      u_nom(accel_idx, t) = std::clamp(point.acceleration_mps2, min_accel, max_accel);

      float steer = point.front_wheel_angle_rad;
      if (std::abs(steer) <= 1.0E-6F && idx + 1U < reference.points.size()) {
        const auto & next = reference.points[idx + 1U];
        const float dx = static_cast<float>(next.pose.position.x - point.pose.position.x);
        const float dy = static_cast<float>(next.pose.position.y - point.pose.position.y);
        const float ds = std::hypot(dx, dy);
        if (ds > 1.0E-6F) {
          const float yaw0 = static_cast<float>(tf2::getYaw(point.pose.orientation));
          const float yaw1 = static_cast<float>(tf2::getYaw(next.pose.orientation));
          const float dyaw = std::atan2(std::sin(yaw1 - yaw0), std::cos(yaw1 - yaw0));
          steer = std::atan(wheel_base * (dyaw / ds));
        }
      }
      u_nom(steer_idx, t) = std::clamp(steer, -max_steer, max_steer);
    }
  }

  void updateDiffusionReference(
    const Trajectory & reference, const Odometry & odometry,
    const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
    const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
    const TrackedObjects & tracked_objects_in)
  {
    if (!initialized) {
      setup();
    }

    if (force_cold_start_each_step) {
      resetTrackingState();
    }

    diffusion_reference = reference;
    tracked_objects = ignore_obstacles ? TrackedObjects{} : tracked_objects_in;
    path = trajectoryToPath2D(reference);
    obstacles.clear();
    // Boundary crash is disabled on this stack (isEgoOutsideDrivableArea always false).
    // ignore_drivable_area remains an ablation API flag; it does not reintroduce road borders.
    (void)ignore_drivable_area;

    const size_t new_start_idx = findTrackingStartIndex(reference, odometry);
    const bool large_index_jump =
      step_count > 0 && (new_start_idx > tracking_start_idx + kTrackingIndexResetThreshold ||
                         tracking_start_idx > new_start_idx + kTrackingIndexResetThreshold);
    if (step_count == 0 || large_index_jump) {
      resetTrackingState();
    }
    tracking_start_idx = new_start_idx;
    seedNominalControlFromDiffusionReference(reference, tracking_start_idx);

    const float ego_yaw = yawFromOdometry(odometry);
    const float ego_v = static_cast<float>(odometry.twist.twist.linear.x);

    x = model.getZeroState();
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)) =
      static_cast<float>(odometry.pose.pose.position.x);
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)) =
      static_cast<float>(odometry.pose.pose.position.y);
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)) = ego_yaw;
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)) = ego_v;
    const float ego_accel = std::clamp(
      longitudinalAccelerationMps2(acceleration), vehicle_params.min_accel(),
      vehicle_params.max_accel());
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::ACCELERATION)) = ego_accel;
    const float ego_steer = std::clamp(
      steeringTireAngleRad(steering_status), -vehicle_params.max_steer_angle,
      vehicle_params.max_steer_angle);
    x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE)) = ego_steer;

    const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(
      path, x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)),
      x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)), arc_length);
    arc_length = proj.arc_length_s;
  }

  FirstOrderDubinsMppiControl runStep()
  {
    const std::vector<mppi::path::PathReferenceSample> ref =
      buildDiffusionReferenceHorizon(diffusion_reference, tracking_start_idx, path);
    mppi::cost::fillFirstOrderDubinsBicycleCostFromPathReference<kRefHorizon>(cost, ref);

    int obstacle_count = 0;
    if (!tracked_objects.objects.empty()) {
      buildObstacleTrajectoryBuffersFromTrackedObjects(
        tracked_objects, kDt, kRefHorizon, obs_traj_x, obs_traj_y, obs_traj_yaw, obs_half_length,
        obs_half_width);
      obstacle_count = trackedObjectObstacleCount(tracked_objects);
    } else if (!obstacles.empty()) {
      mppi::cost::buildObstacleTrajectoryBuffers(
        obstacles, sim_time, kDt, kRefHorizon, obs_traj_x, obs_traj_y, obs_traj_yaw,
        obs_half_length, obs_half_width);
      obstacle_count =
        static_cast<int>(std::min(obstacles.size(), static_cast<size_t>(kMaxMppiObstacles)));
    } else {
      obs_traj_x.clear();
      obs_traj_y.clear();
      obs_traj_yaw.clear();
      obs_half_length.clear();
      obs_half_width.clear();
    }
    mppi::cost::fillFirstOrderDubinsBicycleCostObstacleTrajectories<kRefHorizon>(
      cost, obstacle_count > 0 ? obs_traj_x.data() : nullptr,
      obstacle_count > 0 ? obs_traj_y.data() : nullptr,
      obstacle_count > 0 ? obs_traj_yaw.data() : nullptr,
      obstacle_count > 0 ? obs_half_length.data() : nullptr,
      obstacle_count > 0 ? obs_half_width.data() : nullptr, obstacle_count, kRefHorizon);

    controller->updateImportanceSampler(u_nom);
    controller->computeControl(x, 1);
    cudaStreamSynchronize(controller->stream_);

    const Mppi::control_trajectory u_opt_traj = controller->getControlSeq();
    u_opt = u_opt_traj;

    DYN::control_array u_apply = u_opt_traj.col(0);
    model.enforceConstraints(x, u_apply);

    DYN::state_array x_next = model.getZeroState();
    DYN::state_array xdot = model.getZeroState();
    DYN::output_array y = DYN::output_array::Zero();
    model.step(x, x_next, xdot, u_apply, y, static_cast<float>(step_count), kDt);
    x = x_next;

    const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(
      path, x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)),
      x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)), arc_length);
    arc_length = proj.arc_length_s;

    ++step_count;
    sim_time += kDt;

    FirstOrderDubinsMppiControl control;
    control.accel_cmd =
      u_apply(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD));
    control.steer_cmd =
      u_apply(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD));

    RCLCPP_DEBUG(
      mppiLogger(),
      "MPPI track step %d: start_idx=%zu arc_s=%.2f ref_v0=%.2f u_accel=%.3f u_steer=%.3f "
      "ego_v=%.2f baseline_cost=%.2f",
      step_count, tracking_start_idx, arc_length, ref.empty() ? 0.0F : ref.front().v,
      control.accel_cmd, control.steer_cmd,
      x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)),
      controller->getBaselineCost());

    return control;
  }

  void teardown()
  {
    if (initialized) {
      cost.freeCudaMem();
      initialized = false;
    }
  }
};

FirstOrderDubinsMppiInterface::FirstOrderDubinsMppiInterface() : impl_(std::make_unique<Impl>())
{
}

FirstOrderDubinsMppiInterface::~FirstOrderDubinsMppiInterface()
{
  if (impl_) {
    impl_->teardown();
  }
}

FirstOrderDubinsMppiInterface::FirstOrderDubinsMppiInterface(
  FirstOrderDubinsMppiInterface && other) noexcept = default;

FirstOrderDubinsMppiInterface & FirstOrderDubinsMppiInterface::operator=(
  FirstOrderDubinsMppiInterface && other) noexcept = default;

void FirstOrderDubinsMppiInterface::initialize()
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->setup();
}

bool FirstOrderDubinsMppiInterface::isInitialized() const
{
  return impl_ && impl_->initialized;
}

void FirstOrderDubinsMppiInterface::setVehicleParams(
  const FirstOrderDubinsMppiVehicleParams & params)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  if (impl_->initialized) {
    impl_->teardown();
  }
  impl_->vehicle_params = params;
}

void FirstOrderDubinsMppiInterface::setCostParams(const FirstOrderDubinsMppiCostParams & params)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  if (impl_->initialized) {
    impl_->teardown();
  }
  impl_->user_cost_params_ = params;
}

void FirstOrderDubinsMppiInterface::setDebugTrajectoryLogging(
  const bool enable, const std::string & directory)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->debug_trajectory_logger.configure(enable, directory);
  impl_->debug_trajectory_logger.writeParamsOnce(impl_->user_cost_params_, impl_->vehicle_params);
}

void FirstOrderDubinsMppiInterface::setAblationOptions(
  const bool ignore_obstacles, const bool ignore_drivable_area,
  const bool force_cold_start_each_step)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  impl_->ignore_obstacles = ignore_obstacles;
  impl_->ignore_drivable_area = ignore_drivable_area;
  impl_->force_cold_start_each_step = force_cold_start_each_step;
  RCLCPP_INFO(
    mppiLogger(),
    "MPPI ablation options: ignore_obstacles=%s ignore_drivable_area=%s "
    "force_cold_start_each_step=%s",
    ignore_obstacles ? "true" : "false", ignore_drivable_area ? "true" : "false",
    force_cold_start_each_step ? "true" : "false");
}

bool FirstOrderDubinsMppiInterface::copySampleCostDistribution(
  std::vector<float> & raw_costs, std::vector<float> & normalized_weights, const int stride) const
{
  raw_costs.clear();
  normalized_weights.clear();
  if (!impl_ || !impl_->controller || !impl_->initialized) {
    return false;
  }

  // IMPORTANT: take by value (not const-ref-to-temporary). nvcc has historically broken
  // lifetime extension for large Eigen return temporaries, which caused heap corruption
  // (munmap_chunk: invalid pointer) when reading getSampledCostSeq() via const auto&.
  const Mppi::sampled_cost_traj importance = impl_->controller->getSampledCostSeq();
  const float baseline = impl_->controller->getBaselineCost();
  const float normalizer = impl_->controller->getNormalizerCost();
  const float lambda = std::max(impl_->user_cost_params_.lambda, 1.0e-6F);
  const int stride_n = std::max(1, stride);
  const int num_rollouts = static_cast<int>(importance.size());
  const int kept = (num_rollouts + stride_n - 1) / stride_n;
  raw_costs.reserve(static_cast<size_t>(kept));
  normalized_weights.reserve(static_cast<size_t>(kept));

  for (int i = 0; i < num_rollouts; i += stride_n) {
    const float w = importance(i);
    normalized_weights.push_back((normalizer > 0.0F) ? (w / normalizer) : 0.0F);
    raw_costs.push_back((w > 0.0F) ? (baseline - lambda * std::log(w)) : (baseline + 1.0e6F));
  }
  return !raw_costs.empty();
}

FirstOrderDubinsMppiControl FirstOrderDubinsMppiInterface::computeStep(
  FirstOrderDubinsMppiState & state, float & arc_length, float sim_time)
{
  if (!impl_ || !impl_->initialized) {
    throw std::runtime_error(
      "FirstOrderDubinsMppiInterface must be initialized before computeStep");
  }

  fromHostState(impl_->x, state);
  impl_->arc_length = arc_length;
  impl_->sim_time = sim_time;
  const FirstOrderDubinsMppiControl control = impl_->runStep();
  state = toHostState(impl_->x);
  arc_length = impl_->arc_length;
  return control;
}

FirstOrderDubinsMppiOptimizationResult FirstOrderDubinsMppiInterface::optimizeTrajectory(
  const Trajectory & input, const Odometry & odometry,
  const std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> & acceleration,
  const std::optional<autoware_vehicle_msgs::msg::SteeringReport> & steering_status,
  const TrackedObjects & tracked_objects)
{
  if (!impl_) {
    throw std::runtime_error("FirstOrderDubinsMppiInterface implementation is missing");
  }
  FirstOrderDubinsMppiOptimizationResult result;
  if (input.points.size() < 2U) {
    RCLCPP_WARN(
      mppiLogger(), "MPPI skipped: trajectory has %zu points (need >= 2)", input.points.size());
    result.trajectory = input;
    result.debug.reference_trajectory = input;
    result.debug.optimized_trajectory = input;
    return result;
  }

  const auto start_time = std::chrono::steady_clock::now();

  impl_->updateDiffusionReference(input, odometry, acceleration, steering_status, tracked_objects);
  const FirstOrderDubinsMppiControl control = impl_->runStep();

  const auto state_trajectory = impl_->controller->getActualStateSeq();
  const Mppi::control_trajectory u_opt_traj = impl_->controller->getControlSeq();
  Trajectory output = input;

  const int pos_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int pos_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);
  const int yaw_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW);
  const int vel_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X);
  const int accel_cmd_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD);
  const int steer_cmd_idx =
    static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD);

  // GPU costs post-step state at timestep t against ref[t] (= DP[t] at time (t+1)*dt).
  // Map: points[i] <- state[i+1], control[i].
  //
  // Vendor mismatch: GPU rollouts take H steps (mppi_common.cu: t = 0..H-1) and cost the
  // final post-step state x[H], but host getActualStateSeq() only runs H-1 steps
  // (controller.cuh: i < num_timesteps - 1) and stores x[0..H-1]. Reconstruct x[H] here
  // with one dynamics step so the last DP point is not left as a duplicate of x[H-1].
  const int n_state = static_cast<int>(state_trajectory.cols());
  const int n_ctrl = static_cast<int>(u_opt_traj.cols());
  const size_t num_points = std::min(
    {output.points.size(), static_cast<size_t>(std::max(0, n_state)),
     static_cast<size_t>(std::max(0, n_ctrl))});

  DYN::state_array x_final = DYN::state_array::Zero();
  DYN::state_array x_final_dot = DYN::state_array::Zero();
  DYN::output_array y_final = DYN::output_array::Zero();
  const bool have_final_state = n_state > 0 && n_ctrl > 0;
  if (have_final_state) {
    DYN::state_array x_tail = state_trajectory.col(n_state - 1);
    DYN::control_array u_tail = u_opt_traj.col(n_ctrl - 1);
    impl_->model.enforceConstraints(x_tail, u_tail);
    impl_->model.step(
      x_tail, x_final, x_final_dot, u_tail, y_final, static_cast<float>(n_state - 1), kDt);
  }

  float max_pos_delta = 0.0F;
  float max_vel_delta = 0.0F;
  size_t i = 0;
  for (auto & out_point : output.points) {
    if (i >= num_points) {
      break;
    }
    const auto & in_point = input.points[i];
    const int control_col = static_cast<int>(i);
    const bool use_final = (control_col + 1 >= n_state) && have_final_state;

    const float tracked_x =
      use_final ? x_final(pos_x_idx) : state_trajectory(pos_x_idx, control_col + 1);
    const float tracked_y =
      use_final ? x_final(pos_y_idx) : state_trajectory(pos_y_idx, control_col + 1);
    const float tracked_yaw =
      use_final ? x_final(yaw_idx) : state_trajectory(yaw_idx, control_col + 1);
    const float tracked_v =
      use_final ? x_final(vel_x_idx) : state_trajectory(vel_x_idx, control_col + 1);

    const float ref_x = static_cast<float>(in_point.pose.position.x);
    const float ref_y = static_cast<float>(in_point.pose.position.y);
    const float ref_v = in_point.longitudinal_velocity_mps;

    max_pos_delta = std::max(max_pos_delta, std::hypot(tracked_x - ref_x, tracked_y - ref_y));
    max_vel_delta = std::max(max_vel_delta, std::abs(tracked_v - ref_v));

    out_point.pose.position.x = tracked_x;
    out_point.pose.position.y = tracked_y;
    out_point.pose.position.z = in_point.pose.position.z;
    out_point.pose.orientation = quaternionFromYaw(tracked_yaw);
    out_point.longitudinal_velocity_mps = tracked_v;
    out_point.acceleration_mps2 = u_opt_traj(accel_cmd_idx, control_col);
    out_point.front_wheel_angle_rad = u_opt_traj(steer_cmd_idx, control_col);
    ++i;
  }

  const auto elapsed_ms =
    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start_time)
      .count();

  result.trajectory = output;
  result.debug.reference_trajectory = input;
  result.debug.optimized_trajectory = output;
  // Rollout visualization disabled (CPU replay of top-K samples was ~80ms).
  fillOptimalHorizonPoints(impl_->controller->getActualStateSeq(), result.debug.optimal_horizon);
  result.debug.baseline_cost = impl_->controller->getBaselineCost();

  MppiDebugEgoState ego;
  ego.x = odometry.pose.pose.position.x;
  ego.y = odometry.pose.pose.position.y;
  ego.z = odometry.pose.pose.position.z;
  ego.yaw = yawFromOdometry(odometry);
  ego.v = odometry.twist.twist.linear.x;
  ego.accel = longitudinalAccelerationMps2(acceleration);
  ego.steer = steeringTireAngleRad(steering_status);
  impl_->debug_trajectory_logger.writeParamsOnce(impl_->user_cost_params_, impl_->vehicle_params);
  impl_->debug_trajectory_logger.logFrame(
    result.debug.reference_trajectory, result.debug.optimized_trajectory, ego,
    result.debug.baseline_cost);

  RCLCPP_INFO(
    mppiLogger(),
    "MPPI tracked diffusion ref in %.1f ms: start_idx=%zu steps=%d output points size=%zu "
    "points=%zu rollouts=%zu "
    "obstacles=%zu u_accel=%.3f u_steer=%.3f baseline_cost=%.2f max_pos_err=%.3f m "
    "max_vel_err=%.3f m/s",
    elapsed_ms, impl_->tracking_start_idx, impl_->step_count, output.points.size(), num_points,
    result.debug.rollouts.size(), tracked_objects.objects.size(), control.accel_cmd,
    control.steer_cmd, result.debug.baseline_cost, max_pos_delta, max_vel_delta);

  return result;
}

}  // namespace autoware::mppi_optimizer
