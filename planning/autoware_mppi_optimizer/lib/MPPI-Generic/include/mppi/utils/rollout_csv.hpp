/**
 * Header-only CSV writers for MPPI rollout / iteration analysis.
 *
 * All writers are templated on the dynamics type so they can be reused across
 * different examples. The dynamics type must expose `DYN_T::DYN_PARAMS_T` with
 * `StateIndex`, `OutputIndex`, and `ControlIndex` enums.
 *
 * Schemas (in lock-step with `scripts/mppi/plot_mppi_rollout_analysis.py` and
 * `scripts/mppi/plot_mppi_rollouts_at_step.py`):
 *   <prefix>_centerline.csv : x_m,y_m
 *   <prefix>_meta.csv       : key,value
 *   <prefix>_costs.csv      : rollout_index,raw_cost,unnormalized_importance,normalized_weight
 *   <prefix>_combined.csv   : step,t,x,y,yaw,vel_x,steer,u_accel,u_steer
 *   <prefix>_rollouts_xy.csv: rollout_index,step,x,y,yaw,vel_x
 *   <prefix>_rollouts_controls.csv: rollout_index,step,u_accel,u_steer
 *   <prefix>_context.csv       : index,value  (diffusion obstacle context vector)
 */
#ifndef MPPI_UTILS_ROLLOUT_CSV_HPP_
#define MPPI_UTILS_ROLLOUT_CSV_HPP_

#include <Eigen/Dense>
#include <mppi/path/path2d.hpp>

#include <fstream>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace mppi
{
namespace rollout_csv
{
inline void writeCenterline(const mppi::path::Path2D& path, const std::string& prefix, bool log_path = true)
{
  const std::string out = prefix + "_centerline.csv";
  std::ofstream f(out.c_str());
  if (!f)
  {
    return;
  }
  f << "x_m,y_m\n";
  for (const mppi::path::PathAnchor& a : path.anchors())
  {
    f << a.x << "," << a.y << "\n";
  }
  if (log_path)
  {
    std::cout << "Wrote centerline: " << out << "\n";
  }
}

/**
 * Sister to `writeCenterline` for closed-loop tracking examples that pass
 * a full log path like "foo.csv": writes "foo_centerline.csv".
 */
inline void writeCenterlineForLog(const mppi::path::Path2D& path, const std::string& log_csv_path)
{
  std::string out = log_csv_path;
  const size_t n = out.size();
  if (n > 4U && out.compare(n - 4U, 4U, ".csv") == 0)
  {
    out.insert(n - 4U, "_centerline");
  }
  else
  {
    out += "_centerline.csv";
  }
  std::ofstream f(out.c_str());
  if (!f)
  {
    return;
  }
  f << "x_m,y_m\n";
  for (const mppi::path::PathAnchor& a : path.anchors())
  {
    f << a.x << "," << a.y << "\n";
  }
  std::cout << "Wrote centerline for plot: " << out << "\n";
}

template <class DYN_T>
void writeMeta(const std::string& path, const typename DYN_T::state_array& x, float dt, float lambda, int horizon,
               int num_rollouts, int num_logged_traj, float baseline, float normalizer, int sim_step = -1,
               float sim_time = -1.0F, float boundary_left = -1.0F, float boundary_right = -1.0F)
{
  using S = typename DYN_T::DYN_PARAMS_T::StateIndex;
  std::ofstream f(path.c_str());
  if (!f)
  {
    return;
  }
  f << "key,value\n";
  f << "dt," << dt << "\n";
  f << "horizon," << horizon << "\n";
  f << "num_rollouts," << num_rollouts << "\n";
  f << "num_logged_trajectories," << num_logged_traj << "\n";
  f << "lambda," << lambda << "\n";
  f << "baseline," << baseline << "\n";
  f << "normalizer," << normalizer << "\n";
  if (sim_step >= 0)
  {
    f << "sim_step," << sim_step << "\n";
  }
  if (sim_time >= 0.0F)
  {
    f << "sim_time," << sim_time << "\n";
  }
  f << "init_pos_x," << x(static_cast<int>(S::POS_X)) << "\n";
  f << "init_pos_y," << x(static_cast<int>(S::POS_Y)) << "\n";
  f << "init_yaw," << x(static_cast<int>(S::YAW)) << "\n";
  f << "init_vel_x," << x(static_cast<int>(S::VEL_X)) << "\n";
  f << "init_steer," << x(static_cast<int>(S::STEER_ANGLE)) << "\n";
  if (boundary_left >= 0.0F)
  {
    f << "boundary_threshold_left," << boundary_left << "\n";
  }
  if (boundary_right >= 0.0F)
  {
    f << "boundary_threshold_right," << boundary_right << "\n";
  }
  if (boundary_left >= 0.0F && boundary_right < 0.0F)
  {
    f << "boundary_threshold," << boundary_left << "\n";
  }
  else if (boundary_left >= 0.0F && boundary_right >= 0.0F && std::abs(boundary_left - boundary_right) < 1.0E-6F)
  {
    f << "boundary_threshold," << boundary_left << "\n";
  }
}

inline void writeCosts(const std::string& path, const std::vector<float>& raw_costs,
                       const std::vector<float>& unnormalized_importance,
                       const std::vector<float>& normalized_weights)
{
  std::ofstream f(path.c_str());
  if (!f)
  {
    return;
  }
  f << "rollout_index,raw_cost,unnormalized_importance,normalized_weight\n";
  const int n = static_cast<int>(raw_costs.size());
  for (int i = 0; i < n; ++i)
  {
    f << i << "," << raw_costs[i] << "," << unnormalized_importance[i] << "," << normalized_weights[i] << "\n";
  }
}

/**
 * Same as writeCosts but only writes the selected rollout indices (preserving
 * the original rollout_index column for cross-referencing).
 */
inline void writeCostsIndexed(const std::string& path, const std::vector<int>& rollout_indices,
                                const std::vector<float>& raw_costs,
                                const std::vector<float>& unnormalized_importance,
                                const std::vector<float>& normalized_weights)
{
  std::ofstream f(path.c_str());
  if (!f)
  {
    return;
  }
  f << "rollout_index,raw_cost,unnormalized_importance,normalized_weight\n";
  const int n = static_cast<int>(rollout_indices.size());
  for (int i = 0; i < n; ++i)
  {
    f << rollout_indices[static_cast<size_t>(i)] << "," << raw_costs[static_cast<size_t>(i)] << ","
      << unnormalized_importance[static_cast<size_t>(i)] << "," << normalized_weights[static_cast<size_t>(i)] << "\n";
  }
}

template <class DYN_T, class CONTROL_TRAJ_T>
void writeCombinedTrajectory(DYN_T& model, const typename DYN_T::state_array& x0, const CONTROL_TRAJ_T& u,
                             const std::string& path, float dt)
{
  using S = typename DYN_T::DYN_PARAMS_T::StateIndex;
  std::ofstream f(path.c_str());
  if (!f)
  {
    return;
  }
  f << "step,t,x,y,yaw,vel_x,steer,u_accel,u_steer\n";
  typename DYN_T::state_array x = x0;
  typename DYN_T::state_array x_next = model.getZeroState();
  typename DYN_T::state_array xdot = model.getZeroState();
  typename DYN_T::output_array y = DYN_T::output_array::Zero();
  typename DYN_T::control_array u_step = DYN_T::control_array::Zero();

  f << "0,0," << x(static_cast<int>(S::POS_X)) << "," << x(static_cast<int>(S::POS_Y)) << ","
    << x(static_cast<int>(S::YAW)) << "," << x(static_cast<int>(S::VEL_X)) << ","
    << x(static_cast<int>(S::STEER_ANGLE)) << ",0,0\n";

  const int steps = static_cast<int>(u.cols());
  for (int k = 0; k < steps; ++k)
  {
    u_step = u.col(k);
    model.enforceConstraints(x, u_step);
    model.step(x, x_next, xdot, u_step, y, static_cast<float>(k), dt);
    const float t = static_cast<float>(k + 1) * dt;
    f << (k + 1) << "," << t << "," << x_next(static_cast<int>(S::POS_X)) << "," << x_next(static_cast<int>(S::POS_Y))
      << "," << x_next(static_cast<int>(S::YAW)) << "," << x_next(static_cast<int>(S::VEL_X)) << ","
      << x_next(static_cast<int>(S::STEER_ANGLE)) << "," << u_step(0) << "," << u_step(1) << "\n";
    x = x_next;
  }
}

template <class DYN_T, class TrajT>
void writeRolloutTrajectories(const std::string& path, const typename DYN_T::state_array& x0, int horizon,
                              const std::vector<TrajT>& outputs, const std::vector<int>& rollout_indices, int out_x_idx,
                              int out_y_idx, int out_yaw_idx, int out_vel_idx)
{
  using S = typename DYN_T::DYN_PARAMS_T::StateIndex;
  std::ofstream f(path.c_str());
  if (!f)
  {
    return;
  }
  f << "rollout_index,step,x,y,yaw,vel_x\n";
  const int n_out = static_cast<int>(outputs.size());

  for (int r = 0; r < n_out; ++r)
  {
    const int rollout_index = rollout_indices.empty() ? r : rollout_indices[static_cast<size_t>(r)];
    f << rollout_index << ",0," << x0(static_cast<int>(S::POS_X)) << "," << x0(static_cast<int>(S::POS_Y)) << ","
      << x0(static_cast<int>(S::YAW)) << "," << x0(static_cast<int>(S::VEL_X)) << "\n";
    const TrajT& traj = outputs[static_cast<size_t>(r)];
    for (int c = 0; c < horizon; ++c)
    {
      f << rollout_index << "," << (c + 1) << "," << traj(out_x_idx, c) << "," << traj(out_y_idx, c) << ","
        << traj(out_yaw_idx, c) << "," << traj(out_vel_idx, c) << "\n";
    }
  }
}

template <class DYN_T, class TrajT>
void writeRolloutTrajectories(const std::string& path, const typename DYN_T::state_array& x0, int horizon,
                              const std::vector<TrajT>& outputs, const std::vector<int>& rollout_indices)
{
  using O = typename DYN_T::DYN_PARAMS_T::OutputIndex;
  writeRolloutTrajectories<DYN_T, TrajT>(path, x0, horizon, outputs, rollout_indices,
                                         static_cast<int>(O::POS_X), static_cast<int>(O::POS_Y),
                                         static_cast<int>(O::YAW), static_cast<int>(O::VEL_X));
}

template <class DYN_T, class TrajT>
void writeRolloutTrajectories(const std::string& path, const typename DYN_T::state_array& x0, int horizon,
                              const std::vector<TrajT>& outputs)
{
  writeRolloutTrajectories<DYN_T, TrajT>(path, x0, horizon, outputs, {});
}

template <class DYN_T>
void writeRolloutControls(const std::string& path, const std::vector<float>& host_controls, int horizon,
                          int num_rollouts, const std::vector<int>& rollout_indices)
{
  std::ofstream f(path.c_str());
  if (!f)
  {
    return;
  }
  f << "rollout_index,step,u_accel,u_steer\n";
  const size_t stride = static_cast<size_t>(horizon) * static_cast<size_t>(DYN_T::CONTROL_DIM);
  for (size_t out_idx = 0; out_idx < rollout_indices.size(); ++out_idx)
  {
    const int rollout_index = rollout_indices[out_idx];
    for (int t = 0; t < horizon; ++t)
    {
      const float* src = host_controls.data() +
                         (out_idx * stride + static_cast<size_t>(t) * static_cast<size_t>(DYN_T::CONTROL_DIM));
      f << rollout_index << "," << t << "," << src[0] << "," << src[1] << "\n";
    }
  }
}

inline void writeContextVector(const std::string& path, const float* context, int context_dim)
{
  std::ofstream f(path.c_str());
  if (!f || context == nullptr || context_dim <= 0)
  {
    return;
  }
  f << "index,value\n";
  for (int i = 0; i < context_dim; ++i)
  {
    f << i << "," << context[i] << "\n";
  }
}
}  // namespace rollout_csv
}  // namespace mppi

#endif  // MPPI_UTILS_ROLLOUT_CSV_HPP_
