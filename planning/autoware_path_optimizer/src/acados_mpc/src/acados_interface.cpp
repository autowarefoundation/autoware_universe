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

#include "acados_interface.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>

AcadosInterface::AcadosInterface(int max_iter, double tol)
{
  capsule_ = curvilinear_bicycle_model_spatial_acados_create_capsule();
  curvilinear_bicycle_model_spatial_acados_create(capsule_);

  // Example: set initial state to zero (will be overridden by setInitialState)
  double lbx0[CURVILINEAR_BICYCLE_MODEL_SPATIAL_NX] = {0};
  double ubx0[CURVILINEAR_BICYCLE_MODEL_SPATIAL_NX] = {0};

  nlp_config_ = curvilinear_bicycle_model_spatial_acados_get_nlp_config(capsule_);
  nlp_dims_ = curvilinear_bicycle_model_spatial_acados_get_nlp_dims(capsule_);
  nlp_in_ = curvilinear_bicycle_model_spatial_acados_get_nlp_in(capsule_);
  nlp_out_ = curvilinear_bicycle_model_spatial_acados_get_nlp_out(capsule_);
  nlp_solver_ = curvilinear_bicycle_model_spatial_acados_get_nlp_solver(capsule_);

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);

  setSolverOptions(max_iter, tol);
}

AcadosInterface::~AcadosInterface()
{
  curvilinear_bicycle_model_spatial_acados_free(capsule_);
  curvilinear_bicycle_model_spatial_acados_free_capsule(capsule_);
}

void AcadosInterface::setSolverOptions(int max_iter, double tol)
{
  void * nlp_opts = curvilinear_bicycle_model_spatial_acados_get_nlp_opts(capsule_);
  if (nlp_opts == nullptr) {
    std::cerr << "Warning: nlp_opts is null, cannot set solver options" << std::endl;
    return;
  }

  // set max iterations
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "max_iter", &max_iter);

  // set tolerances: set KKT/stat tolerance and related tolerances
  double tol_val = tol;
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_stat", &tol_val);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_eq", &tol_val);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_ineq", &tol_val);
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts, "tol_comp", &tol_val);
}

void AcadosInterface::setParameters(int stage, std::array<double, NP> params)
{
  curvilinear_bicycle_model_spatial_acados_update_params(
    capsule_, stage, const_cast<double *>(params.data()), NP);
}

void AcadosInterface::setParametersAllStages(std::array<double, NP> params)
{
  ocp_nlp_dims * nlp_dims = curvilinear_bicycle_model_spatial_acados_get_nlp_dims(capsule_);
  for (int i = 0; i <= nlp_dims->N; ++i) {
    curvilinear_bicycle_model_spatial_acados_update_params(
      capsule_, i, const_cast<double *>(params.data()), NP);
  }
}

void AcadosInterface::setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0)
{
  // Apply warm-start initial guesses to ocp_nlp_out for all stages
  ocp_nlp_dims * nlp_dims = curvilinear_bicycle_model_spatial_acados_get_nlp_dims(capsule_);
  for (int i = 0; i < nlp_dims->N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", const_cast<double *>(x0.data()));
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", const_cast<double *>(u0.data()));
  }
  // final state
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, nlp_dims->N, "x", const_cast<double *>(x0.data()));
}

void AcadosInterface::setInitialState(std::array<double, NX> x0)
{
  // set both lbx and ubx at stage 0 to force initial state
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", const_cast<double *>(x0.data()));
  // Do not seed the full trajectory in ocp_nlp_out here; Python only sets lbx/ubx at stage 0.
}

void AcadosInterface::setInequalityBounds(
  int stage, std::array<double, NH> lh, std::array<double, NH> uh)
{
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "lh", const_cast<double *>(lh.data()));
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, "uh", const_cast<double *>(uh.data()));
}

void AcadosInterface::applyCircleConstraintsToParams(
  int stage, std::array<double, NP> & params, const std::array<double, NH> & beta,
  const std::array<double, NH> & lh, const std::array<double, NH> & uh)
{
  // Layout at the end of p (see generator): [... base ... | cos_beta[0..NH-1] | sin_beta[0..NH-1] |
  // lon_offset[0..NH-1]]
  const size_t cos_beta_offset = NP - 3 * NH;
  const size_t sin_beta_offset = NP - 2 * NH;
  for (size_t i = 0; i < NH; ++i) {
    params[cos_beta_offset + i] = std::cos(beta[i]);
    params[sin_beta_offset + i] = std::sin(beta[i]);
  }
  setInequalityBounds(stage, lh, uh);
}

void AcadosInterface::setSoftConstraintLinearWeight(int stage, double w)
{
  // acados uses separate slack variables for lower/upper sides (s_l, s_u), each of length NSH.
  // Linear penalties are provided via "zl" and "zu".
  std::array<double, NSH> zl{};
  std::array<double, NSH> zu{};
  zl.fill(w);
  zu.fill(w);
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, stage, "zl", const_cast<double *>(zl.data()));
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, stage, "zu", const_cast<double *>(zu.data()));
}

std::array<std::array<double, NX>, N + 1> AcadosInterface::getStateTrajectory() const
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  for (size_t ii = 0; ii <= size_t(nlp_dims_->N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii]);
  }

  return xtraj;
}

std::array<std::array<double, NU>, N> AcadosInterface::getControlTrajectory() const
{
  std::array<std::array<double, NU>, N> utraj;
  for (size_t ii = 0; ii < size_t(N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii]);
  }
  return utraj;
}

int AcadosInterface::solve()
{
  int status = curvilinear_bicycle_model_spatial_acados_solve(capsule_);
  return status;
}

AcadosSolution AcadosInterface::getControl(std::array<double, NX> x0)
{
  // initial value for control input
  std::array<double, NU> u0;
  u0.fill(0.0);

  // prepare evaluation
  double kkt_norm_inf;
  double elapsed_time;
  int sqp_iter;

  // initialize solution
  setInitialState(x0);

  int status = curvilinear_bicycle_model_spatial_acados_solve(capsule_);
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);

  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

  curvilinear_bicycle_model_spatial_acados_print_stats(capsule_);

  std::stringstream ss;
  ss << "\nSolver info:" << std::endl;
  ss << " SQP iterations " << sqp_iter << "\n minimum time for " << 1 << " solve "
     << elapsed_time * 1000 << " [ms]\n KKT " << kkt_norm_inf << std::endl;

  AcadosSolution solution;

  solution.xtraj = getStateTrajectory();
  solution.utraj = getControlTrajectory();

  solution.sqp_iter = sqp_iter;
  solution.kkt_norm_inf = kkt_norm_inf;
  solution.elapsed_time = elapsed_time;

  solution.status = status;
  solution.info = ss.str();

  return solution;
}
