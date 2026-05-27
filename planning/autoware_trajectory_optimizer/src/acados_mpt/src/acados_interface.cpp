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

#include <iostream>
#include <sstream>

namespace temporal_mpt
{

AcadosInterface::AcadosInterface()
{
  capsule_ = kinematic_bicycle_temporal_acados_create_capsule();
  kinematic_bicycle_temporal_acados_create(capsule_);

  double lbx0[KINEMATIC_BICYCLE_TEMPORAL_NX] = {0};
  double ubx0[KINEMATIC_BICYCLE_TEMPORAL_NX] = {0};

  nlp_config_ = kinematic_bicycle_temporal_acados_get_nlp_config(capsule_);
  nlp_dims_ = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  nlp_in_ = kinematic_bicycle_temporal_acados_get_nlp_in(capsule_);
  nlp_out_ = kinematic_bicycle_temporal_acados_get_nlp_out(capsule_);
  nlp_solver_ = kinematic_bicycle_temporal_acados_get_nlp_solver(capsule_);

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", lbx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", ubx0);
}

AcadosInterface::~AcadosInterface()
{
  kinematic_bicycle_temporal_acados_free(capsule_);
  kinematic_bicycle_temporal_acados_free_capsule(capsule_);
}

void AcadosInterface::setParameters(int stage, std::array<double, NP> params)
{
  kinematic_bicycle_temporal_acados_update_params(
    capsule_, stage, const_cast<double *>(params.data()), NP);
}

void AcadosInterface::setParametersAllStages(std::array<double, NP> params)
{
  ocp_nlp_dims * nlp_dims = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  for (int i = 0; i <= nlp_dims->N; ++i) {
    kinematic_bicycle_temporal_acados_update_params(
      capsule_, i, const_cast<double *>(params.data()), NP);
  }
}

void AcadosInterface::setStageReference(int stage, std::array<double, NY> yref)
{
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, stage, "yref", const_cast<double *>(yref.data()));
}

void AcadosInterface::setTerminalReference(std::array<double, NYN> yref_e)
{
  ocp_nlp_cost_model_set(
    nlp_config_, nlp_dims_, nlp_in_, static_cast<int>(N), "yref",
    const_cast<double *>(yref_e.data()));
}

void AcadosInterface::setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0)
{
  ocp_nlp_dims * nlp_dims = kinematic_bicycle_temporal_acados_get_nlp_dims(capsule_);
  for (int i = 0; i < nlp_dims->N; ++i) {
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", const_cast<double *>(x0.data()));
    ocp_nlp_out_set(
      nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", const_cast<double *>(u0.data()));
  }
  ocp_nlp_out_set(
    nlp_config_, nlp_dims_, nlp_out_, nlp_in_, nlp_dims->N, "x", const_cast<double *>(x0.data()));
}

void AcadosInterface::setInitialState(std::array<double, NX> x0)
{
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "lbx", const_cast<double *>(x0.data()));
  ocp_nlp_constraints_model_set(
    nlp_config_, nlp_dims_, nlp_in_, nlp_out_, 0, "ubx", const_cast<double *>(x0.data()));
}

std::array<std::array<double, NX>, N + 1> AcadosInterface::getStateTrajectory() const
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  for (size_t ii = 0; ii <= static_cast<size_t>(nlp_dims_->N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "x", &xtraj[ii]);
  }
  return xtraj;
}

std::array<std::array<double, NU>, N> AcadosInterface::getControlTrajectory() const
{
  std::array<std::array<double, NU>, N> utraj;
  for (size_t ii = 0; ii < static_cast<size_t>(N); ii++) {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, ii, "u", &utraj[ii]);
  }
  return utraj;
}

int AcadosInterface::solve()
{
  return kinematic_bicycle_temporal_acados_solve(capsule_);
}

AcadosSolution AcadosInterface::getControl(std::array<double, NX> x0)
{
  double kkt_norm_inf = 0.0;
  double elapsed_time = 0.0;
  int sqp_iter = 0;

  setInitialState(x0);

  int status = kinematic_bicycle_temporal_acados_solve(capsule_);
  ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

  kinematic_bicycle_temporal_acados_print_stats(capsule_);

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

}  // namespace temporal_mpt
