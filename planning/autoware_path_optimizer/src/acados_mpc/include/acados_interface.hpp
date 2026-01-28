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

#pragma once

#include <array>
#include <cstddef>
#include <string>

// Undefine MAX_ITER to avoid conflict between OSQP and acados/hpipm
#ifdef MAX_ITER
#undef MAX_ITER
#endif

extern "C" {
#include "c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.h"
}

constexpr size_t NX = CURVILINEAR_BICYCLE_MODEL_SPATIAL_NX;
constexpr size_t NP = CURVILINEAR_BICYCLE_MODEL_SPATIAL_NP;
constexpr size_t NU = CURVILINEAR_BICYCLE_MODEL_SPATIAL_NU;
// Number of nonlinear path constraints h(x,u,p) per stage (fixed at codegen time).
constexpr size_t NH = CURVILINEAR_BICYCLE_MODEL_SPATIAL_NH;
// Number of soft constraints on h (slacks) per stage.
constexpr size_t NSH = CURVILINEAR_BICYCLE_MODEL_SPATIAL_NSH;
constexpr size_t N = CURVILINEAR_BICYCLE_MODEL_SPATIAL_N;

struct AcadosSolution
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  std::array<std::array<double, NU>, N> utraj;

  int sqp_iter;
  double kkt_norm_inf;
  double elapsed_time;

  int status;
  std::string info;
};

class AcadosInterface
{
public:
  AcadosInterface(int max_iter, double tol);
  ~AcadosInterface();

  int solve();
  AcadosSolution getControl(std::array<double, NX> x0);
  // Set parameters for a single stage
  void setParameters(int stage, std::array<double, NP> params);
  // Set the same parameters for all stages
  void setParametersAllStages(std::array<double, NP> params);
  // Warm start: set initial guesses for all states and controls
  void setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0);
  // Set the initial state constraint (lbx/ubx) at stage 0
  void setInitialState(std::array<double, NX> x0);
  // Set MPT-style inequality bounds for h constraints at a single stage: lh <= h(x,p) <= uh.
  // No-op if NH == 0 in the generated solver.
  void setInequalityBounds(int stage, std::array<double, NH> lh, std::array<double, NH> uh);
  // Apply per-stage beta (yaw difference) and bounds to the solver:
  // - Writes cos(beta)/sin(beta) into the correct trailing entries of p (if NH>0)
  // - Calls setInequalityBounds(stage, lh, uh)
  // No-op if NH == 0 in the generated solver.
  void applyCircleConstraintsToParams(
    int stage, std::array<double, NP> & params, const std::array<double, NH> & beta,
    const std::array<double, NH> & lh, const std::array<double, NH> & uh);

  // Set linear slack penalties for soft h-constraints at a stage:
  // cost += zl^T s_l + zu^T s_u
  // No-op if NSH == 0 in the generated solver.
  void setSoftConstraintLinearWeight(int stage, double w);
  // Set solver options at runtime: max_iter and tolerance (KKT tol)
  void setSolverOptions(int max_iter, double tol);

private:
  // Retrieve full horizon of states (length (N+1)*NX)
  std::array<std::array<double, NX>, N + 1> getStateTrajectory() const;
  // Retrieve full horizon of controls (length N*NU)
  std::array<std::array<double, NU>, N> getControlTrajectory() const;

  curvilinear_bicycle_model_spatial_solver_capsule * capsule_;
  ocp_nlp_config * nlp_config_;
  ocp_nlp_dims * nlp_dims_;
  ocp_nlp_in * nlp_in_;
  ocp_nlp_out * nlp_out_;
  ocp_nlp_solver * nlp_solver_;
};
