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

#ifdef MAX_ITER
#undef MAX_ITER
#endif

extern "C" {
#include "c_generated_code/acados_solver_kinematic_bicycle_temporal.h"
}

namespace temporal_mpt
{
constexpr size_t NX = KINEMATIC_BICYCLE_TEMPORAL_NX;
constexpr size_t NP = KINEMATIC_BICYCLE_TEMPORAL_NP;
constexpr size_t NU = KINEMATIC_BICYCLE_TEMPORAL_NU;
constexpr size_t N = KINEMATIC_BICYCLE_TEMPORAL_N;
constexpr size_t NY = KINEMATIC_BICYCLE_TEMPORAL_NY;
constexpr size_t NYN = KINEMATIC_BICYCLE_TEMPORAL_NYN;

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
  AcadosInterface();
  ~AcadosInterface();

  int solve();
  AcadosSolution getControl(std::array<double, NX> x0);
  void setParameters(int stage, std::array<double, NP> params);
  void setParametersAllStages(std::array<double, NP> params);
  void setStageReference(int stage, std::array<double, NY> yref);
  void setTerminalReference(std::array<double, NYN> yref_e);
  void setWarmStart(std::array<double, NX> x0, std::array<double, NU> u0);
  void setInitialState(std::array<double, NX> x0);

private:
  std::array<std::array<double, NX>, N + 1> getStateTrajectory() const;
  std::array<std::array<double, NU>, N> getControlTrajectory() const;

  kinematic_bicycle_temporal_solver_capsule * capsule_;
  ocp_nlp_config * nlp_config_;
  ocp_nlp_dims * nlp_dims_;
  ocp_nlp_in * nlp_in_;
  ocp_nlp_out * nlp_out_;
  ocp_nlp_solver * nlp_solver_;
};
}  // namespace temporal_mpt
