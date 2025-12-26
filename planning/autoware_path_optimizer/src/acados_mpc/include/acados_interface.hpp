#pragma once

#include <stddef.h>

#include <array>
#include <string>

// Undefine MAX_ITER to avoid conflict between OSQP and acados/hpipm
#ifdef MAX_ITER
#undef MAX_ITER
#endif

extern "C" {
#include "c_generated_code/acados_solver_curvilinear_bicycle_model_spatial.h"
}

#define NX CURVILINEAR_BICYCLE_MODEL_SPATIAL_NX
#define NP CURVILINEAR_BICYCLE_MODEL_SPATIAL_NP
#define NU CURVILINEAR_BICYCLE_MODEL_SPATIAL_NU
constexpr size_t N = CURVILINEAR_BICYCLE_MODEL_SPATIAL_N;

struct AcadosSolution
{
  std::array<std::array<double, NX>, N + 1> xtraj;
  std::array<std::array<double, NU>, N> utraj;

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
