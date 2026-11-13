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
//
// Stub header for static analysis (clang-tidy) and IDE support.
// The real header is generated at build time into the build directory.

#ifndef ACADOS_MPT__C_GENERATED_CODE__ACADOS_SOLVER_KINEMATIC_BICYCLE_TEMPORAL_H_
#define ACADOS_MPT__C_GENERATED_CODE__ACADOS_SOLVER_KINEMATIC_BICYCLE_TEMPORAL_H_

#include "acados/utils/types.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

#define KINEMATIC_BICYCLE_TEMPORAL_NX 4
#define KINEMATIC_BICYCLE_TEMPORAL_NZ 0
#define KINEMATIC_BICYCLE_TEMPORAL_NU 2
#define KINEMATIC_BICYCLE_TEMPORAL_NP 2
#define KINEMATIC_BICYCLE_TEMPORAL_NP_GLOBAL 0
#define KINEMATIC_BICYCLE_TEMPORAL_NBX 0
#define KINEMATIC_BICYCLE_TEMPORAL_NBX0 4
#define KINEMATIC_BICYCLE_TEMPORAL_NBU 2
#define KINEMATIC_BICYCLE_TEMPORAL_NSBX 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSBU 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSH 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSH0 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSG 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSPHI 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSHN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSGN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSPHIN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSPHI0 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSBXN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NS 0
#define KINEMATIC_BICYCLE_TEMPORAL_NS0 0
#define KINEMATIC_BICYCLE_TEMPORAL_NSN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NG 0
#define KINEMATIC_BICYCLE_TEMPORAL_NBXN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NGN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NY0 6
#define KINEMATIC_BICYCLE_TEMPORAL_NY 6
#define KINEMATIC_BICYCLE_TEMPORAL_NYN 4
#define KINEMATIC_BICYCLE_TEMPORAL_N 80
#define KINEMATIC_BICYCLE_TEMPORAL_NH 1
#define KINEMATIC_BICYCLE_TEMPORAL_NHN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NH0 0
#define KINEMATIC_BICYCLE_TEMPORAL_NPHI0 0
#define KINEMATIC_BICYCLE_TEMPORAL_NPHI 0
#define KINEMATIC_BICYCLE_TEMPORAL_NPHIN 0
#define KINEMATIC_BICYCLE_TEMPORAL_NR 0

#ifdef __cplusplus
extern "C" {
#endif

// ** capsule for solver data **
typedef struct kinematic_bicycle_temporal_solver_capsule
{
  ocp_nlp_in * nlp_in;
  ocp_nlp_out * nlp_out;
  ocp_nlp_out * sens_out;
  ocp_nlp_solver * nlp_solver;
  void * nlp_opts;
  ocp_nlp_plan_t * nlp_solver_plan;
  ocp_nlp_config * nlp_config;
  ocp_nlp_dims * nlp_dims;

  unsigned int nlp_np;

  external_function_external_param_casadi * expl_vde_forw;
  external_function_external_param_casadi * expl_ode_fun;
  external_function_external_param_casadi * expl_vde_adj;

  external_function_external_param_casadi * nl_constr_h_fun_jac;
  external_function_external_param_casadi * nl_constr_h_fun;
} kinematic_bicycle_temporal_solver_capsule;

ACADOS_SYMBOL_EXPORT kinematic_bicycle_temporal_solver_capsule *
kinematic_bicycle_temporal_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_free_capsule(
  kinematic_bicycle_temporal_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_create(
  kinematic_bicycle_temporal_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_reset(
  kinematic_bicycle_temporal_solver_capsule * capsule, int reset_qp_solver_mem);

ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_create_with_discretization(
  kinematic_bicycle_temporal_solver_capsule * capsule, int n_time_steps, double * new_time_steps);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_update_time_steps(
  kinematic_bicycle_temporal_solver_capsule * capsule, int N, double * new_time_steps);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_update_qp_solver_cond_N(
  kinematic_bicycle_temporal_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_update_params(
  kinematic_bicycle_temporal_solver_capsule * capsule, int stage, double * value, int np);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_update_params_sparse(
  kinematic_bicycle_temporal_solver_capsule * capsule, int stage, int * idx, double * p,
  int n_update);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_set_p_global_and_precompute_dependencies(
  kinematic_bicycle_temporal_solver_capsule * capsule, double * data, int data_len);

ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_solve(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_setup_qp_matrices_and_factorize(
  kinematic_bicycle_temporal_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_free(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void kinematic_bicycle_temporal_acados_print_stats(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int kinematic_bicycle_temporal_acados_custom_update(
  kinematic_bicycle_temporal_solver_capsule * capsule, double * data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in * kinematic_bicycle_temporal_acados_get_nlp_in(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out * kinematic_bicycle_temporal_acados_get_nlp_out(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out * kinematic_bicycle_temporal_acados_get_sens_out(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver * kinematic_bicycle_temporal_acados_get_nlp_solver(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config * kinematic_bicycle_temporal_acados_get_nlp_config(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void * kinematic_bicycle_temporal_acados_get_nlp_opts(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims * kinematic_bicycle_temporal_acados_get_nlp_dims(
  kinematic_bicycle_temporal_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t * kinematic_bicycle_temporal_acados_get_nlp_plan(
  kinematic_bicycle_temporal_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_MPT__C_GENERATED_CODE__ACADOS_SOLVER_KINEMATIC_BICYCLE_TEMPORAL_H_
