/*
 * Created on Thu Jun 11 2020 by Bogdan
 */
#ifndef MPPI_COST_FUNCTIONS_QUADROTOR_QUADRATIC_COST_CUH_
#define MPPI_COST_FUNCTIONS_QUADROTOR_QUADRATIC_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/quadrotor/quadrotor_dynamics.cuh>
#include <mppi/utils/math_utils.h>
struct QuadrotorQuadraticCostParams : public CostParams<4>
{
  /**
   * State for this class is defined as follows:
   *    x - position in 3D space (x, y, z) - meters
   *    v - velocity in 3D space (v_x, v_y_ v_z) - meters/sec
   *    q - quaternion (q_w, q_x, q_y, q_z)
   *    w - angular velocities (roll_dot, pitch_dot, yaw_dot) - rad/sec
   *
   * Coordinate Frame is NWU
   */
  float s_goal[13] = { 0, 0, 0,     // x
                       0, 0, 0,     // v
                       1, 0, 0, 0,  // q
                       0, 0, 0 };   // w

  float* x_goal()
  {
    return &s_goal[0];
  }
  float* v_goal()
  {
    return &s_goal[3];
  }
  float* q_goal()
  {
    return &s_goal[6];
  }
  float* w_goal()
  {
    return &s_goal[10];
  }

  float x_coeff = 1.0;
  float v_coeff = 1.0;
  // Euler Angle or Quaternion scoring
  bool use_euler = true;
  float q_coeff = 1.0;  // Quaternion Modifier
  float roll_coeff = 1.0;
  float pitch_coeff = 1.0;
  float yaw_coeff = 1.0;

  float w_coeff = 1.0;
  float terminal_cost_coeff = 0;

  QuadrotorQuadraticCostParams()
  {
    control_cost_coeff[0] = 2.0;
    control_cost_coeff[1] = 2.0;
    control_cost_coeff[2] = 2.0;
    control_cost_coeff[3] = 2.0;
  }

  Eigen::Matrix<float, 13, 1> getDesiredState()
  {
    Eigen::Matrix<float, 13, 1> s;
    s << s_goal[0], s_goal[1], s_goal[2], s_goal[3], s_goal[4], s_goal[5], s_goal[6], s_goal[7], s_goal[8], s_goal[9],
        s_goal[10], s_goal[11], s_goal[12];
    return s;
  }
};

class QuadrotorQuadraticCost
  : public Cost<QuadrotorQuadraticCost, QuadrotorQuadraticCostParams, QuadrotorDynamicsParams>
{
  /**
   * State for this class is defined as follows:
   *    x - position in 3D space (x, y, z) - meters
   *    v - velocity in 3D space (v_x, v_y_ v_z) - meters/sec
   *    q - quaternion (q_w, q_x, q_y, q_z)
   *    w - angular velocities (roll_dot, pitch_dot, yaw_dot) - rad/sec
   *
   * Coordinate Frame is NWU
   *
   * Control:
   *    roll_rate  - rad/sec
   *    pitch_rate - rad/sec
   *    yaw_rate   - rad/sec
   *    thrust     - Newtons
   */

public:
  QuadrotorQuadraticCost(cudaStream_t stream = nullptr);
  static constexpr float MAX_COST_VALUE = 1e16;

  /**
   * Host Functions
   */
  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);

  float terminalCost(const Eigen::Ref<const output_array> s);

  /**
   * Device Functions
   */
  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);

  __device__ float terminalCost(float* s, float* theta_c = nullptr);
};

#if __CUDACC__
#include "quadrotor_quadratic_cost.cu"
#endif

#endif  // MPPI_COST_FUNCTIONS_QUADROTOR_QUADRATIC_COSTS_CUH_
