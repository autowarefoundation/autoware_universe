#pragma once

#ifndef CARTPOLE_QUADRATIC_COST_CUH_
#define CARTPOLE_QUADRATIC_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/dynamics/cartpole/cartpole_dynamics.cuh>
#include <mppi/utils/file_utils.h>

struct CartpoleQuadraticCostParams : public CostParams<1>
{
  float cart_position_coeff = 1000;
  float cart_velocity_coeff = 100;
  float pole_angle_coeff = 2000;
  float pole_angular_velocity_coeff = 100;
  float terminal_cost_coeff = 0;
  float desired_terminal_state[4] = { 0, 0, M_PI, 0 };

  CartpoleQuadraticCostParams()
  {
    this->control_cost_coeff[0] = 10.0;
  }
};

class CartpoleQuadraticCost : public Cost<CartpoleQuadraticCost, CartpoleQuadraticCostParams, CartpoleDynamicsParams>
{
public:
  /**
   * Constructor
   * @param width
   * @param height
   */
  CartpoleQuadraticCost(cudaStream_t stream = 0);

  /**
   * @brief Compute the state cost
   */
  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);

  /**
   * @brief Compute the state cost on the CPU
   */
  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);

  /**
   * @brief Compute the terminal cost of the system
   */
  __device__ float terminalCost(float* s, float* theta_c);

  float terminalCost(const Eigen::Ref<const output_array> s);

protected:
};

#if __CUDACC__
#include "cartpole_quadratic_cost.cu"
#endif

#endif  // CARTPOLE_QUADRATIC_COST_CUH_// Include the cart pole cost.
