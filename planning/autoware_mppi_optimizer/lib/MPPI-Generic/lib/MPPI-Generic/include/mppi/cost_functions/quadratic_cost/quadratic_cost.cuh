#pragma once
/*
 * Created on Wed Dec 16 2020 by Bogdan
 */

#ifndef MPPI_COST_FUNCTIONS_QUADRATIC_COST_CUH_
#define MPPI_COST_FUNCTIONS_QUADRATIC_COST_CUH_

#include <mppi/cost_functions/cost.cuh>
#include <mppi/utils/math_utils.h>

template <class DYN_T, int SIM_TIME_HORIZON = 1>
struct QuadraticCostTrajectoryParams : public CostParams<DYN_T::CONTROL_DIM>
{
  typedef DYN_T TEMPLATED_DYN;
  static const int TIME_HORIZON = SIM_TIME_HORIZON;
  /**
   * Defines a general desired state and coeffs
   */
  float s_goal[DYN_T::OUTPUT_DIM * SIM_TIME_HORIZON] = { 0 };

  float s_coeffs[DYN_T::OUTPUT_DIM] = { 0 };

  int current_time = 0;

  QuadraticCostTrajectoryParams()
  {
    for (int i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      this->control_cost_coeff[i] = 0;
    }
    for (int i = 0; i < DYN_T::OUTPUT_DIM; i++)
    {
      this->s_coeffs[i] = 1;
    }
  }

  const Eigen::Matrix<float, DYN_T::OUTPUT_DIM, 1> getDesiredState(int t)
  {
    Eigen::Matrix<float, DYN_T::OUTPUT_DIM, 1> s(s_goal + this->getIndex(t));
    return s;
  }

  __device__ float* getGoalStatePointer(int t)
  {
    return s_goal + this->getIndex(t);
  }

  __host__ __device__ int getIndex(int t)
  {
    int index = (this->current_time + t);
    if (index >= TIME_HORIZON)
    {
      index = (TIME_HORIZON - 1);
    }
    index *= DYN_T::OUTPUT_DIM;
    return index;
  }
  __host__ __device__ void setCurrentTime(int new_time)
  {
    current_time = new_time;
  }
};

template <class CLASS_T, class DYN_T, class PARAMS_T>
class QuadraticCostImpl : public Cost<CLASS_T, PARAMS_T, typename DYN_T::DYN_PARAMS_T>
{
public:
  typedef Cost<CLASS_T, PARAMS_T, typename DYN_T::DYN_PARAMS_T> PARENT_CLASS;
  using output_array = typename PARENT_CLASS::output_array;
  QuadraticCostImpl(cudaStream_t stream = nullptr);
  static constexpr float MAX_COST_VALUE = 1e16;

  /**
   * Host Functions
   */
  virtual std::string getCostFunctionName() const override
  {
    return "Quadratic Cost";
  }

  float computeStateCost(const Eigen::Ref<const output_array> s, int timestep = 0, int* crash_status = nullptr);

  float terminalCost(const Eigen::Ref<const output_array> s);

  float getLipshitzConstantCost()
  {
    // Find the spectral radius of the state matrix
    float rho_Q = fabsf(this->params_.s_coeffs[0]);
    for (int i = 1; i < DYN_T::OUTPUT_DIM; i++)
    {
      rho_Q = fmaxf(rho_Q, fabsf(this->params_.s_coeffs[i]));
    }
    return 2 * rho_Q;
  };

  /**
   * Device Functions
   */
  __device__ float computeStateCost(float* s, int timestep = 0, float* theta_c = nullptr, int* crash_status = nullptr);

  // Custom implementation that does a Nan check.
  // __device__ float computeRunningCost(float* s, float* u, float* noise, float* std_dev, float lambda, float alpha,
  // int timestep, float* theta_c,  int* crash_status);

  __device__ float terminalCost(float* s, float* theta_c);
};

#if __CUDACC__
#include "quadratic_cost.cu"
#endif

template <class DYN_T, int SIM_TIME_HORIZON>
class QuadraticCostTrajectory : public QuadraticCostImpl<QuadraticCostTrajectory<DYN_T, SIM_TIME_HORIZON>, DYN_T,
                                                         QuadraticCostTrajectoryParams<DYN_T, SIM_TIME_HORIZON>>
{
public:
  QuadraticCostTrajectory(cudaStream_t stream = nullptr)
    : QuadraticCostImpl<QuadraticCostTrajectory, DYN_T, QuadraticCostTrajectoryParams<DYN_T, SIM_TIME_HORIZON>>(
          stream){};
};

template <class DYN_T>
class QuadraticCost : public QuadraticCostImpl<QuadraticCost<DYN_T>, DYN_T, QuadraticCostTrajectoryParams<DYN_T>>
{
public:
  QuadraticCost(cudaStream_t stream = nullptr)
    : QuadraticCostImpl<QuadraticCost, DYN_T, QuadraticCostTrajectoryParams<DYN_T>>(stream){};
};

#endif  // MPPI_COST_FUNCTIONS_QUADRATIC_COST_CUH_
