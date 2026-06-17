#pragma once
/*
Header file for costs
*/

#ifndef COSTS_CUH_
#define COSTS_CUH_

#include <Eigen/Dense>
#include <stdio.h>
#include <math.h>
#include <mppi/dynamics/dynamics.cuh>
#include <mppi/utils/managed.cuh>

#include <stdexcept>

template <int C_DIM>
struct CostParams
{
  static const int CONTROL_DIM = C_DIM;
  float control_cost_coeff[C_DIM];
  float discount = 1.0f;
  CostParams()
  {
    // Default set all controls to 1
    for (int i = 0; i < C_DIM; ++i)
    {
      control_cost_coeff[i] = 1.0f;
    }
  }
};

// https://cboard.cprogramming.com/cplusplus-programming/122412-crtp-how-pass-type.html
template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T = DynamicsParams>
class Cost : public Managed
{
public:
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * typedefs for access to templated class from outside classes
   */
  using ControlIndex = typename DYN_PARAMS_T::ControlIndex;
  using OutputIndex = typename DYN_PARAMS_T::OutputIndex;
  using TEMPLATED_DYN_PARAMS = DYN_PARAMS_T;
  static const int CONTROL_DIM = E_INDEX(ControlIndex, NUM_CONTROLS);
  static const int OUTPUT_DIM = E_INDEX(OutputIndex, NUM_OUTPUTS);  // TODO
  typedef CLASS_T COST_T;
  typedef PARAMS_T COST_PARAMS_T;
  typedef Eigen::Matrix<float, CONTROL_DIM, 1> control_array;             // Control at a time t
  typedef Eigen::Matrix<float, CONTROL_DIM, CONTROL_DIM> control_matrix;  // Control at a time t
  typedef Eigen::Matrix<float, OUTPUT_DIM, 1> output_array;               // Output at a time t

  Cost() = default;
  /**
   * Destructor must be virtual so that children are properly
   * destroyed when called from a basePlant reference
   */
  virtual ~Cost()
  {
    freeCudaMem();
  }

  virtual std::string getCostFunctionName() const
  {
    return "cost function name not set";
  }

  void GPUSetup();

  bool getDebugDisplayEnabled()
  {
    return false;
  }

  /**
   * Updates the cost parameters
   * @param params
   */
  void setParams(const PARAMS_T& params)
  {
    params_ = params;
    if (GPUMemStatus_)
    {
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice();
    }
  }

  __host__ __device__ PARAMS_T getParams()
  {
    return params_;
  }

  void paramsToDevice();

  /**
   *
   * @param description
   * @param data
   */
  void updateCostmap(std::vector<int> description, std::vector<float> data){};

  /**
   * deallocates the allocated cuda memory for an object
   */
  void freeCudaMem();

  /**
   * Computes the feedback control cost on CPU for RMPPI
   */
  float computeFeedbackCost(const Eigen::Ref<const control_array> fb_u, const Eigen::Ref<const control_array> std_dev,
                            const float lambda = 1.0f, const float alpha = 0.0f)
  {
    float cost = 0.0f;
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      cost += params_.control_cost_coeff[i] * SQ(fb_u(i)) / SQ(std_dev(i));
    }

    return 0.5f * lambda * (1.0f - alpha) * cost;
  }

  /**
   * Computes the control cost on CPU. This is the normal control cost calculation
   * in MPPI and Tube-MPPI
   */
  float computeControlCost(const Eigen::Ref<const control_array> u, int timestep, int* crash)
  {
    return 0.0f;
  }
  // =================== METHODS THAT SHOULD HAVE NO DEFAULT ==========================
  /**
   * Computes the state cost on the CPU. Should be implemented in subclasses
   */
  float computeStateCost(const Eigen::Ref<const output_array> y, int timestep, int* crash_status)
  {
    throw std::logic_error("SubClass did not implement computeStateCost");
  }

  /**
   *
   * @param s current state as a float array
   * @return state cost on GPU
   */
  __device__ float computeStateCost(float* y, int timestep, float* theta_c, int* crash_status);

  /**
   * Computes the state cost on the CPU. Should be implemented in subclasses
   */
  float terminalCost(const Eigen::Ref<const output_array> y)
  {
    throw std::logic_error("SubClass did not implement terminalCost");
  }

  /**
   *
   * @param s terminal state as float array
   * @return terminal cost on GPU
   */
  __device__ float terminalCost(float* y, float* theta_c);

  /**
   * Method to allow setup of costs on the CPU. This is needed for
   * initializing the memory of an LSTM for example
   */
  void initializeCosts(const Eigen::Ref<const output_array>& output, const Eigen::Ref<const control_array>& control,
                       float t_0, float dt)
  {
  }

  /**
   * Method to allow setup of costs on the GPU. This is needed for
   * initializing the memory of an LSTM for example
   */
  __device__ void initializeCosts(float* output, float* control, float* theta_c, float t_0, float dt)
  {
  }

  // ================ END OF METHODS WITH NO DEFAULT ===========================

  // =================== METHODS THAT SHOULD NOT BE OVERWRITTEN ================
  /**
   * Computes the feedback control cost on GPU used in RMPPI. There is an
   * assumption that we are provided std_dev and the covriance matrix is
   * diagonal.
   */
  __device__ float computeFeedbackCost(float* fb_u, float* std_dev, float lambda = 1.0f, float alpha = 0.0f)
  {
    float cost = 0.0f;
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      cost += params_.control_cost_coeff[i] * SQ(fb_u[i] / std_dev[i]);
    }
    return 0.5f * lambda * (1.0f - alpha) * cost;
  }

  /**
   * Computes the normal control cost for MPPI and Tube-MPPI
   * 0.5 * lambda * (u*^T \Sigma^{-1} u*^T + 2 * u*^T \Sigma^{-1} (u*^T + noise))
   * On the GPU, u = u* + noise already, so we need the following to create
   * the original cost:
   * 0.5 * lambda * (u - noise)^T \Sigma^{-1} (u + noise)
   */
  __device__ float computeControlCost(float* u, int timestep, float* theta_c, int* crash)
  {
    return 0.0f;
  }
  // =================== END METHODS THAT SHOULD NOT BE OVERWRITTEN ============

  // =================== METHODS THAT CAN BE OVERWRITTEN =======================
  float computeRunningCost(const Eigen::Ref<const output_array> y, const Eigen::Ref<const control_array> u,
                           int timestep, int* crash)
  {
    CLASS_T* derived = static_cast<CLASS_T*>(this);

    return derived->computeStateCost(y, timestep, crash) +
           derived->computeControlCost(u, timestep, crash);
  }

  __device__ float computeRunningCost(float* y, float* u,
                                      int timestep, float* theta_c, int* crash);
  // =================== END METHODS THAT CAN BE OVERWRITTEN ===================

  inline __host__ __device__ PARAMS_T getParams() const
  {
    return params_;
  }

  CLASS_T* cost_d_ = nullptr;

protected:
  PARAMS_T params_;
};

#if __CUDACC__
#include "cost.cu"
#endif

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
const int Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>::CONTROL_DIM;

template <class CLASS_T, class PARAMS_T, class DYN_PARAMS_T>
const int Cost<CLASS_T, PARAMS_T, DYN_PARAMS_T>::OUTPUT_DIM;
#endif  // COSTS_CUH_
