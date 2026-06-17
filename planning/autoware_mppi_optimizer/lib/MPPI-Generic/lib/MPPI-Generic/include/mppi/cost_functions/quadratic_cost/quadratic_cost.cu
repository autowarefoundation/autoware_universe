#include <mppi/cost_functions/quadratic_cost/quadratic_cost.cuh>

template <class CLASS_T, class DYN_T, class PARAMS_T>
QuadraticCostImpl<CLASS_T, DYN_T, PARAMS_T>::QuadraticCostImpl(cudaStream_t stream)
{
  this->bindToStream(stream);
}

template <class CLASS_T, class DYN_T, class PARAMS_T>
float QuadraticCostImpl<CLASS_T, DYN_T, PARAMS_T>::computeStateCost(const Eigen::Ref<const output_array> s,
                                                                    int timestep, int* crash_status)
{
  float cost = 0;
  output_array des_state = this->params_.getDesiredState(timestep);

  Eigen::Matrix<float, DYN_T::OUTPUT_DIM, DYN_T::OUTPUT_DIM> coeffs;
  coeffs = Eigen::Matrix<float, DYN_T::OUTPUT_DIM, DYN_T::OUTPUT_DIM>::Zero();
  for (int i = 0; i < DYN_T::OUTPUT_DIM; i++)
  {
    coeffs(i, i) = this->params_.s_coeffs[i];
  }
  output_array error = s - des_state;
  cost = error.transpose() * coeffs * error;

  return cost;
}

template <class CLASS_T, class DYN_T, class PARAMS_T>
float QuadraticCostImpl<CLASS_T, DYN_T, PARAMS_T>::terminalCost(const Eigen::Ref<const output_array> s)
{
  return 0.0;
}

template <class CLASS_T, class DYN_T, class PARAMS_T>
__device__ float QuadraticCostImpl<CLASS_T, DYN_T, PARAMS_T>::computeStateCost(float* s, int timestep, float* theta_c,
                                                                               int* crash_status)
{
  float cost = 0;

  float* desired_state = this->params_.getGoalStatePointer(timestep);

  for (int i = 0; i < DYN_T::OUTPUT_DIM; i++)
  {
    cost += powf(s[i] - desired_state[i], 2) * this->params_.s_coeffs[i];
  }

  return cost;
}

template <class CLASS_T, class DYN_T, class PARAMS_T>
__device__ float QuadraticCostImpl<CLASS_T, DYN_T, PARAMS_T>::terminalCost(float* s, float* theta_c)
{
  return 0.0;
}
