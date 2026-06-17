#pragma once

namespace mppi
{
class RiskMeasure
{
public:
  enum FUNC_TYPE : int
  {
    MAX = 0,
    MIN,
    VAR,
    CVAR,
    MEAN,
    MEDIAN,
    NUM_FUNCS
  };

  FUNC_TYPE func_ = CVAR;
  float alpha_ = 0.8;

  __host__ __device__ float shaping_func(float* __restrict__ costs, const int num_costs)
  {
    return shaping_func(costs, num_costs, func_, alpha_);
  }

  static __host__ __device__ float shaping_func(float* __restrict__ costs, const int num_costs,
                                                const FUNC_TYPE type = MEAN, const float risk_tolerance = 0.5f)
  {
    float cost = 0.0f;
    if (num_costs == 1)
    {
      return costs[0];
    }
    switch (type)
    {
      case CVAR:
        cost = cvar(costs, num_costs, risk_tolerance);
        break;
      case MAX:
        cost = max_measure(costs, num_costs);
        break;
      case MEDIAN:
        cost = var(costs, num_costs, 0.5f);
        break;
      case MIN:
        cost = min_measure(costs, num_costs);
        break;
      case VAR:
        cost = var(costs, num_costs, risk_tolerance);
        break;
      default: // go to mean case
      case MEAN:
        cost = mean_measure(costs, num_costs);
        break;
    }
    return cost;
  }

  static __host__ __device__ float max_measure(const float* __restrict__ costs, const int num_costs)
  {
    float max_cost = costs[0];
    for (int i = 1; i < num_costs; i++)
    {
      if (costs[i] > max_cost)
      {
        max_cost = costs[i];
      }
    }
    return max_cost;
  }

  static __host__ __device__ float min_measure(const float* __restrict__ costs, const int num_costs)
  {
    float min_cost = costs[0];
    for (int i = 1; i < num_costs; i++)
    {
      if (costs[i] < min_cost)
      {
        min_cost = costs[i];
      }
    }
    return min_cost;
  }

  static __host__ __device__ float mean_measure(const float* __restrict__ costs, const int num_costs)
  {
    float cost = 0.0f;
    for (int i = 0; i < num_costs; i++)
    {
      cost += costs[i];
    }
    return cost / num_costs;
  }

  static __host__ __device__ float h_index(const int num_costs, const float alpha)
  {
    return alpha * (num_costs - 1);
  }

  static __host__ __device__ float var(float* __restrict__ costs, const int num_costs, float alpha);

  static __host__ __device__ float cvar(float* __restrict__ costs, const int num_costs, float alpha);
};
}  // namespace mppi

#ifdef __CUDACC__
#include "risk_utils.cu"
#endif
