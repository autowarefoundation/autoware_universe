#include <mppi/utils/risk_utils.cuh>

// #include <thrust/device_vector.h>
// #include <thrust/sort.h>

#include <algorithm>

namespace mppi
{
template <class T = float>
__host__ __device__ void insertionSort(T* __restrict__ array, const int N)
{
  T temp;
  int j;
  for (int i = 1; i < N; i++)
  {
    temp = array[i];
    j = i - 1;
    while (j >= 0 && array[j] > temp)
    {
      array[j + 1] = array[j];
      --j;
    }
    array[j + 1] = temp;
  }
}

__host__ __device__ float RiskMeasure::var(float* __restrict__ costs, const int num_costs, float alpha)
{
  float cost = 0.0f;
#ifdef __CUDA_ARCH__
  // thrust::device_ptr<float> thrust_ptr = thrust::device_pointer_cast(costs);
  // thrust::sort(thrust::seq, thrust_ptr, thrust_ptr + num_costs);
  insertionSort<>(costs, num_costs);
#else
  std::sort(costs, costs + num_costs);
#endif
  float h_idx = h_index(num_costs, alpha);
  int next_idx = min((int)ceilf(h_idx), num_costs - 1);
  int prev_idx = max((int)floorf(h_idx), 0);
  cost = costs[prev_idx] + (h_idx - prev_idx) * (costs[next_idx] - costs[prev_idx]);
  return cost;
}

__host__ __device__ float RiskMeasure::cvar(float* __restrict__ costs, const int num_costs, float alpha)
{
  float cost = 0.0f;
  float value_at_risk = var(costs, num_costs, alpha);  // also sorts costs
  int num_costs_above = 1;
  float sum_costs_above = value_at_risk;
  float h_idx = h_index(num_costs, alpha);
  for (int i = ceilf(h_idx); i < num_costs; i++)
  {
    num_costs_above++;
    sum_costs_above += costs[i];
  }
  cost = sum_costs_above / num_costs_above;
  return cost;
}
}  // namespace mppi
