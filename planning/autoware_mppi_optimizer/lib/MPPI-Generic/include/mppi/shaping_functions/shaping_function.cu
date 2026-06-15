#include "shaping_function.cuh"
#include <mppi/utils/math_utils.h>
namespace mppi_common
{
template <class CLASS_T, int NUM_ROLLOUTS>
__global__ void weightKernel(float* trajectory_costs_d, float baseline, CLASS_T* shape_function)
{
  int global_idx = (blockDim.x * blockIdx.x + threadIdx.x) * blockDim.z + threadIdx.z;

  if (global_idx < NUM_ROLLOUTS * blockDim.z)
  {
    trajectory_costs_d[global_idx] = shape_function->computeWeight(trajectory_costs_d, baseline, global_idx);
  }
}
}  // namespace mppi_common

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::paramsToDevice()
{
  if (GPUMemStatus_)
  {
    HANDLE_ERROR(
        cudaMemcpyAsync(&shaping_function_d_->params_, &params_, sizeof(PARAMS_T), cudaMemcpyHostToDevice, stream_));
    HANDLE_ERROR(cudaStreamSynchronize(stream_));
  }
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::freeCudaMem()
{
  if (GPUMemStatus_)
  {
    cudaFree(shaping_function_d_);
    GPUMemStatus_ = false;
    shaping_function_d_ = nullptr;
  }
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::GPUSetup()
{
  CLASS_T* derived = static_cast<CLASS_T*>(this);
  if (!GPUMemStatus_)
  {
    shaping_function_d_ = Managed::GPUSetup<CLASS_T>(derived);
  }
  else
  {
    std::cout << "GPU Memory already set" << std::endl;  // TODO should this be an exception?
  }
  derived->paramsToDevice();
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
__host__ __device__ float ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::computeWeight(float* traj_cost,
                                                                                                      float baseline,
                                                                                                      int global_idx)
{
  return expf(-this->params_.lambda_inv * (traj_cost[global_idx] - baseline));
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::computeWeights(cost_traj& trajectory_costs,
                                                                                  float* trajectory_costs_d,
                                                                                  cudaStream_t stream)
{
  // Copy the costs back to the host
  HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs.data(), trajectory_costs_d, NUM_ROLLOUTS * sizeof(float),
                               cudaMemcpyDeviceToHost, this->stream_));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  baseline_ = trajectory_costs.minCoeff();

  // launch the WeightKernel to calculate the weights
  CLASS_T* derived = static_cast<CLASS_T*>(this);
  derived->launchWeightKernel(trajectory_costs_d, baseline_, stream);

  HANDLE_ERROR(cudaMemcpyAsync(trajectory_costs.data(), trajectory_costs_d, NUM_ROLLOUTS * sizeof(float),
                               cudaMemcpyDeviceToHost, stream));
  HANDLE_ERROR(cudaStreamSynchronize(stream));

  normalizer_ = trajectory_costs.sum();

  // TODO after this you should grab the baseline value and the normalizer
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::launchWeightKernel(float* trajectory_costs_d,
                                                                                      float baseline,
                                                                                      cudaStream_t stream)
{
  dim3 dimBlock(BDIM_X, 1, 1);
  dim3 dimGrid((NUM_ROLLOUTS - 1) / BDIM_X + 1, 1, 1);
  mppi_common::weightKernel<CLASS_T, NUM_ROLLOUTS>
      <<<1, NUM_ROLLOUTS>>>(trajectory_costs_d, baseline, this->shaping_function_d_);
  CudaCheckError();
}

template <class CLASS_T, class PARAMS_T, int NUM_ROLLOUTS, int BDIM_X>
void ShapingFunctionImpl<CLASS_T, PARAMS_T, NUM_ROLLOUTS, BDIM_X>::computeFreeEnergy(
    float& free_energy, float& free_energy_var, float& free_energy_modified, float* cost_rollouts_host, float baseline)
{
  float var = 0;
  float norm = 0;
  for (int i = 0; i < NUM_ROLLOUTS; i++)
  {
    norm += cost_rollouts_host[i];
    var += SQ(cost_rollouts_host[i]);
  }
  norm /= NUM_ROLLOUTS;
  free_energy = -1.0f / this->params_.lambda_inv * logf(norm) + baseline;
  free_energy_var = 1.0f / this->params_.lambda_inv * (var / NUM_ROLLOUTS - SQ(norm));
  // TODO Figure out the point of the following lines
  float weird_term = free_energy_var / (norm * sqrtf(1.0 * NUM_ROLLOUTS));
  free_energy_modified = 1.0f / this->params_.lambda_inv * (weird_term + 0.5 * SQ(weird_term));
}
