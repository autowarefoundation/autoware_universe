#include "weightedreduction_kernel_test.cuh"

__global__ void setInitialControlToZero_KernelTest(int control_dim, float* u_d, float* u_intermediate)
{
  int thread_idx = blockDim.x * blockIdx.x + threadIdx.x;
  mppi::kernels::setInitialControlToZero(control_dim, thread_idx, u_d, u_intermediate);
}

template <int num_threads, int control_dim>
void launchSetInitialControlToZero_KernelTest(std::array<float, control_dim>& u_host,
                                              std::array<float, num_threads * control_dim>& u_intermediate_host)
{
  float* u_dev;
  float* u_intermediate_dev;

  // Allocate Memory
  HANDLE_ERROR(cudaMalloc((void**)&u_dev, sizeof(float) * u_host.size()))
  HANDLE_ERROR(cudaMalloc((void**)&u_intermediate_dev, sizeof(float) * u_intermediate_host.size()))

  setInitialControlToZero_KernelTest<<<1, num_threads>>>(control_dim, u_dev, u_intermediate_dev);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(u_host.data(), u_dev, sizeof(float) * u_host.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(u_intermediate_host.data(), u_intermediate_dev, sizeof(float) * u_intermediate_host.size(),
                          cudaMemcpyDeviceToHost))

  cudaFree(u_dev);
  cudaFree(u_intermediate_dev);
}

template <int control_dim>
__global__ void strideControlWeightReduction_KernelTest(int num_rollouts, int num_timesteps, int sum_stride,
                                                        float* exp_costs_d, float normalizer, float* du_d,
                                                        float* u_intermediate)
{
  int thread_idx = threadIdx.x;
  int block_idx = blockIdx.x;

  float u_thread[control_dim];
  float* u_intermediate_thread = &u_intermediate[block_idx * control_dim * ((num_rollouts - 1) / sum_stride + 1)];

  mppi::kernels::strideControlWeightReduction(num_rollouts, num_timesteps, sum_stride, thread_idx, block_idx,
                                              control_dim, exp_costs_d, normalizer, du_d, u_thread,
                                              u_intermediate_thread);
}

template <int control_dim, int num_rollouts, int num_timesteps, int sum_stride>
void launchStrideControlWeightReduction_KernelTest(
    float normalizer, const std::array<float, num_rollouts>& exp_costs_host,
    const std::array<float, num_rollouts * num_timesteps * control_dim>& du_host,
    std::array<float, num_timesteps * control_dim*((num_rollouts - 1) / sum_stride + 1)>& u_intermediate_host)
{
  float* exp_costs_dev;
  float* du_dev;
  float* u_intermediate_dev;

  // Allocate Memory
  HANDLE_ERROR(cudaMalloc((void**)&exp_costs_dev, sizeof(float) * exp_costs_host.size()));
  HANDLE_ERROR(cudaMalloc((void**)&du_dev, sizeof(float) * du_host.size()))
  HANDLE_ERROR(cudaMalloc((void**)&u_intermediate_dev, sizeof(float) * u_intermediate_host.size()));

  HANDLE_ERROR(
      cudaMemcpy(exp_costs_dev, exp_costs_host.data(), sizeof(float) * exp_costs_host.size(), cudaMemcpyHostToDevice))
  HANDLE_ERROR(cudaMemcpy(du_dev, du_host.data(), sizeof(float) * du_host.size(), cudaMemcpyHostToDevice))

  dim3 blockdim((num_rollouts - 1) / sum_stride + 1, 1, 1);
  dim3 griddim(num_timesteps, 1, 1);

  strideControlWeightReduction_KernelTest<control_dim><<<griddim, blockdim>>>(
      num_rollouts, num_timesteps, sum_stride, exp_costs_dev, normalizer, du_dev, u_intermediate_dev);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(u_intermediate_host.data(), u_intermediate_dev, sizeof(float) * u_intermediate_host.size(),
                          cudaMemcpyDeviceToHost))

  cudaFree(exp_costs_dev);
  cudaFree(du_dev);
  cudaFree(u_intermediate_dev);
}

template <int control_dim>
__global__ void rolloutWeightReductionAndSaveControl_KernelTest(int num_rollouts, int num_timesteps, int sum_stride,
                                                                float* u_intermediate, float* du_new_d)
{
  int thread_idx = threadIdx.x;  // Current cell
  int block_idx = blockIdx.x;    // Current timestep

  float u[control_dim];
  float* u_intermediate_thread = &u_intermediate[block_idx * control_dim * ((num_rollouts - 1) / sum_stride + 1)];
  mppi::kernels::rolloutWeightReductionAndSaveControl(thread_idx, block_idx, num_rollouts, num_timesteps, control_dim,
                                                      sum_stride, u, u_intermediate_thread, du_new_d);
}

template <int control_dim, int num_rollouts, int num_timesteps, int sum_stride>
void launchRolloutWeightReductionAndSaveControl_KernelTest(
    const std::array<float, num_timesteps * control_dim*((num_rollouts - 1) / sum_stride + 1)>& u_intermediate_host,
    std::array<float, num_timesteps * control_dim>& du_new_host)
{
  float* u_intermediate_dev;
  float* du_new_dev;

  // Allocate Memory
  HANDLE_ERROR(cudaMalloc((void**)&u_intermediate_dev, sizeof(float) * u_intermediate_host.size()))
  HANDLE_ERROR(cudaMalloc((void**)&du_new_dev, sizeof(float) * du_new_host.size()))

  HANDLE_ERROR(cudaMemcpy(u_intermediate_dev, u_intermediate_host.data(), sizeof(float) * u_intermediate_host.size(),
                          cudaMemcpyHostToDevice))
  dim3 blockdim((num_rollouts - 1) / sum_stride + 1, 1, 1);
  dim3 griddim(num_timesteps, 1, 1);

  rolloutWeightReductionAndSaveControl_KernelTest<control_dim>
      <<<griddim, blockdim>>>(num_rollouts, num_timesteps, sum_stride, u_intermediate_dev, du_new_dev);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(du_new_host.data(), du_new_dev, sizeof(float) * du_new_host.size(), cudaMemcpyDeviceToHost))

  cudaFree(u_intermediate_dev);
  cudaFree(du_new_dev);
}

// Old Weighted Reduction Kernel Implementation for comparison
template <int CONTROL_DIM, int NUM_ROLLOUTS, int BLOCKSIZE_WRX>
__global__ void autorallyWeightedReductionKernel(float* states_d, float* du_d, float normalizer, int num_timesteps)
{
  int tdx = threadIdx.x;
  int bdx = blockIdx.x;

  __shared__ float u_system[CONTROL_DIM * ((NUM_ROLLOUTS - 1) / BLOCKSIZE_WRX + 1)];
  int stride = BLOCKSIZE_WRX;

  float u[CONTROL_DIM];

  int i, j;
  for (i = 0; i < CONTROL_DIM; i++)
  {
    u[i] = 0;
  }

  for (j = 0; j < CONTROL_DIM; j++)
  {
    u_system[tdx * CONTROL_DIM + j] = 0;
  }
  __syncthreads();

  if (BLOCKSIZE_WRX * tdx < NUM_ROLLOUTS)
  {
    float weight = 0;
    for (i = 0; i < stride; i++)
    {
      if (stride * tdx + i < NUM_ROLLOUTS)
      {
        weight = states_d[stride * tdx + i] / normalizer;
        for (j = 0; j < CONTROL_DIM; j++)
        {
          u[j] = du_d[(stride * tdx + i) * (num_timesteps * CONTROL_DIM) + bdx * CONTROL_DIM + j];
          u_system[tdx * CONTROL_DIM + j] += weight * u[j];
        }
      }
    }
  }
  __syncthreads();
  if (tdx == 0 && bdx < num_timesteps)
  {
    for (i = 0; i < CONTROL_DIM; i++)
    {
      u[i] = 0;
    }
    for (i = 0; i < (NUM_ROLLOUTS - 1) / BLOCKSIZE_WRX + 1; i++)
    {
      for (j = 0; j < CONTROL_DIM; j++)
      {
        u[j] += u_system[CONTROL_DIM * i + j];
      }
    }
    for (i = 0; i < CONTROL_DIM; i++)
    {
      du_d[CONTROL_DIM * bdx + i] = u[i];
    }
  }
}

template <int CONTROL_DIM, int NUM_ROLLOUTS, int BLOCKSIZE_WRX, int NUM_TIMESTEPS>
void launchAutoRallyWeightedReductionKernelTest(
    std::array<float, NUM_ROLLOUTS> exp_costs,
    std::array<float, CONTROL_DIM * NUM_ROLLOUTS * NUM_TIMESTEPS> perturbed_controls, float normalizer,
    std::array<float, CONTROL_DIM * NUM_TIMESTEPS>& controls_out, cudaStream_t stream)
{
  float* exp_costs_d_;
  float* perturbed_controls_d_;

  // Allocate CUDA memory for the device pointers
  HANDLE_ERROR(cudaMalloc((void**)&exp_costs_d_, sizeof(float) * exp_costs.size()));
  HANDLE_ERROR(cudaMalloc((void**)&perturbed_controls_d_, sizeof(float) * perturbed_controls.size()));

  // Memcpy the data to the device
  HANDLE_ERROR(cudaMemcpyAsync(exp_costs_d_, exp_costs.data(), sizeof(float) * exp_costs.size(), cudaMemcpyHostToDevice,
                               stream));
  HANDLE_ERROR(cudaMemcpyAsync(perturbed_controls_d_, perturbed_controls.data(),
                               sizeof(float) * perturbed_controls.size(), cudaMemcpyHostToDevice, stream));

  // Setup the block and grid sizes
  dim3 dimBlock((NUM_ROLLOUTS - 1) / BLOCKSIZE_WRX + 1, 1, 1);
  dim3 dimGrid(NUM_TIMESTEPS, 1, 1);

  // Kernel launch and error check
  autorallyWeightedReductionKernel<CONTROL_DIM, NUM_ROLLOUTS, BLOCKSIZE_WRX>
      <<<dimGrid, dimBlock, 0, stream>>>(exp_costs_d_, perturbed_controls_d_, normalizer, NUM_TIMESTEPS);
  CudaCheckError();

  // Copy the result back to the GPU
  HANDLE_ERROR(cudaMemcpyAsync(controls_out.data(), perturbed_controls_d_, sizeof(float) * controls_out.size(),
                               cudaMemcpyDeviceToHost, stream));

  // Free the CUDA memory
  HANDLE_ERROR(cudaFree(exp_costs_d_));
  HANDLE_ERROR(cudaFree(perturbed_controls_d_));
}

template <int CONTROL_DIM, int NUM_ROLLOUTS, int SUM_STRIDE, int NUM_TIMESTEPS>
void launchWeightedReductionKernelTest(std::array<float, NUM_ROLLOUTS> exp_costs,
                                       std::array<float, CONTROL_DIM * NUM_ROLLOUTS * NUM_TIMESTEPS> perturbed_controls,
                                       float normalizer, std::array<float, CONTROL_DIM * NUM_TIMESTEPS>& controls_out,
                                       cudaStream_t stream)
{
  float* exp_costs_d_;
  float* perturbed_controls_d_;
  float* optimal_controls_d_;

  // Allocate CUDA memory for the device pointers
  HANDLE_ERROR(cudaMalloc((void**)&exp_costs_d_, sizeof(float) * exp_costs.size()));
  HANDLE_ERROR(cudaMalloc((void**)&perturbed_controls_d_, sizeof(float) * perturbed_controls.size()));
  HANDLE_ERROR(cudaMalloc((void**)&optimal_controls_d_, sizeof(float) * controls_out.size()));

  // Memcpy the data to the device
  HANDLE_ERROR(cudaMemcpyAsync(exp_costs_d_, exp_costs.data(), sizeof(float) * exp_costs.size(), cudaMemcpyHostToDevice,
                               stream));
  HANDLE_ERROR(cudaMemcpyAsync(perturbed_controls_d_, perturbed_controls.data(),
                               sizeof(float) * perturbed_controls.size(), cudaMemcpyHostToDevice, stream));

  // Setup the block and grid sizes
  dim3 dimBlock((NUM_ROLLOUTS - 1) / SUM_STRIDE + 1, 1, 1);
  dim3 dimGrid(NUM_TIMESTEPS, 1, 1);

  mppi::kernels::weightedReductionKernel<CONTROL_DIM><<<dimGrid, dimBlock, 0, stream>>>(
      exp_costs_d_, perturbed_controls_d_, optimal_controls_d_, normalizer, NUM_TIMESTEPS, NUM_ROLLOUTS, SUM_STRIDE);
  CudaCheckError();

  // Copy the result back to the GPU
  HANDLE_ERROR(cudaMemcpyAsync(controls_out.data(), optimal_controls_d_, sizeof(float) * controls_out.size(),
                               cudaMemcpyDeviceToHost, stream));

  // Free the CUDA memory
  HANDLE_ERROR(cudaFree(exp_costs_d_));
  HANDLE_ERROR(cudaFree(perturbed_controls_d_));
  HANDLE_ERROR(cudaFree(optimal_controls_d_));
}
