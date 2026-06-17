#include "cost_generic_kernel_tests.cuh"

template <class COST_T>
__global__ void computeRunningCostTestKernel(COST_T* __restrict__ cost, const float* __restrict__ y_d,
                                             const float* __restrict__ u_d, int num_rollouts, int num_timesteps,
                                             float dt, float* __restrict__ output_cost)
{
  const int sample_idx = blockIdx.x;
  const int t = threadIdx.x;
  const int max_time_iters = ceilf((float)num_timesteps / blockDim.x);
  const int sys_index = threadIdx.z + blockDim.z * blockIdx.z;

  extern __shared__ float entire_buffer[];
  float* y_shared = entire_buffer;
  float* u_shared = &y_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::OUTPUT_DIM)];
  // int* crash_shared = (int*)&u_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::CONTROL_DIM)];
  // float* running_cost_shared = (float*)&crash_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z)];
  // float* theta_c_shared = &running_cost_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * blockDim.y)];

  float* running_cost_shared = &u_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::CONTROL_DIM)];
  int* crash_shared = (int*)&running_cost_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * blockDim.y)];
  float* theta_c_shared = (float*)&crash_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z)];

  const int shared_idx = threadIdx.x + blockDim.x * threadIdx.z;
  const int cost_idx = threadIdx.x + blockDim.x * (threadIdx.y + blockDim.y * threadIdx.z);
  float* y = &y_shared[shared_idx * COST_T::OUTPUT_DIM];
  float* u = &u_shared[shared_idx * COST_T::CONTROL_DIM];
  float* running_cost = &running_cost_shared[cost_idx];

  int* crash = &crash_shared[shared_idx];
  *crash = 0;

  // Fill in shared memory
  const int global_mem_index = (num_rollouts * sys_index + sample_idx) * num_timesteps + t;
  mp1::loadArrayParallel<COST_T::OUTPUT_DIM>(y, 0, y_d, global_mem_index * COST_T::OUTPUT_DIM);
  mp1::loadArrayParallel<COST_T::CONTROL_DIM>(u, 0, u_d, global_mem_index * COST_T::CONTROL_DIM);
  __syncthreads();

  cost->initializeCosts(y, u, theta_c_shared, 0, dt);
  __syncthreads();
  running_cost[0] = cost->computeRunningCost(y, u, t, theta_c_shared, crash);

  __syncthreads();
  // if (threadIdx.y == 0)
  // {
  //   int num_zeros = 0;
  //   for (int i = 0; i < blockDim.y; i++)
  //   {
  //     if (running_cost[i * blockDim.x] == 0)
  //     {
  //       num_zeros++;
  //     }
  //   }
  //   if (num_zeros != blockDim.y - 1)
  //   {
  //     printf("Sample %d, t %d block_y %d: ", sample_idx, t, blockDim.y);
  //     for (int i = 0; i < blockDim.y; i++)
  //     {
  //       printf("%f, ", running_cost[i * blockDim.x]);
  //     }
  //     printf("\n");
  //   }
  // }
  // if (sample_idx == 0 && t == 0 && blockDim.y < 4)
  // {
  //   printf("Cost y %d: %f\n", threadIdx.y, running_cost[0]);
  // }
  // __syncthreads();

  running_cost = &running_cost_shared[threadIdx.x + blockDim.x * blockDim.y * threadIdx.z];
  __syncthreads();
  mppi::kernels::costArrayReduction(running_cost, blockDim.y, threadIdx.y, blockDim.y, threadIdx.y == 0, blockDim.x);
  // int prev_size = blockDim.y;
  // // Allow for better computation when blockDim.x is a power of 2
  // const bool block_power_of_2 = (prev_size & (prev_size - 1)) == 0;
  // const int stop_condition = (block_power_of_2) ? 32 : 0;
  // int size;
  // const int xy_index = threadIdx.y;
  // const int xy_step = blockDim.y;

  // for (size = prev_size / 2; size > stop_condition; size /= 2)
  // {
  //   for (int j = xy_index; j < size; j += xy_step)
  //   {
  //     running_cost[j * blockDim.x] += running_cost[(j + size) * blockDim.x];
  //   }
  //   __syncthreads();
  //   if (prev_size - 2 * size == 1 && xy_index == 0)
  //   {
  //     running_cost[(size - 1) * blockDim.x] += running_cost[(prev_size - 1) * blockDim.x];
  //   }
  //   __syncthreads();
  //   prev_size = size;
  // }

  // if (xy_index < 32 && stop_condition != 0)
  // {  // unroll the last warp
  //   switch (size * 2)
  //   {
  //     case 64:
  //       mppi::kernels::warpReduceAdd<64>(running_cost, xy_index, blockDim.x);
  //       break;
  //     case 32:
  //       mppi::kernels::warpReduceAdd<32>(running_cost, xy_index, blockDim.x);
  //       break;
  //     case 16:
  //       mppi::kernels::warpReduceAdd<16>(running_cost, xy_index, blockDim.x);
  //       break;
  //     case 8:
  //       mppi::kernels::warpReduceAdd<8>(running_cost, xy_index, blockDim.x);
  //       break;
  //     case 4:
  //       mppi::kernels::warpReduceAdd<4>(running_cost, xy_index, blockDim.x);
  //       break;
  //     case 2:
  //       mppi::kernels::warpReduceAdd<2>(running_cost, xy_index, blockDim.x);
  //       break;
  //     case 1:
  //       mppi::kernels::warpReduceAdd<1>(running_cost, xy_index, blockDim.x);
  //       break;
  //   }
  // }
  // __syncthreads();
  // __syncthreads();
  // if (sample_idx == 0 && t == 0 && blockDim.y < 4)
  // {
  //   printf("Final Cost y %d: %f\n", threadIdx.y, running_cost[0]);
  // }
  __syncthreads();
  if (threadIdx.y == 0)
  {
    output_cost[global_mem_index] = running_cost[0];
  }
}

template <class COST_T>
__global__ void computeTerminalCostTestKernel(COST_T* __restrict__ cost, const float* __restrict__ y_d,
                                              int num_rollouts, float dt, float* __restrict__ output_cost)
{
  const int sample_idx = threadIdx.x + blockIdx.x * blockDim.x;
  const int sys_index = threadIdx.z + blockDim.z * blockIdx.z;

  extern __shared__ float entire_buffer[];
  float* y_shared = entire_buffer;
  float* u_shared = &y_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::OUTPUT_DIM)];
  float* theta_c_shared = &u_shared[mppi::math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::CONTROL_DIM)];

  const int shared_idx = threadIdx.x + blockDim.x * threadIdx.z;
  float* y = &y_shared[shared_idx * COST_T::OUTPUT_DIM];
  float* u = &u_shared[shared_idx * COST_T::CONTROL_DIM];

  // Fill in shared memory
  const int global_mem_index = num_rollouts * sys_index + sample_idx;
  mp1::loadArrayParallel<mp1::Parallel1Dir::THREAD_Y>(y, 0, y_d, global_mem_index * COST_T::OUTPUT_DIM);
  for (int i = threadIdx.y; i < COST_T::CONTROL_DIM; i += blockDim.y)
  {
    u[i] = 0.0f;
  }
  __syncthreads();

  cost->initializeCosts(y, u, theta_c_shared, 0, dt);
  __syncthreads();
  output_cost[global_mem_index] = cost->terminalCost(y, theta_c_shared);
}

template <class COST_T>
void launchRunningCostTestKernel(COST_T& cost, std::vector<std::array<float, COST_T::OUTPUT_DIM>>& y,
                                 std::vector<std::array<float, COST_T::CONTROL_DIM>>& u, int num_rollouts,
                                 int num_timesteps, float dt, int dim_y, std::vector<float>& output_costs)
{
  if (y.size() != num_rollouts * num_timesteps)
  {
    std::cout << "Number of outputs does not match num_rollouts * num_timesteps. Output: " << y.size()
              << ", num_rollouts: " << num_rollouts << ", num_timesteps: " << num_timesteps << std::endl;
    exit(-1);
  }
  if (u.size() != y.size())
  {
    std::cout << "Number of controls does not match number of outputs. Output: " << y.size()
              << ", Control: " << u.size() << std::endl;
    exit(-1);
  }
  dim3 block_dim = dim3(num_timesteps, dim_y, 1);
  dim3 grid_dim = dim3(num_rollouts, 1, 1);
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  // Global Memory setup
  float* y_d;
  float* u_d;
  float* output_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&y_d, sizeof(float) * num_rollouts * num_timesteps * COST_T::OUTPUT_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&u_d, sizeof(float) * num_rollouts * num_timesteps * COST_T::CONTROL_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&output_costs_d, sizeof(float) * num_rollouts * num_timesteps));

  // Copy data to GPU
  for (int k = 0; k < num_rollouts; k++)
  {
    for (int t = 0; t < num_timesteps; t++)
    {
      const int index = k * num_timesteps + t;
      HANDLE_ERROR(cudaMemcpyAsync(y_d + index * COST_T::OUTPUT_DIM, y[index].data(),
                                   sizeof(float) * COST_T::OUTPUT_DIM, cudaMemcpyHostToDevice, stream));
      HANDLE_ERROR(cudaMemcpyAsync(u_d + index * COST_T::CONTROL_DIM, u[index].data(),
                                   sizeof(float) * COST_T::CONTROL_DIM, cudaMemcpyHostToDevice, stream));
    }
  }

  // Figure shared memory size
  const int block_num_shared = block_dim.x * block_dim.z;
  const int block_cost_shared_mem_size =
      mppi::math::int_multiple_const(COST_T::SHARED_MEM_REQUEST_GRD_BYTES, sizeof(float4)) +
      block_num_shared * mppi::math::int_multiple_const(COST_T::SHARED_MEM_REQUEST_BLK_BYTES, sizeof(float4));
  unsigned compute_cost_shared_mem_size = (mppi::math::nearest_multiple_4(block_num_shared * COST_T::OUTPUT_DIM) +
                                           mppi::math::nearest_multiple_4(block_num_shared * COST_T::CONTROL_DIM) +
                                           mppi::math::nearest_multiple_4(block_num_shared * dim_y)) *
                                              sizeof(float) +
                                          mppi::math::nearest_multiple_4(block_num_shared) * sizeof(int) +
                                          block_cost_shared_mem_size;
  // Launch kernel
  computeRunningCostTestKernel<COST_T><<<grid_dim, block_dim, compute_cost_shared_mem_size, stream>>>(
      cost.cost_d_, y_d, u_d, num_rollouts, num_timesteps, dt, output_costs_d);

  // Copy memory back to CPU
  output_costs.resize(num_rollouts * num_timesteps);
  HANDLE_ERROR(cudaMemcpyAsync(output_costs.data(), output_costs_d, sizeof(float) * num_rollouts * num_timesteps,
                               cudaMemcpyDeviceToHost, stream));

  // Free memory
  HANDLE_ERROR(cudaFree(y_d));
  HANDLE_ERROR(cudaFree(u_d));
  HANDLE_ERROR(cudaFree(output_costs_d));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
}

template <class COST_T, class SAMPLING_T>
void launchRolloutCostKernel(COST_T& cost, SAMPLING_T& sampler, std::vector<std::array<float, COST_T::OUTPUT_DIM>>& y,
                             int num_rollouts, int num_timesteps, float dt, int dim_y, std::vector<float>& output_costs)
{
  if (y.size() != num_rollouts * num_timesteps)
  {
    std::cout << "Number of outputs does not match num_rollouts * num_timesteps. Output: " << y.size()
              << ", num_rollouts: " << num_rollouts << ", num_timesteps: " << num_timesteps << std::endl;
    exit(-1);
  }
  dim3 block_dim = dim3(num_timesteps, dim_y, 1);
  dim3 grid_dim = dim3(num_rollouts, 1, 1);
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));
  sampler.bindToStream(stream);

  // Global Memory setup
  float* y_d;
  float* output_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&y_d, sizeof(float) * num_rollouts * num_timesteps * COST_T::OUTPUT_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&output_costs_d, sizeof(float) * num_rollouts * num_timesteps));

  // Copy data to GPU
  for (int k = 0; k < num_rollouts; k++)
  {
    for (int t = 0; t < num_timesteps; t++)
    {
      const int index = k * num_timesteps + t;
      HANDLE_ERROR(cudaMemcpyAsync(y_d + index * COST_T::OUTPUT_DIM, y[index].data(),
                                   sizeof(float) * COST_T::OUTPUT_DIM, cudaMemcpyHostToDevice, stream));
    }
  }

  // Figure shared memory size
  // const int block_num_shared = block_dim.x * block_dim.z;
  unsigned compute_cost_shared_mem_size = mppi::kernels::calcRolloutCostKernelSharedMemSize(&cost, &sampler, block_dim);
  //       sizeof(float) * (mppi::math::nearest_multiple_4(block_num_shared * COST_T::OUTPUT_DIM) +
  //                        mppi::math::nearest_multiple_4(block_num_shared * COST_T::CONTROL_DIM) +
  //                        mppi::math::nearest_multiple_4(block_num_shared * block_dim.y)) +
  //       sizeof(int) * mppi::math::nearest_multiple_4(block_num_shared) +
  //       mppi::math::int_multiple_const(COST_T::SHARED_MEM_REQUEST_GRD_BYTES, sizeof(float4)) +
  //       block_num_shared * mppi::math::int_multiple_const(COST_T::SHARED_MEM_REQUEST_BLK_BYTES, sizeof(float4)) +
  //       mppi::math::int_multiple_const(SAMPLING_T::SHARED_MEM_REQUEST_GRD_BYTES, sizeof(float4)) +
  //       block_num_shared * mppi::math::int_multiple_const(SAMPLING_T::SHARED_MEM_REQUEST_BLK_BYTES, sizeof(float4));
  // #ifdef USE_CUDA_BARRIERS_COST
  //   compute_cost_shared_mem_size += mppi::math::int_multiple_const(block_num_shared * sizeof(barrier), 16);
  // #endif
  // Launch kernel
  mppi::kernels::rolloutCostKernel<COST_T, SAMPLING_T><<<grid_dim, block_dim, compute_cost_shared_mem_size, stream>>>(
      cost.cost_d_, sampler.sampling_d_, dt, num_timesteps, num_rollouts, 1.0, 0.0, y_d, output_costs_d);

  // Copy memory back to CPU
  output_costs.resize(num_rollouts);
  HANDLE_ERROR(cudaMemcpyAsync(output_costs.data(), output_costs_d, sizeof(float) * num_rollouts,
                               cudaMemcpyDeviceToHost, stream));

  // Free memory
  HANDLE_ERROR(cudaFree(y_d));
  HANDLE_ERROR(cudaFree(output_costs_d));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
}

template <class COST_T>
void launchTerminalCostTestKernel(COST_T& cost, std::vector<std::array<float, COST_T::OUTPUT_DIM>>& y, float dt,
                                  int dim_x, int dim_y, std::vector<float>& output_costs)
{
  const int num_rollouts = y.size();
  dim3 block_dim = dim3(dim_x, dim_y, 1);
  dim3 grid_dim = dim3(num_rollouts, 1, 1);
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  // Global Memory setup
  float* y_d;
  float* output_costs_d;
  HANDLE_ERROR(cudaMalloc((void**)&y_d, sizeof(float) * num_rollouts * COST_T::OUTPUT_DIM));
  HANDLE_ERROR(cudaMalloc((void**)&output_costs_d, sizeof(float) * num_rollouts));

  // Copy data to GPU
  for (int k = 0; k < num_rollouts; k++)
  {
    HANDLE_ERROR(cudaMemcpyAsync(y_d + k * COST_T::OUTPUT_DIM, y[k].data(), sizeof(float) * COST_T::OUTPUT_DIM,
                                 cudaMemcpyHostToDevice, stream));
  }

  // Figure shared memory size
  const int block_num_shared = block_dim.x * block_dim.z;
  const int block_cost_shared_mem_size =
      mppi::math::int_multiple_const(COST_T::SHARED_MEM_REQUEST_GRD_BYTES, sizeof(float4)) +
      block_num_shared * mppi::math::int_multiple_const(COST_T::SHARED_MEM_REQUEST_BLK_BYTES, sizeof(float4));
  unsigned compute_cost_shared_mem_size = (mppi::math::nearest_multiple_4(block_num_shared * COST_T::OUTPUT_DIM) +
                                           mppi::math::nearest_multiple_4(block_num_shared * COST_T::CONTROL_DIM)) *
                                              sizeof(float) +
                                          block_cost_shared_mem_size;
  // Launch kernel
  computeTerminalCostTestKernel<COST_T><<<grid_dim, block_dim, compute_cost_shared_mem_size, stream>>>(
      cost.cost_d_, y_d, num_rollouts, dt, output_costs_d);

  // Copy memory back to CPU
  HANDLE_ERROR(cudaMemcpyAsync(output_costs.data(), output_costs_d, sizeof(float) * num_rollouts,
                               cudaMemcpyDeviceToHost, stream));

  // Free memory
  HANDLE_ERROR(cudaFree(y_d));
  HANDLE_ERROR(cudaFree(output_costs_d));
  HANDLE_ERROR(cudaStreamSynchronize(stream));
}

template <class COST_T>
void checkGPURolloutCost(COST_T& cost, float dt)
{
  cost.GPUSetup();
  const int num_rollouts = 512;
  const int num_timesteps = 8;
#ifdef USE_ROLLOUT_COST_KERNEL
  using SAMPLER_T = mppi::sampling_distributions::GaussianDistribution<typename COST_T::TEMPLATED_DYN_PARAMS>;
  using SAMPLER_PARAMS = typename SAMPLER_T::SAMPLING_PARAMS_T;
  SAMPLER_PARAMS params;
  for (int i = 0; i < COST_T::CONTROL_DIM; i++)
  {
    params.std_dev[i] = 1.0;
    params.control_cost_coeff[i] = 0.0;
  }
  params.num_rollouts = num_rollouts;
  params.num_timesteps = num_timesteps;
  SAMPLER_T sampler(params);
  sampler.GPUSetup();

  // Curand generator
  cudaStream_t stream;
  cudaStreamCreate(&stream);
  curandGenerator_t gen;
  int seed = 42;
  curandCreateGenerator(&gen, CURAND_RNG_PSEUDO_DEFAULT);
  curandSetPseudoRandomGeneratorSeed(gen, seed);
  curandSetGeneratorOffset(gen, 0);
  curandSetStream(gen, stream);
  sampler.bindToStream(stream);
#endif

  using control_trajectory = Eigen::Matrix<float, COST_T::CONTROL_DIM, num_timesteps>;
  using output_trajectory = Eigen::Matrix<float, COST_T::OUTPUT_DIM, num_timesteps>;
  std::vector<std::array<float, COST_T::OUTPUT_DIM>> y(num_timesteps * num_rollouts);
#ifndef USE_ROLLOUT_COST_KERNEL
  std::vector<std::array<float, COST_T::CONTROL_DIM>> u(num_timesteps * num_rollouts);
#endif

  std::vector<float> output_cost_gpu;

  for (int n = 0; n < num_rollouts; n++)
  {
#ifndef USE_ROLLOUT_COST_KERNEL
    control_trajectory u_traj = control_trajectory::Random();
#endif
    output_trajectory y_traj = output_trajectory::Random();
    for (int t = 0; t < num_timesteps; t++)
    {
      int index = t + n * num_timesteps;
      for (int i = 0; i < COST_T::OUTPUT_DIM; i++)
      {
        y[index][i] = y_traj(i, t);
      }
#ifndef USE_ROLLOUT_COST_KERNEL
      for (int i = 0; i < COST_T::CONTROL_DIM; i++)
      {
        u[index][i] = u_traj(i, t);
      }
#endif
    }
  }

#ifdef USE_ROLLOUT_COST_KERNEL
  sampler.generateSamples(0, 0, gen, true);
#endif

  for (int y_dim = 1; y_dim < 32; y_dim++)
  {
#ifdef USE_ROLLOUT_COST_KERNEL
    launchRolloutCostKernel<COST_T, SAMPLER_T>(cost, sampler, y, num_rollouts, num_timesteps, dt, y_dim,
                                               output_cost_gpu);
#else
    launchRunningCostTestKernel<COST_T>(cost, y, u, num_rollouts, num_timesteps, dt, y_dim, output_cost_gpu);
#endif
    for (int n = 0; n < num_rollouts; n++)
    {
#ifdef USE_ROLLOUT_COST_KERNEL
      control_trajectory u_traj_n;
      HANDLE_ERROR(cudaMemcpy(u_traj_n.data(), sampler.getControlSample(n, 0, 0),
                              sizeof(float) * num_timesteps * COST_T::CONTROL_DIM, cudaMemcpyDeviceToHost));
      float total_cost = 0;
      for (int t = 0; t < num_timesteps; t++)
      {
        int index = t + n * num_timesteps;
        typename COST_T::control_array u_index = u_traj_n.col(t);
        Eigen::Map<typename COST_T::output_array> y_index(y[index].data());
#else
      for (int t = 0; t < num_timesteps; t++)
      {
        int index = t + n * num_timesteps;
        Eigen::Map<typename COST_T::control_array> u_index(u[index].data());
        Eigen::Map<typename COST_T::output_array> y_index(y[index].data());
#endif

        int crash = 0;
        float cpu_cost = cost.computeRunningCost(y_index, u_index, t, &crash);
#ifdef USE_ROLLOUT_COST_KERNEL
        cpu_cost += sampler.computeLikelihoodRatioCost(u_index, n, t, 0);
        total_cost += cpu_cost;
#else
        ASSERT_NEAR(cpu_cost, output_cost_gpu[index], cpu_cost * 0.0015)
            << " Sample " << n << ", t " << t << " y_dim " << y_dim;
#endif
      }
#ifdef USE_ROLLOUT_COST_KERNEL
      total_cost /= num_timesteps;
      Eigen::Map<typename COST_T::output_array> y_index(y[(n + 1) * num_timesteps - 1].data());
      total_cost += cost.terminalCost(y_index) / num_timesteps;
      ASSERT_NEAR(total_cost, output_cost_gpu[n], total_cost * 0.001) << " Sample " << n << " y_dim " << y_dim;
#endif
    }
  }
}
