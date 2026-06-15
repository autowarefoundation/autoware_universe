#include <mppi/core/mppi_common.cuh>
#include <curand.h>
#include <mppi/utils/gpu_err_chk.cuh>
#include <mppi/utils/math_utils.h>
#include <mppi/utils/cuda_math_utils.cuh>

// CUDA barriers were first implemented in CUDA 11
#if defined(CMAKE_USE_CUDA_BARRIERS) && defined(CUDART_VERSION) && CUDART_VERSION > 11000
#include <cuda/barrier>
using barrier = cuda::barrier<cuda::thread_scope_block>;

// Turn on/off various CUDA barriers from CMake configuration
#ifdef CMAKE_USE_CUDA_BARRIERS_DYN
#define USE_CUDA_BARRIERS_DYN
#endif
#ifdef CMAKE_USE_CUDA_BARRIERS_COST
#define USE_CUDA_BARRIERS_COST
#endif
#ifdef CMAKE_USE_CUDA_BARRIERS_ROLLOUT
#define USE_CUDA_BARRIERS_ROLLOUT
#endif
#endif

#include <cooperative_groups.h>
namespace cg = cooperative_groups;
namespace mp1 = mppi::p1;

namespace mppi
{
namespace kernels
{
/*******************************************************************************************************************
 * Kernel Functions
 *******************************************************************************************************************/
template <class DYN_T, class COST_T, class SAMPLING_T>
__global__ void rolloutKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling,
                              COST_T* __restrict__ costs, float dt, const int num_timesteps, const int num_rollouts,
                              const float* __restrict__ init_x_d, float lambda, float alpha,
                              float* __restrict__ trajectory_costs_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int block_idx = blockIdx.x;
  const int global_idx = blockDim.x * block_idx + thread_idx;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int distribution_idx = threadIdx.z;
  const int distribution_dim = blockDim.z;
  const int sample_dim = blockDim.x;
  // Ensure that there is enough room for the SHARED_MEM_REQUEST_GRD_BYTES and SHARED_MEM_REQUEST_BLK_BYTES portions to
  // be aligned to the float4 boundary.
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];

  float* x_shared = entire_buffer;
  float* x_next_shared = &x_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* y_shared = &x_next_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* x_dot_shared = &y_shared[math::nearest_multiple_4(sample_dim * DYN_T::OUTPUT_DIM * distribution_dim)];
  float* u_shared = &x_dot_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* theta_s_shared = &u_shared[math::nearest_multiple_4(sample_dim * DYN_T::CONTROL_DIM * distribution_dim)];
  float* theta_d_shared = &theta_s_shared[size_of_theta_s_bytes / sizeof(float)];
  float* theta_c_shared = &theta_d_shared[size_of_theta_d_bytes / sizeof(float)];
  float* running_cost_shared = &theta_c_shared[size_of_theta_c_bytes / sizeof(float)];
  int* crash_status_shared = (int*)&running_cost_shared[math::nearest_multiple_4(blockDim.x * blockDim.y * blockDim.z)];

#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* barrier_shared = (barrier*)&crash_status_shared[math::nearest_multiple_4(sample_dim * distribution_dim)];
#endif

  // Create local state, state dot and controls
  int running_cost_index = thread_idx + blockDim.x * (thread_idy + blockDim.y * thread_idz);
  float* x = &x_shared[shared_idx * DYN_T::STATE_DIM];
  float* x_next = &x_next_shared[shared_idx * DYN_T::STATE_DIM];
  float* x_temp;
  float* xdot = &x_dot_shared[shared_idx * DYN_T::STATE_DIM];
  float* u = &u_shared[shared_idx * DYN_T::CONTROL_DIM];
  float* y = &y_shared[shared_idx * DYN_T::OUTPUT_DIM];
  float* running_cost = &running_cost_shared[running_cost_index];
  running_cost[0] = 0.0f;
  int* crash_status = &crash_status_shared[shared_idx];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* bar = &barrier_shared[shared_idx];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  // Load global array to shared array
  loadGlobalToShared<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM>(num_rollouts, blockDim.y, global_idx, thread_idy, thread_idz,
                                                           init_x_d, x, xdot, u);
  __syncthreads();

  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0f, dt);
  sampling->initializeDistributions(y, 0.0f, dt, theta_d_shared);
  costs->initializeCosts(y, u, theta_c_shared, 0.0f, dt);
  __syncthreads();
  for (int t = 0; t < num_timesteps; t++)
  {
    // Load noise trajectories scaled by the exploration factor
    sampling->readControlSample(global_idx, t, distribution_idx, u, theta_d_shared, blockDim.y, thread_idy, y);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // applies constraints as defined in dynamics.cuh see specific dynamics class for what happens here
    // usually just control clamping
    dynamics->enforceConstraints(x, u);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    // Copy control constraints back to global memory
    sampling->writeControlSample(global_idx, t, distribution_idx, u, theta_d_shared, blockDim.y, thread_idy, y);

    // Increment states
    dynamics->step(x, x_next, xdot, u, y, theta_s_shared, t, dt);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    running_cost[0] +=
        costs->computeRunningCost(y, u, t, theta_c_shared, crash_status) +
        sampling->computeLikelihoodRatioCost(u, theta_d_shared, global_idx, t, distribution_idx, lambda, alpha);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    x_temp = x;
    x = x_next;
    x_next = x_temp;
  }

  // Add all costs together
  running_cost = &running_cost_shared[thread_idx + blockDim.x * blockDim.y * thread_idz];
  __syncthreads();
  costArrayReduction(running_cost, blockDim.y, thread_idy, blockDim.y, thread_idy == 0, blockDim.x);
  // Compute terminal cost and the final cost for each thread
  computeAndSaveCost(num_rollouts, num_timesteps, global_idx, costs, y, running_cost[0] / (num_timesteps),
                     theta_c_shared, trajectory_costs_d);
}

template <class COST_T, class SAMPLING_T, bool COALESCE>
__global__ void rolloutCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                                  const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                  const float* __restrict__ y_d, float* __restrict__ trajectory_costs_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int global_idx = blockIdx.x;
  const int distribution_idx = threadIdx.z;
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);

  int running_cost_index = thread_idx + blockDim.x * (thread_idy + blockDim.y * thread_idz);
  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];
  float* y_shared = entire_buffer;
  float* u_shared = &y_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::OUTPUT_DIM)];
  float* running_cost_shared = &u_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::CONTROL_DIM)];
  int* crash_status_shared = (int*)&running_cost_shared[math::nearest_multiple_4(blockDim.x * blockDim.y * blockDim.z)];
  float* theta_c = (float*)&crash_status_shared[math::nearest_multiple_4(blockDim.x * blockDim.z)];
  float* theta_d = &theta_c[size_of_theta_c_bytes / sizeof(float)];
#ifdef USE_CUDA_BARRIERS_COST
  barrier* barrier_shared = (barrier*)&theta_d[size_of_theta_d_bytes / sizeof(float)];
#endif

  // Create local state, state dot and controls
  float* y;
  float* u;
  int* crash_status;

  // Initialize running cost and total cost
  float* running_cost;
  int sample_time_offset = 0;

  // Load global array to shared array
  y = &y_shared[(blockDim.x * thread_idz + thread_idx) * COST_T::OUTPUT_DIM];
  u = &u_shared[(blockDim.x * thread_idz + thread_idx) * COST_T::CONTROL_DIM];
  crash_status = &crash_status_shared[thread_idz * blockDim.x + thread_idx];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
  running_cost = &running_cost_shared[running_cost_index];
  running_cost[0] = 0.0f;
#ifdef USE_CUDA_BARRIERS_COST
  barrier* bar = &barrier_shared[(blockDim.x * thread_idz + thread_idx)];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  /*<----Start of simulation loop-----> */
#ifdef USE_COST_WITH_OFF_NUM_TIMESTEPS
  const int max_time_iters = ceilf((float)(num_timesteps - 2) / blockDim.x);
#else
  const int max_time_iters = ceilf((float)num_timesteps / blockDim.x);
#endif
  costs->initializeCosts(y, u, theta_c, 0.0f, dt);
  sampling->initializeDistributions(y, 0.0f, dt, theta_d);
  __syncthreads();
  for (int time_iter = 0; time_iter < max_time_iters; ++time_iter)
  {
    int t = thread_idx + time_iter * blockDim.x;
    if (COALESCE)
    {  // Fill entire shared mem sequentially using sequential threads_idx
      int amount_to_fill = (time_iter + 1) * blockDim.x > num_timesteps ? num_timesteps % blockDim.x : blockDim.x;
      mp1::loadArrayParallel<mp1::Parallel1Dir::THREAD_XY>(
          y_shared, blockDim.x * thread_idz * COST_T::OUTPUT_DIM, y_d,
          ((num_rollouts * thread_idz + global_idx) * num_timesteps + time_iter * blockDim.x) * COST_T::OUTPUT_DIM,
          COST_T::OUTPUT_DIM * amount_to_fill);
    }
    else if (t < num_timesteps)
    {  // t = num_timesteps is the terminal state for outside this for-loop
      sample_time_offset = (num_rollouts * thread_idz + global_idx) * num_timesteps + t;
      mp1::loadArrayParallel<COST_T::OUTPUT_DIM>(y, 0, y_d, sample_time_offset * COST_T::OUTPUT_DIM);
    }
    if (t < num_timesteps)
    {  // load controls from t = 0 to t = num_timesteps - 1
      sampling->readControlSample(global_idx, t, distribution_idx, u, theta_d, blockDim.y, thread_idy, y);
    }
#ifdef USE_CUDA_BARRIERS_COST
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // Compute cost
    if (t < num_timesteps)
    {
      running_cost[0] +=
          costs->computeRunningCost(y, u, t, theta_c, crash_status) +
          sampling->computeLikelihoodRatioCost(u, theta_d, global_idx, t, distribution_idx, lambda, alpha);
    }
#ifdef USE_CUDA_BARRIERS_COST
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
  }

  // Add all costs together
  running_cost = &running_cost_shared[blockDim.x * blockDim.y * thread_idz];
  __syncthreads();
  costArrayReduction(running_cost, blockDim.x * blockDim.y, thread_idx + blockDim.x * thread_idy,
                     blockDim.x * blockDim.y, thread_idx == blockDim.x - 1 && thread_idy == 0);
  // point every thread to the last output at t = NUM_TIMESTEPS for terminal cost calculation
  const int last_y_index = (num_timesteps - 1) % blockDim.x;
  y = &y_shared[(blockDim.x * thread_idz + last_y_index) * COST_T::OUTPUT_DIM];
#ifdef USE_COST_WITH_OFF_NUM_TIMESTEPS
  // load last output array
  const int t = num_timesteps - 1;
  mp1::loadArrayParallel<mp1::Parallel1Dir::THREAD_XY>(
      y_shared, (blockDim.x * thread_idz + last_y_index) * COST_T::OUTPUT_DIM, y_d,
      ((global_idx + num_rollouts * thread_idz) * num_timesteps + t) * COST_T::OUTPUT_DIM, COST_T::OUTPUT_DIM);
  __syncthreads();
#endif
  // Compute terminal cost and the final cost for each thread
  computeAndSaveCost(num_rollouts, num_timesteps, global_idx, costs, y, running_cost[0] / (num_timesteps), theta_c,
                     trajectory_costs_d);
}

template <class DYN_T, class SAMPLING_T>
__global__ void rolloutDynamicsKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling, float dt,
                                      const int num_timesteps, const int num_rollouts,
                                      const float* __restrict__ init_x_d, float* __restrict__ y_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int block_idx = blockIdx.x;
  const int global_idx = blockDim.x * block_idx + thread_idx;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int distribution_idx = threadIdx.z;
  const int distribution_dim = blockDim.z;
  const int sample_dim = blockDim.x;
  // Ensure that there is enough room for the SHARED_MEM_REQUEST_GRD_BYTES and SHARED_MEM_REQUEST_BLK_BYTES portions to
  // be aligned to the float4 boundary.
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];

  float* x_shared = entire_buffer;
  float* x_next_shared = &x_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* y_shared = &x_next_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* x_dot_shared = &y_shared[math::nearest_multiple_4(sample_dim * DYN_T::OUTPUT_DIM * distribution_dim)];
  float* u_shared = &x_dot_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* theta_s_shared = &u_shared[math::nearest_multiple_4(sample_dim * DYN_T::CONTROL_DIM * distribution_dim)];
  float* theta_d_shared = &theta_s_shared[size_of_theta_s_bytes / sizeof(float)];
#ifdef USE_CUDA_BARRIERS_DYN
  barrier* barrier_shared = (barrier*)&theta_d_shared[size_of_theta_d_bytes / sizeof(float)];
#endif

  // Create local state, state dot and controls
  float* x = &x_shared[shared_idx * DYN_T::STATE_DIM];
  float* x_next = &x_next_shared[shared_idx * DYN_T::STATE_DIM];
  float* x_temp;
  float* xdot = &x_dot_shared[shared_idx * DYN_T::STATE_DIM];
  float* u = &u_shared[shared_idx * DYN_T::CONTROL_DIM];
  float* y = &y_shared[shared_idx * DYN_T::OUTPUT_DIM];
#ifdef USE_CUDA_BARRIERS_DYN
  barrier* bar = &barrier_shared[shared_idx];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  // Load global array to shared array
  loadGlobalToShared<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM>(num_rollouts, blockDim.y, global_idx, thread_idy, thread_idz,
                                                           init_x_d, x, xdot, u);
  __syncthreads();

  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0f, dt);
  sampling->initializeDistributions(y, 0.0f, dt, theta_d_shared);
  __syncthreads();
  for (int t = 0; t < num_timesteps; t++)
  {
    // Load noise trajectories scaled by the exploration factor
    sampling->readControlSample(global_idx, t, distribution_idx, u, theta_d_shared, blockDim.y, thread_idy, y);
#ifdef USE_CUDA_BARRIERS_DYN
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // applies constraints as defined in dynamics.cuh see specific dynamics class for what happens here
    // usually just control clamping
    dynamics->enforceConstraints(x, u);
#ifdef USE_CUDA_BARRIERS_DYN
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    // Copy control constraints back to global memory
    sampling->writeControlSample(global_idx, t, distribution_idx, u, theta_d_shared, blockDim.y, thread_idy, y);

    // Increment states
    dynamics->step(x, x_next, xdot, u, y, theta_s_shared, t, dt);
#ifdef USE_CUDA_BARRIERS_DYN
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    x_temp = x;
    x = x_next;
    x_next = x_temp;
    // Copy state to global memory
    int sample_time_offset = (num_rollouts * thread_idz + global_idx) * num_timesteps + t;
    mp1::loadArrayParallel<DYN_T::OUTPUT_DIM>(y_d, sample_time_offset * DYN_T::OUTPUT_DIM, y, 0);
  }
}

template <class DYN_T, class COST_T, class SAMPLING_T>
__global__ void visualizeKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling,
                                COST_T* __restrict__ costs, float dt, const int num_timesteps, const int num_rollouts,
                                const float* __restrict__ init_x_d, float lambda, float alpha, float* __restrict__ y_d,
                                float* __restrict__ cost_traj_d, int* __restrict__ crash_status_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int block_idx = blockIdx.x;
  const int global_idx = blockDim.x * block_idx + thread_idx;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int distribution_idx = threadIdx.z;
  const int distribution_dim = blockDim.z;
  const int sample_dim = blockDim.x;
  // Ensure that there is enough room for the SHARED_MEM_REQUEST_GRD_BYTES and SHARED_MEM_REQUEST_BLK_BYTES portions to
  // be aligned to the float4 boundary.
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];

  float* x_shared = entire_buffer;
  float* x_next_shared = &x_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* y_shared = &x_next_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* x_dot_shared = &y_shared[math::nearest_multiple_4(sample_dim * DYN_T::OUTPUT_DIM * distribution_dim)];
  float* u_shared = &x_dot_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* theta_s_shared = &u_shared[math::nearest_multiple_4(sample_dim * DYN_T::CONTROL_DIM * distribution_dim)];
  float* theta_d_shared = &theta_s_shared[size_of_theta_s_bytes / sizeof(float)];
  float* theta_c_shared = &theta_d_shared[size_of_theta_d_bytes / sizeof(float)];
  float* running_cost_shared = &theta_c_shared[size_of_theta_c_bytes / sizeof(float)];
  int* crash_status_shared =
      (int*)&running_cost_shared[math::nearest_multiple_4(num_timesteps * blockDim.y * blockDim.z)];

#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* barrier_shared = (barrier*)&crash_status_shared[math::nearest_multiple_4(sample_dim * distribution_dim)];
#endif

  // Create local state, state dot and controls
  float* x = &x_shared[shared_idx * DYN_T::STATE_DIM];
  float* x_next = &x_next_shared[shared_idx * DYN_T::STATE_DIM];
  float* x_temp;
  float* xdot = &x_dot_shared[shared_idx * DYN_T::STATE_DIM];
  float* u = &u_shared[shared_idx * DYN_T::CONTROL_DIM];
  float* y = &y_shared[shared_idx * DYN_T::OUTPUT_DIM];
  float* running_cost = &running_cost_shared[blockDim.x * (thread_idz * blockDim.y + thread_idy)];
  int* crash_status = &crash_status_shared[shared_idx];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
  int cost_index;
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* bar = &barrier_shared[shared_idx];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  // Load global array to shared array
  loadGlobalToShared<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM>(num_rollouts, blockDim.y, global_idx, thread_idy, thread_idz,
                                                           init_x_d, x, xdot, u);
  __syncthreads();

  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0f, dt);
  sampling->initializeDistributions(y, 0.0f, dt, theta_d_shared);
  costs->initializeCosts(y, u, theta_c_shared, 0.0f, dt);
  __syncthreads();
  for (int t = 0; t < num_timesteps; t++)
  {
    // Load noise trajectories scaled by the exploration factor
    sampling->readVisControlSample(global_idx, t, distribution_idx, u, theta_d_shared, blockDim.y, thread_idy, y);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // applies constraints as defined in dynamics.cuh see specific dynamics class for what happens here
    // usually just control clamping
    dynamics->enforceConstraints(x, u);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // Increment states
    dynamics->step(x, x_next, xdot, u, y, theta_s_shared, t, dt);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    if (t > 0)
    {
      float cost =
          costs->computeRunningCost(y, u, t, theta_c_shared, crash_status) +
          sampling->computeLikelihoodRatioCost(u, theta_d_shared, global_idx, t, distribution_idx, lambda, alpha);
      running_cost[t - 1] = cost / (num_timesteps);
      crash_status_d[global_idx * num_timesteps + t] = crash_status[0];
    }
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    x_temp = x;
    x = x_next;
    x_next = x_temp;
    // Copy state to global memory
    int sample_time_offset = (num_rollouts * thread_idz + global_idx) * num_timesteps + t;
    mp1::loadArrayParallel<DYN_T::OUTPUT_DIM>(y_d, sample_time_offset * DYN_T::OUTPUT_DIM, y, 0);
  }

  // Add all thread_y components of cost together
  running_cost = &running_cost_shared[thread_idx + blockDim.x * blockDim.y * thread_idz];
  __syncthreads();
  costArrayReduction(running_cost, blockDim.y, thread_idy, blockDim.y, thread_idy == blockDim.y - 1, blockDim.x);
  // Compute terminal cost for each thread
  if (threadIdx.x == 0 && threadIdx.y == 0)
  {
    cost_index = (threadIdx.z * num_rollouts + global_idx) * (num_timesteps + 1) + num_timesteps;
    cost_traj_d[cost_index] = costs->terminalCost(y, theta_c_shared) / (num_timesteps);
  }
  __syncthreads();
  // Copy to global memory
  int parallel_index, step;
  mp1::getParallel1DIndex<mp1::Parallel1Dir::THREAD_X>(parallel_index, step);
  if (num_timesteps % 4 == 0)
  {
    float4* cost_traj_d4 =
        reinterpret_cast<float4*>(&cost_traj_d[(thread_idz * num_rollouts + global_idx) * num_timesteps]);
    float4* running_cost_shared4 =
        reinterpret_cast<float4*>(&running_cost_shared[thread_idz * num_timesteps * blockDim.y]);
    for (int i = parallel_index; i < num_timesteps / 4; i += step)
    {
      cost_traj_d4[i] = running_cost_shared4[i];
    }
  }
  else if (num_timesteps % 2 == 0)
  {
    float2* cost_traj_d2 =
        reinterpret_cast<float2*>(&cost_traj_d[(thread_idz * num_rollouts + global_idx) * num_timesteps]);
    float2* running_cost_shared2 =
        reinterpret_cast<float2*>(&running_cost_shared[thread_idz * num_timesteps * blockDim.y]);
    for (int i = parallel_index; i < num_timesteps / 2; i += step)
    {
      cost_traj_d2[i] = running_cost_shared2[i];
    }
  }
  else
  {
    for (int i = parallel_index; i < num_timesteps; i += step)
    {
      cost_traj_d[(thread_idz * num_rollouts + global_idx) * num_timesteps + i] =
          running_cost_shared[thread_idz * num_timesteps * blockDim.y + i];
    }
  }
}

template <class COST_T, class SAMPLING_T, bool COALESCE>
__global__ void visualizeCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                                    const int num_timesteps, const int num_rollouts, const float lambda, float alpha,
                                    const float* __restrict__ y_d, float* __restrict__ cost_traj_d,
                                    int* __restrict__ crash_status_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int global_idx = blockIdx.x;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int distribution_idx = threadIdx.z;

  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];

  float* y_shared = entire_buffer;
  float* u_shared = &y_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::OUTPUT_DIM)];
  float* running_cost_shared = &u_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::CONTROL_DIM)];
  int* crash_status_shared =
      (int*)&running_cost_shared[math::nearest_multiple_4(blockDim.y * blockDim.z * num_timesteps)];
  float* theta_c = (float*)&crash_status_shared[math::nearest_multiple_4(blockDim.x * blockDim.z)];
  float* theta_d = &theta_c[size_of_theta_c_bytes / sizeof(float)];
#ifdef USE_CUDA_BARRIERS_COST
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  barrier* barrier_shared = (barrier*)&theta_d[size_of_theta_d_bytes / sizeof(float)];
#endif
  // Create local state, state dot and controls
  float* y;
  float* u;
  int* crash_status;

  // Initialize running cost and total cost
  float* running_cost;
  int sample_time_offset = 0;
  int cost_index = 0;

  // Load global array to shared array
  y = &y_shared[shared_idx * COST_T::OUTPUT_DIM];
  u = &u_shared[shared_idx * COST_T::CONTROL_DIM];
  crash_status = &crash_status_shared[shared_idx];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
#ifdef USE_CUDA_BARRIERS_COST
  barrier* bar = &barrier_shared[(blockDim.x * thread_idz + thread_idx)];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  /*<----Start of simulation loop-----> */
#ifdef USE_COST_WITH_OFF_NUM_TIMESTEPS
  const int max_time_iters = ceilf((float)(num_timesteps - 2) / blockDim.x);
#else
  const int max_time_iters = ceilf((float)num_timesteps / blockDim.x);
#endif
  costs->initializeCosts(y, u, theta_c, 0.0f, dt);
  sampling->initializeDistributions(y, 0.0f, dt, theta_d);
  __syncthreads();
  for (int time_iter = 0; time_iter < max_time_iters; ++time_iter)
  {
    int t = thread_idx + time_iter * blockDim.x;
    cost_index = (thread_idz * num_rollouts + global_idx) * (num_timesteps) + t - 1;
    running_cost = &running_cost_shared[blockDim.x * (thread_idz * blockDim.y + thread_idy) + t - 1];
    if (COALESCE)
    {  // Fill entire shared mem sequentially using sequential threads_idx
      int amount_to_fill = (time_iter + 1) * blockDim.x > num_timesteps ? num_timesteps % blockDim.x : blockDim.x;
      mp1::loadArrayParallel<mp1::Parallel1Dir::THREAD_XY>(
          y_shared, blockDim.x * thread_idz * COST_T::OUTPUT_DIM, y_d,
          ((num_rollouts * thread_idz + global_idx) * num_timesteps + time_iter * blockDim.x) * COST_T::OUTPUT_DIM,
          COST_T::OUTPUT_DIM * amount_to_fill);
    }
    else if (t < num_timesteps)
    {  // t = num_timesteps is the terminal state for outside this for-loop
      sample_time_offset = (num_rollouts * thread_idz + global_idx) * num_timesteps + t;
      mp1::loadArrayParallel<COST_T::OUTPUT_DIM>(y, 0, y_d, sample_time_offset * COST_T::OUTPUT_DIM);
    }
    if (t < num_timesteps)
    {  // load controls from t = 0 to t = num_timesteps - 1
      sampling->readVisControlSample(global_idx, t, distribution_idx, u, theta_d, blockDim.y, thread_idy, y);
    }
#ifdef USE_CUDA_BARRIERS_COST
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // Compute cost
    if (t < num_timesteps)
    {
      float cost = costs->computeRunningCost(y, u, t, theta_c, crash_status) +
                   sampling->computeLikelihoodRatioCost(u, theta_d, global_idx, t, distribution_idx, lambda, alpha);
      running_cost[0] = cost / (num_timesteps);
      crash_status_d[global_idx * num_timesteps + t] = crash_status[0];
    }
#ifdef USE_CUDA_BARRIERS_COST
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
  }
  // consolidate y threads into single cost
  running_cost = &running_cost_shared[thread_idx + blockDim.x * blockDim.y * thread_idz];
  __syncthreads();
  costArrayReduction(running_cost, blockDim.y, thread_idy, blockDim.y, thread_idy == blockDim.y - 1, blockDim.x);
  // point every thread to the last output at t = NUM_TIMESTEPS for terminal cost calculation
  const int last_y_index = (num_timesteps - 1) % blockDim.x;
  y = &y_shared[(blockDim.x * thread_idz + last_y_index) * COST_T::OUTPUT_DIM];
#ifdef USE_COST_WITH_OFF_NUM_TIMESTEPS
  // load last output array
  const int t = num_timesteps - 1;
  mp1::loadArrayParallel<mp1::Parallel1Dir::THREAD_XY>(
      y_shared, (blockDim.x * thread_idz + last_y_index) * COST_T::OUTPUT_DIM, y_d,
      ((global_idx + num_rollouts * thread_idz) * num_timesteps + t) * COST_T::OUTPUT_DIM, COST_T::OUTPUT_DIM);
  __syncthreads();
#endif
  // Compute terminal cost for each thread
  if (threadIdx.x == 0 && threadIdx.y == 0)
  {
    cost_index = (threadIdx.z * num_rollouts + global_idx) * (num_timesteps + 1) + num_timesteps;
    cost_traj_d[cost_index] = costs->terminalCost(y, theta_c) / (num_timesteps);
  }
  __syncthreads();
  // Copy to global memory
  if (num_timesteps % 4 == 0)
  {
    float4* cost_traj_d4 =
        reinterpret_cast<float4*>(&cost_traj_d[(thread_idz * num_rollouts + global_idx) * num_timesteps]);
    float4* running_cost_shared4 =
        reinterpret_cast<float4*>(&running_cost_shared[thread_idz * num_timesteps * blockDim.y]);
    for (int i = thread_idx; i < num_timesteps / 4; i += blockDim.x)
    {
      cost_traj_d4[i] = running_cost_shared4[i];
    }
  }
  else if (num_timesteps % 2 == 0)
  {
    float2* cost_traj_d2 =
        reinterpret_cast<float2*>(&cost_traj_d[(thread_idz * num_rollouts + global_idx) * num_timesteps]);
    float2* running_cost_shared2 =
        reinterpret_cast<float2*>(&running_cost_shared[thread_idz * num_timesteps * blockDim.y]);
    for (int i = thread_idx; i < num_timesteps / 2; i += blockDim.x)
    {
      cost_traj_d2[i] = running_cost_shared2[i];
    }
  }
  else
  {
    for (int i = thread_idx; i < num_timesteps; i += blockDim.x)
    {
      cost_traj_d[(thread_idz * num_rollouts + global_idx) * num_timesteps + i] =
          running_cost_shared[thread_idz * num_timesteps * blockDim.y + i];
    }
  }
}

__global__ void normExpKernel(int num_rollouts, float* trajectory_costs_d, float lambda_inv, float baseline)
{
  int global_idx = (blockDim.x * blockIdx.x + threadIdx.x) * blockDim.z + threadIdx.z;
  int global_step = blockDim.x * gridDim.x * blockDim.z * gridDim.z;
  // #if defined(CUDA_VERSION) && CUDA_VERSION > 11060
  //   auto block = cg::this_grid();
  //   int global_idx_b = block.thread_rank() + block.block_rank() * block.num_threads();
  //   int global_step_b = block.num_threads() * block.num_blocks();
  //   if (global_idx == 200 && threadIdx.y == 0 && threadIdx.z == 0)
  //   {
  //     printf("Global ind: %d, thread_rank: %d\n", global_idx, global_idx_b);
  //     printf("Global step: %d, thread_rank: %d\n", global_step, global_step_b);
  //   }
  // #endif
  normExpTransform(num_rollouts * blockDim.z, trajectory_costs_d, lambda_inv, baseline, global_idx, global_step);
}

__global__ void TsallisKernel(int num_rollouts, float* trajectory_costs_d, float gamma, float r, float baseline)
{
  int global_idx = (blockDim.x * blockIdx.x + threadIdx.x) * blockDim.z + threadIdx.z;
  int global_step = blockDim.x * gridDim.x * blockDim.z * gridDim.z;
  TsallisTransform(num_rollouts * blockDim.z, trajectory_costs_d, gamma, r, baseline, global_idx, global_step);
}

template <int CONTROL_DIM>
__global__ void weightedReductionKernel(const float* __restrict__ exp_costs_d, const float* __restrict__ du_d,
                                        float* __restrict__ new_u_d, const float normalizer, const int num_timesteps,
                                        const int num_rollouts, const int sum_stride)
{
  int thread_idx = threadIdx.x;  // Rollout index
  int block_idx = blockIdx.x;    // Timestep

  // Create a shared array for intermediate sums: CONTROL_DIM x NUM_THREADS
  extern __shared__ float u_intermediate[];

  float u[CONTROL_DIM];
  setInitialControlToZero(CONTROL_DIM, thread_idx, u, u_intermediate);

  __syncthreads();

  // Sum the weighted control variations at a desired stride
  strideControlWeightReduction(num_rollouts, num_timesteps, sum_stride, thread_idx, block_idx, CONTROL_DIM, exp_costs_d,
                               normalizer, du_d, u, u_intermediate);

  __syncthreads();

  // Sum all weighted control variations
  rolloutWeightReductionAndSaveControl(thread_idx, block_idx, num_rollouts, num_timesteps, CONTROL_DIM, sum_stride, u,
                                       u_intermediate, new_u_d);

  __syncthreads();
}

template <int CONTROL_DIM, int NUM_ROLLOUTS, int SUM_STRIDE>
__global__ void weightedReductionKernel(float* exp_costs_d, float* du_d, float* du_new_d,
                                        float2* baseline_and_normalizer_d, int num_timesteps)
{
  int thread_idx = threadIdx.x;  // Rollout index
  int block_idx = blockIdx.x;    // Timestep

  // Create a shared array for intermediate sums: CONTROL_DIM x NUM_THREADS
  __shared__ float u_intermediate[CONTROL_DIM * ((NUM_ROLLOUTS - 1) / SUM_STRIDE + 1)];

  float u[CONTROL_DIM];
  setInitialControlToZero(CONTROL_DIM, thread_idx, u, u_intermediate);

  __syncthreads();

  // Sum the weighted control variations at a desired stride
  strideControlWeightReduction(NUM_ROLLOUTS, num_timesteps, SUM_STRIDE, thread_idx, block_idx, CONTROL_DIM, exp_costs_d,
                               baseline_and_normalizer_d->y, du_d, u, u_intermediate);

  __syncthreads();

  // Sum all weighted control variations
  rolloutWeightReductionAndSaveControl(thread_idx, block_idx, NUM_ROLLOUTS, num_timesteps, CONTROL_DIM, SUM_STRIDE, u,
                                       u_intermediate, du_new_d);

  __syncthreads();
}

/*******************************************************************************************************************
 * Rollout Kernel Helpers
 *******************************************************************************************************************/
template <int STATE_DIM, int CONTROL_DIM>
__device__ void loadGlobalToShared(const int num_rollouts, const int blocksize_y, const int global_idx,
                                   const int thread_idy, const int thread_idz, const float* __restrict__ x_device,
                                   float* __restrict__ x_thread, float* __restrict__ xdot_thread,
                                   float* __restrict__ u_thread)
{
  // Transfer to shared memory
  int i;
  if (global_idx < num_rollouts)
  {
#if true
    mp1::loadArrayParallel<STATE_DIM>(x_thread, 0, x_device, STATE_DIM * thread_idz);
    if (STATE_DIM % 4 == 0)
    {
      float4* xdot4_t = reinterpret_cast<float4*>(xdot_thread);
      for (i = thread_idy; i < STATE_DIM / 4; i += blocksize_y)
      {
        xdot4_t[i] = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      }
    }
    else if (STATE_DIM % 2 == 0)
    {
      float2* xdot2_t = reinterpret_cast<float2*>(xdot_thread);
      for (i = thread_idy; i < STATE_DIM / 2; i += blocksize_y)
      {
        xdot2_t[i] = make_float2(0.0f, 0.0f);
      }
    }
    else
    {
      for (i = thread_idy; i < STATE_DIM; i += blocksize_y)
      {
        xdot_thread[i] = 0.0f;
      }
    }

    if (CONTROL_DIM % 4 == 0)
    {
      float4* u4_t = reinterpret_cast<float4*>(u_thread);
      for (i = thread_idy; i < CONTROL_DIM / 4; i += blocksize_y)
      {
        u4_t[i] = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
      }
    }
    else if (CONTROL_DIM % 2 == 0)
    {
      float2* u2_t = reinterpret_cast<float2*>(u_thread);
      for (i = thread_idy; i < CONTROL_DIM / 2; i += blocksize_y)
      {
        u2_t[i] = make_float2(0.0f, 0.0f);
      }
    }
    else
    {
      for (i = thread_idy; i < CONTROL_DIM; i += blocksize_y)
      {
        u_thread[i] = 0.0f;
      }
    }
#else
    for (i = thread_idy; i < STATE_DIM; i += blocksize_y)
    {
      x_thread[i] = x_device[i + STATE_DIM * thread_idz];
      xdot_thread[i] = 0.0f;
    }
    for (i = thread_idy; i < CONTROL_DIM; i += blocksize_y)
    {
      u_thread[i] = 0.0f;
    }
#endif
  }
}

template <class COST_T>
__device__ void computeAndSaveCost(int num_rollouts, int num_timesteps, int global_idx, COST_T* costs, float* output,
                                   float running_cost, float* theta_c, float* cost_rollouts_device)
{
  // only want to save 1 cost per trajectory
  if (threadIdx.y == 0 && global_idx < num_rollouts)
  {
    cost_rollouts_device[global_idx + num_rollouts * threadIdx.z] =
        running_cost + costs->terminalCost(output, theta_c) / (num_timesteps);
  }
}

/*******************************************************************************************************************
 * NormExp Kernel Helpers
 *******************************************************************************************************************/
float computeBaselineCost(float* cost_rollouts_host, int num_rollouts)
{  // TODO if we use standard containers in MPPI, should this be replaced with a min algorithm?
  int best_idx = computeBestIndex(cost_rollouts_host, num_rollouts);
  return cost_rollouts_host[best_idx];
}

float constructBestWeights(float* cost_rollouts_host, int num_rollouts)
{
  int best_idx = computeBestIndex(cost_rollouts_host, num_rollouts);
  float best_cost = cost_rollouts_host[best_idx];

  for (int i = 0; i < num_rollouts; i++)
  {
    if (i == best_idx)
    {
      cost_rollouts_host[i] = 1.0;
    }
    else
    {
      cost_rollouts_host[i] = 0.0;
    }
  }

  // printf("Best idx: %d, cost: %f\n", best_cost_idx, best_cost);
  return best_cost;
}

int computeBestIndex(float* cost_rollouts_host, int num_rollouts)
{
  float best_cost = cost_rollouts_host[0];
  int best_cost_idx = 0;
  for (int i = 1; i < num_rollouts; i++)
  {
    if (cost_rollouts_host[i] < best_cost)
    {
      best_cost = cost_rollouts_host[i];
      best_cost_idx = i;
    }
  }

  // printf("Best idx: %d, cost: %f\n", best_cost_idx, best_cost);
  return best_cost_idx;
}

__device__ inline float computeBaselineCost(int num_rollouts, const float* __restrict__ trajectory_costs_d,
                                            float* __restrict__ reduction_buffer, int rollout_idx_global,
                                            int rollout_idx_step)
{
  // Copy costs to shared memory
  float min_cost = 0.0;
#if false
  // potential method to speed up copying costs
  int prev_size = min(blockDim.x, num_rollouts);
  float my_val = (rollout_idx_global < num_rollouts) ? trajectory_costs_d[rollout_idx_global] : INFINITY;
  for (int i = rollout_idx_global + rollout_idx_step; i < num_rollouts; i += rollout_idx_step)
  {
    my_val = min(trajectory_costs_d[i], my_val);
  }
  reduction_buffer[rollout_idx_global] = my_val;
  // __syncthreads();
  // if (threadIdx.x == 0)
  // {
  //   for (int i = 0; i < min(blockDim.x, num_rollouts); i++)
  //   {
  //     printf("buff %d: %f\n", i, reduction_buffer[i]);
  //   }
  //   printf("Num rollouts; %d\n", num_rollouts);
  // }
#else
  int prev_size = num_rollouts / 2;
  for (int i = rollout_idx_global; i < prev_size; i += rollout_idx_step)
  {
    reduction_buffer[i] = min(trajectory_costs_d[i], trajectory_costs_d[i + prev_size]);
  }
  if (num_rollouts - 2 * prev_size == 1 && threadIdx.x == blockDim.x - 1)
  {
    reduction_buffer[prev_size - 1] = min(reduction_buffer[num_rollouts - 1], reduction_buffer[prev_size - 1]);
  }
#endif

  __syncthreads();
  // find min along the entire array
  for (int size = prev_size / 2; size > 0; size /= 2)
  {
    for (int i = rollout_idx_global; i < size; i += rollout_idx_step)
    {
      reduction_buffer[i] = min(reduction_buffer[i], reduction_buffer[i + size]);
    }
    __syncthreads();
    if (prev_size - 2 * size == 1 && threadIdx.x == blockDim.x - 1)
    {
      reduction_buffer[size - 1] = min(reduction_buffer[size - 1], reduction_buffer[prev_size - 1]);
    }
    __syncthreads();
    prev_size = size;
  }
  min_cost = reduction_buffer[0];
  return min_cost;
}

__device__ __host__ inline void normExpTransform(int num_rollouts, float* __restrict__ trajectory_costs_d,
                                                 float lambda_inv, float baseline, int global_idx, int rollout_idx_step)
{
  for (int i = global_idx; i < num_rollouts; i += rollout_idx_step)
  {
    float cost_dif = trajectory_costs_d[i] - baseline;
    trajectory_costs_d[i] = expf(-lambda_inv * cost_dif);
  }
}

__device__ __host__ inline void TsallisTransform(int num_rollouts, float* __restrict__ trajectory_costs_d, float gamma,
                                                 float r, float baseline, int global_idx, int rollout_idx_step)
{
  for (int i = global_idx; i < num_rollouts; i += rollout_idx_step)
  {
    float cost_dif = trajectory_costs_d[i] - baseline;
    // trajectory_costs_d[i] = mppi::math::expr(-lambda_bar_inv * cost_dif);
    // trajectory_costs_d[i] = (cost_dif < gamma) * expf(logf(1.0 - cost_dif / gamma) / (r - 1));
    if (cost_dif < gamma)
    {
      trajectory_costs_d[i] = expf(logf(1.0 - cost_dif / gamma) / (r - 1));
    }
    else
    {
      trajectory_costs_d[i] = 0;
    }
  }
}

__device__ inline float computeNormalizer(int num_rollouts, const float* __restrict__ trajectory_costs_d,
                                          float* __restrict__ reduction_buffer, int rollout_idx_global,
                                          int rollout_idx_step)
{
  // Copy costs to shared memory
#if false
  // potential method to speed up copying costs
  int prev_size = min(blockDim.x, num_rollouts);
  float my_val = (rollout_idx_global < num_rollouts) ? trajectory_costs_d[rollout_idx_global] : 0;
  for (int i = rollout_idx_global + rollout_idx_step; i < num_rollouts; i += rollout_idx_step)
  {
    my_val += trajectory_costs_d[i];
  }
  reduction_buffer[rollout_idx_global] = my_val;
#else
  int prev_size = num_rollouts / 2;
  for (int i = rollout_idx_global; i < prev_size; i += rollout_idx_step)
  {
    reduction_buffer[i] = trajectory_costs_d[i] + trajectory_costs_d[i + prev_size];
  }
  if (num_rollouts - 2 * prev_size == 1 && threadIdx.x == blockDim.x - 1)
  {
    reduction_buffer[prev_size - 1] += reduction_buffer[num_rollouts - 1];
  }
#endif
  __syncthreads();
  // sum the entire array
  for (int size = prev_size / 2; size > 0; size /= 2)
  {
    for (int i = rollout_idx_global; i < size; i += rollout_idx_step)
    {
      reduction_buffer[i] += reduction_buffer[i + size];
    }
    __syncthreads();
    if (prev_size - 2 * size == 1 && threadIdx.x == blockDim.x - 1)
    {
      reduction_buffer[size - 1] += reduction_buffer[prev_size - 1];
    }
    __syncthreads();
    prev_size = size;
  }
  return reduction_buffer[0];
}

template <int NUM_ROLLOUTS, int BLOCKSIZE_X = 1024>
__global__ void fullGPUcomputeWeights(float* __restrict__ trajectory_costs_d, float lambda_inv,
                                      float2* __restrict__ output)
{
  __shared__ float reduction_buffer[NUM_ROLLOUTS];
  // int global_idx = blockIdx.x * blockDim.x + threadIdx.x;
  // int better_global_idx = (blockIdx.x * blockDim.x + threadIdx.x) * blockDim.y + threadIdx.y;
  // int global_idx = (blockDim.x * blockIdx.x + threadIdx.x) * blockDim.z + threadIdx.z;
  // int global_step = blockDim.x * gridDim.x;
  // int better_global_step = blockDim.x * gridDim.x  * blockDim.y * gridDim.y;
  int global_idx = threadIdx.x;
  int global_step = blockDim.x;

  float baseline = computeBaselineCost(NUM_ROLLOUTS, trajectory_costs_d, reduction_buffer, global_idx, global_step);
  normExpTransform(NUM_ROLLOUTS, trajectory_costs_d, lambda_inv, baseline, global_idx, global_step);
  __syncthreads();
  float normalizer = computeNormalizer(NUM_ROLLOUTS, trajectory_costs_d, reduction_buffer, global_idx, global_step);
  __syncthreads();
  if (threadIdx.x == 0)
  {
    *output = make_float2(baseline, normalizer);
  }
}

float computeNormalizer(float* cost_rollouts_host, int num_rollouts)
{
  double normalizer = 0.0;
  for (int i = 0; i < num_rollouts; ++i)
  {
    normalizer += cost_rollouts_host[i];
  }
  return normalizer;
}

void computeFreeEnergy(float& free_energy, float& free_energy_var, float& free_energy_modified,
                       float* cost_rollouts_host, int num_rollouts, float baseline, float lambda)
{
  float var = 0;
  float norm = 0;
  for (int i = 0; i < num_rollouts; i++)
  {
    norm += cost_rollouts_host[i];
    var += SQ(cost_rollouts_host[i]);
  }
  norm /= num_rollouts;
  free_energy = -lambda * logf(norm) + baseline;
  free_energy_var = lambda * (var / num_rollouts - SQ(norm));
  // TODO Figure out the point of the following lines
  float weird_term = free_energy_var / (norm * sqrtf(1.0 * num_rollouts));
  free_energy_modified = lambda * (weird_term + 0.5 * SQ(weird_term));
}

/*******************************************************************************************************************
 * Weighted Reduction Kernel Helpers
 *******************************************************************************************************************/
__device__ void setInitialControlToZero(int control_dim, int thread_idx, float* __restrict__ u,
                                        float* __restrict__ u_intermediate)
{
  if (control_dim % 4 == 0)
  {
    for (int i = 0; i < control_dim / 4; i++)
    {
      reinterpret_cast<float4*>(u)[i] = make_float4(0, 0, 0, 0);
      reinterpret_cast<float4*>(&u_intermediate[thread_idx * control_dim])[i] = make_float4(0, 0, 0, 0);
    }
  }
  else if (control_dim % 2 == 0)
  {
    for (int i = 0; i < control_dim / 2; i++)
    {
      reinterpret_cast<float2*>(u)[i] = make_float2(0, 0);
      reinterpret_cast<float2*>(&u_intermediate[thread_idx * control_dim])[i] = make_float2(0, 0);
    }
  }
  else
  {
    for (int i = 0; i < control_dim; i++)
    {
      u[i] = 0;
      u_intermediate[thread_idx * control_dim + i] = 0;
    }
  }
}

__device__ void strideControlWeightReduction(const int num_rollouts, const int num_timesteps, const int sum_stride,
                                             const int thread_idx, const int block_idx, const int control_dim,
                                             const float* __restrict__ exp_costs_d, const float normalizer,
                                             const float* __restrict__ du_d, float* __restrict__ u,
                                             float* __restrict__ u_intermediate)
{
  // int index = thread_idx * sum_stride + i;
  for (int i = 0; i < sum_stride; ++i)
  {  // Iterate through the size of the subsection
    if ((thread_idx * sum_stride + i) < num_rollouts)
    {                                                                        // Ensure we do not go out of bounds
      float weight = exp_costs_d[thread_idx * sum_stride + i] / normalizer;  // compute the importance sampling weight
      for (int j = 0; j < control_dim; ++j)
      {  // Iterate through the control dimensions
        // Rollout index: (thread_idx*sum_stride + i)*(num_timesteps*control_dim)
        // Current timestep: block_idx*control_dim
        u[j] = du_d[(thread_idx * sum_stride + i) * (num_timesteps * control_dim) + block_idx * control_dim + j];
        u_intermediate[thread_idx * control_dim + j] += weight * u[j];
      }
    }
  }
}

__device__ void rolloutWeightReductionAndSaveControl(int thread_idx, int block_idx, int num_rollouts, int num_timesteps,
                                                     int control_dim, int sum_stride, float* u, float* u_intermediate,
                                                     float* du_new_d)
{
  if (thread_idx == 0 && block_idx < num_timesteps)
  {  // block index refers to the current timestep
    for (int i = 0; i < control_dim; ++i)
    {  // TODO replace with memset?
      u[i] = 0;
    }
    for (int i = 0; i < ((num_rollouts - 1) / sum_stride + 1); ++i)
    {  // iterate through the each subsection
      for (int j = 0; j < control_dim; ++j)
      {
        u[j] += u_intermediate[i * control_dim + j];
      }
    }
    for (int i = 0; i < control_dim; i++)
    {
      du_new_d[block_idx * control_dim + i] = u[i];
    }
  }
}

template <int BLOCKSIZE>
__device__ void warpReduceAdd(volatile float* sdata, const int tid, const int stride)
{
  if (BLOCKSIZE >= 64)
  {
    sdata[tid * stride] += sdata[(tid + 32) * stride];
  }
  if (BLOCKSIZE >= 32)
  {
    sdata[tid * stride] += sdata[(tid + 16) * stride];
  }
  if (BLOCKSIZE >= 16)
  {
    sdata[tid * stride] += sdata[(tid + 8) * stride];
  }
  if (BLOCKSIZE >= 8)
  {
    sdata[tid * stride] += sdata[(tid + 4) * stride];
  }
  if (BLOCKSIZE >= 4)
  {
    sdata[tid * stride] += sdata[(tid + 2) * stride];
  }
  if (BLOCKSIZE >= 2)
  {
    sdata[tid * stride] += sdata[(tid + 1) * stride];
  }
}

__device__ void costArrayReduction(float* running_cost, const int start_size, const int index, const int step,
                                   const bool catch_condition, const int stride)
{
  int prev_size = start_size;
  const bool block_power_of_2 = (prev_size & (prev_size - 1)) == 0;
  const int stop_condition = (block_power_of_2) ? 32 : 0;
  int size;
  int j;

  for (size = prev_size / 2; size > stop_condition; size /= 2)
  {
    for (j = index; j < size; j += step)
    {
      running_cost[j * stride] += running_cost[(j + size) * stride];
    }
    __syncthreads();
    if (prev_size - 2 * size == 1 && catch_condition)
    {
      running_cost[(size - 1) * stride] += running_cost[(prev_size - 1) * stride];
    }
    __syncthreads();
    prev_size = size;
  }
  switch (size * 2)
  {
    case 64:
      if (index < 32)
      {
        warpReduceAdd<64>(running_cost, index, stride);
      }
      break;
    case 32:
      if (index < 16)
      {
        warpReduceAdd<32>(running_cost, index, stride);
      }
      break;
    case 16:
      if (index < 8)
      {
        warpReduceAdd<16>(running_cost, index, stride);
      }
      break;
    case 8:
      if (index < 4)
      {
        warpReduceAdd<8>(running_cost, index, stride);
      }
      break;
    case 4:
      if (index < 2)
      {
        warpReduceAdd<4>(running_cost, index, stride);
      }
      break;
    case 2:
      if (index < 1)
      {
        warpReduceAdd<2>(running_cost, index, stride);
      }
      break;
  }
  __syncthreads();
}

/*******************************************************************************************************************
 * Launch Functions
 *******************************************************************************************************************/
template <class DYN_T, class COST_T, typename SAMPLING_T, bool COALESCE>
void launchSplitRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                              SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                              const int num_rollouts, float lambda, float alpha, float* __restrict__ init_x_d,
                              float* __restrict__ y_d, float* __restrict__ trajectory_costs, dim3 dimDynBlock,
                              dim3 dimCostBlock, cudaStream_t stream, bool synchronize)
{
  if (num_rollouts % dimDynBlock.x != 0)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_rollouts (" << num_rollouts
              << ") must be evenly divided by dynamics block size x (" << dimDynBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }
  if (num_timesteps < dimCostBlock.x)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_timesteps (" << num_timesteps
              << ") must be greater than or equal to cost block size x (" << dimCostBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }
  // Run Dynamics
  const int gridsize_x = math::int_ceil(num_rollouts, dimDynBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned dynamics_shared_size = calcRolloutDynamicsKernelSharedMemSize(dynamics, sampling, dimDynBlock);
  HANDLE_ERROR(cudaFuncSetAttribute(rolloutDynamicsKernel<DYN_T, SAMPLING_T>,
                                    cudaFuncAttributeMaxDynamicSharedMemorySize, dynamics_shared_size));
  rolloutDynamicsKernel<DYN_T, SAMPLING_T><<<dimGrid, dimDynBlock, dynamics_shared_size, stream>>>(
      dynamics->model_d_, sampling->sampling_d_, dt, num_timesteps, num_rollouts, init_x_d, y_d);
  HANDLE_ERROR(cudaGetLastError());
  // Run Costs
  dim3 dimCostGrid(num_rollouts, 1, 1);
  unsigned cost_shared_size = calcRolloutCostKernelSharedMemSize(costs, sampling, dimCostBlock);
  rolloutCostKernel<COST_T, SAMPLING_T, COALESCE><<<dimCostGrid, dimCostBlock, cost_shared_size, stream>>>(
      costs->cost_d_, sampling->sampling_d_, dt, num_timesteps, num_rollouts, lambda, alpha, y_d, trajectory_costs);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <class DYN_T, class COST_T, typename SAMPLING_T>
void launchRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling,
                         float dt, const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                         float* __restrict__ init_x_d, float* __restrict__ trajectory_costs, dim3 dimBlock,
                         cudaStream_t stream, bool synchronize)
{
  if (num_rollouts % dimBlock.x != 0)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_rollouts (" << num_rollouts
              << ") must be evenly divided by rollout thread block size x (" << dimBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }

  const int gridsize_x = math::int_ceil(num_rollouts, dimBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned shared_mem_size = calcRolloutCombinedKernelSharedMemSize(dynamics, costs, sampling, dimBlock);
  HANDLE_ERROR(cudaFuncSetAttribute(rolloutKernel<DYN_T, COST_T, SAMPLING_T>,
                                    cudaFuncAttributeMaxDynamicSharedMemorySize, shared_mem_size));
  rolloutKernel<DYN_T, COST_T, SAMPLING_T><<<dimGrid, dimBlock, shared_mem_size, stream>>>(
      dynamics->model_d_, sampling->sampling_d_, costs->cost_d_, dt, num_timesteps, num_rollouts, init_x_d, lambda,
      alpha, trajectory_costs);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <class DYN_T, class COST_T, typename SAMPLING_T>
void launchVisualizeKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling,
                           float dt, const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                           float* __restrict__ init_x_d, float* __restrict__ y_d, float* __restrict__ trajectory_costs,
                           int* __restrict__ crash_status_d, dim3 dimVisBlock, cudaStream_t stream, bool synchronize)
{
  if (num_rollouts <= 1)
  {  // Not enough samples to visualize
    std::cerr << "Not enough samples to visualize" << std::endl;
    return;
  }
  if (num_rollouts % dimVisBlock.x != 0)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_rollouts (" << num_rollouts
              << ") must be evenly divided by vis block size x (" << dimVisBlock.x << ")" << std::endl;
    return;
  }

  const int gridsize_x = math::int_ceil(num_rollouts, dimVisBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned shared_mem_size = calcVisualizeKernelSharedMemSize(dynamics, costs, sampling, num_timesteps, dimVisBlock);
  HANDLE_ERROR(cudaFuncSetAttribute(rolloutKernel<DYN_T, COST_T, SAMPLING_T>,
                                    cudaFuncAttributeMaxDynamicSharedMemorySize, shared_mem_size));
  visualizeKernel<DYN_T, COST_T, SAMPLING_T><<<dimGrid, dimVisBlock, shared_mem_size, stream>>>(
      dynamics->model_d_, sampling->sampling_d_, costs->cost_d_, dt, num_timesteps, num_rollouts, init_x_d, lambda,
      alpha, y_d, trajectory_costs, crash_status_d);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <class COST_T, class SAMPLING_T, bool COALESCE>
void launchVisualizeCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                               const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                               float* __restrict__ y_d, int* __restrict__ sampled_crash_status_d,
                               float* __restrict__ cost_traj_result, dim3 dimBlock, cudaStream_t stream,
                               bool synchronize)
{
  if (num_rollouts <= 1)
  {  // Not enough samples to visualize
    std::cerr << "Not enough samples to visualize" << std::endl;
    return;
  }

  dim3 dimCostGrid(num_rollouts, 1, 1);
  unsigned shared_mem_size = calcVisCostKernelSharedMemSize(costs, sampling, num_timesteps, dimBlock);
  visualizeCostKernel<COST_T, SAMPLING_T, COALESCE><<<dimCostGrid, dimBlock, shared_mem_size, stream>>>(
      costs->cost_d_, sampling->sampling_d_, dt, num_timesteps, num_rollouts, lambda, alpha, y_d, cost_traj_result,
      sampled_crash_status_d);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <int CONTROL_DIM>
void launchWeightedReductionKernel(const float* __restrict__ exp_costs_d, const float* __restrict__ du_d,
                                   float* __restrict__ new_u_d, const float normalizer, const int num_timesteps,
                                   const int num_rollouts, const int sum_stride, cudaStream_t stream, bool synchronize)
{
  dim3 dimBlock(math::int_ceil(num_rollouts, sum_stride), 1, 1);
  dim3 dimGrid(num_timesteps, 1, 1);
  unsigned shared_mem_size = math::nearest_multiple_4(CONTROL_DIM * dimBlock.x) * sizeof(float);
  weightedReductionKernel<CONTROL_DIM><<<dimGrid, dimBlock, shared_mem_size, stream>>>(
      exp_costs_d, du_d, new_u_d, normalizer, num_timesteps, num_rollouts, sum_stride);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

void launchNormExpKernel(int num_rollouts, int blocksize_x, float* trajectory_costs_d, float lambda_inv, float baseline,
                         cudaStream_t stream, bool synchronize)
{
  dim3 dimBlock(blocksize_x, 1, 1);
  dim3 dimGrid((num_rollouts - 1) / blocksize_x + 1, 1, 1);
  normExpKernel<<<dimGrid, dimBlock, 0, stream>>>(num_rollouts, trajectory_costs_d, lambda_inv, baseline);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

void launchTsallisKernel(int num_rollouts, int blocksize_x, float* trajectory_costs_d, float gamma, float r,
                         float baseline, cudaStream_t stream, bool synchronize)
{
  dim3 dimBlock(blocksize_x, 1, 1);
  dim3 dimGrid((num_rollouts - 1) / blocksize_x + 1, 1, 1);
  TsallisKernel<<<dimGrid, dimBlock, 0, stream>>>(num_rollouts, trajectory_costs_d, gamma, r, baseline);
  // CudaCheckError();
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <int NUM_ROLLOUTS>
void launchWeightTransformKernel(float* __restrict__ costs_d, float2* __restrict__ baseline_and_norm_d,
                                 const float lambda_inv, const int num_systems, cudaStream_t stream, bool synchronize)
{
  // Figure out max size of threads from the device properties (slows down this method a lot)
  // int device_id = 0;
  // cudaDeviceProp deviceProp;
  // cudaGetDeviceProperties(&deviceProp, device_id);
  // int blocksize_x = deviceProp.maxThreadsDim[0];
  const int blocksize_x = 1024;
  dim3 dimBlock(blocksize_x, 1, 1);
  // Can't be split into multiple blocks because we want to do all the math in shared memory
  dim3 dimGrid(1, 1, 1);
  for (int i = 0; i < num_systems; i++)
  {
    fullGPUcomputeWeights<NUM_ROLLOUTS>
        <<<dimGrid, dimBlock, 0, stream>>>(costs_d + i * NUM_ROLLOUTS, lambda_inv, baseline_and_norm_d + i);
    HANDLE_ERROR(cudaGetLastError());
  }
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <class DYN_T, int NUM_ROLLOUTS, int SUM_STRIDE>
void launchweightedReductionKernel(float* exp_costs_d, float* du_d, float* du_new_d, float2* baseline_and_normalizer_d,
                                   int num_timesteps, cudaStream_t stream, bool synchronize)
{
  dim3 dimBlock((NUM_ROLLOUTS - 1) / SUM_STRIDE + 1, 1, 1);
  dim3 dimGrid(num_timesteps, 1, 1);
  weightedReductionKernel<DYN_T::CONTROL_DIM, NUM_ROLLOUTS, SUM_STRIDE>
      <<<dimGrid, dimBlock, 0, stream>>>(exp_costs_d, du_d, du_new_d, baseline_and_normalizer_d, num_timesteps);
  // CudaCheckError();
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

/*******************************************************************************************************************
 * Shared Memory Calculation Functions
 *******************************************************************************************************************/
template <class DYN_T, class SAMPLER_T>
unsigned calcRolloutDynamicsKernelSharedMemSize(const DYN_T* dynamics, const SAMPLER_T* sampler, dim3& dimBlock)
{
  const int dynamics_num_shared = dimBlock.x * dimBlock.z;
  unsigned dynamics_shared_size =
      sizeof(float) * (3 * math::nearest_multiple_4(dynamics_num_shared * DYN_T::STATE_DIM) +
                       math::nearest_multiple_4(dynamics_num_shared * DYN_T::OUTPUT_DIM) +
                       math::nearest_multiple_4(dynamics_num_shared * DYN_T::CONTROL_DIM)) +
      calcClassSharedMemSize<DYN_T>(dynamics, dimBlock) + calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock);
#ifdef USE_CUDA_BARRIERS_DYN
  dynamics_shared_size += math::int_multiple_const(dynamics_num_shared * sizeof(barrier), 16);
#endif
  return dynamics_shared_size;
}

template <class COST_T, class SAMPLER_T>
unsigned calcRolloutCostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, dim3& dimBlock)
{
  const int cost_num_shared = dimBlock.x * dimBlock.z;
  unsigned cost_shared_size = sizeof(float) * (math::nearest_multiple_4(cost_num_shared * COST_T::OUTPUT_DIM) +
                                               math::nearest_multiple_4(cost_num_shared * COST_T::CONTROL_DIM) +
                                               math::nearest_multiple_4(cost_num_shared * dimBlock.y)) +
                              sizeof(int) * math::nearest_multiple_4(cost_num_shared) +
                              calcClassSharedMemSize(cost, dimBlock) +
                              calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock);
#ifdef USE_CUDA_BARRIERS_COST
  cost_shared_size += math::int_multiple_const(cost_num_shared * sizeof(barrier), 16);
#endif
  return cost_shared_size;
}

template <class DYN_T, class COST_T, class SAMPLER_T>
unsigned calcRolloutCombinedKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                                dim3& dimBlock)
{
  const int num_shared = dimBlock.x * dimBlock.z;
  unsigned shared_mem_size = sizeof(float) * (3 * math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM) +
                                              math::nearest_multiple_4(num_shared * DYN_T::OUTPUT_DIM) +
                                              math::nearest_multiple_4(num_shared * DYN_T::CONTROL_DIM) +
                                              math::nearest_multiple_4(num_shared * dimBlock.y)) +
                             sizeof(int) * math::nearest_multiple_4(num_shared) +
                             calcClassSharedMemSize(dynamics, dimBlock) + calcClassSharedMemSize(cost, dimBlock) +
                             calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  shared_mem_size += math::int_multiple_const(num_shared * sizeof(barrier), 16);
#endif
  return shared_mem_size;
}

template <class DYN_T, class COST_T, class SAMPLER_T>
unsigned calcVisualizeKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                          const int& num_timesteps, dim3& dimBlock)
{
  const int num_shared = dimBlock.x * dimBlock.z;
  unsigned shared_mem_size = sizeof(float) * (3 * math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM) +
                                              math::nearest_multiple_4(num_shared * DYN_T::OUTPUT_DIM) +
                                              math::nearest_multiple_4(num_shared * DYN_T::CONTROL_DIM) +
                                              math::nearest_multiple_4(num_timesteps * dimBlock.y * dimBlock.z)) +
                             sizeof(int) * math::nearest_multiple_4(num_shared) +
                             calcClassSharedMemSize(dynamics, dimBlock) + calcClassSharedMemSize(cost, dimBlock) +
                             calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  shared_mem_size += math::int_multiple_const(num_shared * sizeof(barrier), 16);
#endif
  return shared_mem_size;
}

template <class COST_T, class SAMPLER_T>
unsigned calcVisCostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, const int& num_timesteps,
                                        dim3& dimBlock)
{
  const int shared_num = dimBlock.x * dimBlock.z;
  unsigned shared_mem_size = sizeof(float) * (math::nearest_multiple_4(shared_num * COST_T::OUTPUT_DIM) +
                                              math::nearest_multiple_4(shared_num * COST_T::CONTROL_DIM) +
                                              math::nearest_multiple_4(dimBlock.z * num_timesteps * dimBlock.y)) +
                             sizeof(int) * math::nearest_multiple_4(dimBlock.z * num_timesteps) +
                             calcClassSharedMemSize(cost, dimBlock) +
                             calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock);
#ifdef USE_CUDA_BARRIERS_COST
  shared_mem_size += math::int_multiple_const(shared_num * sizeof(barrier), 16);
#endif
  return shared_mem_size;
}

template <class T>
__host__ __device__ unsigned calcClassSharedMemSize(const T* class_ptr, const dim3& dimBlock)
{
  const int num_shared = dimBlock.x * dimBlock.z;
  unsigned shared_size = math::int_multiple_const(class_ptr->getGrdSharedSizeBytes(), sizeof(float4)) +
                         num_shared * math::int_multiple_const(class_ptr->getBlkSharedSizeBytes(), sizeof(float4));
  return shared_size;
}
}  // namespace kernels
}  // namespace mppi
