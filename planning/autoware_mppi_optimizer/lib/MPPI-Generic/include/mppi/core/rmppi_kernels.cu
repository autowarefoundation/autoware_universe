#include <mppi/core/mppi_common.cuh>

namespace mp1 = mppi::p1;

namespace mppi
{
namespace kernels
{
namespace rmppi
{
template <class DYN_T, class SAMPLING_T>
__global__ void initEvalDynKernel(DYN_T* __restrict__ dynamics, SAMPLING_T* __restrict__ sampling, float dt,
                                  const int num_timesteps, const int num_rollouts, int samples_per_condition,
                                  const int* __restrict__ strides_d, const float* __restrict__ states_d,
                                  float* __restrict__ y_d)
{
  // Get thread and block id
  const int global_idx = blockDim.x * blockIdx.x + threadIdx.x;
  const int distribution_idx = threadIdx.z;
  const int candidate_idx = global_idx / samples_per_condition;
  const int candidate_sample_idx = global_idx % samples_per_condition;
  const int tdy = threadIdx.y;
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);

  const int shared_idx = blockDim.x * threadIdx.z + threadIdx.x;
  const int distribution_dim = blockDim.z;
  const int sample_dim = blockDim.x;

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
  float* x = &(reinterpret_cast<float*>(x_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_next = &(reinterpret_cast<float*>(x_next_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_temp;
  float* xdot = &(reinterpret_cast<float*>(x_dot_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* u = &(reinterpret_cast<float*>(u_shared)[shared_idx * DYN_T::CONTROL_DIM]);
  float* y = &(reinterpret_cast<float*>(y_shared)[shared_idx * DYN_T::OUTPUT_DIM]);

  // Load global array to shared array
  mppi::p1::loadArrayParallel<DYN_T::STATE_DIM>(x, 0, states_d, candidate_idx * DYN_T::STATE_DIM);
  int stride = strides_d[candidate_idx];
  for (int i = tdy; i < DYN_T::CONTROL_DIM; i += blockDim.y)
  {
    u[i] = 0;
  }
  for (int i = tdy; i < DYN_T::OUTPUT_DIM; i += blockDim.y)
  {
    y[i] = 0;
  }
  __syncthreads();

#ifdef USE_CUDA_BARRIERS_DYN
  barrier* bar = &barrier_shared[(shared_idx)];
  if (tdy == 0)
  {
    init(bar, blockDim.y);
  }
#endif
  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0, dt);
  sampling->initializeDistributions(y, 0.0, dt, theta_d_shared);
  __syncthreads();
  for (int t = 0; t < num_timesteps; t++)
  {
    // Load noise trajectories scaled by the exploration factor
    int candidate_t = min(t + stride, num_timesteps - 1);
    sampling->readControlSample(candidate_sample_idx, candidate_t, distribution_idx, u, theta_d_shared, blockDim.y, tdy,
                                y);
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
    int sample_time_offset = (num_rollouts * threadIdx.z + global_idx) * num_timesteps + t;
    mp1::loadArrayParallel<DYN_T::OUTPUT_DIM>(y_d, sample_time_offset * DYN_T::OUTPUT_DIM, y, 0);
  }
}

template <class COST_T, class SAMPLING_T, bool COALESCE>
__global__ void initEvalCostKernel(COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling, float dt,
                                   const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                   const int samples_per_condition, const int* __restrict__ strides_d,
                                   const float* __restrict__ y_d, float* __restrict__ trajectory_costs_d)
{
  // Get thread and block id
  const int global_idx = blockIdx.x;
  const int distribution_idx = threadIdx.z;
  const int candidate_idx = global_idx / samples_per_condition;
  const int candidate_sample_idx = global_idx % samples_per_condition;

  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int shared_idx = blockDim.x * threadIdx.z + threadIdx.x;
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];
  float* y_shared = entire_buffer;
  float* u_shared = &y_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::OUTPUT_DIM)];
  float* running_cost_shared = &u_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * COST_T::CONTROL_DIM)];
  int* crash_status_shared = (int*)&running_cost_shared[math::nearest_multiple_4(blockDim.x * blockDim.z * blockDim.y)];
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
  int stride = strides_d[candidate_idx];

  // Load global array to shared array
  y = &y_shared[(shared_idx)*COST_T::OUTPUT_DIM];
  u = &u_shared[(shared_idx)*COST_T::CONTROL_DIM];
  crash_status = &crash_status_shared[shared_idx];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
  const int running_cost_index = thread_idx + blockDim.x * (thread_idy + blockDim.y * thread_idz);
  running_cost = &running_cost_shared[running_cost_index];
  running_cost[0] = 0;
#ifdef USE_CUDA_BARRIERS_COST
  barrier* bar = &barrier_shared[(blockDim.x * thread_idz + thread_idx)];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  /*<----Start of simulation loop-----> */
  const int max_time_iters = ceilf((float)num_timesteps / blockDim.x);
  costs->initializeCosts(y, u, theta_c, 0.0, dt);
  sampling->initializeDistributions(y, 0.0, dt, theta_d);
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
      int candidate_t = min(t + stride, num_timesteps - 1);
      sampling->readControlSample(candidate_sample_idx, candidate_t, distribution_idx, u, theta_d, blockDim.y,
                                  thread_idy, y);
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
  // Compute terminal cost and the final cost for each thread
  computeAndSaveCost(num_rollouts, num_timesteps, global_idx, costs, y, running_cost[0] / (num_timesteps), theta_c,
                     trajectory_costs_d);
}

template <class DYN_T, class COST_T, class SAMPLING_T>
__global__ void initEvalKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                               SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                               const int num_rollouts, int samples_per_condition, float lambda, float alpha,
                               const int* __restrict__ strides_d, const float* __restrict__ states_d,
                               float* __restrict__ trajectory_costs_d)
{
  // Get thread and block id
  const int global_idx = blockDim.x * blockIdx.x + threadIdx.x;
  const int distribution_idx = threadIdx.z;
  const int candidate_idx = global_idx / samples_per_condition;
  const int candidate_sample_idx = global_idx % samples_per_condition;
  const int tdy = threadIdx.y;
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);

  const int shared_idx = blockDim.x * threadIdx.z + threadIdx.x;
  const int distribution_dim = blockDim.z;
  const int sample_dim = blockDim.x;

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
  int running_cost_index = threadIdx.x + blockDim.x * (tdy + blockDim.y * threadIdx.z);
  float* x = &(reinterpret_cast<float*>(x_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_next = &(reinterpret_cast<float*>(x_next_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_temp;
  float* xdot = &(reinterpret_cast<float*>(x_dot_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* u = &(reinterpret_cast<float*>(u_shared)[shared_idx * DYN_T::CONTROL_DIM]);
  float* y = &(reinterpret_cast<float*>(y_shared)[shared_idx * DYN_T::OUTPUT_DIM]);
  float* running_cost = &running_cost_shared[running_cost_index];
  running_cost[0] = 0.0f;
  int* crash_status = &crash_status_shared[shared_idx];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.

  // Load global array to shared array
  mppi::p1::loadArrayParallel<DYN_T::STATE_DIM>(x, 0, states_d, candidate_idx * DYN_T::STATE_DIM);
  int stride = strides_d[candidate_idx];
  int p_index, step;
  mppi::p1::getParallel1DIndex<mppi::p1::Parallel1Dir::THREAD_Y>(p_index, step);
  for (int i = p_index; i < DYN_T::CONTROL_DIM; i += step)
  {
    u[i] = 0;
  }
  for (int i = p_index; i < DYN_T::OUTPUT_DIM; i += step)
  {
    y[i] = 0;
  }
  __syncthreads();

#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* bar = &barrier_shared[(shared_idx)];
  if (tdy == 0)
  {
    init(bar, blockDim.y);
  }
#endif
  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0, dt);
  sampling->initializeDistributions(y, 0.0, dt, theta_d_shared);
  costs->initializeCosts(y, u, theta_c_shared, 0.0f, dt);
  __syncthreads();
  for (int t = 0; t < num_timesteps; t++)
  {
    // Load noise trajectories scaled by the exploration factor
    int candidate_t = min(t + stride, num_timesteps - 1);
    sampling->readControlSample(candidate_sample_idx, candidate_t, distribution_idx, u, theta_d_shared, blockDim.y, tdy,
                                y);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // applies constraints as defined in dynamics.cuh see specific dynamics class for what happens here
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
  running_cost = &running_cost_shared[threadIdx.x + blockDim.x * blockDim.y * threadIdx.z];
  __syncthreads();
  costArrayReduction(running_cost, blockDim.y, tdy, blockDim.y, tdy == 0, blockDim.x);

  // Compute terminal cost and the final cost for each thread
  computeAndSaveCost(num_rollouts, num_timesteps, global_idx, costs, y, running_cost[0] / (num_timesteps),
                     theta_c_shared, trajectory_costs_d);
}

template <class DYN_T, class FB_T, class SAMPLING_T, int NOMINAL_STATE_IDX>
__global__ void rolloutRMPPIDynamicsKernel(DYN_T* __restrict__ dynamics, FB_T* __restrict__ fb_controller,
                                           SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                                           const int num_rollouts, const float* __restrict__ init_x_d,
                                           float* __restrict__ y_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int block_idx = blockIdx.x;
  const int global_idx = blockDim.x * block_idx + thread_idx;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int shared_nom_idx = blockDim.x * NOMINAL_STATE_IDX + thread_idx;
  const int distribution_idx = threadIdx.z;
  const int distribution_dim = blockDim.z;
  const int sample_dim = blockDim.x;
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_fb_bytes = calcClassSharedMemSize(fb_controller, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];

  float* x_shared = entire_buffer;
  float* x_next_shared = &x_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* y_shared = &x_next_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* x_dot_shared = &y_shared[math::nearest_multiple_4(sample_dim * DYN_T::OUTPUT_DIM * distribution_dim)];
  float* u_shared = &x_dot_shared[math::nearest_multiple_4(sample_dim * DYN_T::STATE_DIM * distribution_dim)];
  float* theta_s_shared = &u_shared[math::nearest_multiple_4(sample_dim * DYN_T::CONTROL_DIM * distribution_dim)];
  float* theta_d_shared = &theta_s_shared[size_of_theta_s_bytes / sizeof(float)];
  float* theta_fb = &theta_d_shared[size_of_theta_d_bytes / sizeof(float)];
#ifdef USE_CUDA_BARRIERS_DYN
  barrier* barrier_shared = (barrier*)&theta_fb[size_of_theta_fb_bytes / sizeof(float)];
#endif

  // Create local state, state dot and controls
  float* x = &(reinterpret_cast<float*>(x_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_next = &(reinterpret_cast<float*>(x_next_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_nom = &(reinterpret_cast<float*>(x_shared)[shared_nom_idx * DYN_T::STATE_DIM]);
  float* x_nom_next = &(reinterpret_cast<float*>(x_next_shared)[shared_nom_idx * DYN_T::STATE_DIM]);
  float* x_temp;
  float* xdot = &(reinterpret_cast<float*>(x_dot_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* u = &(reinterpret_cast<float*>(u_shared)[shared_idx * DYN_T::CONTROL_DIM]);
  float* y = &(reinterpret_cast<float*>(y_shared)[shared_idx * DYN_T::OUTPUT_DIM]);
#ifdef USE_CUDA_BARRIERS_DYN
  barrier* bar = &barrier_shared[shared_idx];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif
  // The array to hold K(x,x*)
  float fb_control[DYN_T::CONTROL_DIM];
  int i;

  // Load global array to shared array
  ::mppi::kernels::loadGlobalToShared<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM>(
      num_rollouts, blockDim.y, global_idx, thread_idy, thread_idz, init_x_d, x, xdot, u);
  __syncthreads();

  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0, dt);
  sampling->initializeDistributions(y, 0.0, dt, theta_d_shared);
  fb_controller->initializeFeedback(x, u, theta_fb, 0.0, dt);
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

    // Now find feedback control
    for (i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      fb_control[i] = 0;
    }

    // we do not apply feedback on the nominal state
    if (thread_idz != NOMINAL_STATE_IDX)
    {
      fb_controller->k(x, x_nom, t, theta_fb, fb_control);
    }

    for (i = thread_idy; i < DYN_T::CONTROL_DIM; i += blockDim.y)
    {
      u[i] += fb_control[i];
    }
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
    // copy back feedback-filled controls to global memory
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
    x_temp = x_nom;
    x_nom = x_nom_next;
    x_nom_next = x_temp;
    // Copy state to global memory
    int sample_time_offset = (num_rollouts * thread_idz + global_idx) * num_timesteps + t;
    mp1::loadArrayParallel<DYN_T::OUTPUT_DIM>(y_d, sample_time_offset * DYN_T::OUTPUT_DIM, y, 0);
  }
}

template <class COST_T, class DYN_T, class SAMPLING_T, class FB_T, int NOMINAL_STATE_IDX, bool COALESCE>
__global__ void rolloutRMPPICostKernel(COST_T* __restrict__ costs, DYN_T* __restrict__ dynamics,
                                       FB_T* __restrict__ fb_controller, SAMPLING_T* __restrict__ sampling, float dt,
                                       const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                       float value_func_threshold, const float* __restrict__ init_x_d,
                                       const float* __restrict__ y_d, float* __restrict__ trajectory_costs_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int global_idx = blockIdx.x;
  const int distribution_idx = threadIdx.z;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int num_shared = blockDim.x * blockDim.z;
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);
  const int size_of_theta_fb_bytes = calcClassSharedMemSize(fb_controller, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];
  float* y_shared = entire_buffer;
  float* u_shared = &y_shared[math::nearest_multiple_4(num_shared * COST_T::OUTPUT_DIM)];
  float* running_cost_shared = &u_shared[math::nearest_multiple_4(num_shared * COST_T::CONTROL_DIM)];
  int* crash_status_shared = (int*)&running_cost_shared[math::nearest_multiple_4(num_shared * 2 * blockDim.y)];
  float* theta_c = (float*)&crash_status_shared[math::nearest_multiple_4(num_shared)];
  float* theta_d = &theta_c[size_of_theta_c_bytes / sizeof(float)];
  float* theta_fb = &theta_d[size_of_theta_d_bytes / sizeof(float)];
#ifdef USE_CUDA_BARRIERS_COST
  barrier* barrier_shared = (barrier*)&theta_fb[size_of_theta_fb_bytes / sizeof(float)];
#endif

  // Initialize running cost and total cost
  int sample_time_offset = 0;

  // The array to hold K(x,x*)
  float fb_control[COST_T::CONTROL_DIM];

  // Load global array to shared array
  float* y = &y_shared[shared_idx * COST_T::OUTPUT_DIM];
  float* y_nom = &y_shared[(blockDim.x * NOMINAL_STATE_IDX + thread_idx) * COST_T::OUTPUT_DIM];
  float* u = &u_shared[shared_idx * COST_T::CONTROL_DIM];
  int* crash_status = &crash_status_shared[shared_idx];
  const int cost_index = blockDim.x * (thread_idz * blockDim.y + thread_idy) + thread_idx;
  float* running_cost = &running_cost_shared[cost_index];
  float* running_cost_extra = &running_cost_shared[cost_index + num_shared * blockDim.y];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
  running_cost[0] = 0;
  running_cost_extra[0] = 0;
#ifdef USE_CUDA_BARRIERS_COST
  barrier* bar = &barrier_shared[(blockDim.x * thread_idz + thread_idx)];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif

  float curr_cost = 0.0f;

  /*<----Start of simulation loop-----> */
  const int max_time_iters = ceilf((float)num_timesteps / blockDim.x);
  costs->initializeCosts(y, u, theta_c, 0.0, dt);
  sampling->initializeDistributions(y, 0.0, dt, theta_d);
  fb_controller->initializeFeedback(y, u, theta_fb, 0.0, dt);
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

    // Get feedback
    float x[DYN_T::STATE_DIM];
    float x_nom[DYN_T::STATE_DIM];
    if (t < num_timesteps)
    {
      dynamics->outputToState(y, x);
      dynamics->outputToState(y_nom, x_nom);
      fb_controller->k(x, x_nom, t, theta_fb, fb_control);
    }
#ifdef USE_CUDA_BARRIERS_COST
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // Compute cost
    if (t < num_timesteps)
    {
      curr_cost = costs->computeRunningCost(y, u, t, theta_c, crash_status);
    }

    if (thread_idz == NOMINAL_STATE_IDX && t < num_timesteps)
    {
      running_cost[0] += curr_cost;
      running_cost_extra[0] +=
          sampling->computeLikelihoodRatioCost(u, theta_d, global_idx, t, distribution_idx, lambda, alpha);
    }

    if (thread_idz != NOMINAL_STATE_IDX && t < num_timesteps)
    {
      running_cost[0] +=
          curr_cost + sampling->computeLikelihoodRatioCost(u, theta_d, global_idx, t, distribution_idx, lambda, alpha);
      running_cost_extra[0] +=
          curr_cost + sampling->computeFeedbackCost(fb_control, theta_d, t, distribution_idx, lambda, alpha);
    }

    // We need running_state_cost_nom and running_control_cost_nom for the nominal system
    // We need running_cost_real and running_cost_feedback for the real system
    // Nominal system needs to know running_state_cost_nom, running_control_cost_nom, and running_cost_feedback
    // Real system needs to know running_cost_real

#ifdef USE_CUDA_BARRIERS_COST
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
  }

  // Add all costs together
  running_cost = &running_cost_shared[blockDim.x * blockDim.y * thread_idz];
  running_cost_extra = &running_cost_shared[blockDim.x * blockDim.y * (blockDim.z + thread_idz)];
  __syncthreads();

  multiCostArrayReduction(running_cost, running_cost_extra, blockDim.x * blockDim.y,
                          thread_idx + blockDim.x * thread_idy, blockDim.x * blockDim.y,
                          thread_idx == blockDim.x - 1 && thread_idy == 0);

  // point every thread to the last output at t = NUM_TIMESTEPS for terminal cost calculation
  const int last_y_index = (num_timesteps - 1) % blockDim.x;
  y = &y_shared[(blockDim.x * thread_idz + last_y_index) * COST_T::OUTPUT_DIM];
  if (thread_idx == 0 && thread_idy == 0)
  {
    *running_cost += costs->terminalCost(y, theta_c);
    if (thread_idz != NOMINAL_STATE_IDX)
    {
      *running_cost_extra += costs->terminalCost(y, theta_c);
    }
    *running_cost /= ((float)num_timesteps);
    *running_cost_extra /= ((float)num_timesteps);
  }
  __syncthreads();
  if (thread_idz != NOMINAL_STATE_IDX && thread_idx == 0 && thread_idy == 0)
  {
    float* running_nom_cost = &running_cost_shared[blockDim.x * blockDim.y * NOMINAL_STATE_IDX];
    const float* running_nom_cost_likelihood_ratio_cost =
        &running_cost_shared[blockDim.x * blockDim.y * (blockDim.z + NOMINAL_STATE_IDX)];
    *running_nom_cost =
        0.5 * *running_nom_cost + 0.5 * fmaxf(fminf(*running_cost_extra, value_func_threshold), *running_nom_cost);
    *running_nom_cost += *running_nom_cost_likelihood_ratio_cost;
  }
  __syncthreads();
  if (thread_idy == 0 && thread_idx == 0)
  {
    trajectory_costs_d[global_idx + thread_idz * num_rollouts] = running_cost[0];
  }
}

template <class DYN_T, class COST_T, class FB_T, class SAMPLING_T, int NOMINAL_STATE_IDX>
__global__ void rolloutRMPPIKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                                   FB_T* __restrict__ fb_controller, SAMPLING_T* __restrict__ sampling, float dt,
                                   const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                   float value_func_threshold, const float* __restrict__ init_x_d,
                                   float* __restrict__ trajectory_costs_d)
{
  // Get thread and block id
  const int thread_idx = threadIdx.x;
  const int thread_idy = threadIdx.y;
  const int thread_idz = threadIdx.z;
  const int block_idx = blockIdx.x;
  const int global_idx = blockDim.x * block_idx + thread_idx;
  const int shared_idx = blockDim.x * thread_idz + thread_idx;
  const int shared_nom_idx = blockDim.x * NOMINAL_STATE_IDX + thread_idx;
  const int distribution_idx = threadIdx.z;
  const int num_shared = blockDim.x * blockDim.z;
  const int size_of_theta_s_bytes = calcClassSharedMemSize(dynamics, blockDim);
  const int size_of_theta_d_bytes = calcClassSharedMemSize(sampling, blockDim);
  const int size_of_theta_c_bytes = calcClassSharedMemSize(costs, blockDim);
  const int size_of_theta_fb_bytes = calcClassSharedMemSize(fb_controller, blockDim);

  // Create shared state and control arrays
  extern __shared__ float entire_buffer[];

  float* x_shared = entire_buffer;
  float* x_next_shared = &x_shared[math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM)];
  float* y_shared = &x_next_shared[math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM)];
  float* x_dot_shared = &y_shared[math::nearest_multiple_4(num_shared * DYN_T::OUTPUT_DIM)];
  float* u_shared = &x_dot_shared[math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM)];
  float* theta_s_shared = &u_shared[math::nearest_multiple_4(num_shared * DYN_T::CONTROL_DIM)];
  float* theta_d_shared = &theta_s_shared[size_of_theta_s_bytes / sizeof(float)];
  float* theta_c_shared = &theta_d_shared[size_of_theta_d_bytes / sizeof(float)];
  float* theta_fb = &theta_c_shared[size_of_theta_c_bytes / sizeof(float)];
  float* running_cost_shared = &theta_fb[size_of_theta_fb_bytes / sizeof(float)];
  int* crash_status_shared = (int*)&running_cost_shared[math::nearest_multiple_4(num_shared * 2 * blockDim.y)];
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* barrier_shared = (barrier*)&crash_status_shared[math::nearest_multiple_4(num_shared)];
#endif

  // Create local state, state dot and controls
  float* x = &(reinterpret_cast<float*>(x_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_next = &(reinterpret_cast<float*>(x_next_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* x_nom = &(reinterpret_cast<float*>(x_shared)[shared_nom_idx * DYN_T::STATE_DIM]);
  float* x_nom_next = &(reinterpret_cast<float*>(x_next_shared)[shared_nom_idx * DYN_T::STATE_DIM]);
  float* x_temp;
  float* xdot = &(reinterpret_cast<float*>(x_dot_shared)[shared_idx * DYN_T::STATE_DIM]);
  float* u = &(reinterpret_cast<float*>(u_shared)[shared_idx * DYN_T::CONTROL_DIM]);
  float* y = &(reinterpret_cast<float*>(y_shared)[shared_idx * DYN_T::OUTPUT_DIM]);
  int* crash_status = &crash_status_shared[shared_idx];
  const int cost_index = blockDim.x * (thread_idz * blockDim.y + thread_idy) + thread_idx;
  float* running_cost = &running_cost_shared[cost_index];
  float* running_cost_extra = &running_cost_shared[cost_index + num_shared * blockDim.y];
  crash_status[0] = 0;  // We have not crashed yet as of the first trajectory.
  running_cost[0] = 0;
  running_cost_extra[0] = 0;
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  barrier* bar = &barrier_shared[shared_idx];
  if (thread_idy == 0)
  {
    init(bar, blockDim.y);
  }
#endif
  // The array to hold K(x,x*)
  float fb_control[DYN_T::CONTROL_DIM];
  int i;
  float curr_cost = 0.0f;

  // Load global array to shared array
  ::mppi::kernels::loadGlobalToShared<DYN_T::STATE_DIM, DYN_T::CONTROL_DIM>(
      num_rollouts, blockDim.y, global_idx, thread_idy, thread_idz, init_x_d, x, xdot, u);
  __syncthreads();

  /*<----Start of simulation loop-----> */
  dynamics->initializeDynamics(x, u, y, theta_s_shared, 0.0, dt);
  sampling->initializeDistributions(y, 0.0, dt, theta_d_shared);
  costs->initializeCosts(y, u, theta_c_shared, 0.0, dt);
  fb_controller->initializeFeedback(x, u, theta_fb, 0.0, dt);
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

    // Now find feedback control
    for (i = 0; i < DYN_T::CONTROL_DIM; i++)
    {
      fb_control[i] = 0;
    }

    // we do not apply feedback on the nominal state
    if (thread_idz != NOMINAL_STATE_IDX)
    {
      fb_controller->k(x, x_nom, t, theta_fb, fb_control);
    }

    for (i = thread_idy; i < DYN_T::CONTROL_DIM; i += blockDim.y)
    {
      u[i] += fb_control[i];
    }
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
    // copy back feedback-filled controls to global memory
    sampling->writeControlSample(global_idx, t, distribution_idx, u, theta_d_shared, blockDim.y, thread_idy, y);

    // Increment states
    dynamics->step(x, x_next, xdot, u, y, theta_s_shared, t, dt);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif

    // Compute cost
    if (t < num_timesteps)
    {
      curr_cost = costs->computeRunningCost(y, u, t, theta_c_shared, crash_status);
    }

    if (thread_idz == NOMINAL_STATE_IDX && t < num_timesteps)
    {
      running_cost[0] += curr_cost;
      running_cost_extra[0] +=
          sampling->computeLikelihoodRatioCost(u, theta_d_shared, global_idx, t, distribution_idx, lambda, alpha);
    }

    if (thread_idz != NOMINAL_STATE_IDX && t < num_timesteps)
    {
      running_cost[0] += curr_cost + sampling->computeLikelihoodRatioCost(u, theta_d_shared, global_idx, t,
                                                                          distribution_idx, lambda, alpha);
      running_cost_extra[0] +=
          curr_cost + sampling->computeFeedbackCost(fb_control, theta_d_shared, t, distribution_idx, lambda, alpha);
    }

    // We need running_state_cost_nom and running_control_cost_nom for the nominal system
    // We need running_cost_real and running_cost_feedback for the real system
    // Nominal system needs to know running_state_cost_nom, running_control_cost_nom, and running_cost_feedback
    // Real system needs to know running_cost_real

#ifdef USE_CUDA_BARRIERS_ROLLOUT
    bar->arrive_and_wait();
#else
    __syncthreads();
#endif
    x_temp = x;
    x = x_next;
    x_next = x_temp;
    x_temp = x_nom;
    x_nom = x_nom_next;
    x_nom_next = x_temp;
  }

  // Add all costs together
  running_cost = &running_cost_shared[thread_idx + blockDim.x * blockDim.y * thread_idz];
  running_cost_extra = &running_cost_shared[thread_idx + blockDim.x * blockDim.y * (blockDim.z + thread_idz)];
  __syncthreads();
  multiCostArrayReduction(running_cost, running_cost_extra, blockDim.y, thread_idy, blockDim.y, thread_idy == 0,
                          blockDim.x);

  if (thread_idy == 0)
  {
    *running_cost += costs->terminalCost(y, theta_c_shared);
    if (thread_idz != NOMINAL_STATE_IDX)
    {
      *running_cost_extra += costs->terminalCost(y, theta_c_shared);
    }
    *running_cost /= ((float)num_timesteps);
    *running_cost_extra /= ((float)num_timesteps);
  }
  __syncthreads();
  if (thread_idz != NOMINAL_STATE_IDX && thread_idy == 0)
  {
    float* running_nom_cost = &running_cost_shared[blockDim.x * blockDim.y * NOMINAL_STATE_IDX];
    const float* running_nom_cost_likelihood_ratio_cost =
        &running_cost_shared[blockDim.x * blockDim.y * (blockDim.z + NOMINAL_STATE_IDX)];
    *running_nom_cost =
        0.5 * *running_nom_cost + 0.5 * fmaxf(fminf(*running_cost_extra, value_func_threshold), *running_nom_cost);
    *running_nom_cost += *running_nom_cost_likelihood_ratio_cost;
  }
  __syncthreads();
  if (thread_idy == 0)
  {
    trajectory_costs_d[global_idx + thread_idz * num_rollouts] = running_cost[0];
  }
}

template <class DYN_T, class COST_T, typename SAMPLING_T, bool COALESCE>
void launchSplitInitEvalKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                               SAMPLING_T* __restrict__ sampling, float dt, const int num_timesteps,
                               const int num_rollouts, float lambda, float alpha, int samples_per_condition,
                               int* __restrict__ strides_d, float* __restrict__ init_x_d, float* __restrict__ y_d,
                               float* __restrict__ trajectory_costs, dim3 dimDynBlock, dim3 dimCostBlock,
                               cudaStream_t stream, bool synchronize)
{
  if (num_rollouts % dimDynBlock.x != 0)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_rollouts (" << num_rollouts
              << ") must be evenly divided by eval dynamics block size x (" << dimDynBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }
  if (num_timesteps < dimCostBlock.x)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_timesteps (" << num_timesteps
              << ") must be greater than or equal to eval cost block size x (" << dimCostBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }

  // Run Dynamics
  const int gridsize_x = math::int_ceil(num_rollouts, dimDynBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned dynamics_shared_size = calcEvalDynKernelSharedMemSize(dynamics, sampling, dimDynBlock);
  initEvalDynKernel<DYN_T, SAMPLING_T><<<dimGrid, dimDynBlock, dynamics_shared_size, stream>>>(
      dynamics->model_d_, sampling->sampling_d_, dt, num_timesteps, num_rollouts, samples_per_condition, strides_d,
      init_x_d, y_d);

  // Run Costs
  dim3 dimCostGrid(num_rollouts, 1, 1);
  unsigned cost_shared_size =
      calcEvalCostKernelSharedMemSize(costs, sampling, num_rollouts, samples_per_condition, dimCostBlock);

  initEvalCostKernel<COST_T, SAMPLING_T, COALESCE><<<dimCostGrid, dimCostBlock, cost_shared_size, stream>>>(
      costs->cost_d_, sampling->sampling_d_, dt, num_timesteps, num_rollouts, lambda, alpha, samples_per_condition,
      strides_d, y_d, trajectory_costs);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <class DYN_T, class COST_T, typename SAMPLING_T>
void launchInitEvalKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs, SAMPLING_T* __restrict__ sampling,
                          float dt, const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                          int samples_per_condition, int* __restrict__ strides_d, float* __restrict__ init_x_d,
                          float* __restrict__ trajectory_costs, dim3 dimBlock, cudaStream_t stream, bool synchronize)
{
  if (num_rollouts % dimBlock.x != 0)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_rollouts (" << num_rollouts
              << ") must be evenly divided by eval kernel block size x (" << dimBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }
  // Run initEvalKernel
  const int gridsize_x = math::int_ceil(num_rollouts, dimBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned shared_size =
      calcEvalCombinedKernelSharedMemSize(dynamics, costs, sampling, num_rollouts, samples_per_condition, dimBlock);
  initEvalKernel<DYN_T, COST_T, SAMPLING_T><<<dimGrid, dimBlock, shared_size, stream>>>(
      dynamics->model_d_, costs->cost_d_, sampling->sampling_d_, dt, num_timesteps, num_rollouts, samples_per_condition,
      lambda, alpha, strides_d, init_x_d, trajectory_costs);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
}

template <class DYN_T, class COST_T, class SAMPLING_T, class FB_T, int NOMINAL_STATE_IDX, bool COALESCE>
void launchSplitRMPPIRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                                   SAMPLING_T* __restrict__ sampling, FB_T* __restrict__ fb_controller, float dt,
                                   const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                                   float value_func_threshold, float* __restrict__ init_x_d, float* __restrict__ y_d,
                                   float* __restrict__ trajectory_costs, dim3 dimDynBlock, dim3 dimCostBlock,
                                   cudaStream_t stream, bool synchronize)
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
  if (dimDynBlock.z < 2 || dimDynBlock.z != dimCostBlock.z)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): number of dynamics systems (" << dimDynBlock.z
              << ") and cost systems (" << dimCostBlock.z << ") must be equal to each other and greater than 2"
              << std::endl;
    exit(EXIT_FAILURE);
  }
  // Run Dynamics
  const int gridsize_x = math::int_ceil(num_rollouts, dimDynBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned dynamics_shared_size = calcRMPPIDynKernelSharedMemSize(dynamics, sampling, fb_controller, dimDynBlock);
  rolloutRMPPIDynamicsKernel<DYN_T, FB_T, SAMPLING_T, NOMINAL_STATE_IDX>
      <<<dimGrid, dimDynBlock, dynamics_shared_size, stream>>>(dynamics->model_d_, fb_controller->feedback_d_,
                                                               sampling->sampling_d_, dt, num_timesteps, num_rollouts,
                                                               init_x_d, y_d);
  HANDLE_ERROR(cudaGetLastError());

  // Run Costs
  dim3 dimCostGrid(num_rollouts, 1, 1);
  unsigned cost_shared_size = calcRMPPICostKernelSharedMemSize(costs, sampling, fb_controller, dimCostBlock);
  rolloutRMPPICostKernel<COST_T, DYN_T, SAMPLING_T, FB_T, NOMINAL_STATE_IDX, COALESCE>
      <<<dimCostGrid, dimCostBlock, cost_shared_size, stream>>>(
          costs->cost_d_, dynamics->model_d_, fb_controller->feedback_d_, sampling->sampling_d_, dt, num_timesteps,
          num_rollouts, lambda, alpha, value_func_threshold, init_x_d, y_d, trajectory_costs);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize || true)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
  HANDLE_ERROR(cudaGetLastError());
}

template <class DYN_T, class COST_T, class SAMPLING_T, class FB_T, int NOMINAL_STATE_IDX>
void launchRMPPIRolloutKernel(DYN_T* __restrict__ dynamics, COST_T* __restrict__ costs,
                              SAMPLING_T* __restrict__ sampling, FB_T* __restrict__ fb_controller, float dt,
                              const int num_timesteps, const int num_rollouts, float lambda, float alpha,
                              float value_func_threshold, float* __restrict__ init_x_d,
                              float* __restrict__ trajectory_costs, dim3 dimBlock, cudaStream_t stream,
                              bool synchronize)
{
  if (num_rollouts % dimBlock.x != 0)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): num_rollouts (" << num_rollouts
              << ") must be evenly divided by dynamics block size x (" << dimBlock.x << ")" << std::endl;
    exit(EXIT_FAILURE);
  }
  // if (num_timesteps < dimCostBlock.x)
  // {
  //   std::cerr << __FILE__ << " (" << __LINE__ << "): num_timesteps (" << num_timesteps
  //             << ") must be greater than or equal to cost block size x (" << dimCostBlock.x << ")" << std::endl;
  //   exit(EXIT_FAILURE);
  // }
  if (dimBlock.z < 2)
  {
    std::cerr << __FILE__ << " (" << __LINE__ << "): number of dynamics systems (" << dimBlock.z
              << ")  must be greater than 2" << std::endl;
    exit(EXIT_FAILURE);
  }
  // Run RMPPI Rollout kernel
  const int gridsize_x = math::int_ceil(num_rollouts, dimBlock.x);
  dim3 dimGrid(gridsize_x, 1, 1);
  unsigned shared_size = calcRMPPICombinedKernelSharedMemSize(dynamics, costs, sampling, fb_controller, dimBlock);
  rolloutRMPPIKernel<DYN_T, COST_T, FB_T, SAMPLING_T, NOMINAL_STATE_IDX><<<dimGrid, dimBlock, shared_size, stream>>>(
      dynamics->model_d_, costs->cost_d_, fb_controller->feedback_d_, sampling->sampling_d_, dt, num_timesteps,
      num_rollouts, lambda, alpha, value_func_threshold, init_x_d, trajectory_costs);
  HANDLE_ERROR(cudaGetLastError());
  if (synchronize || true)
  {
    HANDLE_ERROR(cudaStreamSynchronize(stream));
  }
  HANDLE_ERROR(cudaGetLastError());
}

template <class DYN_T, class SAMPLER_T>
unsigned calcEvalDynKernelSharedMemSize(const DYN_T* dynamics, const SAMPLER_T* sampler, dim3& dimBlock)
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
unsigned calcEvalCostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, const int num_rollouts,
                                         const int samples_per_condition, dim3& dimBlock)
{
  const int cost_num_shared = dimBlock.x * dimBlock.z;
  unsigned cost_shared_size = sizeof(float) * (math::nearest_multiple_4(cost_num_shared * COST_T::OUTPUT_DIM) +
                                               math::nearest_multiple_4(cost_num_shared * COST_T::CONTROL_DIM) +
                                               math::nearest_multiple_4(cost_num_shared * dimBlock.y)) +
                              sizeof(int) * math::nearest_multiple_4(cost_num_shared) +
                              calcClassSharedMemSize(cost, dimBlock) +
                              calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock) +
                              math::nearest_multiple_4(num_rollouts / samples_per_condition) * sizeof(float);
#ifdef USE_CUDA_BARRIERS_COST
  cost_shared_size += math::int_multiple_const(cost_num_shared * sizeof(barrier), 16);
#endif
  return cost_shared_size;
}

template <class DYN_T, class COST_T, class SAMPLER_T>
unsigned calcEvalCombinedKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                             const int num_rollouts, const int samples_per_condition, dim3& dimBlock)
{
  const int num_shared = dimBlock.x * dimBlock.z;
  unsigned shared_size =
      sizeof(float) * (3 * math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM) +
                       math::nearest_multiple_4(num_shared * DYN_T::OUTPUT_DIM) +
                       math::nearest_multiple_4(num_shared * DYN_T::CONTROL_DIM) +
                       math::nearest_multiple_4(num_shared * dimBlock.y)) +
      sizeof(int) * math::nearest_multiple_4(num_shared) + calcClassSharedMemSize<DYN_T>(dynamics, dimBlock) +
      calcClassSharedMemSize<COST_T>(cost, dimBlock) + calcClassSharedMemSize<SAMPLER_T>(sampler, dimBlock);
#ifdef USE_CUDA_BARRIERS_ROLLOUT
  shared_size += math::int_multiple_const(num_shared * sizeof(barrier), 16);
#endif
  return shared_size;
}

template <class DYN_T, class SAMPLER_T, class FB_T>
unsigned calcRMPPIDynKernelSharedMemSize(const DYN_T* dynamics, const SAMPLER_T* sampler, const FB_T* fb_controller,
                                         dim3& dimBlock)
{
  const int dynamics_num_shared = dimBlock.x * dimBlock.z;
  unsigned dynamics_shared_size =
      sizeof(float) * (3 * math::nearest_multiple_4(dynamics_num_shared * DYN_T::STATE_DIM) +
                       math::nearest_multiple_4(dynamics_num_shared * DYN_T::OUTPUT_DIM) +
                       math::nearest_multiple_4(dynamics_num_shared * DYN_T::CONTROL_DIM)) +
      calcClassSharedMemSize(dynamics, dimBlock) + calcClassSharedMemSize(sampler, dimBlock) +
      calcClassSharedMemSize(fb_controller, dimBlock);
#ifdef USE_CUDA_BARRIERS_DYN
  dynamics_shared_size += math::int_multiple_const(dynamics_num_shared * sizeof(barrier), 16);
#endif
  return dynamics_shared_size;
}

template <class COST_T, class SAMPLER_T, class FB_T>
unsigned calcRMPPICostKernelSharedMemSize(const COST_T* cost, const SAMPLER_T* sampler, const FB_T* fb_controller,
                                          dim3& dimBlock)
{
  const int cost_num_shared = dimBlock.x * dimBlock.z;
  unsigned cost_shared_size = sizeof(float) * (math::nearest_multiple_4(cost_num_shared * COST_T::OUTPUT_DIM) +
                                               math::nearest_multiple_4(cost_num_shared * COST_T::CONTROL_DIM) +
                                               math::nearest_multiple_4(cost_num_shared * dimBlock.y * 2)) +
                              sizeof(int) * math::nearest_multiple_4(cost_num_shared) +
                              calcClassSharedMemSize(cost, dimBlock) + calcClassSharedMemSize(sampler, dimBlock) +
                              calcClassSharedMemSize(fb_controller, dimBlock);
#ifdef USE_CUDA_BARRIERS_COST
  cost_shared_size += math::int_multiple_const(cost_num_shared * sizeof(barrier), 16);
#endif
  return cost_shared_size;
}

template <class DYN_T, class COST_T, class SAMPLER_T, class FB_T>
unsigned calcRMPPICombinedKernelSharedMemSize(const DYN_T* dynamics, const COST_T* cost, const SAMPLER_T* sampler,
                                              const FB_T* fb_controller, dim3& dimBlock)
{
  const int num_shared = dimBlock.x * dimBlock.z;
  unsigned shared_size = sizeof(float) * (3 * math::nearest_multiple_4(num_shared * DYN_T::STATE_DIM) +
                                          math::nearest_multiple_4(num_shared * DYN_T::OUTPUT_DIM) +
                                          math::nearest_multiple_4(num_shared * DYN_T::CONTROL_DIM) +
                                          math::nearest_multiple_4(num_shared * dimBlock.y * 2)) +
                         sizeof(int) * math::nearest_multiple_4(num_shared) +
                         calcClassSharedMemSize(dynamics, dimBlock) + calcClassSharedMemSize(sampler, dimBlock) +
                         calcClassSharedMemSize(fb_controller, dimBlock) + calcClassSharedMemSize(cost, dimBlock);
#ifdef USE_CUDA_BARRIERS_DYN
  shared_size += math::int_multiple_const(num_shared * sizeof(barrier), 16);
#endif
  return shared_size;
}

__device__ void multiCostArrayReduction(float* running_cost, float* running_cost_extra, const int start_size,
                                        const int index, const int step, const bool catch_condition, const int stride)
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
      running_cost_extra[j * stride] += running_cost_extra[(j + size) * stride];
    }
    __syncthreads();
    if (prev_size - 2 * size == 1 && catch_condition)
    {
      running_cost[(size - 1) * stride] += running_cost[(prev_size - 1) * stride];
      running_cost_extra[(size - 1) * stride] += running_cost_extra[(prev_size - 1) * stride];
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
        warpReduceAdd<64>(running_cost_extra, index, stride);
      }
      break;
    case 32:
      if (index < 16)
      {
        warpReduceAdd<32>(running_cost, index, stride);
        warpReduceAdd<32>(running_cost_extra, index, stride);
      }
      break;
    case 16:
      if (index < 8)
      {
        warpReduceAdd<16>(running_cost, index, stride);
        warpReduceAdd<16>(running_cost_extra, index, stride);
      }
      break;
    case 8:
      if (index < 4)
      {
        warpReduceAdd<8>(running_cost, index, stride);
        warpReduceAdd<8>(running_cost_extra, index, stride);
      }
      break;
    case 4:
      if (index < 2)
      {
        warpReduceAdd<4>(running_cost, index, stride);
        warpReduceAdd<4>(running_cost_extra, index, stride);
      }
      break;
    case 2:
      if (index < 1)
      {
        warpReduceAdd<2>(running_cost, index, stride);
        warpReduceAdd<2>(running_cost_extra, index, stride);
      }
      break;
  }
  __syncthreads();
}
}  // namespace rmppi
}  // namespace kernels
}  // namespace mppi
