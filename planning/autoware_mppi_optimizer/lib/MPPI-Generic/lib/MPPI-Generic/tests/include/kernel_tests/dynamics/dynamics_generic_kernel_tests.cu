#include <mppi/core/mppi_common.cuh>

template <typename CLASS_T, typename PARAMS_T>
__global__ void parameterTestKernel(CLASS_T* class_t, PARAMS_T& params)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid == 0)
  {
    params = class_t->getParams();
  }
}

template <typename CLASS_T, typename PARAMS_T>
void launchParameterTestKernel(CLASS_T& class_t, PARAMS_T& params)
{
  PARAMS_T* params_d;
  HANDLE_ERROR(cudaMalloc((void**)&params_d, sizeof(PARAMS_T)))

  parameterTestKernel<CLASS_T, PARAMS_T><<<1, 1>>>(static_cast<CLASS_T*>(class_t.model_d_), *params_d);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(&params, params_d, sizeof(PARAMS_T), cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(params_d);
}

template <class DYN_T>
__global__ void getSharedMemorySizesKernel(DYN_T* __restrict__ dynamics, int* __restrict__ output_d)
{
  output_d[0] = dynamics->getGrdSharedSizeBytes();
  output_d[1] = dynamics->getBlkSharedSizeBytes();
}

template <typename DYNAMICS_T>
void launchGetSharedMemorySizesKernel(DYNAMICS_T& dynamics, int shared_mem_sizes[2])
{
  int* shared_mem_sizes_d;
  HANDLE_ERROR(cudaMalloc((void**)&shared_mem_sizes_d, sizeof(int) * 2));

  getSharedMemorySizesKernel<DYNAMICS_T><<<1, 1>>>(dynamics.model_d_, shared_mem_sizes_d);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(shared_mem_sizes, shared_mem_sizes_d, sizeof(int) * 2, cudaMemcpyDeviceToHost));

  HANDLE_ERROR(cudaFree(shared_mem_sizes_d));
}

template <typename DYNAMICS_T, int C_DIM>
__global__ void controlRangesTestKernel(DYNAMICS_T* dynamics, float2* control_rngs)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid == 0)
  {
    float2* raw_ptr = dynamics->getControlRangesRaw();
    for (int i = 0; i < C_DIM; i++)
    {
      control_rngs[i].x = raw_ptr[i].x;
      control_rngs[i].y = raw_ptr[i].y;
    }
  }
}

template <typename DYNAMICS_T, int C_DIM>
void launchControlRangesTestKernel(DYNAMICS_T& dynamics, std::array<float2, C_DIM>& control_rngs)
{
  float2* ranges_d;
  HANDLE_ERROR(cudaMalloc((void**)&ranges_d, sizeof(float2) * control_rngs.size()))

  controlRangesTestKernel<DYNAMICS_T, C_DIM><<<1, 1>>>(static_cast<DYNAMICS_T*>(dynamics.model_d_), ranges_d);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(control_rngs.data(), ranges_d, sizeof(float2) * control_rngs.size(), cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(ranges_d);
}

template <typename DYNAMICS_T, int S_DIM, int C_DIM>
__global__ void enforceConstraintTestKernel(DYNAMICS_T* dynamics, float* state, float* control, int num)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    dynamics->enforceConstraints(&state[tid * S_DIM], &control[tid * C_DIM]);
  }
}

template <typename DYNAMICS_T, int S_DIM, int C_DIM>
void launchEnforceConstraintTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                       std::vector<std::array<float, C_DIM>>& control, int dim_y)
{
  int count = state.size();
  float* state_d;
  float* control_d;
  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * S_DIM * state.size()))
  HANDLE_ERROR(cudaMalloc((void**)&control_d, sizeof(float) * C_DIM * control.size()))

  HANDLE_ERROR(cudaMemcpy(state_d, state.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(control_d, control.data(), sizeof(float) * C_DIM * count, cudaMemcpyHostToDevice));

  dim3 threadsPerBlock(count, dim_y);
  dim3 numBlocks(1, 1);
  enforceConstraintTestKernel<DYNAMICS_T, S_DIM, C_DIM>
      <<<numBlocks, threadsPerBlock>>>(static_cast<DYNAMICS_T*>(dynamics.model_d_), state_d, control_d, count);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(state.data(), state_d, sizeof(float) * S_DIM * state.size(), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(control.data(), control_d, sizeof(float) * C_DIM * control.size(), cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(control_d);
}

template <typename DYNAMICS_T, int S_DIM>
__global__ void updateStateTestKernel(DYNAMICS_T* dynamics, float* state, float* state_der, float dt, int num)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    dynamics->updateState(state + (tid * S_DIM), state_der + (tid * S_DIM), dt);
  }
}

template <typename DYNAMICS_T, int S_DIM>
void launchUpdateStateTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                 std::vector<std::array<float, S_DIM>>& state_der, float dt, int dim_y)
{
  int count = state.size();
  float* state_d;
  float* state_der_d;
  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * S_DIM * count))
  HANDLE_ERROR(cudaMalloc((void**)&state_der_d, sizeof(float) * S_DIM * count))

  HANDLE_ERROR(cudaMemcpy(state_d, state.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(state_der_d, state_der.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));

  dim3 threadsPerBlock(count, dim_y);
  dim3 numBlocks(1, 1);
  updateStateTestKernel<DYNAMICS_T, S_DIM>
      <<<numBlocks, threadsPerBlock>>>(static_cast<DYNAMICS_T*>(dynamics.model_d_), state_d, state_der_d, dt, count);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(state.data(), state_d, sizeof(float) * S_DIM * state.size(), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(state_der.data(), state_der_d, sizeof(float) * S_DIM * state_der.size(), cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(state_der_d);
}

template <typename DYNAMICS_T, int S_DIM>
__global__ void computeKinematicsTestKernel(DYNAMICS_T* dynamics, float* state, float* state_der, int num)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num && threadIdx.y == 0)
  {
    dynamics->computeKinematics(state + (tid * S_DIM), state_der + (tid * S_DIM));
  }
}

template <typename DYNAMICS_T, int S_DIM>
void launchComputeKinematicsTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                       std::vector<std::array<float, S_DIM>>& state_der, int dim_y)
{
  int count = state.size();
  float* state_d;
  float* state_der_d;
  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * S_DIM * state.size()))
  HANDLE_ERROR(cudaMalloc((void**)&state_der_d, sizeof(float) * S_DIM * state_der.size()))

  HANDLE_ERROR(cudaMemcpy(state_d, state.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(state_der_d, state_der.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));

  dim3 threadsPerBlock(count, dim_y);
  dim3 numBlocks(1, 1);
  computeKinematicsTestKernel<DYNAMICS_T, S_DIM>
      <<<numBlocks, threadsPerBlock>>>(static_cast<DYNAMICS_T*>(dynamics.model_d_), state_d, state_der_d, count);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(state.data(), state_d, sizeof(float) * S_DIM * state.size(), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(state_der.data(), state_der_d, sizeof(float) * S_DIM * state_der.size(), cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(state_der_d);
}

template <class DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X>
__global__ void computeDynamicsTestKernel(DYNAMICS_T* model, float* state, float* control, float* state_der, int count)
{
  extern __shared__ float entire_buffer[];

  float* output = entire_buffer;
  float* theta = &output[mppi::math::nearest_multiple_4(blockDim.x * DYNAMICS_T::OUTPUT_DIM)];

  int tid = blockIdx.x * blockDim.x + threadIdx.x;

  model->initializeDynamics(state, control, output, theta, 0.0f, 0.0f);
  __syncthreads();

  if (tid < count)
  {
    model->computeDynamics(state + (tid * S_DIM), control + (tid * C_DIM), state_der + (tid * S_DIM), theta);
  }
}

template <class DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X>
void launchComputeDynamicsTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                     std::vector<std::array<float, C_DIM>>& control,
                                     std::vector<std::array<float, S_DIM>>& state_der, int dim_y)
{
  int count = state.size();
  float* state_d;
  float* state_der_d;
  float* control_d;

  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * S_DIM * count))
  HANDLE_ERROR(cudaMalloc((void**)&state_der_d, sizeof(float) * S_DIM * count))
  HANDLE_ERROR(cudaMalloc((void**)&control_d, sizeof(float) * C_DIM * count))

  HANDLE_ERROR(cudaMemcpy(state_d, state.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(state_der_d, state_der.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(control_d, control.data(), sizeof(float) * C_DIM * count, cudaMemcpyHostToDevice));

  // make sure you cannot use invalid inputs
  const int gridsize_x = (count - 1) / BLOCKDIM_X + 1;
  dim3 threadsPerBlock(BLOCKDIM_X, dim_y);
  dim3 numBlocks(gridsize_x, 1);
  unsigned shared_mem = mppi::kernels::calcClassSharedMemSize(&dynamics, threadsPerBlock) +
                        mppi::math::nearest_multiple_4(threadsPerBlock.x * DYNAMICS_T::OUTPUT_DIM);
  // launch kernel
  computeDynamicsTestKernel<DYNAMICS_T, S_DIM, C_DIM, BLOCKDIM_X>
      <<<numBlocks, threadsPerBlock, shared_mem>>>(dynamics.model_d_, state_d, control_d, state_der_d, count);
  HANDLE_ERROR(cudaGetLastError());

  HANDLE_ERROR(cudaMemcpy(state.data(), state_d, sizeof(float) * S_DIM * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(state_der.data(), state_der_d, sizeof(float) * S_DIM * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(control.data(), control_d, sizeof(float) * C_DIM * count, cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(state_der_d);
  cudaFree(control_d);
}

template <typename DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X>
__global__ void computeStateDerivTestKernel(DYNAMICS_T* dynamics, float* state, float* control, float* state_der,
                                            int num)
{
  extern __shared__ float entire_buffer[];

  float* output = entire_buffer;
  float* theta = &output[mppi::math::nearest_multiple_4(blockDim.x * DYNAMICS_T::OUTPUT_DIM)];

  dynamics->initializeDynamics(state, control, output, theta, 0.0f, 0.0f);
  __syncthreads();

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid < num)
  {
    dynamics->computeStateDeriv(state + (tid * S_DIM), control + (tid * C_DIM), state_der + (tid * S_DIM), theta);
  }
}

template <typename DYNAMICS_T, int S_DIM, int C_DIM, int BLOCKDIM_X>
void launchComputeStateDerivTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, S_DIM>>& state,
                                       std::vector<std::array<float, C_DIM>>& control,
                                       std::vector<std::array<float, S_DIM>>& state_der, int dim_y)
{
  int count = state.size();
  float* state_d;
  float* control_d;
  float* state_der_d;
  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * S_DIM * state.size()))
  HANDLE_ERROR(cudaMalloc((void**)&state_der_d, sizeof(float) * S_DIM * state_der.size()))
  HANDLE_ERROR(cudaMalloc((void**)&control_d, sizeof(float) * C_DIM * control.size()))

  HANDLE_ERROR(cudaMemcpy(state_d, state.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(state_der_d, state_der.data(), sizeof(float) * S_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(control_d, control.data(), sizeof(float) * C_DIM * count, cudaMemcpyHostToDevice));

  const int gridsize_x = (count - 1) / BLOCKDIM_X + 1;
  dim3 threadsPerBlock(BLOCKDIM_X, dim_y);
  dim3 numBlocks(gridsize_x, 1);

  unsigned shared_mem = mppi::kernels::calcClassSharedMemSize(&dynamics, threadsPerBlock) +
                        mppi::math::nearest_multiple_4(threadsPerBlock.x * DYNAMICS_T::OUTPUT_DIM);
  computeStateDerivTestKernel<DYNAMICS_T, S_DIM, C_DIM, BLOCKDIM_X><<<numBlocks, threadsPerBlock, shared_mem>>>(
      static_cast<DYNAMICS_T*>(dynamics.model_d_), state_d, control_d, state_der_d, count);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(state.data(), state_d, sizeof(float) * S_DIM * state.size(), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(state_der.data(), state_der_d, sizeof(float) * S_DIM * state_der.size(), cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(control.data(), control_d, sizeof(float) * C_DIM * control.size(), cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(state_der_d);
  cudaFree(control_d);
}

template <typename DYNAMICS_T>
__global__ void stepTestKernel(DYNAMICS_T* dynamics, float* state, float* control, float* state_der, float* next_state,
                               float* output, int t, float dt, int num)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;

  extern __shared__ float entire_buffer[];
  float* theta = entire_buffer;

  // float* theta = reinterpret_cast<float*>(theta_s4);
  float* x = state + (tid * DYNAMICS_T::STATE_DIM);
  float* x_dot = state_der + (tid * DYNAMICS_T::STATE_DIM);
  float* x_next = next_state + (tid * DYNAMICS_T::STATE_DIM);
  float* u = control + (tid * DYNAMICS_T::CONTROL_DIM);
  float* y = output + (tid * DYNAMICS_T::OUTPUT_DIM);

  if (tid < num)
  {
    dynamics->initializeDynamics(state, control, output, theta, 0.0f, dt);
  }
  __syncthreads();

  if (tid < num)
  {
    dynamics->enforceConstraints(x, u);
    dynamics->step(x, x_next, x_dot, u, y, theta, t, dt);
  }
}

template <typename DYNAMICS_T, int BLOCKDIM_X = 32>  // here for compatability
void launchStepTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, DYNAMICS_T::STATE_DIM>>& state,
                          std::vector<std::array<float, DYNAMICS_T::CONTROL_DIM>>& control,
                          std::vector<std::array<float, DYNAMICS_T::STATE_DIM>>& state_der,
                          std::vector<std::array<float, DYNAMICS_T::STATE_DIM>>& next_state, int t, float dt, int dim_y,
                          int dim_x = 32)
{
  std::vector<std::array<float, DYNAMICS_T::OUTPUT_DIM>> output(state.size());
  launchStepTestKernel(dynamics, state, control, state_der, next_state, output, t, dt, dim_y, dim_x);
}

template <typename DYNAMICS_T>
void launchStepTestKernel(DYNAMICS_T& dynamics, std::vector<std::array<float, DYNAMICS_T::STATE_DIM>>& state,
                          std::vector<std::array<float, DYNAMICS_T::CONTROL_DIM>>& control,
                          std::vector<std::array<float, DYNAMICS_T::STATE_DIM>>& state_der,
                          std::vector<std::array<float, DYNAMICS_T::STATE_DIM>>& next_state,
                          std::vector<std::array<float, DYNAMICS_T::OUTPUT_DIM>>& output, int t, float dt, int dim_y,
                          int dim_x)
{
  if (state.size() != control.size())
  {
    std::cerr << "Num States doesn't match num controls" << std::endl;
    return;
  }
  if (state.size() != state_der.size())
  {
    std::cerr << "Num States doesn't match num state_ders" << std::endl;
    return;
  }
  if (state.size() != next_state.size())
  {
    std::cerr << "Num States doesn't match num next_states" << std::endl;
    return;
  }
  if (state.size() % dim_x != 0)
  {
    std::cerr << "Num States needs be evenly divisible by dim_x: " << state.size()
              << " state queries, " << dim_x << " parallelizable threads per block" << std::endl;
    return;
  }
  int count = state.size();
  float* state_d;
  float* control_d;
  float* state_der_d;
  float* next_state_d;
  float* output_d;
  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * DYNAMICS_T::STATE_DIM * count));
  HANDLE_ERROR(cudaMalloc((void**)&state_der_d, sizeof(float) * DYNAMICS_T::STATE_DIM * count));
  HANDLE_ERROR(cudaMalloc((void**)&next_state_d, sizeof(float) * DYNAMICS_T::STATE_DIM * count));
  HANDLE_ERROR(cudaMalloc((void**)&control_d, sizeof(float) * DYNAMICS_T::CONTROL_DIM * count));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * DYNAMICS_T::OUTPUT_DIM * count));

  HANDLE_ERROR(
      cudaMemcpy(state_d, state.data(), sizeof(float) * DYNAMICS_T::STATE_DIM * count, cudaMemcpyHostToDevice));
  // HANDLE_ERROR(
  //     cudaMemcpy(state_der_d, state_der.data(), sizeof(float) * DYNAMICS_T::STATE_DIM * count,
  //     cudaMemcpyHostToDevice));
  // HANDLE_ERROR(cudaMemcpy(next_state_d, next_state.data(), sizeof(float) * DYNAMICS_T::STATE_DIM * count,
  //                         cudaMemcpyHostToDevice));
  HANDLE_ERROR(
      cudaMemcpy(control_d, control.data(), sizeof(float) * DYNAMICS_T::CONTROL_DIM * count, cudaMemcpyHostToDevice));

  const int gridsize_x = (count - 1) / dim_x + 1;
  dim3 threadsPerBlock(dim_x, dim_y);
  dim3 numBlocks(gridsize_x, 1);

  unsigned shared_mem = mppi::kernels::calcClassSharedMemSize(&dynamics, threadsPerBlock);
  stepTestKernel<DYNAMICS_T><<<numBlocks, threadsPerBlock, shared_mem>>>(
      dynamics.model_d_, state_d, control_d, state_der_d, next_state_d, output_d, t, dt, count);
  HANDLE_ERROR(cudaGetLastError());

  // Copy the memory back to the host
  HANDLE_ERROR(
      cudaMemcpy(state.data(), state_d, sizeof(float) * DYNAMICS_T::STATE_DIM * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(state_der.data(), state_der_d, sizeof(float) * DYNAMICS_T::STATE_DIM * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(next_state.data(), next_state_d, sizeof(float) * DYNAMICS_T::STATE_DIM * count,
                          cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(control.data(), control_d, sizeof(float) * DYNAMICS_T::CONTROL_DIM * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(output.data(), output_d, sizeof(float) * DYNAMICS_T::OUTPUT_DIM * count, cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(state_der_d);
  cudaFree(control_d);
  cudaFree(next_state_d);
  cudaFree(output_d);
}

template <class DYN_T>
void checkGPUComputationStep(DYN_T& dynamics, float dt, int max_y_dim, int x_dim,
                             typename DYN_T::buffer_trajectory buffer, double tol)
{
  CudaCheckError();
  dynamics.GPUSetup();
  CudaCheckError();

  const int num_points = 1024;
  Eigen::Matrix<float, DYN_T::CONTROL_DIM, num_points> control_trajectory;
  control_trajectory = Eigen::Matrix<float, DYN_T::CONTROL_DIM, num_points>::Random();
  Eigen::Matrix<float, DYN_T::STATE_DIM, num_points> state_trajectory;
  state_trajectory = Eigen::Matrix<float, DYN_T::STATE_DIM, num_points>::Random();

  std::vector<std::array<float, DYN_T::STATE_DIM>> s(num_points);
  std::vector<std::array<float, DYN_T::STATE_DIM>> s_der(num_points);
  std::vector<std::array<float, DYN_T::STATE_DIM>> s_next(num_points);
  std::vector<std::array<float, DYN_T::OUTPUT_DIM>> output(num_points);
  // steering, throttle
  std::vector<std::array<float, DYN_T::CONTROL_DIM>> u(num_points);
  for (int state_index = 0; state_index < s.size(); state_index++)
  {
    for (int dim = 0; dim < s[0].size(); dim++)
    {
      s[state_index][dim] = state_trajectory.col(state_index)(dim);
    }
    for (int dim = 0; dim < u[0].size(); dim++)
    {
      u[state_index][dim] = control_trajectory.col(state_index)(dim);
    }
  }

  // Run dynamics on GPU
  for (int y_dim = 1; y_dim <= max_y_dim; y_dim++)
  {
    if (dynamics.checkRequiresBuffer())
    {
      dynamics.updateFromBuffer(buffer);
    }
    launchStepTestKernel<DYN_T>(dynamics, s, u, s_der, s_next, output, 0, dt, y_dim, x_dim);
    for (int point = 0; point < num_points; point++)
    {
      typename DYN_T::state_array state = state_trajectory.col(point);
      typename DYN_T::state_array next_state = DYN_T::state_array::Zero();
      typename DYN_T::control_array control = control_trajectory.col(point);
      typename DYN_T::state_array state_der_cpu = DYN_T::state_array::Zero();
      typename DYN_T::output_array output_array_cpu = DYN_T::output_array::Zero();
      dynamics.initializeDynamics(state, control, output_array_cpu, 0, dt);

      dynamics.step(state, next_state, state_der_cpu, control, output_array_cpu, 0.0f, dt);

      for (int dim = 0; dim < DYN_T::STATE_DIM; dim++)
      {
        EXPECT_NEAR(state(dim), s[point][dim], tol)
            << "at sample " << point << ", state dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s[point][dim]));
      }
      for (int dim = 0; dim < DYN_T::CONTROL_DIM; dim++)
      {
        EXPECT_NEAR(control(dim), u[point][dim], tol)
            << "at sample " << point << ", control dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(u[point][dim]));
      }
      for (int dim = 0; dim < DYN_T::STATE_DIM; dim++)
      {
        EXPECT_NEAR(state_der_cpu(dim), s_der[point][dim], tol)
            << "at sample " << point << ", state deriv dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s_der[point][dim]));
      }
      for (int dim = 0; dim < DYN_T::STATE_DIM; dim++)
      {
        EXPECT_NEAR(next_state(dim), s_next[point][dim], tol)
            << "at sample " << point << ", next state dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s[point][dim]));
      }
      for (int dim = 0; dim < DYN_T::OUTPUT_DIM; dim++)
      {
        if (isnan(output_array_cpu(dim)) && isnan(output[point][dim]))
        {
          continue;
        }
        EXPECT_NEAR(output_array_cpu(dim), output[point][dim], tol * 1000)  // TODO this is a stupid hack
            << "at sample " << point << ", output dim: " << dim << " with y_dim " << y_dim;
        EXPECT_TRUE(isfinite(s_der[point][dim]));
      }
    }
  }

  dynamics.freeCudaMem();
}
