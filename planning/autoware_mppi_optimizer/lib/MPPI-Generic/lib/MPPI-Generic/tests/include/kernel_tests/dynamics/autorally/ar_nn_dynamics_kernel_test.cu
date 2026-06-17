template <class NETWORK_T, int THETA_SIZE, int STRIDE_SIZE, int NUM_LAYERS>
__global__ void parameterCheckTestKernel(NETWORK_T* model, float* theta, int* stride, int* net_structure)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  if (tid == 0)
  {
    for (int i = 0; i < THETA_SIZE; i++)
    {
      theta[i] = model->getThetaPtr()[i];
    }
    for (int i = 0; i < STRIDE_SIZE; i++)
    {
      stride[i] = model->getStrideIdcsPtr()[i];
    }
    for (int i = 0; i < NUM_LAYERS; i++)
    {
      net_structure[i] = model->getNetStructurePtr()[i];
    }
  }
}

template <class NETWORK_T, int THETA_SIZE, int STRIDE_SIZE, int NUM_LAYERS>
void launchParameterCheckTestKernel(NETWORK_T& model, std::array<float, THETA_SIZE>& theta,
                                    std::array<int, STRIDE_SIZE>& stride, std::array<int, NUM_LAYERS>& net_structure)
{
  float* theta_d;
  int* stride_d;
  int* net_structure_d;

  HANDLE_ERROR(cudaMalloc((void**)&theta_d, sizeof(float) * theta.size()))
  HANDLE_ERROR(cudaMalloc((void**)&stride_d, sizeof(int) * stride.size()))
  HANDLE_ERROR(cudaMalloc((void**)&net_structure_d, sizeof(int) * net_structure.size()))

  dim3 threadsPerBlock(1, 1);
  dim3 numBlocks(1, 1);
  parameterCheckTestKernel<NETWORK_T, THETA_SIZE, STRIDE_SIZE, NUM_LAYERS>
      <<<numBlocks, threadsPerBlock>>>(model.model_d_, theta_d, stride_d, net_structure_d);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(theta.data(), theta_d, sizeof(float) * theta.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(stride.data(), stride_d, sizeof(int) * stride.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(
      cudaMemcpy(net_structure.data(), net_structure_d, sizeof(int) * net_structure.size(), cudaMemcpyDeviceToHost))
  cudaDeviceSynchronize();

  cudaFree(theta_d);
  cudaFree(stride_d);
  cudaFree(net_structure_d);
}

template <class NETWORK_T, int S_DIM, int C_DIM, int BLOCKDIM_X, int BLOCKDIM_Z>
__global__ void fullARNNTestKernel(NETWORK_T* model, float* state, float* control, float* state_der, float dt)
{
  __shared__ float theta[NETWORK_T::SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float) + 1 +
                         NETWORK_T::SHARED_MEM_REQUEST_BLK_BYTES * BLOCKDIM_X * BLOCKDIM_Z];
  __shared__ float output[S_DIM * BLOCKDIM_X * BLOCKDIM_Z];
  __shared__ float s_der[S_DIM * BLOCKDIM_X * BLOCKDIM_Z];

  model->initializeDynamics(state, control, output, theta, 0.0f, 0.0f);

  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  // calls enforce constraints -> compute state derivative -> increment state

  // printf("before enforceConstraints %d, %d\n", threadIdx.x, threadIdx.y);
  // printf("enforceConstraints %d, %d\n", threadIdx.x, threadIdx.y);
  // printf("before enforce Constraints %f, %f\n", (control+(tid*C_DIM))[0], (control+(tid*C_DIM))[1]);
  model->enforceConstraints(state + (tid * S_DIM), control + (tid * C_DIM));
  __syncthreads();
  // printf("after enforce Constraints %f, %f\n", (control+(tid*C_DIM))[0], (control+(tid*C_DIM))[1]);
  model->computeStateDeriv(state + (tid * S_DIM), control + (tid * C_DIM), state_der + (tid * S_DIM), theta);
  __syncthreads();

  int shared_indexer = BLOCKDIM_X * threadIdx.z + threadIdx.x;
  for (int i = threadIdx.y; i < S_DIM; i += blockDim.y)
  {
    // printf("index for shared %d, %d\n", S_DIM*shared_indexer + i, S_DIM*tid+i);
    s_der[S_DIM * shared_indexer + i] = state_der[S_DIM * tid + i];
  }
  __syncthreads();

  // if(threadIdx.y == 0) {
  // printf("state_der = %f, %f, %f, %f, %f, %f, %f\n", state_der[0], state_der[1], state_der[2], state_der[3],
  // state_der[4], state_der[5], state_der[6]); printf("state = %f, %f, %f, %f, %f, %f, %f\n", state[0], state[1],
  // state[2], state[3], state[4], state[5], state[6]); printf("s_der = %f, %f, %f, %f, %f, %f, %f\n", s_der[0],
  // s_der[1], s_der[2], s_der[3], s_der[4], s_der[5], s_der[6]);
  //}

  model->updateState(state + (tid * S_DIM), s_der + (shared_indexer * S_DIM), dt);
}

template <class NETWORK_T, int S_DIM, int C_DIM, int BLOCKDIM_X = 1, int BLOCKDIM_Z = 1>
void launchFullARNNTestKernel(NETWORK_T& model, std::vector<std::array<float, S_DIM>>& state,
                              std::vector<std::array<float, C_DIM>>& control,
                              std::vector<std::array<float, S_DIM>>& state_der, float dt, int dim_y)
{
  float* state_d;
  float* state_der_d;
  float* control_d;

  HANDLE_ERROR(cudaMalloc((void**)&state_d, sizeof(float) * S_DIM * state.size() * BLOCKDIM_Z))
  HANDLE_ERROR(cudaMalloc((void**)&state_der_d, sizeof(float) * S_DIM * state_der.size() * BLOCKDIM_Z))
  HANDLE_ERROR(cudaMalloc((void**)&control_d, sizeof(float) * C_DIM * control.size() * BLOCKDIM_Z))

  HANDLE_ERROR(cudaMemcpy(state_d, state.data(), sizeof(float) * S_DIM * state.size(), cudaMemcpyHostToDevice))
  HANDLE_ERROR(
      cudaMemcpy(state_der_d, state_der.data(), sizeof(float) * S_DIM * state_der.size(), cudaMemcpyHostToDevice))
  HANDLE_ERROR(cudaMemcpy(control_d, control.data(), sizeof(float) * C_DIM * control.size(), cudaMemcpyHostToDevice))

  // make sure you cannot use invalid inputs
  dim3 threadsPerBlock(state.size(), dim_y, BLOCKDIM_Z);
  dim3 numBlocks(1, 1);
  // launch kernel
  fullARNNTestKernel<NETWORK_T, S_DIM, C_DIM, BLOCKDIM_X, BLOCKDIM_Z>
      <<<numBlocks, threadsPerBlock>>>(model.model_d_, state_d, control_d, state_der_d, dt);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(state.data(), state_d, sizeof(float) * S_DIM * state.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(
      cudaMemcpy(state_der.data(), state_der_d, sizeof(float) * S_DIM * state_der.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(control.data(), control_d, sizeof(float) * C_DIM * control.size(), cudaMemcpyDeviceToHost))
  cudaDeviceSynchronize();

  cudaFree(state_d);
  cudaFree(state_der_d);
  cudaFree(control_d);
}
