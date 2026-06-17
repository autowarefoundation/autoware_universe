//
// Created by jason on 8/19/22.
//

#ifndef MPPIGENERIC_NETWORK_HELPER_KERNEL_TEST_CUH
#define MPPIGENERIC_NETWORK_HELPER_KERNEL_TEST_CUH

#include <mppi/utils/math_utils.h>
#include <mppi/core/mppi_common.cuh>

// TODO check on multiple different blocks

template <class NETWORK_T>
__global__ void parameterCheckTestKernel(NETWORK_T* model, float* theta, int* stride, int* net_structure,
                                         float* shared_theta, int* shared_stride, int* shared_net_structure)
{
  extern __shared__ float theta_s[];
  model->initialize(theta_s);

  float* theta_shared_ptr = theta_s;
  int* stride_shared_ptr = (int*)(theta_s + model->getNumParams());
  int* structure_shared_ptr = (int*)(theta_s + model->getNumParams() + model->getStrideSize());

  for (int i = 0; i < model->getNumParams(); i++)
  {
    theta[i] = model->getThetaPtr()[i];
    shared_theta[i] = theta_shared_ptr[i];
  }
  for (int i = 0; i < model->getStrideSize(); i++)
  {
    stride[i] = model->getStrideIdcsPtr()[i];
    shared_stride[i] = stride_shared_ptr[i];
  }
  for (int i = 0; i < model->getNumLayers(); i++)
  {
    net_structure[i] = model->getNetStructurePtr()[i];
    shared_net_structure[i] = structure_shared_ptr[i];
  }
}

template <class NETWORK_T, int THETA_SIZE, int STRIDE_SIZE, int NUM_LAYERS>
void launchParameterCheckTestKernel(NETWORK_T& model, std::array<float, THETA_SIZE>& theta,
                                    std::array<int, STRIDE_SIZE>& stride, std::array<int, NUM_LAYERS>& net_structure,
                                    std::array<float, THETA_SIZE>& shared_theta,
                                    std::array<int, STRIDE_SIZE>& shared_stride,
                                    std::array<int, NUM_LAYERS>& shared_net_structure)
{
  float* theta_d;
  int* stride_d;
  int* net_structure_d;

  float* shared_theta_d;
  int* shared_stride_d;
  int* shared_net_structure_d;

  HANDLE_ERROR(cudaMalloc((void**)&theta_d, sizeof(float) * theta.size()))
  HANDLE_ERROR(cudaMalloc((void**)&stride_d, sizeof(int) * stride.size()))
  HANDLE_ERROR(cudaMalloc((void**)&net_structure_d, sizeof(int) * net_structure.size()))
  HANDLE_ERROR(cudaMalloc((void**)&shared_theta_d, sizeof(float) * theta.size()))
  HANDLE_ERROR(cudaMalloc((void**)&shared_stride_d, sizeof(int) * stride.size()))
  HANDLE_ERROR(cudaMalloc((void**)&shared_net_structure_d, sizeof(int) * net_structure.size()))

  dim3 threadsPerBlock(3, 1);
  dim3 numBlocks(10, 1);
  unsigned shared_size = mppi::kernels::calcClassSharedMemSize(&model, threadsPerBlock);
  parameterCheckTestKernel<NETWORK_T><<<numBlocks, threadsPerBlock, shared_size>>>(
      model.network_d_, theta_d, stride_d, net_structure_d, shared_theta_d, shared_stride_d, shared_net_structure_d);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(theta.data(), theta_d, sizeof(float) * theta.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(stride.data(), stride_d, sizeof(int) * stride.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(
      cudaMemcpy(net_structure.data(), net_structure_d, sizeof(int) * net_structure.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(shared_theta.data(), shared_theta_d, sizeof(float) * theta.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(shared_stride.data(), shared_stride_d, sizeof(int) * stride.size(), cudaMemcpyDeviceToHost))
  HANDLE_ERROR(cudaMemcpy(shared_net_structure.data(), shared_net_structure_d, sizeof(int) * net_structure.size(),
                          cudaMemcpyDeviceToHost))
  cudaDeviceSynchronize();

  cudaFree(theta_d);
  cudaFree(stride_d);
  cudaFree(net_structure_d);
  cudaFree(shared_theta_d);
  cudaFree(shared_stride_d);
  cudaFree(shared_net_structure_d);
}

template <class NETWORK_T>
__global__ void parameterCheckTestKernel(NETWORK_T* model, float* lstm_params, float* shared_lstm_params,
                                         float* fnn_params, float* shared_fnn_params)
{
  extern __shared__ float theta_s[];
  model->initialize(theta_s);
  uint tid = blockIdx.x;

  float* lstm_params_start =
      lstm_params + tid * (model->getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model->getHiddenDim());
  memcpy(lstm_params_start, model->getWeights(),
         model->getLSTMGrdSharedSizeBytes() + 2 * model->getHiddenDim() * sizeof(float));

  float* fnn_params_start = fnn_params + tid * model->getOutputGrdSharedSizeBytes() / sizeof(float);
  memcpy(fnn_params_start, model->getOutputWeights(), model->getOutputGrdSharedSizeBytes());

  float* shared_lstm_params_start =
      shared_lstm_params + tid * (model->getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model->getHiddenDim());
  memcpy(shared_lstm_params_start, theta_s, model->getLSTMGrdSharedSizeBytes());
  memcpy(shared_lstm_params_start + model->getLSTMGrdSharedSizeBytes() / sizeof(float),
         theta_s + model->getGrdSharedSizeBytes() / sizeof(float), model->getHiddenDim() * 2 * sizeof(float));

  float* shared_fnn_params_start = shared_fnn_params + tid * model->getOutputGrdSharedSizeBytes() / sizeof(float);
  memcpy(shared_fnn_params_start, theta_s + model->getLSTMGrdSharedSizeBytes() / sizeof(float),
         model->getOutputGrdSharedSizeBytes());
}

template <class NETWORK_T>
void launchParameterCheckTestKernel(NETWORK_T& model, std::vector<float>& lstm_params,
                                    std::vector<float>& shared_lstm_params, std::vector<float>& fnn_params,
                                    std::vector<float>& shared_fnn_params, int num)
{
  lstm_params.resize((model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim()) * num);
  shared_lstm_params.resize((model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim()) * num);
  fnn_params.resize(model.getOutputGrdSharedSizeBytes() / sizeof(float) * num);
  shared_fnn_params.resize(model.getOutputGrdSharedSizeBytes() / sizeof(float) * num);

  std::fill(lstm_params.begin(), lstm_params.end(), -1);
  std::fill(shared_lstm_params.begin(), shared_lstm_params.end(), -1);
  std::fill(fnn_params.begin(), fnn_params.end(), -2);
  std::fill(shared_fnn_params.begin(), shared_fnn_params.end(), -2);

  float* lstm_params_d = nullptr;
  float* shared_lstm_params_d = nullptr;
  float* fnn_params_d = nullptr;
  float* shared_fnn_params_d = nullptr;

  HANDLE_ERROR(cudaMalloc((void**)&lstm_params_d,
                          (model.getLSTMGrdSharedSizeBytes() + 2 * model.getHiddenDim() * sizeof(float)) * num));
  HANDLE_ERROR(cudaMalloc((void**)&shared_lstm_params_d,
                          (model.getLSTMGrdSharedSizeBytes() + 2 * model.getHiddenDim() * sizeof(float)) * num));
  HANDLE_ERROR(cudaMalloc((void**)&fnn_params_d, model.getOutputGrdSharedSizeBytes() * num));
  HANDLE_ERROR(cudaMalloc((void**)&shared_fnn_params_d, model.getOutputGrdSharedSizeBytes() * num));

  dim3 threadsPerBlock(1, 1);
  dim3 numBlocks(num, 1);
  unsigned shared_size = mppi::kernels::calcClassSharedMemSize(&model, threadsPerBlock);
  parameterCheckTestKernel<NETWORK_T><<<numBlocks, threadsPerBlock, shared_size>>>(
      model.network_d_, lstm_params_d, shared_lstm_params_d, fnn_params_d, shared_fnn_params_d);
  CudaCheckError();

  HANDLE_ERROR(cudaMemcpy(lstm_params.data(), lstm_params_d,
                          (model.getLSTMGrdSharedSizeBytes() + 2 * model.getHiddenDim() * sizeof(float)) * num,
                          cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(shared_lstm_params.data(), shared_lstm_params_d,
                          (model.getLSTMGrdSharedSizeBytes() + 2 * model.getHiddenDim() * sizeof(float)) * num,
                          cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(fnn_params.data(), fnn_params_d, model.getOutputGrdSharedSizeBytes() * num, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(shared_fnn_params.data(), shared_fnn_params_d, model.getOutputGrdSharedSizeBytes() * num,
                          cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(lstm_params_d);
  cudaFree(shared_lstm_params_d);
  cudaFree(fnn_params_d);
  cudaFree(shared_fnn_params_d);
}

template <typename NETWORK_T, int BLOCKSIZE_X>
__global__ void forwardTestKernel(NETWORK_T* network, float* input, float* output, int num, int steps)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  extern __shared__ float theta_s[];
  float* local_input = input + (tid * network->getInputDim());
  float* local_output = output + (tid * network->getOutputDim());

  network->initialize(theta_s);

  if (tid < num)
  {
    float* curr_act = nullptr;
    for (uint step = 0; step < steps; step++)
    {
      curr_act = network->forward(local_input, theta_s);
    }
    for (uint i = threadIdx.y; i < network->getOutputDim(); i += blockDim.y)
    {
      local_output[i] = curr_act[i];
    }
    __syncthreads();
  }
}

template <typename NETWORK_T, int INPUT_DIM, int OUTPUT_DIM, int BLOCKSIZE_X>
void launchForwardTestKernel(NETWORK_T& model, std::vector<std::array<float, INPUT_DIM>>& input,
                             std::vector<std::array<float, OUTPUT_DIM>>& output, int dim_y, int steps = 1)
{
  if (input.size() != output.size())
  {
    std::cerr << "Num States doesn't match num controls" << std::endl;
    return;
  }
  int count = input.size();
  float* input_d;
  float* output_d;
  HANDLE_ERROR(cudaMalloc((void**)&input_d, sizeof(float) * model.getInputDim() * count));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * model.getOutputDim() * count));

  HANDLE_ERROR(cudaMemcpy(input_d, input.data(), sizeof(float) * model.getInputDim() * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(
      cudaMemcpy(output_d, input.data(), sizeof(float) * model.getOutputDim() * count, cudaMemcpyHostToDevice));

  const int gridsize_x = (count - 1) / BLOCKSIZE_X + 1;
  dim3 threadsPerBlock(BLOCKSIZE_X, dim_y);
  dim3 numBlocks(gridsize_x, 1);
  unsigned shared_size = mppi::kernels::calcClassSharedMemSize(&model, threadsPerBlock);
  forwardTestKernel<NETWORK_T, BLOCKSIZE_X>
      <<<numBlocks, threadsPerBlock, shared_size>>>(model.network_d_, input_d, output_d, count, steps);
  CudaCheckError();

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(input.data(), input_d, sizeof(float) * model.getInputDim() * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(
      cudaMemcpy(output.data(), output_d, sizeof(float) * model.getOutputDim() * count, cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(input_d);
  cudaFree(output_d);
}

template <typename NETWORK_T, int BLOCKSIZE_X>
__global__ void forwardTestKernelPreload(NETWORK_T* network, float* input, float* output, int num, int steps)
{
  int tid = blockIdx.x * blockDim.x + threadIdx.x;
  extern __shared__ float theta_s[];
  float* local_input = input + (tid * network->getInputDim());
  float* local_output = output + (tid * network->getOutputDim());

  network->initialize(theta_s);

  if (tid < num)
  {
    float* input_loc = network->getInputLocation(theta_s);
    for (int i = threadIdx.y; i < network->getInputDim(); i += blockDim.y)
    {
      input_loc[i] = local_input[i];
    }
    __syncthreads();

    float* curr_act = nullptr;
    for (uint step = 0; step < steps; step++)
    {
      curr_act = network->forward(nullptr, theta_s);
    }
    for (uint i = threadIdx.y; i < network->getOutputDim(); i += blockDim.y)
    {
      local_output[i] = curr_act[i];
    }
    __syncthreads();
  }
}

template <typename NETWORK_T, int INPUT_DIM, int OUTPUT_DIM, int BLOCKSIZE_X>
void launchForwardTestKernelPreload(NETWORK_T& model, std::vector<std::array<float, INPUT_DIM>>& input,
                                    std::vector<std::array<float, OUTPUT_DIM>>& output, int dim_y, int steps = 1)
{
  if (input.size() != output.size())
  {
    std::cerr << "Num States doesn't match num controls" << std::endl;
    return;
  }
  int count = input.size();
  float* input_d;
  float* output_d;
  HANDLE_ERROR(cudaMalloc((void**)&input_d, sizeof(float) * INPUT_DIM * count));
  HANDLE_ERROR(cudaMalloc((void**)&output_d, sizeof(float) * OUTPUT_DIM * count));

  HANDLE_ERROR(cudaMemcpy(input_d, input.data(), sizeof(float) * INPUT_DIM * count, cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(output_d, input.data(), sizeof(float) * OUTPUT_DIM * count, cudaMemcpyHostToDevice));

  const int gridsize_x = (count - 1) / BLOCKSIZE_X + 1;
  dim3 threadsPerBlock(BLOCKSIZE_X, dim_y);
  dim3 numBlocks(gridsize_x, 1);
  unsigned shared_size = mppi::kernels::calcClassSharedMemSize(&model, threadsPerBlock);
  forwardTestKernelPreload<NETWORK_T, BLOCKSIZE_X>
      <<<numBlocks, threadsPerBlock, shared_size>>>(model.network_d_, input_d, output_d, count, steps);
  CudaCheckError();

  // Copy the memory back to the host
  HANDLE_ERROR(cudaMemcpy(input.data(), input_d, sizeof(float) * INPUT_DIM * count, cudaMemcpyDeviceToHost));
  HANDLE_ERROR(cudaMemcpy(output.data(), output_d, sizeof(float) * OUTPUT_DIM * count, cudaMemcpyDeviceToHost));
  cudaDeviceSynchronize();

  cudaFree(input_d);
  cudaFree(output_d);
}

#endif  // MPPIGENERIC_NETWORK_HELPER_KERNEL_TEST_CUH
