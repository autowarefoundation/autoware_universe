//
// Created by jason on 8/18/22.
//

#include "fnn_helper.cuh"
template <bool USE_SHARED>
FNNHelper<USE_SHARED>::FNNHelper(const std::vector<int>& layers, cudaStream_t stream) : Managed(stream)
{
  setupMemory(layers);
}

template <bool USE_SHARED>
FNNHelper<USE_SHARED>::FNNHelper(std::string model_path, cudaStream_t stream) : Managed(stream)
{
  loadParams(model_path);
}

template <bool USE_SHARED>
FNNHelper<USE_SHARED>::FNNHelper(const cnpy::npz_t& param_dict, cudaStream_t stream) : Managed(stream)
{
  loadParams(param_dict);
}

template <bool USE_SHARED>
FNNHelper<USE_SHARED>::FNNHelper(const cnpy::npz_t& param_dict, std::string prefix, cudaStream_t stream)
  : Managed(stream)
{
  loadParams(prefix, param_dict);
}

template <bool USE_SHARED>
FNNHelper<USE_SHARED>::~FNNHelper()
{
  if (this->GPUMemStatus_)
  {
    freeCudaMem();
  }
  delete theta_;
  delete stride_idcs_;
  delete net_structure_;
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::loadParams(const std::string& model_path)
{
  int i, j, k;
  std::string bias_name = "";
  std::string weight_name = "";
  if (!fileExists(model_path))
  {
    std::cerr << "Could not load neural net model at path: " << model_path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(model_path);
  loadParams(param_dict);
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::loadParams(const cnpy::npz_t& param_dict)
{
  loadParams("", param_dict);
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::loadParams(std::string prefix, const cnpy::npz_t& param_dict, bool add_slash)
{
  int i, j, k;
  std::string bias_name = "";
  std::string weight_name = "";

  if (add_slash && !prefix.empty() && *prefix.rbegin() != '/')
  {
    prefix.append("/");
  }

  int counter = 1;
  bias_name = prefix + "dynamics_b" + std::to_string(counter);
  weight_name = prefix + "dynamics_W" + std::to_string(counter);

  assert(static_cast<int>(param_dict.at(weight_name).num_vals) % static_cast<int>(param_dict.at(bias_name).num_vals) ==
         0);

  std::vector<int> layers = { static_cast<int>(param_dict.at(weight_name).num_vals) /
                              static_cast<int>(param_dict.at(bias_name).num_vals) };
  while (param_dict.find(bias_name) != param_dict.end())
  {
    layers.push_back(static_cast<int>(param_dict.at(bias_name).num_vals));
    counter += 1;
    bias_name = prefix + "dynamics_b" + std::to_string(counter);
  }
  // TODO more asserts for proper sizing of weights
  // TODO only setup memory if it is different than stored
  setupMemory(layers);

  for (i = 0; i < NUM_LAYERS - 1; i++)
  {
    // NN index from 1
    bias_name = prefix + "dynamics_b" + std::to_string(i + 1);
    weight_name = prefix + "dynamics_W" + std::to_string(i + 1);

    cnpy::NpyArray weight_i_raw = param_dict.at(weight_name);
    cnpy::NpyArray bias_i_raw = param_dict.at(bias_name);
    double* weight_i = weight_i_raw.data<double>();
    double* bias_i = bias_i_raw.data<double>();

    // copy over the weights
    for (j = 0; j < net_structure_[i + 1]; j++)
    {
      for (k = 0; k < net_structure_[i]; k++)
      {
        theta_[stride_idcs_[2 * i] + j * net_structure_[i] + k] = weight_i[j * net_structure_[i] + k];
      }
    }
    // copy over the bias
    for (j = 0; j < net_structure_[i + 1]; j++)
    {
      theta_[stride_idcs_[2 * i + 1] + j] = bias_i[j];
    }
  }
  for (int i = 0; i < NUM_PARAMS; i++)
  {
    assert(isfinite(theta_[i]));
  }
  changed_weights_ = true;
  // Save parameters to GPU memory
  paramsToDevice();
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::setupMemory(const std::vector<int>& layers)
{
  assert(layers.size() >= 2);

  bool setupGPU = this->GPUMemStatus_;
  // TODO should see if this is different or not
  if (setupGPU)
  {
    freeCudaMem();
  }

  NUM_LAYERS = layers.size();
  LARGEST_LAYER = layers[0];
  STRIDE_SIZE = (NUM_LAYERS - 1) * 2;
  for (int i = 1; i < layers.size(); i++)
  {
    // the +1 here is from the bias
    NUM_PARAMS += (layers[i - 1] + 1) * layers[i];
    if (layers[i] > LARGEST_LAYER)
    {
      LARGEST_LAYER = layers[i];
    }
  }
  INPUT_DIM = layers.front();
  OUTPUT_DIM = layers.back();
  // TODO allocate more memory so we can copy as float4's
  if (theta_ != nullptr)
  {
    delete theta_;
    delete net_structure_;
    delete stride_idcs_;
  }

  theta_ = (float*)::operator new(sizeof(float) * NUM_PARAMS);
  net_structure_ = (int*)::operator new(sizeof(int) * NUM_LAYERS);
  stride_idcs_ = (int*)::operator new(sizeof(int) * STRIDE_SIZE);

  memset(theta_, 0.0, sizeof(float) * NUM_PARAMS);
  memset(net_structure_, -1, sizeof(int) * NUM_LAYERS);
  memset(stride_idcs_, -1, sizeof(int) * STRIDE_SIZE);

  for (int i = 0; i < NUM_LAYERS; i++)
  {
    net_structure_[i] = layers[i];
  }

  int stride = 0;
  for (int i = 0; i < NUM_LAYERS - 1; i++)
  {
    stride_idcs_[2 * i] = stride;
    stride += net_structure_[i + 1] * net_structure_[i];
    stride_idcs_[2 * i + 1] = stride;
    stride += net_structure_[i + 1];
  }

  PARAM_SIZE = (NUM_LAYERS + NUM_PARAMS + STRIDE_SIZE) * sizeof(float);

  this->SHARED_MEM_REQUEST_GRD_BYTES = mppi::math::int_multiple_const(PARAM_SIZE * USE_SHARED, sizeof(float4));
  this->SHARED_MEM_REQUEST_BLK_BYTES =
      mppi::math::int_multiple_const(2 * LARGEST_LAYER * sizeof(float), sizeof(float4));

  weighted_in_.resize(NUM_LAYERS - 1);

  // zeros out every matrix
  for (int i = 1; i < NUM_LAYERS; i++)
  {
    weighted_in_[i - 1] = Eigen::MatrixXf::Zero(net_structure_[i], 1);
  }

  if (setupGPU)
  {
    GPUSetup();
  }
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::updateModel(const std::vector<float>& data)
{
  assert(data.size() == NUM_PARAMS);
  std::vector<int> description(NUM_LAYERS);
  for (int i = 0; i < NUM_LAYERS; i++)
  {
    description[i] = net_structure_[i];
  }
  updateModel(description, data);
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::updateModel(const std::vector<int>& description, const std::vector<float>& data)
{
  // TODO can change the description of the network
  for (int i = 0; i < description.size(); i++)
  {
    if (description[i] != net_structure_[i])
    {
      throw std::invalid_argument("Invalid model trying to to be set for NN");
    }
  }
  for (int i = 0; i < NUM_LAYERS - 1; i++)
  {
    for (int j = 0; j < net_structure_[i + 1]; j++)
    {
      for (int k = 0; k < net_structure_[i]; k++)
      {
        theta_[stride_idcs_[2 * i] + j * net_structure_[i] + k] = data[stride_idcs_[2 * i] + j * net_structure_[i] + k];
      }
    }
  }
  for (int i = 0; i < NUM_LAYERS - 1; i++)
  {
    for (int j = 0; j < net_structure_[i + 1]; j++)
    {
      theta_[stride_idcs_[2 * i + 1] + j] = data[stride_idcs_[2 * i + 1] + j];
    }
  }
  for (int i = 0; i < NUM_PARAMS; i++)
  {
    assert(isfinite(theta_[i]));
  }
  changed_weights_ = true;
  if (this->GPUMemStatus_)
  {
    paramsToDevice();
  }
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::freeCudaMem()
{
  if (this->GPUMemStatus_)
  {
    HANDLE_ERROR(cudaFree(weights_d_));
    HANDLE_ERROR(cudaFree(network_d_));
    this->GPUMemStatus_ = false;
  }
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::GPUSetup()
{
  if (!this->GPUMemStatus_)
  {
    network_d_ = Managed::GPUSetup<FNNHelper<USE_SHARED>>(this);
    HANDLE_ERROR(cudaMalloc((void**)&(this->weights_d_), PARAM_SIZE));

    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->weights_d_), &(weights_d_), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->theta_), &(weights_d_), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    float* incr_ptr = (weights_d_ + NUM_PARAMS);
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->stride_idcs_), &(incr_ptr), sizeof(int*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr = (weights_d_ + NUM_PARAMS + STRIDE_SIZE);
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->net_structure_), &(incr_ptr), sizeof(int*), cudaMemcpyHostToDevice,
                                 this->stream_));
    changed_weights_ = true;
    paramsToDevice();
  }
  else
  {
    this->logger_->debug("FNN GPU Memory Already set\n");
  }
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::paramsToDevice()
{
  if (this->GPUMemStatus_ && changed_weights_)
  {
    HANDLE_ERROR(
        cudaMemcpyAsync(this->weights_d_, theta_, NUM_PARAMS * sizeof(float), cudaMemcpyHostToDevice, this->stream_));
    float* incr_ptr = weights_d_ + NUM_PARAMS;
    HANDLE_ERROR(
        cudaMemcpyAsync(incr_ptr, stride_idcs_, STRIDE_SIZE * sizeof(int), cudaMemcpyHostToDevice, this->stream_));
    incr_ptr = weights_d_ + NUM_PARAMS + STRIDE_SIZE;
    HANDLE_ERROR(
        cudaMemcpyAsync(incr_ptr, net_structure_, NUM_LAYERS * sizeof(int), cudaMemcpyHostToDevice, this->stream_));
    changed_weights_ = false;
  }
}

template <bool USE_SHARED>
bool FNNHelper<USE_SHARED>::computeGrad(const Eigen::Ref<const Eigen::MatrixXf>& input, Eigen::Ref<Eigen::MatrixXf> A)
{
  // compute forward to see gradient values
  Eigen::VectorXf output = getZeroOutputVector();
  forward(input, output);
  return computeGrad(A);
}

template <bool USE_SHARED>
bool FNNHelper<USE_SHARED>::computeGrad(Eigen::Ref<Eigen::MatrixXf> A)
{
  // Start backprop
  Eigen::MatrixXf ip_delta = Eigen::MatrixXf::Identity(OUTPUT_DIM, OUTPUT_DIM);

  // Main backprop loop
  for (int i = NUM_LAYERS - 2; i > 0; i--)
  {
    Eigen::MatrixXf zp = weighted_in_[i - 1];
    for (int j = 0; j < net_structure_[i]; j++)
    {
      zp(j) = mppi::nn::tanh_deriv(zp(j));
    }
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> cur_weights(
        theta_ + stride_idcs_[2 * i], net_structure_[i + 1], net_structure_[i]);
    ip_delta = ((cur_weights).transpose() * ip_delta).eval();
    for (int j = 0; j < OUTPUT_DIM; j++)
    {
      ip_delta.col(j) = ip_delta.col(j).array() * zp.array();
    }
  }
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> cur_weights(
      theta_ + stride_idcs_[0], net_structure_[1], net_structure_[0]);

  ip_delta = (((cur_weights).transpose()) * ip_delta).transpose().eval();
  for (int i = 0; i < INPUT_DIM; i++)
  {
    A.col(i) = ip_delta.col(i);
  }
  return true;
}

template <bool USE_SHARED>
void FNNHelper<USE_SHARED>::forward(const Eigen::Ref<const Eigen::MatrixXf>& input, Eigen::Ref<Eigen::MatrixXf> output)
{
  int i, j;
  Eigen::MatrixXf acts = input;
  for (i = 0; i < NUM_LAYERS - 1; i++)
  {
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> cur_weights(
        theta_ + stride_idcs_[2 * i], net_structure_[i + 1], net_structure_[i]);
    Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> cur_bias(
        theta_ + stride_idcs_[2 * i + 1], net_structure_[i + 1], 1);
    weighted_in_[i] = (cur_weights * acts + cur_bias).eval();
    acts = Eigen::MatrixXf::Zero(net_structure_[i + 1], 1);
    if (i < NUM_LAYERS - 2)
    {  // Last layer doesn't apply any non-linearity
      for (j = 0; j < net_structure_[i + 1]; j++)
      {
        acts(j) = mppi::nn::tanh((weighted_in_[i])(j));  // Nonlinear component.
      }
    }
    else
    {
      for (j = 0; j < net_structure_[i + 1]; j++)
      {
        acts(j) = (weighted_in_[i])(j);
      }
    }
  }
  output = acts;
}

template <bool USE_SHARED>
__device__ float* FNNHelper<USE_SHARED>::initialize(float* theta_s)
{
  float* new_theta_s = theta_s;
  if (USE_SHARED)
  {
    // if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0 && USE_SHARED != 0)
    // {
    //   memcpy(theta_s, theta_, NUM_PARAMS * sizeof(float));
    //   new_theta_s += NUM_PARAMS;
    //   memcpy(new_theta_s, stride_idcs_, STRIDE_SIZE * sizeof(int));
    //   new_theta_s += STRIDE_SIZE;
    //   memcpy(new_theta_s, net_structure_, NUM_LAYERS * sizeof(int));
    //   new_theta_s += NUM_LAYERS;
    // }
    for (int i = blockDim.x * threadIdx.y + threadIdx.x; i < NUM_PARAMS; i += blockDim.x * blockDim.y)
    {
      theta_s[i] = theta_[i];
    }
    int* stride_idcs = (int*)(theta_s + NUM_PARAMS);
    for (int i = threadIdx.y; i < STRIDE_SIZE; i += blockDim.y)
    {
      stride_idcs[i] = stride_idcs_[i];
    }
    int* net_structure = (int*)(theta_s + NUM_PARAMS + STRIDE_SIZE);
    for (int i = threadIdx.y; i < NUM_LAYERS; i += blockDim.y)
    {
      net_structure[i] = net_structure_[i];
    }
    __syncthreads();
  }
  return new_theta_s;
}

template <bool USE_SHARED>
__device__ float* FNNHelper<USE_SHARED>::forward(float* input, float* theta_s, float* curr_act)
{
  float* next_act;
  float* tmp_act;
  float tmp;
  float* W;
  float* b;
  uint tdy = threadIdx.y;
  uint i, j, k;
  uint tdx = threadIdx.x;
  uint tdz = threadIdx.z;

  float* theta = theta_;
  int* stride_idcs = stride_idcs_;
  int* net_structure = net_structure_;
  if (USE_SHARED != 0)
  {
    theta = theta_s;
    stride_idcs = (int*)(theta_s + NUM_PARAMS);
    net_structure = (int*)(theta_s + NUM_PARAMS + STRIDE_SIZE);
  }

  next_act = &curr_act[LARGEST_LAYER];

  // iterate through the part of the state that should be an input to the NN
  if (input != nullptr)
  {
    for (i = tdy; i < INPUT_DIM; i += blockDim.y)
    {
      curr_act[i] = input[i];
    }
    __syncthreads();
  }
  // iterate through each layer
  for (i = 0; i < NUM_LAYERS - 1; i++)
  {
    W = &theta[stride_idcs[2 * i]];      // weights
    b = &theta[stride_idcs[2 * i + 1]];  // biases

    // for first non input layer until last layer this thread deals with
    // calculates the next activation based on current
    for (j = tdy; j < net_structure[i + 1]; j += blockDim.y)
    {
      tmp = 0;
      // apply each neuron activation from current layer
      for (k = 0; k < net_structure[i]; k++)
      {
        // No atomic add necessary.
        tmp += W[j * net_structure[i] + k] * curr_act[k];
      }
      // add bias from next layer and neuron
      tmp += b[j];
      if (i < NUM_LAYERS - 2)
      {
        tmp = mppi::nn::tanh(tmp);
      }
      next_act[j] = tmp;
    }
    // Swap the two pointers
    tmp_act = curr_act;
    curr_act = next_act;
    next_act = tmp_act;
    __syncthreads();
  }
  return curr_act;
}

template <bool USE_SHARED>
__device__ float* FNNHelper<USE_SHARED>::forward(float* input, float* theta_s)
{
  uint tdx = threadIdx.x;
  uint tdz = threadIdx.z;
  float* curr_act =
      &theta_s[SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float) + (2 * LARGEST_LAYER) * (blockDim.x * tdz + tdx)];

  return forward(input, theta_s, curr_act);
}
template <bool USE_SHARED>
__device__ float* FNNHelper<USE_SHARED>::getInputLocation(float* theta_s)
{
  uint tdx = threadIdx.x;
  uint tdz = threadIdx.z;
  return theta_s + SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float) + (2 * LARGEST_LAYER) * (blockDim.x * tdz + tdx);
}
