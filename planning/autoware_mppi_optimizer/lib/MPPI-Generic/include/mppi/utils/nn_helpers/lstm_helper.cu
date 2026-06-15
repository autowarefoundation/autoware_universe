//
// Created by jason on 8/20/22.
//

#include "lstm_helper.cuh"

template <bool USE_SHARED>
LSTMHelper<USE_SHARED>::LSTMHelper(LSTMConfig& config, cudaStream_t stream)
  : LSTMHelper<USE_SHARED>(config.input_dim, config.hidden_dim, config.output_layers, stream)
{
}

template <bool USE_SHARED>
LSTMHelper<USE_SHARED>::LSTMHelper(int input_dim, int hidden_dim, std::vector<int>& output_layers, cudaStream_t stream)
  : Managed(stream)
{
  output_nn_ = new FNNHelper<USE_SHARED>(output_layers, this->stream_);
  setupMemory(input_dim, hidden_dim);
}

template <bool USE_SHARED>
LSTMHelper<USE_SHARED>::LSTMHelper(std::string path, cudaStream_t stream) : Managed(stream)
{
  loadParams(path);
}

template <bool USE_SHARED>
LSTMHelper<USE_SHARED>::LSTMHelper(const cnpy::npz_t& param_dict, std::string prefix, bool add_slash,
                                   cudaStream_t stream)
  : Managed(stream)
{
  loadParams(prefix, param_dict, add_slash);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::setupMemory(int input_dim, int hidden_dim)
{
  HIDDEN_DIM = hidden_dim;
  INPUT_DIM = input_dim;
  OUTPUT_DIM = output_nn_->getOutputDim();
  assert(output_nn_->getInputDim() == INPUT_DIM + HIDDEN_DIM);

  bool setupGPU = this->GPUMemStatus_;
  if (setupGPU)
  {
    freeCudaMem();
  }

  HIDDEN_HIDDEN_SIZE = HIDDEN_DIM * HIDDEN_DIM;
  INPUT_HIDDEN_SIZE = HIDDEN_DIM * INPUT_DIM;

  // The initial cell and hidden does not need to be stored in shared memory
  LSTM_PARAM_SIZE_BYTES = (HIDDEN_DIM * 4 + HIDDEN_HIDDEN_SIZE * 4 + INPUT_HIDDEN_SIZE * 4) * sizeof(float);
  LSTM_SHARED_MEM_GRD_BYTES = mppi::math::int_multiple_const(LSTM_PARAM_SIZE_BYTES * USE_SHARED, sizeof(float4));

  SHARED_MEM_REQUEST_GRD_BYTES = output_nn_->getGrdSharedSizeBytes() + LSTM_SHARED_MEM_GRD_BYTES;
  SHARED_MEM_REQUEST_BLK_BYTES = output_nn_->getBlkSharedSizeBytes() +
                                 mppi::math::int_multiple_const((2 * HIDDEN_DIM) * sizeof(float), sizeof(float4));

  hidden_state_ = Eigen::VectorXf(HIDDEN_DIM);
  hidden_state_.setZero();
  cell_state_ = Eigen::VectorXf(HIDDEN_DIM);
  cell_state_.setZero();

  if (weights_ != nullptr)
  {
    delete weights_;
  }

  // allocate all the memory dynamically
  weights_ = (float*)::operator new(LSTM_PARAM_SIZE_BYTES + 2 * HIDDEN_DIM * sizeof(float));
  W_im_ = weights_;
  W_fm_ = weights_ + HIDDEN_HIDDEN_SIZE;
  W_om_ = weights_ + 2 * HIDDEN_HIDDEN_SIZE;
  W_cm_ = weights_ + 3 * HIDDEN_HIDDEN_SIZE;

  W_ii_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE;
  W_fi_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + INPUT_HIDDEN_SIZE;
  W_oi_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 2 * INPUT_HIDDEN_SIZE;
  W_ci_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 3 * INPUT_HIDDEN_SIZE;

  b_i_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE;
  b_f_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + HIDDEN_DIM;
  b_o_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 2 * HIDDEN_DIM;
  b_c_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 3 * HIDDEN_DIM;

  initial_hidden_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 4 * HIDDEN_DIM;
  initial_cell_ = weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 5 * HIDDEN_DIM;

  memset(weights_, 0.0f, LSTM_PARAM_SIZE_BYTES + HIDDEN_DIM * 2 * sizeof(float));

  if (setupGPU)
  {
    GPUSetup();
  }
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::updateLSTMInitialStates(const Eigen::Ref<const Eigen::VectorXf> hidden,
                                                     const Eigen::Ref<const Eigen::VectorXf> cell)
{
  setHiddenState(hidden);
  setCellState(cell);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::GPUSetup()
{
  output_nn_->GPUSetup();
  if (!this->GPUMemStatus_)
  {
    network_d_ = Managed::GPUSetup<LSTMHelper<USE_SHARED>>(this);
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->output_nn_), &(this->output_nn_->network_d_),
                                 sizeof(OUTPUT_FNN_T*), cudaMemcpyHostToDevice, this->stream_));
    HANDLE_ERROR(cudaMalloc((void**)&(this->weights_d_), LSTM_PARAM_SIZE_BYTES + HIDDEN_DIM * 2 * sizeof(float)));

    // copies all pointers to be right on the GPU side
    float* incr_ptr = weights_d_;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->weights_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_im_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += HIDDEN_HIDDEN_SIZE;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_fm_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += HIDDEN_HIDDEN_SIZE;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_om_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += HIDDEN_HIDDEN_SIZE;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_cm_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += HIDDEN_HIDDEN_SIZE;

    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_ii_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += INPUT_HIDDEN_SIZE;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_fi_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += INPUT_HIDDEN_SIZE;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_oi_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += INPUT_HIDDEN_SIZE;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->W_ci_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice,
                                 this->stream_));
    incr_ptr += INPUT_HIDDEN_SIZE;

    HANDLE_ERROR(
        cudaMemcpyAsync(&(this->network_d_->b_i_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice, this->stream_));
    incr_ptr += HIDDEN_DIM;
    HANDLE_ERROR(
        cudaMemcpyAsync(&(this->network_d_->b_f_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice, this->stream_));
    incr_ptr += HIDDEN_DIM;
    HANDLE_ERROR(
        cudaMemcpyAsync(&(this->network_d_->b_o_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice, this->stream_));
    incr_ptr += HIDDEN_DIM;
    HANDLE_ERROR(
        cudaMemcpyAsync(&(this->network_d_->b_c_), &(incr_ptr), sizeof(float*), cudaMemcpyHostToDevice, this->stream_));
    incr_ptr += HIDDEN_DIM;

    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->initial_hidden_), &(incr_ptr), sizeof(float*),
                                 cudaMemcpyHostToDevice, this->stream_));
    incr_ptr += HIDDEN_DIM;
    HANDLE_ERROR(cudaMemcpyAsync(&(this->network_d_->initial_cell_), &(incr_ptr), sizeof(float*),
                                 cudaMemcpyHostToDevice, this->stream_));
    paramsToDevice();
  }
  else
  {
    this->logger_->debug("LSTM GPU Memory already set\n");
  }
}
template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::freeCudaMem()
{
  output_nn_->freeCudaMem();
  if (this->GPUMemStatus_)
  {
    HANDLE_ERROR(cudaFree(network_d_));
    HANDLE_ERROR(cudaFree(weights_d_));
    this->GPUMemStatus_ = false;
  }
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::paramsToDevice()
{
  if (this->GPUMemStatus_)
  {
    // copies entire params to device
    HANDLE_ERROR(cudaMemcpyAsync(this->weights_d_, this->weights_,
                                 LSTM_PARAM_SIZE_BYTES + HIDDEN_DIM * 2 * sizeof(float), cudaMemcpyHostToDevice,
                                 stream_));
  }
}

template <bool USE_SHARED>
__device__ void LSTMHelper<USE_SHARED>::initialize(float* theta_s)
{
  this->initialize(theta_s, SHARED_MEM_REQUEST_BLK_BYTES, SHARED_MEM_REQUEST_GRD_BYTES, 0);
}

template <bool USE_SHARED>
__device__ void LSTMHelper<USE_SHARED>::initialize(float* theta_s, int blk_size, int grd_size, int blk_offset)
{
  this->initialize(theta_s, blk_size, grd_size, blk_offset, 0);
}

template <bool USE_SHARED>
__device__ void LSTMHelper<USE_SHARED>::initialize(float* theta_s, int blk_size, int grd_size, int blk_offset,
                                                   int grd_offset)
{
  if (USE_SHARED)
  {
    // if (threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0)
    // {
    //   memcpy(theta_s, weights_, LSTM_PARAM_SIZE_BYTES);
    // }
    for (int i = blockDim.x * threadIdx.y + threadIdx.x; i < LSTM_PARAM_SIZE_BYTES / sizeof(float);
         i += blockDim.x * blockDim.y)
    {
      theta_s[i + grd_offset / sizeof(float)] = weights_[i];
    }
    output_nn_->initialize(theta_s + grd_offset / sizeof(float) + LSTM_SHARED_MEM_GRD_BYTES / sizeof(float));
    __syncthreads();
  }

  // copies the initial cell and hidden state to the correct place
  const int shift =
      grd_size / sizeof(float) + blk_size * (threadIdx.x + blockDim.x * threadIdx.z) / sizeof(float) + blk_offset;
  for (int i = threadIdx.y; i < HIDDEN_DIM; i += blockDim.y)
  {
    (theta_s + shift)[i] = (weights_ + LSTM_PARAM_SIZE_BYTES / sizeof(float))[i];
    (theta_s + shift + HIDDEN_DIM)[i] = (weights_ + LSTM_PARAM_SIZE_BYTES / sizeof(float) + HIDDEN_DIM)[i];
  }

  // if (threadIdx.y == 0)
  // {
  //   memcpy(theta_s + shift, weights_ + LSTM_PARAM_SIZE_BYTES / sizeof(float), 2 * HIDDEN_DIM * sizeof(float));
  // }
  __syncthreads();
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::copyHiddenCellToDevice()
{
  if (this->GPUMemStatus_)
  {
    HANDLE_ERROR(cudaMemcpyAsync(this->weights_d_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 4 * HIDDEN_DIM,
                                 this->weights_ + 4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 4 * HIDDEN_DIM,
                                 2 * HIDDEN_DIM * sizeof(float), cudaMemcpyHostToDevice, stream_));
  }
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::updateOutputModel(const std::vector<int>& description, const std::vector<float>& data)
{
  output_nn_->updateModel(description, data);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::updateOutputModel(const std::vector<float>& data)
{
  output_nn_->updateModel(data);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::forward(const Eigen::Ref<const Eigen::VectorXf>& input)
{
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_im(W_im_, HIDDEN_DIM,
                                                                                             HIDDEN_DIM);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_fm(W_fm_, HIDDEN_DIM,
                                                                                             HIDDEN_DIM);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_om(W_om_, HIDDEN_DIM,
                                                                                             HIDDEN_DIM);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_cm(W_cm_, HIDDEN_DIM,
                                                                                             HIDDEN_DIM);

  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_ii(W_ii_, HIDDEN_DIM,
                                                                                             INPUT_DIM);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_fi(W_fi_, HIDDEN_DIM,
                                                                                             INPUT_DIM);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_oi(W_oi_, HIDDEN_DIM,
                                                                                             INPUT_DIM);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_W_ci(W_ci_, HIDDEN_DIM,
                                                                                             INPUT_DIM);

  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_b_i(b_i_, HIDDEN_DIM, 1);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_b_f(b_f_, HIDDEN_DIM, 1);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_b_o(b_o_, HIDDEN_DIM, 1);
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> eig_b_c(b_c_, HIDDEN_DIM, 1);

  Eigen::VectorXf g_i = eig_W_im * hidden_state_ + eig_W_ii * input + eig_b_i;
  Eigen::VectorXf g_f = eig_W_fm * hidden_state_ + eig_W_fi * input + eig_b_f;
  Eigen::VectorXf g_o = eig_W_om * hidden_state_ + eig_W_oi * input + eig_b_o;
  Eigen::VectorXf g_c = eig_W_cm * hidden_state_ + eig_W_ci * input + eig_b_c;
  g_i = g_i.unaryExpr(&mppi::nn::sigmoid);
  g_f = g_f.unaryExpr(&mppi::nn::sigmoid);
  g_o = g_o.unaryExpr(&mppi::nn::sigmoid);
  g_c = g_c.unaryExpr(&mppi::nn::tanh);

  Eigen::VectorXf c_next = g_i.cwiseProduct(g_c) + g_f.cwiseProduct(cell_state_);
  Eigen::VectorXf h_next = g_o.cwiseProduct(c_next.unaryExpr(&mppi::nn::tanh));

  hidden_state_ = h_next;
  cell_state_ = c_next;
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::forward(const Eigen::Ref<const Eigen::VectorXf>& input, Eigen::Ref<Eigen::VectorXf> output)
{
  forward(input);
  Eigen::VectorXf nn_input = output_nn_->getZeroInputVector();
  for (int i = 0; i < HIDDEN_DIM; i++)
  {
    nn_input(i) = hidden_state_(i);
  }
  for (int i = 0; i < INPUT_DIM; i++)
  {
    nn_input(i + HIDDEN_DIM) = input(i);
  }

  output_nn_->forward(nn_input, output);
}

template <bool USE_SHARED>
__device__ float* LSTMHelper<USE_SHARED>::forward(float* input, float* theta_s)
{
  const int block_idx = (blockDim.x * threadIdx.z + threadIdx.x) * SHARED_MEM_REQUEST_BLK_BYTES / sizeof(float) +
                        SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);
  return forward(input, theta_s, theta_s + block_idx);
}

template <bool USE_SHARED>
__device__ float* LSTMHelper<USE_SHARED>::forward(float* input, float* theta_s, float* block_ptr)
{
  float* c = &block_ptr[0];
  float* g_o = &block_ptr[2 * HIDDEN_DIM];  // input gate
  return forward(input, theta_s, c, g_o);
}

template <bool USE_SHARED>
__device__ float* LSTMHelper<USE_SHARED>::forward(float* input, float* theta_s, float* hidden_cell, float* block_ptr)
{
  // Weights
  float* W_ii = this->W_ii_;
  float* W_im = this->W_im_;
  float* W_fi = this->W_fi_;
  float* W_fm = this->W_fm_;
  float* W_oi = this->W_oi_;
  float* W_om = this->W_om_;
  float* W_ci = this->W_ci_;
  float* W_cm = this->W_cm_;

  // Biases
  float* b_i = this->b_i_;  // hidden_size
  float* b_f = this->b_f_;  // hidden_size
  float* b_o = this->b_o_;  // hidden_size
  float* b_c = this->b_c_;  // hidden_size

  if (USE_SHARED)
  {
    // Weights
    W_im = &theta_s[0];
    W_fm = &theta_s[HIDDEN_HIDDEN_SIZE];
    W_om = &theta_s[2 * HIDDEN_HIDDEN_SIZE];
    W_cm = &theta_s[3 * HIDDEN_HIDDEN_SIZE];
    W_ii = &theta_s[4 * HIDDEN_HIDDEN_SIZE];
    W_fi = &theta_s[4 * HIDDEN_HIDDEN_SIZE + INPUT_HIDDEN_SIZE];
    W_oi = &theta_s[4 * HIDDEN_HIDDEN_SIZE + 2 * INPUT_HIDDEN_SIZE];
    W_ci = &theta_s[4 * HIDDEN_HIDDEN_SIZE + 3 * INPUT_HIDDEN_SIZE];

    // Biases
    b_i = &theta_s[4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE];
    b_f = &theta_s[4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + HIDDEN_DIM];
    b_o = &theta_s[4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 2 * HIDDEN_DIM];
    b_c = &theta_s[4 * HIDDEN_HIDDEN_SIZE + 4 * INPUT_HIDDEN_SIZE + 3 * HIDDEN_DIM];
  }

  uint i, j;

  // Intermediate outputs
  // for each block we have prior cell/hidden state
  float* const h = &hidden_cell[0];
  float* const c = &hidden_cell[HIDDEN_DIM];

  // each block has place to compute g_o, and input
  float* const g_o = &block_ptr[0];  // output gate
  float* const x = &block_ptr[HIDDEN_DIM];

  // FNN needs space for input and activations
  float* const output_act = g_o;

  uint tdy = threadIdx.y;

  // load input into theta_s
  if (input != nullptr)
  {
    for (i = tdy; i < INPUT_DIM; i += blockDim.y)
    {
      x[i] = input[i];
    }
    __syncthreads();
  }

  float temp_g_i = 0;
  float temp_g_f = 0;
  float temp_g_o = 0;
  float temp_cell_update = 0;

  // apply each gate in parallel
  for (i = tdy; i < HIDDEN_DIM; i += blockDim.y)
  {
    temp_g_i = 0;
    temp_g_f = 0;
    temp_g_o = 0;
    temp_cell_update = 0;
    for (j = 0; j < INPUT_DIM; j++)
    {
      int index = i * INPUT_DIM + j;
      temp_g_i += W_ii[index] * x[j];
      temp_g_f += W_fi[index] * x[j];
      temp_g_o += W_oi[index] * x[j];
      temp_cell_update += W_ci[index] * x[j];
    }
    for (j = 0; j < HIDDEN_DIM; j++)
    {
      int index = i * HIDDEN_DIM + j;
      temp_g_i += W_im[index] * h[j];
      temp_g_f += W_fm[index] * h[j];
      temp_g_o += W_om[index] * h[j];
      temp_cell_update += W_cm[index] * h[j];
    }
    temp_g_i += b_i[i];
    temp_g_f += b_f[i];
    temp_g_o += b_o[i];
    temp_cell_update += b_c[i];

    temp_g_i = mppi::nn::sigmoid(temp_g_i);
    temp_g_f = mppi::nn::sigmoid(temp_g_f);
    temp_cell_update = mppi::nn::tanh(temp_cell_update);

    g_o[i] = mppi::nn::sigmoid(temp_g_o);

    c[i] = temp_g_i * temp_cell_update + temp_g_f * c[i];
  }
  __syncthreads();

  // copy computed hidden/cell state to theta_s
  for (i = tdy; i < HIDDEN_DIM; i += blockDim.y)
  {
    h[i] = mppi::nn::tanh(c[i]) * g_o[i];  // actually using c_next intentionally
    output_act[i] = h[i];
  }

  // copy input to activation
  // for (i = tdy; i < INPUT_DIM; i += blockDim.y)
  // {
  //   output_act[i + HIDDEN_DIM] = x[i];
  // }
  __syncthreads();

  return output_nn_->forward(nullptr, theta_s + LSTM_SHARED_MEM_GRD_BYTES / sizeof(float), output_act);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::resetHiddenCellCPU()
{
  for (int i = 0; i < HIDDEN_DIM; i++)
  {
    hidden_state_(i) = initial_hidden_[i];
    cell_state_(i) = initial_cell_[i];
  }
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::setHiddenState(const Eigen::Ref<const Eigen::VectorXf> hidden_state)
{
  for (int i = 0; i < HIDDEN_DIM; i++)
  {
    initial_hidden_[i] = hidden_state(i);
    hidden_state_(i) = hidden_state(i);
  }
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::setCellState(const Eigen::Ref<const Eigen::VectorXf> cell_state)
{
  for (int i = 0; i < HIDDEN_DIM; i++)
  {
    initial_cell_[i] = cell_state(i);
    cell_state_(i) = cell_state(i);
  }
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::loadParams(const std::string& model_path)
{
  if (!fileExists(model_path))
  {
    std::cerr << "Could not load neural net model at path: " << model_path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(model_path);
  loadParams(param_dict);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::loadParams(const cnpy::npz_t& param_dict)
{
  loadParams("", param_dict);
}

template <bool USE_SHARED>
void LSTMHelper<USE_SHARED>::loadParams(std::string prefix, const cnpy::npz_t& param_dict, bool add_slash)
{
  if (add_slash && !prefix.empty() && *prefix.rbegin() != '/')
  {
    prefix.append("/");
  }

  if (param_dict.find("model/" + prefix + "lstm/weight_hh_l0") != param_dict.end())
  {
    prefix.insert(0, "model/");
  }

  // assumes it has been unonioned
  if (output_nn_ == nullptr)
  {
    output_nn_ = new FNNHelper<USE_SHARED>(param_dict, prefix + "output/", this->stream_);
  }
  else
  {
    output_nn_->loadParams(prefix + "output/", param_dict);
  }

  cnpy::NpyArray weight_hh_raw = param_dict.at(prefix + "lstm/weight_hh_l0");
  cnpy::NpyArray bias_hh_raw = param_dict.at(prefix + "lstm/bias_hh_l0");
  cnpy::NpyArray weight_ih_raw = param_dict.at(prefix + "lstm/weight_ih_l0");
  cnpy::NpyArray bias_ih_raw = param_dict.at(prefix + "lstm/bias_ih_l0");
  double* weight_hh = weight_hh_raw.data<double>();
  double* bias_hh = bias_hh_raw.data<double>();
  double* weight_ih = weight_ih_raw.data<double>();
  double* bias_ih = bias_ih_raw.data<double>();

  int input_dim = weight_ih_raw.shape[1];
  int hidden_dim = bias_hh_raw.shape[0] / 4;
  setupMemory(input_dim, hidden_dim);

  for (int i = 0; i < HIDDEN_HIDDEN_SIZE; i++)
  {
    W_im_[i] = weight_hh[i];
    W_fm_[i] = weight_hh[i + HIDDEN_HIDDEN_SIZE];
    W_cm_[i] = weight_hh[i + 2 * HIDDEN_HIDDEN_SIZE];
    W_om_[i] = weight_hh[i + 3 * HIDDEN_HIDDEN_SIZE];
    assert(isfinite(W_im_[i]));
    assert(isfinite(W_fm_[i]));
    assert(isfinite(W_cm_[i]));
    assert(isfinite(W_om_[i]));
  }
  for (int i = 0; i < INPUT_HIDDEN_SIZE; i++)
  {
    W_ii_[i] = weight_ih[i];
    W_fi_[i] = weight_ih[i + INPUT_HIDDEN_SIZE];
    W_ci_[i] = weight_ih[i + 2 * INPUT_HIDDEN_SIZE];
    W_oi_[i] = weight_ih[i + 3 * INPUT_HIDDEN_SIZE];
    assert(isfinite(W_ii_[i]));
    assert(isfinite(W_fi_[i]));
    assert(isfinite(W_ci_[i]));
    assert(isfinite(W_oi_[i]));
  }
  for (int i = 0; i < HIDDEN_DIM; i++)
  {
    b_i_[i] = bias_hh[i] + bias_ih[i];
    b_f_[i] = bias_hh[i + HIDDEN_DIM] + bias_ih[i + HIDDEN_DIM];
    b_c_[i] = bias_hh[i + 2 * HIDDEN_DIM] + bias_ih[i + 2 * HIDDEN_DIM];
    b_o_[i] = bias_hh[i + 3 * HIDDEN_DIM] + bias_ih[i + 3 * HIDDEN_DIM];
    assert(isfinite(b_i_[i]));
    assert(isfinite(b_f_[i]));
    assert(isfinite(b_c_[i]));
    assert(isfinite(b_o_[i]));
  }

  // Save parameters to GPU memory
  paramsToDevice();
}

template <bool USE_SHARED>
__device__ float* LSTMHelper<USE_SHARED>::getInputLocation(float* theta_s)
{
  const int block_idx = (blockDim.x * threadIdx.z + threadIdx.x) * SHARED_MEM_REQUEST_BLK_BYTES / sizeof(float) +
                        SHARED_MEM_REQUEST_GRD_BYTES / sizeof(float);
  float* x = theta_s + block_idx + 3 * HIDDEN_DIM;
  return x;
}

template <bool USE_SHARED>
__device__ float* LSTMHelper<USE_SHARED>::getInputLocation(float* theta_s, const int grd_shift, const int blk_shift,
                                                           const int shift)
{
  float* x = theta_s + grd_shift + blk_shift + shift + 3 * HIDDEN_DIM;
  return x;
}

template <bool USE_SHARED>
__device__ float* LSTMHelper<USE_SHARED>::getHiddenCellLocation(float* theta_s, const int grd_shift,
                                                                const int blk_shift, const int shift)
{
  float* x = theta_s + grd_shift + blk_shift + shift;
  return x;
}
