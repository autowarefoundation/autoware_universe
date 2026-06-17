#ifndef MPPIGENERIC_LSTM_HELPER_CUH
#define MPPIGENERIC_LSTM_HELPER_CUH

#include "fnn_helper.cuh"

struct LSTMConfig {
  int input_dim = -1;
  int hidden_dim = -1;
  std::vector<int> output_layers = {-1};

};

template <bool USE_SHARED = true>
class LSTMHelper : public Managed
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef FNNHelper<USE_SHARED> OUTPUT_FNN_T;

  // TODO create destructor that deallocates memory

  LSTMHelper<USE_SHARED>(LSTMConfig& config, cudaStream_t stream = 0);
  LSTMHelper<USE_SHARED>(int input_dim, int hidden_dim, std::vector<int>& output_layers, cudaStream_t stream = 0);
  LSTMHelper<USE_SHARED>(std::string, cudaStream_t stream = 0);
  LSTMHelper<USE_SHARED>(const cnpy::npz_t& param_dict, std::string prefix, bool add_slash = true,
                         cudaStream_t stream = 0);
  ~LSTMHelper()
  {
    freeCudaMem();
  }

  void loadParams(const std::string& model_path);
  void loadParams(const cnpy::npz_t& npz);
  void loadParams(std::string prefix, const cnpy::npz_t& npz, bool add_slash = true);

  __device__ void initialize(float* theta_s);
  __device__ void initialize(float* theta_s, int blk_size, int grd_size, int blk_offset);
  __device__ void initialize(float* theta_s, int blk_size, int grd_size, int blk_offset, int grd_offset);

  void GPUSetup();
  void freeCudaMem();
  void paramsToDevice();
  void copyHiddenCellToDevice();

  void updateOutputModel(const std::vector<int>& description, const std::vector<float>& data);
  void updateOutputModel(const std::vector<float>& data);
  void updateLSTMInitialStates(const Eigen::Ref<const Eigen::VectorXf> hidden,
                               const Eigen::Ref<const Eigen::VectorXf> cell);

  // bool computeGrad(Eigen::Ref<dfdx> A);
  // bool computeGrad(const Eigen::Ref<const Eigen::MatrixXf>& input, Eigen::Ref<dfdx> A);

  void forward(const Eigen::Ref<const Eigen::VectorXf>& input, Eigen::Ref<Eigen::VectorXf> output);
  void forward(const Eigen::Ref<const Eigen::VectorXf>& input);
  __device__ float* forward(float* input, float* theta_s);
  __device__ float* forward(float* input, float* theta_s, float* block_ptr);
  __device__ float* forward(float* input, float* theta_s, float* hidden_cell, float* block_ptr);
  __device__ float* getInputLocation(float* theta_s);
  __device__ float* getInputLocation(float* theta_s, const int grd_shift, const int blk_shift, const int shift);
  __device__ float* getHiddenCellLocation(float* theta_s, const int grd_shift, const int blk_shift, const int shift);

  void resetHiddenCellCPU();

  void setAllValues(std::vector<float>& lstm, std::vector<float>& output)
  {
    assert(lstm.size() == getNumParams());
    memcpy(this->weights_, lstm.data(), LSTM_PARAM_SIZE_BYTES + HIDDEN_DIM * 2 * sizeof(float));
    resetHiddenCellCPU();
    output_nn_->updateModel(output);
  }

  void setAllValues(float input)
  {
    for (int i = 0; i < LSTM_PARAM_SIZE_BYTES / sizeof(float) + 2 * HIDDEN_DIM; i++)
    {
      weights_[i] = input;
    }
    resetHiddenCellCPU();
    output_nn_->setAllWeights(input);
  }

  Eigen::VectorXf getHiddenState()
  {
    return hidden_state_;
  }
  Eigen::VectorXf getCellState()
  {
    return cell_state_;
  }

  __host__ __device__ OUTPUT_FNN_T* getOutputModel() const
  {
    return output_nn_;
  }

  __host__ __device__ float* getOutputWeights() const
  {
    return output_nn_->weights_d_;
  }

  __host__ __device__ int getHiddenDim() const
  {
    return HIDDEN_DIM;
  }
  __host__ __device__ int getInputDim() const
  {
    return INPUT_DIM;
  }
  __host__ __device__ int getOutputDim() const
  {
    return OUTPUT_DIM;
  }
  __host__ __device__ int getHiddenHiddenSize() const
  {
    return HIDDEN_HIDDEN_SIZE;
  }
  __host__ __device__ int getInputHiddenSize() const
  {
    return INPUT_HIDDEN_SIZE;
  }
  __host__ __device__ int getOutputGrdSharedSizeBytes() const
  {
    return output_nn_->getGrdSharedSizeBytes();
  }
  __host__ __device__ int getLSTMGrdSharedSizeBytes() const
  {
    return LSTM_SHARED_MEM_GRD_BYTES;
  }
  __host__ __device__ int getBlkLSTMSharedSizeBytes() const
  {
    return (3 * HIDDEN_DIM + INPUT_DIM) * sizeof(float);
  }
  __host__ __device__ float* getWeights()
  {
    return weights_;
  }

  int getNumParams() const
  {
    return LSTM_PARAM_SIZE_BYTES / sizeof(float) + 2 * HIDDEN_DIM;
  }

  Eigen::VectorXf getZeroInputVector()
  {
    return Eigen::VectorXf::Zero(INPUT_DIM);
  }
  Eigen::VectorXf getZeroOutputVector()
  {
    return Eigen::VectorXf::Zero(OUTPUT_DIM);
  }

  void setHiddenState(const Eigen::Ref<const Eigen::VectorXf> hidden_state);
  void setCellState(const Eigen::Ref<const Eigen::VectorXf> hidden_state);

  // device pointer, null on the device
  LSTMHelper<USE_SHARED>* network_d_ = nullptr;
  float* weights_d_ = nullptr;

private:
  // params
  OUTPUT_FNN_T* output_nn_ = nullptr;

  int HIDDEN_DIM = 0;
  int INPUT_DIM = 0;
  int OUTPUT_DIM = 0;

  int NUM_PARAMS = 1;

  int LSTM_PARAM_SIZE_BYTES = 0;
  int LSTM_SHARED_MEM_GRD_BYTES = 0;  ///< Amount of shared memory the LSTM needs per block

  int HIDDEN_HIDDEN_SIZE = HIDDEN_DIM * HIDDEN_DIM;
  int INPUT_HIDDEN_SIZE = HIDDEN_DIM * (INPUT_DIM);

  float* W_im_ = nullptr;  ///< HIDDEN_HIDDEN_SIZE
  float* W_fm_ = nullptr;  ///< HIDDEN_HIDDEN_SIZE
  float* W_om_ = nullptr;  ///< HIDDEN_HIDDEN_SIZE
  float* W_cm_ = nullptr;  ///< HIDDEN_HIDDEN_SIZE

  float* W_ii_ = nullptr;  ///< INPUT_HIDDEN_SIZE
  float* W_fi_ = nullptr;  ///< INPUT_HIDDEN_SIZE
  float* W_oi_ = nullptr;  ///< INPUT_HIDDEN_SIZE
  float* W_ci_ = nullptr;  ///< INPUT_HIDDEN_SIZE

  float* b_i_ = nullptr;   ///< HIDDEN_SIZE
  float* b_f_ = nullptr;   ///< HIDDEN_SIZE
  float* b_o_ = nullptr;   ///< HIDDEN_SIZE
  float* b_c_ = nullptr;   ///< HIDDEN_SIZE

  float* initial_hidden_ = nullptr;
  float* initial_cell_ = nullptr;

  float* weights_ = nullptr;

  Eigen::VectorXf hidden_state_;
  Eigen::VectorXf cell_state_;

  void setupMemory(int input_dim, int hidden_dim);
};

#if __CUDACC__
#include "lstm_helper.cu"
#endif

#endif  // MPPIGENERIC_LSTM_HELPER_CUH
