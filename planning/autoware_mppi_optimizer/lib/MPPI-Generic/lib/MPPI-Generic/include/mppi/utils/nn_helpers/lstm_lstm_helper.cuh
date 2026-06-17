//
// Created by jason on 8/23/22.
//

#ifndef MPPIGENERIC_LSTM_LSTM_HELPER_CUH
#define MPPIGENERIC_LSTM_LSTM_HELPER_CUH

#include "lstm_helper.cuh"

struct LSTMLSTMConfig
{
  LSTMConfig init_config;
  LSTMConfig pred_config;
  int init_len = 0;
};

template <bool USE_SHARED = true>
class LSTMLSTMHelper
{
public:
  LSTMLSTMHelper<USE_SHARED>(LSTMLSTMConfig config, cudaStream_t stream = 0)
    : LSTMLSTMHelper<USE_SHARED>(config.init_config.input_dim, config.init_config.hidden_dim,
                                 config.init_config.output_layers, config.pred_config.input_dim,
                                 config.pred_config.hidden_dim, config.pred_config.output_layers, config.init_len,
                                 stream)
  {
  }
  LSTMLSTMHelper<USE_SHARED>(LSTMConfig init_config, LSTMConfig pred_config, int init_len, cudaStream_t stream = 0)
    : LSTMLSTMHelper<USE_SHARED>(init_config.input_dim, init_config.hidden_dim, init_config.output_layers,
                                 pred_config.input_dim, pred_config.hidden_dim, pred_config.output_layers, init_len,
                                 stream)
  {
  }
  LSTMLSTMHelper<USE_SHARED>(int init_input_dim, int init_hidden_dim, std::vector<int>& init_output_layers,
                             int input_dim, int hidden_dim, std::vector<int>& output_layers, int init_len,
                             cudaStream_t = 0);
  // LSTMLSTMHelper<USE_SHARED>(std::string init_path, std::string lstm_path, cudaStream_t = 0);
  LSTMLSTMHelper<USE_SHARED>(std::string path, std::string prefix, cudaStream_t = 0);
  LSTMLSTMHelper<USE_SHARED>(std::string path, cudaStream_t = 0);

  void loadParams(std::string prefix, std::string path);

  void loadParamsInit(const std::string& model_path);
  void loadParamsInit(const cnpy::npz_t& npz);

  void loadParamsLSTM(const std::string& model_path);
  void loadParamsLSTM(const cnpy::npz_t& npz);

  void updateOutputModelInit(const std::vector<int>& description, const std::vector<float>& data);
  void updateOutputModel(const std::vector<int>& description, const std::vector<float>& data);

  void updateOutputModelInit(const std::vector<float>& data);
  void updateOutputModel(const std::vector<float>& data);

  void GPUSetup();
  void freeCudaMem();

  Eigen::VectorXf getZeroInputVector()
  {
    return lstm_->getZeroInputVector();
  }
  Eigen::VectorXf getZeroOutputVector()
  {
    return lstm_->getZeroOutputVector();
  }

  void resetInitHiddenCPU()
  {
    init_model_->resetHiddenCellCPU();
  }
  void resetLSTMHiddenCellCPU()
  {
    lstm_->resetHiddenCellCPU();
  }

  Eigen::MatrixXf getEmptyBufferMatrix()
  {
    return Eigen::MatrixXf(init_model_->getInputDim(), init_len_);
  }

  Eigen::MatrixXf getEmptyBufferMatrix(int length)
  {
    return Eigen::MatrixXf(init_model_->getInputDim(), length);
  }

  void initializeLSTM(const Eigen::Ref<const Eigen::MatrixXf>& buffer);

  void forward(const Eigen::Ref<const Eigen::VectorXf>& input, Eigen::Ref<Eigen::VectorXf> output);

  std::shared_ptr<LSTMHelper<false>> getInitModel();
  std::shared_ptr<LSTMHelper<USE_SHARED>> getLSTMModel();
  LSTMHelper<USE_SHARED>* getLSTMDevicePtr()
  {
    return lstm_->network_d_;
  }

  int getInitLen()
  {
    return init_len_;
  }

private:
  std::shared_ptr<LSTMHelper<false>> init_model_ = nullptr;
  std::shared_ptr<LSTMHelper<USE_SHARED>> lstm_ = nullptr;
  int init_len_ = -1;
};

#if __CUDACC__
#include "lstm_lstm_helper.cu"
#endif

#endif  // MPPIGENERIC_LSTM_LSTM_HELPER_CUH
