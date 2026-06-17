#include "lstm_lstm_helper.cuh"

template <bool USE_SHARED>
LSTMLSTMHelper<USE_SHARED>::LSTMLSTMHelper(int init_input_dim, int init_hidden_dim,
                                           std::vector<int>& init_output_layers, int input_dim, int hidden_dim,
                                           std::vector<int>& output_layers, int init_len, cudaStream_t stream)
{
  init_len_ = init_len;
  init_model_ = std::make_shared<LSTMHelper<false>>(init_input_dim, init_hidden_dim, init_output_layers);
  lstm_ = std::make_shared<LSTMHelper<USE_SHARED>>(input_dim, hidden_dim, output_layers, stream);
  assert(init_model_->getOutputDim() == lstm_->getHiddenDim() * 2);
}

template <bool USE_SHARED>
LSTMLSTMHelper<USE_SHARED>::LSTMLSTMHelper(std::string path, cudaStream_t stream)
  : LSTMLSTMHelper<USE_SHARED>::LSTMLSTMHelper(path, "", stream)
{
}

template <bool USE_SHARED>
LSTMLSTMHelper<USE_SHARED>::LSTMLSTMHelper(std::string path, std::string prefix, cudaStream_t stream)
{
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(path);

  init_model_ = std::make_shared<LSTMHelper<false>>(param_dict, prefix + "init_", false);
  lstm_ = std::make_shared<LSTMHelper<USE_SHARED>>(param_dict, prefix, true, stream);

  if (param_dict.find("init_length") != param_dict.end())  // this is the new api but assert is meaningless right now
  {
    // TODO remove once all models are updated to the lastest export API
    assert(param_dict.find("init_length") != param_dict.end());
    init_len_ = static_cast<int>(param_dict.at("init_length").data<double>()[0]) + 1;
  }
  else
  {
    // old api here
    assert(param_dict.find("num_points") != param_dict.end());
    int num_points = param_dict.at("num_points").data<int>()[0];
    assert(param_dict.find("init_input") != param_dict.end());
    init_len_ = param_dict.at("init_input").shape[0] / (num_points * init_model_->getInputDim());
  }
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::initializeLSTM(const Eigen::Ref<const Eigen::MatrixXf>& buffer)
{
  assert(buffer.rows() == init_model_->getInputDim());
  assert(buffer.cols() >= init_len_);
  // reset hidden/cell state
  init_model_->resetHiddenCellCPU();

  int t = buffer.cols() - init_len_;
  for (; t < buffer.cols() - 1; t++)
  {
    init_model_->forward(buffer.col(t));
  }

  // run full model with output at end
  Eigen::VectorXf output = init_model_->getZeroOutputVector();
  init_model_->forward(buffer.col(t), output);

  // set the lstm initial hidden/cell to output
  lstm_->setHiddenState(output.head(lstm_->getHiddenDim()));
  lstm_->setCellState(output.tail(lstm_->getHiddenDim()));

  // only copies the hidden/cell states
  lstm_->copyHiddenCellToDevice();
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::loadParamsInit(const std::string& model_path)
{
  init_model_->loadParams(model_path);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::loadParamsInit(const cnpy::npz_t& npz)
{
  init_model_->loadParams(npz);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::loadParamsLSTM(const std::string& model_path)
{
  lstm_->loadParams(model_path);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::loadParamsLSTM(const cnpy::npz_t& npz)
{
  lstm_->loadParams(npz);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::loadParams(std::string prefix, std::string model_path)
{
  if (!prefix.empty() && *prefix.rbegin() != '/')
  {
    prefix.append("/");
  }

  if (!fileExists(model_path))
  {
    std::cerr << "Could not load neural net model at path: " << model_path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(model_path);

  lstm_->loadParams(prefix, param_dict);
  init_model_->loadParams(prefix + "init_", param_dict, false);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::GPUSetup()
{
  lstm_->GPUSetup();
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::freeCudaMem()
{
  lstm_->freeCudaMem();
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::forward(const Eigen::Ref<const Eigen::VectorXf>& input,
                                         Eigen::Ref<Eigen::VectorXf> output)
{
  lstm_->forward(input, output);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::updateOutputModel(const std::vector<int>& description, const std::vector<float>& data)
{
  lstm_->updateOutputModel(description, data);
}
template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::updateOutputModelInit(const std::vector<int>& description,
                                                       const std::vector<float>& data)
{
  init_model_->updateOutputModel(description, data);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::updateOutputModel(const std::vector<float>& data)
{
  lstm_->updateOutputModel(data);
}

template <bool USE_SHARED>
void LSTMLSTMHelper<USE_SHARED>::updateOutputModelInit(const std::vector<float>& data)
{
  init_model_->updateOutputModel(data);
}

template <bool USE_SHARED>
std::shared_ptr<LSTMHelper<false>> LSTMLSTMHelper<USE_SHARED>::getInitModel()
{
  return init_model_;
}

template <bool USE_SHARED>
std::shared_ptr<LSTMHelper<USE_SHARED>> LSTMLSTMHelper<USE_SHARED>::getLSTMModel()
{
  return lstm_;
}
