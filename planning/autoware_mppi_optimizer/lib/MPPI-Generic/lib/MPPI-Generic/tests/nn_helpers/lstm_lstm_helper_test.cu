// #include <gtest/gtest.h>

#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>
#include <math.h>

#include <mppi/utils/nn_helpers/lstm_lstm_helper.cuh>
// Auto-generated header file
#include <test_networks.h>
#include <kernel_tests/utils/network_helper_kernel_test.cuh>
#include "mppi/ddp/ddp_dynamics.h"

class LSTMLSTMHelperTest : public testing::Test
{
protected:
  void SetUp() override
  {
    generator = std::default_random_engine(7.0);
    distribution = std::normal_distribution<float>(0.0, 1.0);
  }

  void TearDown() override
  {
  }

  std::default_random_engine generator;
  std::normal_distribution<float> distribution;
  std::vector<int> init_output_layers = { 68, 100, 20 };
  std::vector<int> output_layers = { 18, 2 };
  const int init_hidden_dim = 60;
  const int hidden_dim = 10;
  const int init_input_dim = 8;
  const int input_dim = 8;
  const int output_dim = 2;
  const int init_len = 6;
};

// template class FNNParams<18, 3>;
// template class FNNParams<68, 100, 20>;
// template class LSTMParams<8, 10>;
// template class LSTMParams<8, 60>;
// typedef FNNParams<18, 3> FNN_PARAMS;
// typedef FNNParams<68, 100, 20> FNN_INIT_PARAMS;
// typedef LSTMHelper<LSTMParams<8, 10>, FNN_PARAMS> LSTM;
// typedef LSTMHelper<LSTMParams<8, 60>, FNN_INIT_PARAMS> INIT_LSTM;
//
// template class LSTMLSTMHelper<INIT_LSTM, LSTM, 10>;
//
// typedef LSTMLSTMHelper<INIT_LSTM, LSTM, 10> T;

TEST_F(LSTMLSTMHelperTest, BindStreamAndConstructor)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  LSTMLSTMHelper<> helper(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers,
                          init_len, stream);

  EXPECT_EQ(helper.getLSTMModel()->stream_, stream);
  EXPECT_NE(helper.getLSTMModel(), nullptr);

  EXPECT_EQ(helper.getLSTMModel()->getInputDim(), input_dim);
  EXPECT_EQ(helper.getLSTMModel()->getOutputDim(), output_dim);
  EXPECT_EQ(helper.getLSTMModel()->getHiddenDim(), hidden_dim);

  EXPECT_EQ(helper.getInitModel()->getInputDim(), init_input_dim);
  EXPECT_EQ(helper.getInitModel()->getOutputDim(), 2 * hidden_dim);
  EXPECT_EQ(helper.getInitModel()->getHiddenDim(), init_hidden_dim);

  auto init_lstm = helper.getInitModel();
  EXPECT_NE(init_lstm, nullptr);

  auto hidden = init_lstm->getHiddenState();
  auto cell = init_lstm->getCellState();
  for (int i = 0; i < init_lstm->getHiddenDim(); i++)

  {
    EXPECT_FLOAT_EQ(hidden(i), 0.0f);
    EXPECT_FLOAT_EQ(cell(i), 0.0f);
  }
}

TEST_F(LSTMLSTMHelperTest, BindStreamAndConstructorConfigLSTM)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));
  LSTMConfig init_config, pred_config;
  init_config.input_dim = init_input_dim;
  init_config.hidden_dim = init_hidden_dim;
  init_config.output_layers = init_output_layers;
  pred_config.input_dim = input_dim;
  pred_config.hidden_dim = hidden_dim;
  pred_config.output_layers = output_layers;

  LSTMLSTMHelper<> helper(init_config, pred_config, init_len, stream);

  EXPECT_EQ(helper.getLSTMModel()->stream_, stream);
  EXPECT_NE(helper.getLSTMModel(), nullptr);

  EXPECT_EQ(helper.getLSTMModel()->getInputDim(), input_dim);
  EXPECT_EQ(helper.getLSTMModel()->getOutputDim(), output_dim);
  EXPECT_EQ(helper.getLSTMModel()->getHiddenDim(), hidden_dim);

  EXPECT_EQ(helper.getInitModel()->getInputDim(), init_input_dim);
  EXPECT_EQ(helper.getInitModel()->getOutputDim(), 2 * hidden_dim);
  EXPECT_EQ(helper.getInitModel()->getHiddenDim(), init_hidden_dim);

  auto init_lstm = helper.getInitModel();
  EXPECT_NE(init_lstm, nullptr);

  auto hidden = init_lstm->getHiddenState();
  auto cell = init_lstm->getCellState();
  for (int i = 0; i < init_lstm->getHiddenDim(); i++)

  {
    EXPECT_FLOAT_EQ(hidden(i), 0.0f);
    EXPECT_FLOAT_EQ(cell(i), 0.0f);
  }
}

TEST_F(LSTMLSTMHelperTest, BindStreamAndConstructorConfigLSTMLSTM)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));
  LSTMLSTMConfig config;
  config.init_config.input_dim = init_input_dim;
  config.init_config.hidden_dim = init_hidden_dim;
  config.init_config.output_layers = init_output_layers;
  config.pred_config.input_dim = input_dim;
  config.pred_config.hidden_dim = hidden_dim;
  config.pred_config.output_layers = output_layers;
  config.init_len = init_len;

  LSTMLSTMHelper<> helper(config, stream);

  EXPECT_EQ(helper.getLSTMModel()->stream_, stream);
  EXPECT_NE(helper.getLSTMModel(), nullptr);

  EXPECT_EQ(helper.getLSTMModel()->getInputDim(), input_dim);
  EXPECT_EQ(helper.getLSTMModel()->getOutputDim(), output_dim);
  EXPECT_EQ(helper.getLSTMModel()->getHiddenDim(), hidden_dim);

  EXPECT_EQ(helper.getInitModel()->getInputDim(), init_input_dim);
  EXPECT_EQ(helper.getInitModel()->getOutputDim(), 2 * hidden_dim);
  EXPECT_EQ(helper.getInitModel()->getHiddenDim(), init_hidden_dim);

  auto init_lstm = helper.getInitModel();
  EXPECT_NE(init_lstm, nullptr);

  auto hidden = init_lstm->getHiddenState();
  auto cell = init_lstm->getCellState();
  for (int i = 0; i < init_lstm->getHiddenDim(); i++)

  {
    EXPECT_FLOAT_EQ(hidden(i), 0.0f);
    EXPECT_FLOAT_EQ(cell(i), 0.0f);
  }
}

TEST_F(LSTMLSTMHelperTest, initializeLSTMLSTMTest)
{
  LSTMLSTMHelper<> helper(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers,
                          init_len);
  Eigen::Matrix<float, 8, 10> buffer;
  buffer.setOnes();

  helper.getInitModel()->setAllValues(1.0);

  helper.initializeLSTM(buffer);

  auto lstm = helper.getLSTMModel();
  auto hidden = lstm->getHiddenState();
  auto cell = lstm->getHiddenState();
  for (int i = 0; i < hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(hidden(i), 101.0f);
    EXPECT_FLOAT_EQ(cell(i), 101.0f);
  }
}

TEST_F(LSTMLSTMHelperTest, GPUSetupAndParamsCheck)
{
  LSTMLSTMHelper<> model(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers,
                         init_len);

  std::vector<float> theta_vec(model.getLSTMModel()->getOutputModel()->getNumParams());
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }

  std::vector<float> lstm_vec(model.getLSTMModel()->getNumParams());
  for (int i = 0; i < lstm_vec.size(); i++)
  {
    lstm_vec[i] = distribution(generator);
  }
  model.getLSTMModel()->setAllValues(lstm_vec, theta_vec);

  int grid_dim = 5;

  std::vector<float> lstm_params(grid_dim);
  std::vector<float> shared_lstm_params(grid_dim);
  std::vector<float> fnn_params(grid_dim);
  std::vector<float> shared_fnn_params(grid_dim);

  EXPECT_EQ(model.getLSTMModel()->GPUMemStatus_, false);
  EXPECT_EQ(model.getLSTMModel()->network_d_, nullptr);
  EXPECT_NE(model.getLSTMModel()->getOutputModel(), nullptr);
  EXPECT_EQ(model.getInitModel()->GPUMemStatus_, false);
  EXPECT_EQ(model.getInitModel()->network_d_, nullptr);
  EXPECT_NE(model.getInitModel()->getOutputModel(), nullptr);

  model.GPUSetup();

  EXPECT_EQ(model.getLSTMModel()->GPUMemStatus_, true);
  EXPECT_NE(model.getLSTMModel()->network_d_, nullptr);
  EXPECT_NE(model.getLSTMModel()->getOutputModel(), nullptr);
  EXPECT_EQ(model.getInitModel()->GPUMemStatus_, false);
  EXPECT_EQ(model.getInitModel()->network_d_, nullptr);
  EXPECT_NE(model.getInitModel()->getOutputModel(), nullptr);

  // launch kernel
  launchParameterCheckTestKernel<LSTMHelper<>>(*model.getLSTMModel(), lstm_params, shared_lstm_params, fnn_params,
                                               shared_fnn_params, grid_dim);

  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getGrdSharedSizeBytes() / sizeof(float), 44);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getBlkSharedSizeBytes() / sizeof(float), 36);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNumParams(), 38);

  int grd_size = model.getLSTMModel()->getOutputModel()->getGrdSharedSizeBytes() / sizeof(float);

  for (int grid = 0; grid < grid_dim; grid++)
  {
    // ensure that the output nn matches
    for (int i = 0; i < theta_vec.size(); i++)
    {
      EXPECT_FLOAT_EQ(fnn_params[grid * grd_size + i], theta_vec[i]) << "at grid " << grid << " at index " << i;
    }

    const int h = model.getLSTMModel()->getHiddenDim();
    const int hh = h * h;
    const int ih = model.getLSTMModel()->getInputDim() * model.getLSTMModel()->getHiddenDim();
    int shift = (h * 6 + hh * 4 + ih * 4) * grid;

    for (int i = 0; i < hh; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + i], lstm_vec[i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + hh + i], lstm_vec[hh + i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 2 * hh + i], lstm_vec[2 * hh + i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 3 * hh + i], lstm_vec[3 * hh + i]) << "at grid " << grid << " at index " << i;
    }

    for (int i = 0; i < ih; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + i], lstm_vec[4 * hh + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + ih + i], lstm_vec[4 * hh + ih + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 2 * ih + i], lstm_vec[4 * hh + 2 * ih + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 3 * ih + i], lstm_vec[4 * hh + 3 * ih + i]);
    }

    for (int i = 0; i < h; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + i], lstm_vec[4 * hh + 4 * ih + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + h + i], lstm_vec[4 * hh + 4 * ih + h + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 2 * h + i], lstm_vec[4 * hh + 4 * ih + 2 * h + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 3 * h + i], lstm_vec[4 * hh + 4 * ih + 3 * h + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 4 * h + i], lstm_vec[4 * hh + 4 * ih + 4 * h + i])
          << "at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 5 * h + i], lstm_vec[4 * hh + 4 * ih + 5 * h + i])
          << "at index " << i;
    }
  }

  for (int grid = 0; grid < grid_dim; grid++)
  {
    // ensure that the output nn matches
    for (int i = 0; i < theta_vec.size(); i++)
    {
      EXPECT_FLOAT_EQ(shared_fnn_params[grid * grd_size + i], theta_vec[i]) << "at grid " << grid << " at index " << i;
    }

    const int h = model.getLSTMModel()->getHiddenDim();
    const int hh = h * h;
    const int ih = model.getLSTMModel()->getInputDim() * model.getLSTMModel()->getHiddenDim();
    int shift = (h * 6 + hh * 4 + ih * 4) * grid;

    for (int i = 0; i < hh; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + i], lstm_vec[i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + hh + i], lstm_vec[hh + i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 2 * hh + i], lstm_vec[2 * hh + i])
          << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 3 * hh + i], lstm_vec[3 * hh + i])
          << "at grid " << grid << " at index " << i;
    }

    for (int i = 0; i < ih; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + i], lstm_vec[4 * hh + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + ih + i], lstm_vec[4 * hh + ih + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 2 * ih + i], lstm_vec[4 * hh + 2 * ih + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 3 * ih + i], lstm_vec[4 * hh + 3 * ih + i]);
    }

    for (int i = 0; i < h; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + i], lstm_vec[4 * hh + 4 * ih + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + h + i], lstm_vec[4 * hh + 4 * ih + h + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 2 * h + i], lstm_vec[4 * hh + 4 * ih + 2 * h + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 3 * h + i], lstm_vec[4 * hh + 4 * ih + 3 * h + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 4 * h + i], lstm_vec[4 * hh + 4 * ih + 4 * h + i])
          << "at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 5 * h + i], lstm_vec[4 * hh + 4 * ih + 5 * h + i])
          << "at index " << i;
    }
  }
}

TEST_F(LSTMLSTMHelperTest, LoadModelPathTest)
{
  std::string path = mppi::tests::test_lstm_lstm;
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  LSTMLSTMHelper<> model(path);

  EXPECT_EQ(model.getLSTMModel()->getInputDim(), 3);
  EXPECT_EQ(model.getLSTMModel()->getHiddenDim(), 25);
  EXPECT_EQ(model.getLSTMModel()->getOutputDim(), 2);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[0], 28);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[1], 30);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[2], 30);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[3], 2);

  EXPECT_EQ(model.getInitModel()->getInputDim(), 3);
  EXPECT_EQ(model.getInitModel()->getHiddenDim(), 60);
  EXPECT_EQ(model.getInitModel()->getOutputDim(), 50);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[0], 63);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[1], 15);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[2], 15);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[3], 50);

  EXPECT_EQ(model.getInitLen(), 6);
  assert(model.getInitLen() == 6);

  cnpy::npz_t input_outputs = cnpy::npz_load(path);
  double* inputs = input_outputs.at("input").data<double>();
  double* outputs = input_outputs.at("output").data<double>();
  double* init_inputs = input_outputs.at("init_input").data<double>();
  double* init_hidden = input_outputs.at("init/hidden").data<double>();
  double* init_cell = input_outputs.at("init/cell").data<double>();
  double* hidden = input_outputs.at("hidden").data<double>();
  double* cell = input_outputs.at("cell").data<double>();

  const int hidden_dim = model.getLSTMModel()->getHiddenDim();
  const int input_dim = model.getLSTMModel()->getInputDim();
  const int output_dim = model.getLSTMModel()->getOutputDim();
  const int init_hidden_dim = model.getInitModel()->getHiddenDim();
  const int init_input_dim = model.getInitModel()->getInputDim();

  // TOOD figure out the number of points from the input
  int init_len = model.getInitLen();
  int num_points = input_outputs.at("num_points").data<int>()[0];
  int T = input_outputs.at("output").shape[0] / (num_points * output_dim);

  EXPECT_EQ(num_points, 3);
  EXPECT_EQ(input_dim, init_input_dim);

  double tol = 1e-5;

  auto input = model.getLSTMModel()->getZeroInputVector();
  auto output = model.getLSTMModel()->getZeroOutputVector();

  for (int point = 0; point < num_points; point++)
  {
    // run the init network and ensure initial hidden/cell match
    Eigen::MatrixXf buffer(init_input_dim, init_len);
    model.resetInitHiddenCPU();
    for (int t = 0; t < init_len; t++)
    {
      for (int i = 0; i < init_input_dim; i++)
      {
        buffer(i, t) = init_inputs[point * init_len * init_input_dim + t * init_input_dim + i];
      }
    }
    model.initializeLSTM(buffer);

    for (int i = 0; i < hidden_dim; i++)
    {
      EXPECT_NEAR(model.getLSTMModel()->getHiddenState()(i), init_hidden[hidden_dim * point + i], tol)
          << "at point " << point << " index " << i;
      EXPECT_NEAR(model.getLSTMModel()->getCellState()(i), init_cell[hidden_dim * point + i], tol);
    }
    for (int t = 0; t < T; t++)
    {
      for (int i = 0; i < input_dim; i++)
      {
        input(i) = inputs[point * T * input_dim + t * input_dim + i];
      }
      model.forward(input, output);

      for (int i = 0; i < output_dim; i++)
      {
        EXPECT_NEAR(output[i], outputs[point * T * output_dim + t * output_dim + i], tol)
            << "point " << point << " T " << t << " at dim " << i;
      }
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_NEAR(model.getLSTMModel()->getHiddenState()(i), hidden[point * T * hidden_dim + hidden_dim * t + i], tol)
            << "point " << point << " at dim " << i;
        EXPECT_NEAR(model.getLSTMModel()->getCellState()(i), cell[point * T * hidden_dim + hidden_dim * t + i], tol)
            << "point " << point << " at dim " << i;
      }
    }
  }
}

TEST_F(LSTMLSTMHelperTest, LoadModelPathTestLongBuffer)
{
  std::string path = mppi::tests::test_lstm_lstm;
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  LSTMLSTMHelper<> model(path);

  EXPECT_EQ(model.getLSTMModel()->getInputDim(), 3);
  EXPECT_EQ(model.getLSTMModel()->getHiddenDim(), 25);
  EXPECT_EQ(model.getLSTMModel()->getOutputDim(), 2);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[0], 28);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[1], 30);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[2], 30);
  EXPECT_EQ(model.getLSTMModel()->getOutputModel()->getNetStructurePtr()[3], 2);

  EXPECT_EQ(model.getInitModel()->getInputDim(), 3);
  EXPECT_EQ(model.getInitModel()->getHiddenDim(), 60);
  EXPECT_EQ(model.getInitModel()->getOutputDim(), 50);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[0], 63);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[1], 15);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[2], 15);
  EXPECT_EQ(model.getInitModel()->getOutputModel()->getNetStructurePtr()[3], 50);

  cnpy::npz_t input_outputs = cnpy::npz_load(path);
  double* inputs = input_outputs.at("input").data<double>();
  double* outputs = input_outputs.at("output").data<double>();
  double* init_inputs = input_outputs.at("init_input").data<double>();
  double* init_hidden = input_outputs.at("init/hidden").data<double>();
  double* init_cell = input_outputs.at("init/cell").data<double>();
  double* hidden = input_outputs.at("hidden").data<double>();
  double* cell = input_outputs.at("cell").data<double>();

  const int hidden_dim = model.getLSTMModel()->getHiddenDim();
  const int input_dim = model.getLSTMModel()->getInputDim();
  const int output_dim = model.getLSTMModel()->getOutputDim();
  const int init_hidden_dim = model.getInitModel()->getHiddenDim();
  const int init_input_dim = model.getInitModel()->getInputDim();

  // TOOD figure out the number of points from the input
  const int init_len = model.getInitLen();
  EXPECT_EQ(model.getInitLen(), 6);
  int num_points = input_outputs.at("num_points").data<int>()[0];
  int T = input_outputs.at("output").shape[0] / (num_points * output_dim);

  double tol = 1e-5;

  auto input = model.getLSTMModel()->getZeroInputVector();
  auto output = model.getLSTMModel()->getZeroOutputVector();

  for (int point = 0; point < num_points; point++)
  {
    // run the init network and ensure initial hidden/cell match
    Eigen::MatrixXf buffer(init_input_dim, init_len + 30);
    model.resetInitHiddenCPU();
    for (int t = 0; t < init_len; t++)
    {
      for (int i = 0; i < init_input_dim; i++)
      {
        buffer(i, t + 30) = init_inputs[point * init_len * init_input_dim + t * init_input_dim + i];
      }
    }
    model.initializeLSTM(buffer);

    for (int i = 0; i < hidden_dim; i++)
    {
      EXPECT_NEAR(model.getLSTMModel()->getHiddenState()(i), init_hidden[hidden_dim * point + i], tol)
          << "at point " << point << " index " << i;
      EXPECT_NEAR(model.getLSTMModel()->getCellState()(i), init_cell[hidden_dim * point + i], tol);
    }
    for (int t = 0; t < T; t++)
    {
      for (int i = 0; i < input_dim; i++)
      {
        input(i) = inputs[point * T * input_dim + t * input_dim + i];
      }
      model.forward(input, output);

      for (int i = 0; i < output_dim; i++)
      {
        EXPECT_NEAR(output[i], outputs[point * T * output_dim + t * output_dim + i], tol)
            << "point " << point << " at dim " << i;
      }
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_NEAR(model.getLSTMModel()->getHiddenState()(i), hidden[point * T * hidden_dim + hidden_dim * t + i], tol)
            << "point " << point << " at dim " << i;
        EXPECT_NEAR(model.getLSTMModel()->getCellState()(i), cell[point * T * hidden_dim + hidden_dim * t + i], tol)
            << "point " << point << " at dim " << i;
      }
    }
  }
}

TEST_F(LSTMLSTMHelperTest, forwardCPU)
{
  LSTMLSTMHelper<> model(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers, 10);

  model.getLSTMModel()->setAllValues(1.0f);
  model.getInitModel()->setAllValues(0.1f);
  model.getInitModel()->getOutputModel()->setAllWeights(0.01f);

  for (int i = 0; i < init_hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(model.getInitModel()->getHiddenState()(i), 0.1);
    EXPECT_FLOAT_EQ(model.getInitModel()->getCellState()(i), 0.1);
  }

  for (int i = 0; i < hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(model.getLSTMModel()->getHiddenState()(i), 1.0);
    EXPECT_FLOAT_EQ(model.getLSTMModel()->getCellState()(i), 1.0);
  }

  auto input = model.getLSTMModel()->getZeroInputVector();
  auto output = model.getLSTMModel()->getZeroOutputVector();
  input.setOnes();

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 18.640274);
  EXPECT_FLOAT_EQ(output[1], 18.640274);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 18.950548);
  EXPECT_FLOAT_EQ(output[1], 18.950548);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 18.993294);
  EXPECT_FLOAT_EQ(output[1], 18.993294);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 18.999092);
  EXPECT_FLOAT_EQ(output[1], 18.999092);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 18.999878);
  EXPECT_FLOAT_EQ(output[1], 18.999878);

  auto buffer = model.getEmptyBufferMatrix(10);
  buffer.setOnes();
  buffer = buffer * 0.1;

  model.initializeLSTM(buffer);

  for (int i = 0; i < init_hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(model.getInitModel()->getHiddenState()(i), 0.99790788);
    EXPECT_FLOAT_EQ(model.getInitModel()->getCellState()(i), 9.2190571);
  }
  for (int i = 0; i < hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(model.getLSTMModel()->getHiddenState()(i), 0.558857381);
    EXPECT_FLOAT_EQ(model.getLSTMModel()->getCellState()(i), 0.558857381);
  }

  input *= 0.1;

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 10.945133);
  EXPECT_FLOAT_EQ(output[1], 10.945133);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 11.680509);
  EXPECT_FLOAT_EQ(output[1], 11.680509);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 11.783686);
  EXPECT_FLOAT_EQ(output[1], 11.783686);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 11.797728);
  EXPECT_FLOAT_EQ(output[1], 11.797728);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 11.79963);
  EXPECT_FLOAT_EQ(output[1], 11.79963);
}

TEST_F(LSTMLSTMHelperTest, forwardGPU)
{
  const int num_rollouts = 1;

  LSTMLSTMHelper<> model(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers, 10);

  model.getLSTMModel()->setAllValues(1.0f);
  model.getInitModel()->setAllValues(0.1f);
  model.getInitModel()->getOutputModel()->setAllWeights(0.01f);

  model.GPUSetup();

  std::array<float, 5> true_vals = { 10.945133, 11.680509, 11.783686, 11.797728, 11.79963 };

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 2>> output_arr(num_rollouts);

  auto input = model.getLSTMModel()->getZeroInputVector();
  auto output = model.getLSTMModel()->getZeroOutputVector();
  input.setOnes();
  input = input * 0.1;

  for (int i = 0; i < init_hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(model.getInitModel()->getHiddenState()(i), 0.1);
    EXPECT_FLOAT_EQ(model.getInitModel()->getCellState()(i), 0.1);
  }

  for (int i = 0; i < hidden_dim; i++)
  {
    EXPECT_FLOAT_EQ(model.getLSTMModel()->getHiddenState()(i), 1.0);
    EXPECT_FLOAT_EQ(model.getLSTMModel()->getCellState()(i), 1.0);
  }

  auto buffer = model.getEmptyBufferMatrix(10);
  buffer.setOnes();
  buffer = buffer * 0.1;

  for (int state_index = 0; state_index < num_rollouts; state_index++)
  {
    for (int dim = 0; dim < input_arr[0].size(); dim++)
    {
      input_arr[state_index][dim] = input(dim);
    }
  }

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int step = 1; step < 6; step++)
    {
      model.initializeLSTM(buffer);

      for (int i = 0; i < init_hidden_dim; i++)
      {
        EXPECT_FLOAT_EQ(model.getInitModel()->getHiddenState()(i), 0.99790788);
        EXPECT_FLOAT_EQ(model.getInitModel()->getCellState()(i), 9.2190571);
      }
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_FLOAT_EQ(model.getLSTMModel()->getHiddenState()(i), 0.558857381);
        EXPECT_FLOAT_EQ(model.getLSTMModel()->getCellState()(i), 0.558857381);
      }
      launchForwardTestKernel<LSTMHelper<>, 8, 2, 32>(*model.getLSTMModel(), input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        // model.resetInitHiddenCPU();
        model.resetLSTMHiddenCellCPU();
        input.setOnes();
        input = input * 0.1;

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < input_dim; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < output_dim; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4)
              << "at index " << point << " with y_dim " << y_dim << " dim " << dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
          EXPECT_FLOAT_EQ(output(dim), true_vals[step - 1]) << "at dim " << dim << " step " << step;
        }
      }
    }
  }
}

TEST_F(LSTMLSTMHelperTest, forwardGPUCompareShared)
{
  const int num_rollouts = 3000;

  LSTMLSTMHelper<> model(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers,
                         init_len);

  std::vector<float> theta_vec(model.getLSTMModel()->getOutputModel()->getNumParams());
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  std::vector<float> lstm_vec(model.getLSTMModel()->getNumParams());
  for (int i = 0; i < lstm_vec.size(); i++)
  {
    lstm_vec[i] = distribution(generator);
  }
  model.getLSTMModel()->setAllValues(lstm_vec, theta_vec);

  theta_vec.resize(model.getInitModel()->getOutputModel()->getNumParams());
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  lstm_vec.resize(model.getInitModel()->getNumParams());
  for (int i = 0; i < lstm_vec.size(); i++)
  {
    lstm_vec[i] = distribution(generator);
  }
  model.getInitModel()->setAllValues(lstm_vec, theta_vec);

  model.GPUSetup();

  auto buffer = model.getEmptyBufferMatrix(10);

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 2>> output_arr(num_rollouts);

  Eigen::Matrix<float, 8, num_rollouts> inputs = Eigen::Matrix<float, 8, num_rollouts>::Random();
  auto input = model.getLSTMModel()->getZeroInputVector();
  auto output = model.getLSTMModel()->getZeroOutputVector();

  for (int state_index = 0; state_index < num_rollouts; state_index++)
  {
    for (int dim = 0; dim < input_arr[0].size(); dim++)
    {
      input_arr[state_index][dim] = inputs.col(state_index)(dim);
    }
  }

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int step = 1; step < 6; step++)
    {
      buffer.setRandom();
      model.initializeLSTM(buffer);

      launchForwardTestKernel<LSTMHelper<>, 8, 2, 32>(*model.getLSTMModel(), input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        // model.resetInitHiddenCPU();
        model.resetLSTMHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < input_dim; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < output_dim; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4)
              << "at index " << point << " with y_dim " << y_dim << " dim " << dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
        }
      }
    }
  }
}

TEST_F(LSTMLSTMHelperTest, forwardGPUCompareNoShared)
{
  const int num_rollouts = 3000;

  LSTMLSTMHelper<false> model(init_input_dim, init_hidden_dim, init_output_layers, input_dim, hidden_dim, output_layers,
                              init_len);

  std::vector<float> theta_vec(model.getLSTMModel()->getOutputModel()->getNumParams());
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  std::vector<float> lstm_vec(model.getLSTMModel()->getNumParams());
  for (int i = 0; i < lstm_vec.size(); i++)
  {
    lstm_vec[i] = distribution(generator);
  }
  model.getLSTMModel()->setAllValues(lstm_vec, theta_vec);

  theta_vec.resize(model.getInitModel()->getOutputModel()->getNumParams());
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  lstm_vec.resize(model.getInitModel()->getNumParams());
  for (int i = 0; i < lstm_vec.size(); i++)
  {
    lstm_vec[i] = distribution(generator);
  }
  model.getInitModel()->setAllValues(lstm_vec, theta_vec);

  model.GPUSetup();

  auto buffer = model.getEmptyBufferMatrix(10);

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 2>> output_arr(num_rollouts);

  Eigen::Matrix<float, 8, num_rollouts> inputs = Eigen::Matrix<float, 8, num_rollouts>::Random();
  auto input = model.getLSTMModel()->getZeroInputVector();
  auto output = model.getLSTMModel()->getZeroOutputVector();

  for (int state_index = 0; state_index < num_rollouts; state_index++)
  {
    for (int dim = 0; dim < input_arr[0].size(); dim++)
    {
      input_arr[state_index][dim] = inputs.col(state_index)(dim);
    }
  }

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int step = 1; step < 6; step++)
    {
      buffer.setRandom();
      model.initializeLSTM(buffer);

      launchForwardTestKernel<LSTMHelper<false>, 8, 2, 32>(*model.getLSTMModel(), input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        // model.resetInitHiddenCPU();
        model.resetLSTMHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < input_dim; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < output_dim; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4)
              << "at index " << point << " with y_dim " << y_dim << " dim " << dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
        }
      }
    }
  }
}
//
// // TEST_F(LSTMLSTMHelperTest, TestComputeGradComputationFinite)
// // {
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>> model;
// //   std::vector<float> theta(1412);
// //   for (int i = 0; i < 1412; i++)
// //   {
// //     theta[i] = distribution(generator);
// //   }
// //   model.updateModel({ 6, 32, 32, 4 }, theta);
// //
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx numeric_jac;
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx analytic_jac;
// //
// //   for (int i = 0; i < 1000; i++)
// //   {
// //     LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::input_array input;
// //     input = LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::input_array::Random();
// //
// //     model.computeGrad(input, analytic_jac);
// //     EXPECT_TRUE(analytic_jac.allFinite());
// //   }
// // }
// //
// // TEST_F(LSTMLSTMHelperTest, TestComputeGradComputationCompare)
// // {
// //   GTEST_SKIP();
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>> model;
// //   std::vector<float> theta(1412);
// //   for (int i = 0; i < 1412; i++)
// //   {
// //     theta[i] = distribution(generator);
// //   }
// //   model.updateModel({ 6, 32, 32, 4 }, theta);
// //
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx numeric_jac;
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx analytic_jac;
// //
// //   LSTMLSTMHelper<LSTMParams<6, 32, 32, 4>>::input_array input;
// //   input << 1, 2, 3, 4, 5, 6;
// //
// //   model.computeGrad(input, analytic_jac);
// //
// //   // numeric_jac = num_diff.df(input, numeric_jac);
// //
// //   ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n"
// //                                                        << numeric_jac << "\nAnalytic Jacobian\n"
// //                                                        << analytic_jac;
// // }
