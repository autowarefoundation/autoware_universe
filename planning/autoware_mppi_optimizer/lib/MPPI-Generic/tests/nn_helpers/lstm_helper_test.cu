#include <gtest/gtest.h>

#include <mppi/utils/test_helper.h>
#include <random>
#include <algorithm>
#include <math.h>

#include <mppi/utils/nn_helpers/lstm_helper.cuh>
// Auto-generated header file
#include <test_networks.h>
#include <kernel_tests/utils/network_helper_kernel_test.cuh>
#include <mppi/utils/math_utils.h>
#include <unsupported/Eigen/NumericalDiff>
#include "mppi/ddp/ddp_dynamics.h"

class LSTMHelperTest : public testing::Test
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
};

TEST_F(LSTMHelperTest, ParamsConstructor1)
{
  std::vector<int> vec = { 30, 10, 3 };
  LSTMHelper<> model(5, 25, vec);

  EXPECT_EQ(model.getLSTMGrdSharedSizeBytes(), 12400);
  EXPECT_EQ(model.getGrdSharedSizeBytes(), 13808);
  EXPECT_EQ(model.getBlkSharedSizeBytes(), (25 * 2 + 30 * 2) * sizeof(float) + 8);

  EXPECT_EQ(model.getHiddenDim(), 25);
  EXPECT_EQ(model.getInputDim(), 5);
  EXPECT_EQ(model.getOutputDim(), 3);
  EXPECT_EQ(model.getHiddenHiddenSize(), 25 * 25);
  EXPECT_EQ(model.getInputHiddenSize(), 5 * 25);

  float* W_im = model.getWeights();
  float* W_fm = model.getWeights() + model.getHiddenHiddenSize();
  float* W_om = model.getWeights() + 2 * model.getHiddenHiddenSize();
  float* W_cm = model.getWeights() + 3 * model.getHiddenHiddenSize();
  for (int i = 0; i < 25 * 25; i++)
  {
    EXPECT_FLOAT_EQ(W_im[i], 0.0f);
    EXPECT_FLOAT_EQ(W_fm[i], 0.0f);
    EXPECT_FLOAT_EQ(W_cm[i], 0.0f);
    EXPECT_FLOAT_EQ(W_om[i], 0.0f);
  }

  float* W_ii = model.getWeights() + 4 * model.getHiddenHiddenSize();
  float* W_fi = model.getWeights() + 4 * model.getHiddenHiddenSize() + model.getInputHiddenSize();
  float* W_oi = model.getWeights() + 4 * model.getHiddenHiddenSize() + 2 * model.getInputHiddenSize();
  float* W_ci = model.getWeights() + 4 * model.getHiddenHiddenSize() + 3 * model.getInputHiddenSize();
  for (int i = 0; i < 5 * 25; i++)
  {
    EXPECT_FLOAT_EQ(W_ii[i], 0.0f);
    EXPECT_FLOAT_EQ(W_fi[i], 0.0f);
    EXPECT_FLOAT_EQ(W_oi[i], 0.0f);
    EXPECT_FLOAT_EQ(W_ci[i], 0.0f);
  }

  float* b_i = model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize();
  float* b_f =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + model.getHiddenDim();
  float* b_o =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 2 * model.getHiddenDim();
  float* b_c =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 3 * model.getHiddenDim();
  for (int i = 0; i < 25; i++)
  {
    EXPECT_FLOAT_EQ(b_i[i], 0.0f);
    EXPECT_FLOAT_EQ(b_f[i], 0.0f);
    EXPECT_FLOAT_EQ(b_o[i], 0.0f);
    EXPECT_FLOAT_EQ(b_c[i], 0.0f);
  }

  float* init_hidden =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 4 * model.getHiddenDim();
  float* init_cell =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 5 * model.getHiddenDim();
  for (int i = 0; i < 25; i++)
  {
    EXPECT_FLOAT_EQ(init_hidden[i], 0.0f);
    EXPECT_FLOAT_EQ(init_cell[i], 0.0f);
  }
}

TEST_F(LSTMHelperTest, ConfigConstructor)
{
  LSTMConfig config;
  config.input_dim = 5;
  config.hidden_dim = 25;
  config.output_layers = { 30, 10, 3 };
  LSTMHelper<> model(config);

  EXPECT_EQ(model.getLSTMGrdSharedSizeBytes(), 12400);
  EXPECT_EQ(model.getGrdSharedSizeBytes(), 13808);
  EXPECT_EQ(model.getBlkSharedSizeBytes(), (25 * 2 + 30 * 2) * sizeof(float) + 8);

  EXPECT_EQ(model.getHiddenDim(), 25);
  EXPECT_EQ(model.getInputDim(), 5);
  EXPECT_EQ(model.getOutputDim(), 3);
  EXPECT_EQ(model.getHiddenHiddenSize(), 25 * 25);
  EXPECT_EQ(model.getInputHiddenSize(), 5 * 25);

  float* W_im = model.getWeights();
  float* W_fm = model.getWeights() + model.getHiddenHiddenSize();
  float* W_om = model.getWeights() + 2 * model.getHiddenHiddenSize();
  float* W_cm = model.getWeights() + 3 * model.getHiddenHiddenSize();
  for (int i = 0; i < 25 * 25; i++)
  {
    EXPECT_FLOAT_EQ(W_im[i], 0.0f);
    EXPECT_FLOAT_EQ(W_fm[i], 0.0f);
    EXPECT_FLOAT_EQ(W_cm[i], 0.0f);
    EXPECT_FLOAT_EQ(W_om[i], 0.0f);
  }

  float* W_ii = model.getWeights() + 4 * model.getHiddenHiddenSize();
  float* W_fi = model.getWeights() + 4 * model.getHiddenHiddenSize() + model.getInputHiddenSize();
  float* W_oi = model.getWeights() + 4 * model.getHiddenHiddenSize() + 2 * model.getInputHiddenSize();
  float* W_ci = model.getWeights() + 4 * model.getHiddenHiddenSize() + 3 * model.getInputHiddenSize();
  for (int i = 0; i < 5 * 25; i++)
  {
    EXPECT_FLOAT_EQ(W_ii[i], 0.0f);
    EXPECT_FLOAT_EQ(W_fi[i], 0.0f);
    EXPECT_FLOAT_EQ(W_oi[i], 0.0f);
    EXPECT_FLOAT_EQ(W_ci[i], 0.0f);
  }

  float* b_i = model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize();
  float* b_f =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + model.getHiddenDim();
  float* b_o =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 2 * model.getHiddenDim();
  float* b_c =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 3 * model.getHiddenDim();
  for (int i = 0; i < 25; i++)
  {
    EXPECT_FLOAT_EQ(b_i[i], 0.0f);
    EXPECT_FLOAT_EQ(b_f[i], 0.0f);
    EXPECT_FLOAT_EQ(b_o[i], 0.0f);
    EXPECT_FLOAT_EQ(b_c[i], 0.0f);
  }

  float* init_hidden =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 4 * model.getHiddenDim();
  float* init_cell =
      model.getWeights() + 4 * model.getHiddenHiddenSize() + 4 * model.getInputHiddenSize() + 5 * model.getHiddenDim();
  for (int i = 0; i < 25; i++)
  {
    EXPECT_FLOAT_EQ(init_hidden[i], 0.0f);
    EXPECT_FLOAT_EQ(init_cell[i], 0.0f);
  }
}

TEST_F(LSTMHelperTest, ParamsConstructor2)
{
  int total_amount = 0;

  // delay model
  std::vector<int> delay_vec = { 2, 10, 1 };
  LSTMHelper<> delay(1, 1, delay_vec);
  total_amount += delay.getGrdSharedSizeBytes() / sizeof(float) + 1;
  total_amount += delay.getBlkSharedSizeBytes() * 32;
  // terra model
  std::vector<int> terra_vec = { 18, 10, 3 };
  LSTMHelper<> terra(8, 10, terra_vec);
  total_amount += terra.getGrdSharedSizeBytes() / sizeof(float) + 1;
  total_amount += terra.getBlkSharedSizeBytes() * 32;
  // engine model
  std::vector<int> engine_vec = { 9, 10, 1 };
  LSTMHelper<> engine(4, 5, engine_vec);
  total_amount += engine.getGrdSharedSizeBytes() / sizeof(float) + 1;
  total_amount += engine.getBlkSharedSizeBytes() * 32;
  // steering model
  std::vector<int> steering_vec = { 12, 20, 1 };
  LSTMHelper<> steering(7, 5, steering_vec);
  total_amount += steering.getGrdSharedSizeBytes() / sizeof(float) + 1;
  total_amount += steering.getBlkSharedSizeBytes() * 32;

  std::cout << "total amount: " << total_amount << std::endl;
  EXPECT_LT(total_amount, 49152);
}

TEST_F(LSTMHelperTest, BindStream)
{
  cudaStream_t stream;
  HANDLE_ERROR(cudaStreamCreate(&stream));

  std::vector<int> vec = { 30, 10, 3 };
  LSTMHelper<> helper(5, 25, vec, stream);

  EXPECT_EQ(helper.stream_, stream);
}

// using LSTM = LSTMHelper<LSTMParams<8, 20>, FNNParams<28, 3>>;
TEST_F(LSTMHelperTest, GPUSetupAndParamsCheck)
{
  std::vector<int> vec = { 30, 3 };
  LSTMHelper<> model(5, 25, vec);

  std::vector<float> theta_vec(93);
  for (int i = 0; i < 93; i++)
  {
    // theta_vec[i] = distribution(generator);
    theta_vec[i] = static_cast<float>(i);
  }
  model.updateOutputModel({ 30, 3 }, theta_vec);

  int grid_dim = 5;

  std::vector<float> lstm_params(grid_dim);
  std::vector<float> shared_lstm_params(grid_dim);
  std::vector<float> fnn_params(grid_dim);
  std::vector<float> shared_fnn_params(grid_dim);

  EXPECT_EQ(model.GPUMemStatus_, false);
  EXPECT_EQ(model.network_d_, nullptr);
  EXPECT_NE(model.getOutputModel(), nullptr);

  model.GPUSetup();

  EXPECT_EQ(model.GPUMemStatus_, true);
  EXPECT_NE(model.network_d_, nullptr);

  // launch kernel
  launchParameterCheckTestKernel<LSTMHelper<>>(model, lstm_params, shared_lstm_params, fnn_params, shared_fnn_params,
                                               grid_dim);

  EXPECT_EQ(model.getOutputGrdSharedSizeBytes(), 400);

  for (int grid = 0; grid < grid_dim; grid++)
  {
    // ensure that the output nn matches
    for (int i = 0; i < 93; i++)
    {
      EXPECT_FLOAT_EQ(fnn_params[grid * 100 + i], theta_vec[i]) << "at grid " << grid << " at index " << i;
    }
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 93], 0);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 94], 90);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 95], 30);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 96], 3);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 97], 0);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 98], 0);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 99], 0);

    const int hh = 25 * 25;
    const int ih = 5 * 25;
    int shift = (25 * 6 + hh * 4 + ih * 4) * grid;

    for (int i = 0; i < 25 * 25; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + hh + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 2 * hh + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 3 * hh + i], 0.0f);
    }

    for (int i = 0; i < 8 * 25; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + ih + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 2 * ih + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 3 * ih + i], 0.0f);
    }

    for (int i = 0; i < 25; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 25 + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 50 + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 75 + i], 0.0f);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 100 + i], 0.0f) << "at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 125 + i], 0.0f) << "at index " << i;
    }
  }

  for (int grid = 0; grid < grid_dim; grid++)
  {
    // ensure that the output nn matches
    for (int i = 0; i < 93; i++)
    {
      EXPECT_FLOAT_EQ(shared_fnn_params[grid * 100 + i], theta_vec[i]) << "at grid " << grid << " at index " << i;
    }
    // EXPECT_EQ(static_cast<int>(fnn_params[grid * 97 + 93]), 0);
    // EXPECT_EQ(static_cast<int>(fnn_params[grid * 97 + 94]), 90);
    // EXPECT_EQ(static_cast<int>(fnn_params[grid * 97 + 95]), 30);
    // EXPECT_EQ(static_cast<int>(fnn_params[grid * 97 + 96]), 3);

    const int hh = 25 * 25;
    const int ih = 5 * 25;
    int shift = (20 * 6 + hh * 4 + ih * 4) * grid;

    for (int i = 0; i < 25 * 25; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + hh + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 2 * hh + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 3 * hh + i], 0.0f);
    }

    for (int i = 0; i < 8 * 25; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + ih + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 2 * ih + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 3 * ih + i], 0.0f);
    }

    for (int i = 0; i < 25; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 25 + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 50 + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 75 + i], 0.0f);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 100 + i], 0.0f) << "at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 125 + i], 0.0f) << "at index " << i;
    }
  }
}

TEST_F(LSTMHelperTest, UpdateModel)
{
  std::vector<int> vec = { 30, 3 };
  LSTMHelper<> model(5, 25, vec);

  int grid_dim = 1;

  std::vector<float> theta_vec(93);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = static_cast<float>(i);
  }
  model.updateOutputModel({ 30, 3 }, theta_vec);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = static_cast<float>(i);
  }

  std::vector<float> lstm_params(grid_dim);
  std::vector<float> shared_lstm_params(grid_dim);
  std::vector<float> fnn_params(grid_dim);
  std::vector<float> shared_fnn_params(grid_dim);

  EXPECT_EQ(model.GPUMemStatus_, false);
  EXPECT_EQ(model.network_d_, nullptr);
  EXPECT_NE(model.getOutputModel(), nullptr);

  model.GPUSetup();

  EXPECT_EQ(model.GPUMemStatus_, true);
  EXPECT_NE(model.network_d_, nullptr);

  // launch kernel
  launchParameterCheckTestKernel<LSTMHelper<>>(model, lstm_params, shared_lstm_params, fnn_params, shared_fnn_params,
                                               grid_dim);

  EXPECT_EQ(model.getOutputGrdSharedSizeBytes(), 400);
  for (int grid = 0; grid < grid_dim; grid++)
  {
    // ensure that the output nn matches
    for (int i = 0; i < theta_vec.size(); i++)
    {
      EXPECT_FLOAT_EQ(fnn_params[grid * 100 + i], theta_vec[i]) << "at grid " << grid << " at index " << i;
    }
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 93], 0);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 94], 90);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 95], 30);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 96], 3);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 97], 0);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 98], 0);
    EXPECT_EQ(((int*)fnn_params.data())[grid * 100 + 99], 0);

    const int hh = 25 * 25;
    const int ih = 5 * 25;
    int shift = (25 * 6 + hh * 4 + ih * 4) * grid;

    for (int i = 0; i < 25 * 25; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + i], weights_d[i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + hh + i], weights_d[hh + i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 2 * hh + i], weights_d[2 * hh + i])
          << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 3 * hh + i], weights_d[3 * hh + i])
          << "at grid " << grid << " at index " << i;
    }

    for (int i = 0; i < 8 * 25; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + i], weights_d[4 * hh + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + ih + i], weights_d[4 * hh + ih + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 2 * ih + i], weights_d[4 * hh + 2 * ih + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 3 * ih + i], weights_d[4 * hh + 3 * ih + i]);
    }

    for (int i = 0; i < 25; i++)
    {
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + i], weights_d[4 * hh + 4 * ih + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 25 + i], weights_d[4 * hh + 4 * ih + 25 + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 50 + i], weights_d[4 * hh + 4 * ih + 50 + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 75 + i], weights_d[4 * hh + 4 * ih + 75 + i]);
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 100 + i], weights_d[4 * hh + 4 * ih + 100 + i])
          << "at index " << i;
      EXPECT_FLOAT_EQ(lstm_params[shift + 4 * hh + 4 * ih + 125 + i], weights_d[4 * hh + 4 * ih + 125 + i])
          << "at index " << i;
    }
  }

  for (int grid = 0; grid < grid_dim; grid++)
  {
    // ensure that the output nn matches
    for (int i = 0; i < 93; i++)
    {
      EXPECT_FLOAT_EQ(shared_fnn_params[grid * 97 + i], theta_vec[i]) << "at grid " << grid << " at index " << i;
    }
    // EXPECT_EQ((int)fnn_params[grid * 97 + 93], 0);
    // EXPECT_EQ((int)fnn_params[grid * 97 + 94], 90);
    // EXPECT_EQ((int)fnn_params[grid * 97 + 95], 30);
    // EXPECT_EQ((int)fnn_params[grid * 97 + 96], 3);

    const int hh = 25 * 25;
    const int ih = 5 * 25;
    int shift = (25 * 6 + hh * 4 + ih * 4) * grid;

    for (int i = 0; i < 25 * 25; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + i], weights_d[i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + hh + i], weights_d[hh + i]) << "at grid " << grid << " at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 2 * hh + i], weights_d[2 * hh + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 3 * hh + i], weights_d[3 * hh + i]);
    }

    for (int i = 0; i < 8 * 25; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + i], weights_d[4 * hh + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + ih + i], weights_d[4 * hh + ih + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 2 * ih + i], weights_d[4 * hh + 2 * ih + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 3 * ih + i], weights_d[4 * hh + 3 * ih + i]);
    }

    for (int i = 0; i < 25; i++)
    {
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + i], weights_d[4 * hh + 4 * ih + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 25 + i], weights_d[4 * hh + 4 * ih + 25 + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 50 + i], weights_d[4 * hh + 4 * ih + 50 + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 75 + i], weights_d[4 * hh + 4 * ih + 75 + i]);
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 100 + i], weights_d[4 * hh + 4 * ih + 100 + i])
          << "at index " << i;
      EXPECT_FLOAT_EQ(shared_lstm_params[shift + 4 * hh + 4 * ih + 125 + i], weights_d[4 * hh + 4 * ih + 125 + i])
          << "at index " << i;
    }
  }
}

TEST_F(LSTMHelperTest, LoadModelPathTest)
{
  std::vector<int> vec = { 2, 1 };
  LSTMHelper<> model(1, 1, vec);
  model.GPUSetup();

  std::string path = mppi::tests::test_lstm_lstm;
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  model.loadParams(path);

  cnpy::npz_t input_outputs = cnpy::npz_load(path);
  int num_points = input_outputs.at("num_points").data<int>()[0];
  std::cout << "got T " << input_outputs.at("T").data<double>()[0] << " dt " << input_outputs.at("dt").data<double>()[0]
            << std::endl;
  std::cout << "num points " << num_points << std::endl;
  int T = std::round(input_outputs.at("T").data<double>()[0] / input_outputs.at("dt").data<double>()[0]);
  EXPECT_EQ(num_points, 3);
  EXPECT_EQ(T, 5);

  double* inputs = input_outputs.at("input").data<double>();
  double* outputs = input_outputs.at("output").data<double>();
  double* init_hidden = input_outputs.at("init/hidden").data<double>();
  double* init_cell = input_outputs.at("init/cell").data<double>();
  double* hidden = input_outputs.at("hidden").data<double>();
  double* cell = input_outputs.at("cell").data<double>();

  double tol = 1e-5;

  auto input = model.getZeroInputVector();
  auto output = model.getZeroOutputVector();

  const int hidden_dim = model.getHiddenDim();
  const int input_dim = model.getInputDim();
  const int output_dim = model.getOutputDim();

  for (int point = 0; point < num_points; point++)
  {
    Eigen::VectorXf initial_hidden = Eigen::VectorXf(hidden_dim, 1);
    Eigen::VectorXf initial_cell = Eigen::VectorXf(hidden_dim, 1);
    for (int i = 0; i < hidden_dim; i++)
    {
      initial_hidden(i) = init_hidden[hidden_dim * point + i];
      initial_cell(i) = init_cell[hidden_dim * point + i];
    }
    model.updateLSTMInitialStates(initial_hidden, initial_cell);
    for (int t = 0; t < T; t++)
    {
      for (int i = 0; i < input_dim; i++)
      {
        input(i) = inputs[point * T * input_dim + t * input_dim + i];
      }
      model.forward(input, output);

      for (int i = 0; i < output_dim; i++)
      {
        EXPECT_NEAR(output(i), outputs[point * T * output_dim + t * output_dim + i], tol)
            << "t: " << t << " point " << point << " at dim " << i;
      }
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_NEAR(model.getHiddenState()(i), hidden[point * T * hidden_dim + hidden_dim * t + i], tol)
            << "t: " << t << " point " << point << " at dim " << i;
        EXPECT_NEAR(model.getCellState()(i), cell[point * T * hidden_dim + hidden_dim * t + i], tol)
            << "t: " << t << " point " << point << " at dim " << i;
      }
    }
  }
}

TEST_F(LSTMHelperTest, LoadModelPathInitTest)
{
  std::vector<int> vec = { 2, 1 };
  LSTMHelper<> model(1, 1, vec);
  model.GPUSetup();

  std::string path = mppi::tests::test_lstm_lstm;
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(path);
  model.loadParams("init_", param_dict, false);

  cnpy::npz_t input_outputs = cnpy::npz_load(path);
  int num_points = input_outputs.at("num_points").data<int>()[0];
  int T = std::round(input_outputs.at("tau").data<double>()[0] / input_outputs.at("dt").data<double>()[0]) + 1;
  double* init_inputs = input_outputs.at("init_input").data<double>();
  double* init_hidden = input_outputs.at("init/hidden").data<double>();
  double* init_cell = input_outputs.at("init/cell").data<double>();
  // TODO not the right config to load this data essentially
  // double* init_step_hidden = input_outputs.at("init_step_hidden").data<double>();
  // double* init_step_cell = input_outputs.at("init_step_cell").data<double>();
  EXPECT_EQ(num_points, 3);
  EXPECT_EQ(T, 6);

  double tol = 1e-5;

  auto input = model.getZeroInputVector();
  auto output = model.getZeroOutputVector();

  const int hidden_dim = model.getHiddenDim();
  const int input_dim = model.getInputDim();
  const int output_dim = model.getOutputDim();

  EXPECT_EQ(hidden_dim, 60);
  EXPECT_EQ(input_dim, 3);
  EXPECT_EQ(output_dim, 50);

  for (int point = 0; point < num_points; point++)
  {
    // run the init network and ensure initial hidden/cell match
    model.resetHiddenCellCPU();
    for (int t = 0; t < T; t++)
    {
      for (int i = 0; i < input_dim; i++)
      {
        input(i) = init_inputs[point * T * input_dim + t * input_dim + i];
      }

      model.forward(input, output);
      // for (int i = 0; i < 60; i++)
      // {
      //   EXPECT_NEAR(model.getHiddenState()(i), init_step_hidden[60 * point * T + t * 60 + i], tol)
      //       << "at t " << t << " dim " << i;
      //   EXPECT_NEAR(model.getCellState()(i), init_step_cell[60 * point * T + t * 60 + i], tol)
      //       << "at t " << t << " dim " << i;
      // }
    }
    for (int i = 0; i < 25; i++)
    {
      EXPECT_NEAR(output(i), init_hidden[25 * point + i], tol)
          << "incorrect hidden at point " << point << " index " << i;
      EXPECT_NEAR(output(i + 25), init_cell[25 * point + i], tol)
          << "incorrect cell at point " << point << " index " << i;
    }
  }
}

TEST_F(LSTMHelperTest, LoadModelNPZTest)
{
  std::vector<int> vec = { 2, 1 };
  LSTMHelper<> model(1, 1, vec);
  model.GPUSetup();

  std::string path = mppi::tests::test_lstm_lstm;
  if (!fileExists(path))
  {
    std::cerr << "Could not load neural net model at path: " << path.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(path);
  model.loadParams(param_dict);

  cnpy::npz_t input_outputs = cnpy::npz_load(path);
  int num_points = input_outputs.at("num_points").data<int>()[0];
  int T = std::round(input_outputs.at("T").data<float>()[0] / input_outputs.at("dt").data<float>()[0]);
  double* inputs = input_outputs.at("input").data<double>();
  double* outputs = input_outputs.at("output").data<double>();
  double* init_hidden = input_outputs.at("init/hidden").data<double>();
  double* init_cell = input_outputs.at("init/cell").data<double>();
  double* hidden = input_outputs.at("hidden").data<double>();
  double* cell = input_outputs.at("cell").data<double>();

  double tol = 1e-5;

  auto input = model.getZeroInputVector();
  auto output = model.getZeroOutputVector();

  const int hidden_dim = model.getHiddenDim();
  const int input_dim = model.getInputDim();
  const int output_dim = model.getOutputDim();

  for (int point = 0; point < num_points; point++)
  {
    Eigen::VectorXf initial_hidden = Eigen::VectorXf(hidden_dim, 1);
    Eigen::VectorXf initial_cell = Eigen::VectorXf(hidden_dim, 1);
    for (int i = 0; i < hidden_dim; i++)
    {
      initial_hidden(i) = init_hidden[hidden_dim * point + i];
      initial_cell(i) = init_cell[hidden_dim * point + i];
    }
    model.updateLSTMInitialStates(initial_hidden, initial_cell);
    for (int t = 0; t < T; t++)
    {
      for (int i = 0; i < 3; i++)
      {
        input(i) = inputs[point * T * 3 + t * 3 + i];
      }
      model.forward(input, output);

      for (int i = 0; i < 2; i++)
      {
        EXPECT_NEAR(output[i], outputs[point * T * 2 + t * 2 + i], tol) << "point " << point << " at dim " << i;
      }
      for (int i = 0; i < 25; i++)
      {
        EXPECT_NEAR(model.getHiddenState()(i), hidden[point * T * 25 + 25 * t + i], tol)
            << "point " << point << " at dim " << i;
        EXPECT_NEAR(model.getCellState()(i), cell[point * T * 25 + 25 * t + i], tol)
            << "point " << point << " at dim " << i;
      }
    }
  }
}

TEST_F(LSTMHelperTest, forwardCPU)
{
  std::vector<int> vec = { 28, 3 };
  LSTMHelper<> model(8, 20, vec);

  std::vector<float> theta_vec(87);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = 1.0;
  }
  model.updateOutputModel({ 28, 3 }, theta_vec);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = 1.0f;
  }
  model.resetHiddenCellCPU();

  auto input = model.getZeroInputVector();
  auto output = model.getZeroOutputVector();
  input.setOnes();

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 28.28055);
  EXPECT_FLOAT_EQ(output[1], 28.28055);
  EXPECT_FLOAT_EQ(output[2], 28.28055);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 28.901096);
  EXPECT_FLOAT_EQ(output[1], 28.901096);
  EXPECT_FLOAT_EQ(output[2], 28.901096);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 28.986588);
  EXPECT_FLOAT_EQ(output[1], 28.986588);
  EXPECT_FLOAT_EQ(output[2], 28.986588);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 28.998184);
  EXPECT_FLOAT_EQ(output[1], 28.998184);
  EXPECT_FLOAT_EQ(output[2], 28.998184);

  model.forward(input, output);
  EXPECT_FLOAT_EQ(output[0], 28.999756);
  EXPECT_FLOAT_EQ(output[1], 28.999756);
  EXPECT_FLOAT_EQ(output[2], 28.999756);
}

TEST_F(LSTMHelperTest, forwardGPU)
{
  const int num_rollouts = 1000;

  std::vector<int> vec = { 28, 3 };
  LSTMHelper<> model(8, 20, vec);

  std::vector<float> theta(87);
  std::fill(theta.begin(), theta.end(), 1);
  model.updateOutputModel({ 28, 3 }, theta);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = 1.0f;
  }
  model.resetHiddenCellCPU();

  model.GPUSetup();

  Eigen::Matrix<float, 8, num_rollouts> inputs;
  inputs = Eigen::Matrix<float, 8, num_rollouts>::Ones();
  Eigen::VectorXf output = model.getZeroOutputVector();
  Eigen::VectorXf input = model.getZeroInputVector();

  std::array<float, 5> true_vals = { 28.28055, 28.901096, 28.986588, 28.998184, 28.999756 };

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 3>> output_arr(num_rollouts);

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int state_index = 0; state_index < num_rollouts; state_index++)
    {
      for (int dim = 0; dim < input_arr[0].size(); dim++)
      {
        input_arr[state_index][dim] = inputs.col(state_index)(dim);
      }
    }

    for (int step = 1; step < 6; step++)
    {
      launchForwardTestKernel<LSTMHelper<>, 8, 3, 32>(model, input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        model.resetHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < 8; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < 3; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
          EXPECT_FLOAT_EQ(output(dim), true_vals[step - 1]) << "at dim " << dim << " step " << step;
        }
      }
    }
  }
}

TEST_F(LSTMHelperTest, forwardGPUCompareNoShared)
{
  const int num_rollouts = 1000;

  std::vector<int> vec = { 28, 3 };
  LSTMHelper<false> model(8, 20, vec);
  std::vector<float> theta_vec(87);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  model.updateOutputModel({ 28, 3 }, theta_vec);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = distribution(generator);
  }
  model.resetHiddenCellCPU();

  model.GPUSetup();

  Eigen::Matrix<float, 8, num_rollouts> inputs;
  inputs = Eigen::Matrix<float, 8, num_rollouts>::Random();
  Eigen::VectorXf output = model.getZeroOutputVector();
  Eigen::VectorXf input = model.getZeroInputVector();

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 3>> output_arr(num_rollouts);

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int state_index = 0; state_index < num_rollouts; state_index++)
    {
      for (int dim = 0; dim < input_arr[0].size(); dim++)
      {
        input_arr[state_index][dim] = inputs.col(state_index)(dim);
      }
    }
    for (int step = 1; step < 6; step++)
    {
      launchForwardTestKernel<LSTMHelper<false>, 8, 3, 32>(model, input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        model.resetHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < 8; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < 3; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
        }
      }
    }
  }
}

TEST_F(LSTMHelperTest, forwardGPUCompareShared)
{
  const int num_rollouts = 1000;

  std::vector<int> vec = { 28, 3 };
  LSTMHelper<> model(8, 20, vec);
  std::vector<float> theta_vec(87);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  model.updateOutputModel({ 28, 3 }, theta_vec);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = distribution(generator);
  }
  model.resetHiddenCellCPU();

  model.GPUSetup();

  Eigen::Matrix<float, 8, num_rollouts> inputs;
  inputs = Eigen::Matrix<float, 8, num_rollouts>::Random();
  Eigen::VectorXf output = model.getZeroOutputVector();
  Eigen::VectorXf input = model.getZeroInputVector();

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 3>> output_arr(num_rollouts);

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int state_index = 0; state_index < num_rollouts; state_index++)
    {
      for (int dim = 0; dim < input_arr[0].size(); dim++)
      {
        input_arr[state_index][dim] = inputs.col(state_index)(dim);
      }
    }
    for (int step = 1; step < 6; step++)
    {
      launchForwardTestKernel<LSTMHelper<>, 8, 3, 32>(model, input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        model.resetHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < 8; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < 3; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
        }
      }
    }
  }
}

TEST_F(LSTMHelperTest, forwardGPUComparePreloadNoShared)
{
  const int num_rollouts = 1000;

  std::vector<int> vec = { 28, 3 };
  LSTMHelper<false> model(8, 20, vec);
  std::vector<float> theta_vec(87);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  model.updateOutputModel({ 28, 3 }, theta_vec);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = distribution(generator);
  }
  model.resetHiddenCellCPU();

  model.GPUSetup();

  Eigen::Matrix<float, 8, num_rollouts> inputs;
  inputs = Eigen::Matrix<float, 8, num_rollouts>::Random();
  Eigen::VectorXf output = model.getZeroOutputVector();
  Eigen::VectorXf input = model.getZeroInputVector();

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 3>> output_arr(num_rollouts);

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int state_index = 0; state_index < num_rollouts; state_index++)
    {
      for (int dim = 0; dim < input_arr[0].size(); dim++)
      {
        input_arr[state_index][dim] = inputs.col(state_index)(dim);
      }
    }
    for (int step = 1; step < 6; step++)
    {
      launchForwardTestKernelPreload<LSTMHelper<false>, 8, 3, 32>(model, input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        model.resetHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < 8; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < 3; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
        }
      }
    }
  }
}

TEST_F(LSTMHelperTest, forwardGPUComparePreloadShared)
{
  const int num_rollouts = 1000;

  std::vector<int> vec = { 28, 3 };
  LSTMHelper<> model(8, 20, vec);
  std::vector<float> theta_vec(87);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  model.updateOutputModel({ 28, 3 }, theta_vec);

  float* weights_d = model.getWeights();
  for (int i = 0; i < model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * model.getHiddenDim(); i++)
  {
    weights_d[i] = distribution(generator);
  }
  model.resetHiddenCellCPU();

  model.GPUSetup();

  Eigen::Matrix<float, 8, num_rollouts> inputs;
  inputs = Eigen::Matrix<float, 8, num_rollouts>::Random();
  Eigen::VectorXf output = model.getZeroOutputVector();
  Eigen::VectorXf input = model.getZeroInputVector();

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 3>> output_arr(num_rollouts);

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    for (int state_index = 0; state_index < num_rollouts; state_index++)
    {
      for (int dim = 0; dim < input_arr[0].size(); dim++)
      {
        input_arr[state_index][dim] = inputs.col(state_index)(dim);
      }
    }
    for (int step = 1; step < 6; step++)
    {
      launchForwardTestKernelPreload<LSTMHelper<>, 8, 3, 32>(model, input_arr, output_arr, y_dim, step);
      for (int point = 0; point < num_rollouts; point++)
      {
        model.resetHiddenCellCPU();
        input = inputs.col(point);

        for (int cpu_step = 0; cpu_step < step; cpu_step++)
        {
          model.forward(input, output);
        }
        for (int dim = 0; dim < 8; dim++)
        {
          EXPECT_NEAR(input(dim), input_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
        }
        for (int dim = 0; dim < 3; dim++)
        {
          EXPECT_NEAR(output(dim), output_arr[point][dim], 1e-4) << "at index " << point << " with y_dim " << y_dim;
          EXPECT_TRUE(isfinite(output_arr[point][dim]));
        }
      }
    }
  }
}

TEST_F(LSTMHelperTest, forwardGPUSpeedTest)
{
  const int num_rollouts = 15000;

  std::vector<int> vec = { 28, 3 };
  LSTMHelper<> shared_model(8, 20, vec);
  LSTMHelper<false> global_model(8, 20, vec);
  std::vector<float> theta_vec(87);
  for (int i = 0; i < theta_vec.size(); i++)
  {
    theta_vec[i] = distribution(generator);
  }
  shared_model.updateOutputModel({ 28, 3 }, theta_vec);
  global_model.updateOutputModel({ 28, 3 }, theta_vec);

  float* shared_weights_d = shared_model.getWeights();
  float* global_weights_d = global_model.getWeights();
  for (int i = 0; i < shared_model.getLSTMGrdSharedSizeBytes() / sizeof(float) + 2 * shared_model.getHiddenDim(); i++)
  {
    shared_weights_d[i] = distribution(generator);
    global_weights_d[i] = shared_weights_d[i];
  }
  shared_model.resetHiddenCellCPU();

  shared_model.GPUSetup();

  global_model.resetHiddenCellCPU();

  global_model.GPUSetup();

  Eigen::Matrix<float, 8, 1000> inputs;
  inputs = Eigen::Matrix<float, 8, 1000>::Random();
  Eigen::VectorXf output = shared_model.getZeroOutputVector();
  Eigen::VectorXf input = shared_model.getZeroInputVector();

  std::vector<std::array<float, 8>> input_arr(num_rollouts);
  std::vector<std::array<float, 3>> output_arr(num_rollouts);

  for (int state_index = 0; state_index < num_rollouts; state_index++)
  {
    for (int dim = 0; dim < input_arr[0].size(); dim++)
    {
      input_arr[state_index][dim] = inputs.col(state_index % 1000)(dim);
    }
  }

  for (int y_dim = 1; y_dim < 16; y_dim++)
  {
    // TODO the way of doing this timing is bad
    auto shared_start = std::chrono::steady_clock::now();
    launchForwardTestKernel<LSTMHelper<>, 8, 3, 32>(shared_model, input_arr, output_arr, y_dim, 500);
    auto shared_stop = std::chrono::steady_clock::now();

    auto global_start = std::chrono::steady_clock::now();
    launchForwardTestKernel<LSTMHelper<false>, 8, 3, 32>(global_model, input_arr, output_arr, y_dim, 500);
    auto global_stop = std::chrono::steady_clock::now();

    float shared_time_ms = mppi::math::timeDiffms(shared_stop, shared_start);
    float global_time_ms = mppi::math::timeDiffms(global_stop, global_start);
    std::cout << "for y dim " << y_dim << " got shared: " << shared_time_ms << std::endl;
    std::cout << "for y dim " << y_dim << " got global: " << global_time_ms << std::endl;
  }
}

// TEST_F(LSTMHelperTest, TestComputeGradComputationFinite)
// {
//   LSTMHelper<LSTMParams<6, 32, 32, 4>> model;
//   std::vector<float> theta(1412);
//   for (int i = 0; i < 1412; i++)
//   {
//     theta[i] = distribution(generator);
//   }
//   model.updateModel({ 6, 32, 32, 4 }, theta);
//
//   LSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx numeric_jac;
//   LSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx analytic_jac;
//
//   for (int i = 0; i < 1000; i++)
//   {
//     LSTMHelper<LSTMParams<6, 32, 32, 4>>::input_array input;
//     input = LSTMHelper<LSTMParams<6, 32, 32, 4>>::input_array::Random();
//
//     model.computeGrad(input, analytic_jac);
//     EXPECT_TRUE(analytic_jac.allFinite());
//   }
// }
//
// TEST_F(LSTMHelperTest, TestComputeGradComputationCompare)
// {
//   GTEST_SKIP();
//   LSTMHelper<LSTMParams<6, 32, 32, 4>> model;
//   std::vector<float> theta(1412);
//   for (int i = 0; i < 1412; i++)
//   {
//     theta[i] = distribution(generator);
//   }
//   model.updateModel({ 6, 32, 32, 4 }, theta);
//
//   LSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx numeric_jac;
//   LSTMHelper<LSTMParams<6, 32, 32, 4>>::dfdx analytic_jac;
//
//   LSTMHelper<LSTMParams<6, 32, 32, 4>>::input_array input;
//   input << 1, 2, 3, 4, 5, 6;
//
//   model.computeGrad(input, analytic_jac);
//
//   // numeric_jac = num_diff.df(input, numeric_jac);
//
//   ASSERT_LT((numeric_jac - analytic_jac).norm(), 1e-3) << "Numeric Jacobian\n"
//                                                        << numeric_jac << "\nAnalytic Jacobian\n"
//                                                        << analytic_jac;
// }
