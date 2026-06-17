#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <mppi/dynamics/racer_dubins/racer_dubins_elevation_lstm_unc.cuh>
#include <kernel_tests/dynamics/dynamics_generic_kernel_tests.cuh>
#include <mppi/ddp/ddp_model_wrapper.h>
#include <racer_test_networks.h>
#include <cuda_runtime.h>
#include <mppi/core/mppi_common.cuh>

class RacerDubinsElevationLSTMUncertaintyTest : public ::testing::Test
{
public:
  cudaStream_t stream;
  cudaExtent extent = make_cudaExtent(10, 20, 0);
  float map_resolution = 10.0f;          // [m / px]
  float3 origin = make_float3(1, 2, 3);  // [m, m, m]

  std::vector<float> data_vec;
  std::vector<float4> normal_vec;

  void SetUp() override
  {
    CudaCheckError();
    HANDLE_ERROR(cudaStreamCreate(&stream));

    steer_config.init_config.input_dim = 3;
    steer_config.init_config.hidden_dim = 20;
    steer_config.init_config.output_layers = { 23, 100, 8 };
    steer_config.pred_config.input_dim = 4;
    steer_config.pred_config.hidden_dim = 4;
    steer_config.pred_config.output_layers = { 8, 20, 1 };
    steer_config.init_len = 11;

    unc_config.init_config.input_dim = 10;
    unc_config.init_config.hidden_dim = 20;
    unc_config.init_config.output_layers = { 30, 100, 8 };
    unc_config.pred_config.input_dim = 13;
    unc_config.pred_config.hidden_dim = 4;
    unc_config.pred_config.output_layers = { 17, 20, 5 };
    unc_config.init_len = 11;

    mean_config.init_config.input_dim = 10;
    mean_config.init_config.hidden_dim = 20;
    mean_config.init_config.output_layers = { 30, 100, 8 };
    mean_config.pred_config.input_dim = 12;
    mean_config.pred_config.hidden_dim = 4;
    mean_config.pred_config.output_layers = { 16, 20, 2 };
    mean_config.init_len = 11;

    data_vec.resize(extent.width * extent.height);
    normal_vec.resize(extent.width * extent.height);
    for (int i = 0; i < data_vec.size(); i++)
    {
      data_vec[i] = i * 1.0f;
      Eigen::Vector4f random_normal = Eigen::Vector4f::Random();
      normal_vec[i] = make_float4(random_normal.x(), random_normal.y(), random_normal.z(), random_normal.w());
    }
  }

  void TearDown() override
  {
    CudaCheckError();
    HANDLE_ERROR(cudaStreamDestroy(stream));
  }

  LSTMLSTMConfig steer_config;
  LSTMLSTMConfig mean_config;
  LSTMLSTMConfig unc_config;
};

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, Template)
{
  auto dynamics = RacerDubinsElevationLSTMUncertainty(steer_config, mean_config, unc_config);
  EXPECT_EQ(26, RacerDubinsElevationLSTMUncertainty::STATE_DIM);
  EXPECT_EQ(2, RacerDubinsElevationLSTMUncertainty::CONTROL_DIM);
  EXPECT_TRUE(dynamics.checkRequiresBuffer());
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_NE(dynamics.getHelper(), nullptr);
  EXPECT_NE(dynamics.getUncertaintyHelper(), nullptr);
  EXPECT_NE(dynamics.getMeanHelper(), nullptr);
}

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, BindStreamConstructor)
{
  auto dynamics = RacerDubinsElevationLSTMUncertainty(steer_config, mean_config, unc_config, stream);

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics.getTextureHelper()->stream_, stream);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->stream_, stream);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->stream_, stream);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->stream_, stream);
}

// TEST_F(RacerDubinsElevationLSTMUncertaintyTest, Deconstructor)
// {
//   {
//     auto dynamics = RacerDubinsElevationLSTMUncertainty(steer_config, mean_config, unc_config);
//     dynamics = RacerDubinsElevationLSTMUncertainty(steer_config, mean_config, unc_config);
//   }
// }

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, BindStream)
{
  auto dynamics = RacerDubinsElevationLSTMUncertainty(steer_config, mean_config, unc_config);
  dynamics.bindToStream(stream);

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics.getTextureHelper()->stream_, stream);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->stream_, stream);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->stream_, stream);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->stream_, stream);
}

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, GPUSetup)
{
  auto dynamics = RacerDubinsElevationLSTMUncertainty(steer_config, mean_config, unc_config, stream);

  dynamics.GPUSetup();

  EXPECT_EQ(dynamics.stream_, stream) << "Stream binding failure.";
  EXPECT_NE(dynamics.getTextureHelper(), nullptr);
  EXPECT_EQ(dynamics.getTextureHelper()->stream_, stream);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->stream_, stream);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->stream_, stream);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->stream_, stream);

  EXPECT_EQ(dynamics.GPUMemStatus_, true);
  EXPECT_NE(dynamics.model_d_, nullptr);

  // steering model
  EXPECT_NE(dynamics.network_d_, nullptr);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->GPUMemStatus_, true);
  EXPECT_NE(dynamics.getHelper()->getLSTMDevicePtr(), nullptr);
  EXPECT_NE(dynamics.getHelper()->getLSTMModel()->network_d_, nullptr);

  // mean model
  EXPECT_NE(dynamics.mean_d_, nullptr);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->GPUMemStatus_, true);
  EXPECT_NE(dynamics.getMeanHelper()->getLSTMDevicePtr(), nullptr);
  EXPECT_NE(dynamics.getMeanHelper()->getLSTMModel()->network_d_, nullptr);

  // mean model
  EXPECT_NE(dynamics.uncertainty_d_, nullptr);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->GPUMemStatus_, true);
  EXPECT_NE(dynamics.getUncertaintyHelper()->getLSTMDevicePtr(), nullptr);
  EXPECT_NE(dynamics.getUncertaintyHelper()->getLSTMModel()->network_d_, nullptr);
}

// TODO test case that checks that I get the same as base class here

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, TestLoadParams)
{
  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMUncertainty;
  RacerDubinsElevationLSTMUncertainty dynamics =
      RacerDubinsElevationLSTMUncertainty(mppi::tests::racer_dubins_elevation_uncertainty_test);
  EXPECT_FLOAT_EQ(dynamics.getParams().max_steer_angle, 5.0);
  EXPECT_FLOAT_EQ(dynamics.getParams().max_steer_rate, 17.590296);
  EXPECT_FLOAT_EQ(dynamics.getParams().steering_constant, 3.286375);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_accel_constant, 9.30152702);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_accel_drag_constant, -0.60327667);

  EXPECT_FLOAT_EQ(dynamics.getParams().max_brake_rate_pos, 10);
  EXPECT_FLOAT_EQ(dynamics.getParams().max_brake_rate_neg, 10);
  EXPECT_FLOAT_EQ(dynamics.getParams().pos_quad_brake_c[0], 3);
  EXPECT_FLOAT_EQ(dynamics.getParams().pos_quad_brake_c[1], 0.1);
  EXPECT_FLOAT_EQ(dynamics.getParams().pos_quad_brake_c[2], 0.48);
  EXPECT_FLOAT_EQ(dynamics.getParams().neg_quad_brake_c[0], 5.8);
  EXPECT_FLOAT_EQ(dynamics.getParams().neg_quad_brake_c[1], 0.1);
  EXPECT_FLOAT_EQ(dynamics.getParams().neg_quad_brake_c[2], 1.4);

  EXPECT_FLOAT_EQ(dynamics.getParams().c_t[0], 3.0364573);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_t[1], 4.59772491);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_t[2], 4.06954288);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_b[0], 2.56373692);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_b[1], 6.18813848);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_b[2], 25.52443695);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_v[0], 4.39438224e-01);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_v[1], 2.37689335e-02);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_v[2], 2.12573977e-05);
  EXPECT_FLOAT_EQ(dynamics.getParams().steer_angle_scale, -13.25719738);
  EXPECT_FLOAT_EQ(dynamics.getParams().gravity, -3.1667469);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[0], 0.5);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[1], 0.2);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[2], 0.5);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[3], 0.02);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[4], 0.02);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[5], 0.1);
  EXPECT_FLOAT_EQ(dynamics.getParams().unc_scale[6], 0.1);

  EXPECT_FLOAT_EQ(dynamics.getParams().low_min_throttle, 0.0);
  EXPECT_FLOAT_EQ(dynamics.getParams().c_0, 0.0);
  EXPECT_FLOAT_EQ(dynamics.getParams().wheel_base, 3.0);
  EXPECT_GT(dynamics.getParams().clamp_ax, 10.0f);

  EXPECT_EQ(dynamics.getHelper()->getInitModel()->getInputDim(), 3);
  EXPECT_EQ(dynamics.getHelper()->getInitModel()->getOutputDim(), 8);
  EXPECT_EQ(dynamics.getHelper()->getInitModel()->getHiddenDim(), 20);
  EXPECT_EQ(dynamics.getHelper()->getInitModel()->getOutputModel()->getInputDim(), 23);
  EXPECT_EQ(dynamics.getHelper()->getInitModel()->getOutputModel()->getOutputDim(), 8);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->getInputDim(), 4);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->getOutputDim(), 1);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->getHiddenDim(), 4);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->getOutputModel()->getInputDim(), 8);
  EXPECT_EQ(dynamics.getHelper()->getLSTMModel()->getOutputModel()->getOutputDim(), 1);
  EXPECT_EQ(dynamics.getHelper()->getInitLen(), 11);

  EXPECT_EQ(dynamics.getMeanHelper()->getInitModel()->getInputDim(), 9);
  EXPECT_EQ(dynamics.getMeanHelper()->getInitModel()->getOutputDim(), 8);
  EXPECT_EQ(dynamics.getMeanHelper()->getInitModel()->getHiddenDim(), 20);
  EXPECT_EQ(dynamics.getMeanHelper()->getInitModel()->getOutputModel()->getInputDim(), 29);
  EXPECT_EQ(dynamics.getMeanHelper()->getInitModel()->getOutputModel()->getOutputDim(), 8);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->getInputDim(), 11);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->getOutputDim(), 2);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->getHiddenDim(), 4);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->getOutputModel()->getInputDim(), 15);
  EXPECT_EQ(dynamics.getMeanHelper()->getLSTMModel()->getOutputModel()->getOutputDim(), 2);
  EXPECT_EQ(dynamics.getMeanHelper()->getInitLen(), 11);

  EXPECT_EQ(dynamics.getUncertaintyHelper()->getInitModel()->getInputDim(), 10);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getInitModel()->getOutputDim(), 8);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getInitModel()->getHiddenDim(), 20);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getInitModel()->getOutputModel()->getInputDim(), 30);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getInitModel()->getOutputModel()->getOutputDim(), 8);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->getInputDim(), 12);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->getOutputDim(), 7);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->getHiddenDim(), 4);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->getOutputModel()->getInputDim(), 16);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getLSTMModel()->getOutputModel()->getOutputDim(), 7);
  EXPECT_EQ(dynamics.getUncertaintyHelper()->getInitLen(), 11);
}

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, TestGPUvsCPU)
{
  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMUncertainty;
  RacerDubinsElevationLSTMUncertainty dynamics =
      RacerDubinsElevationLSTMUncertainty(mppi::tests::racer_dubins_elevation_uncertainty_test);

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);

  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  TwoDTextureHelper<float4>* normal_helper = dynamics.getTextureHelperNormals();
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, origin);
  helper->updateTexture(0, data_vec, extent);
  helper->updateResolution(0, map_resolution);
  helper->enableTexture(0);
  helper->copyToDevice(false);
  normal_helper->updateRotation(0, new_rot_mat);
  normal_helper->updateOrigin(0, origin);
  normal_helper->updateTexture(0, normal_vec, extent);
  normal_helper->updateResolution(0, map_resolution);
  normal_helper->enableTexture(0);
  normal_helper->copyToDevice(true);

  DYN::buffer_trajectory buffer;
  buffer["VEL_X"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
  buffer["CAN_STEER_CMD"] = Eigen::VectorXf::Random(51);
  buffer["ROLL"] = Eigen::VectorXf::Random(51);
  buffer["PITCH"] = Eigen::VectorXf::Random(51);
  buffer["CAN_THROTTLE_CMD"] = Eigen::VectorXf::Random(51);
  buffer["BRAKE_STATE"] = Eigen::VectorXf::Random(51);
  buffer["CAN_BRAKE_CMD"] = Eigen::VectorXf::Random(51);
  buffer["OMEGA_Z"] = Eigen::VectorXf::Random(51);

  checkGPUComputationStep<RacerDubinsElevationLSTMUncertainty>(dynamics, 0.02f, 16, 32, buffer, 1.0e-4);
}

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, TestStepGPUvsCPUReverse)
{
  using DYN = RacerDubinsElevationLSTMUncertainty;

  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMUncertainty;
  RacerDubinsElevationLSTMUncertainty dynamics =
      RacerDubinsElevationLSTMUncertainty(mppi::tests::racer_dubins_elevation_uncertainty_test);
  auto params = dynamics.getParams();
  params.gear_sign = -1;
  dynamics.setParams(params);

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);

  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  TwoDTextureHelper<float4>* normal_helper = dynamics.getTextureHelperNormals();
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, origin);
  helper->updateTexture(0, data_vec, extent);
  helper->updateResolution(0, map_resolution);
  helper->enableTexture(0);
  helper->copyToDevice(false);
  normal_helper->updateRotation(0, new_rot_mat);
  normal_helper->updateOrigin(0, origin);
  normal_helper->updateTexture(0, normal_vec, extent);
  normal_helper->updateResolution(0, map_resolution);
  normal_helper->enableTexture(0);
  normal_helper->copyToDevice(true);

  DYN::buffer_trajectory buffer;
  buffer["VEL_X"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
  buffer["CAN_STEER_CMD"] = Eigen::VectorXf::Random(51);
  buffer["ROLL"] = Eigen::VectorXf::Random(51);
  buffer["PITCH"] = Eigen::VectorXf::Random(51);
  buffer["CAN_THROTTLE_CMD"] = Eigen::VectorXf::Random(51);
  buffer["BRAKE_STATE"] = Eigen::VectorXf::Random(51);
  buffer["CAN_BRAKE_CMD"] = Eigen::VectorXf::Random(51);
  buffer["OMEGA_Z"] = Eigen::VectorXf::Random(51);

  checkGPUComputationStep<RacerDubinsElevationLSTMUncertainty>(dynamics, 0.02f, 16, 32, buffer, 1.0e-2);
}

TEST_F(RacerDubinsElevationLSTMUncertaintyTest, TestMatchesPython)
{
  using DYN = RacerDubinsElevationLSTMUncertainty;

  CudaCheckError();
  using DYN = RacerDubinsElevationLSTMUncertainty;
  RacerDubinsElevationLSTMUncertainty dynamics =
      RacerDubinsElevationLSTMUncertainty(mppi::tests::racer_dubins_elevation_uncertainty_test);
  auto params = dynamics.getParams();
  params.K_x = 0.0f;
  params.K_y = 0.0f;
  params.K_yaw = 0.0f;
  params.K_vel_x = 0.0f;
  params.wheel_base = 3.0f;
  dynamics.setParams(params);

  std::array<float3, 3> new_rot_mat{};
  new_rot_mat[0] = make_float3(0, 1, 0);
  new_rot_mat[1] = make_float3(1, 0, 0);
  new_rot_mat[2] = make_float3(0, 0, 1);

  TwoDTextureHelper<float>* helper = dynamics.getTextureHelper();
  TwoDTextureHelper<float4>* normal_helper = dynamics.getTextureHelperNormals();
  helper->updateRotation(0, new_rot_mat);
  helper->updateOrigin(0, origin);
  helper->updateTexture(0, data_vec, extent);
  helper->updateResolution(0, map_resolution);
  helper->enableTexture(0);
  helper->copyToDevice(false);
  normal_helper->updateRotation(0, new_rot_mat);
  normal_helper->updateOrigin(0, origin);
  normal_helper->updateTexture(0, normal_vec, extent);
  normal_helper->updateResolution(0, map_resolution);
  normal_helper->enableTexture(0);
  normal_helper->copyToDevice(true);

  DYN::buffer_trajectory buffer;
  buffer["VEL_X"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE"] = Eigen::VectorXf::Random(51);
  buffer["STEER_ANGLE_RATE"] = Eigen::VectorXf::Random(51);
  buffer["CAN_STEER_CMD"] = Eigen::VectorXf::Random(51);
  buffer["ROLL"] = Eigen::VectorXf::Random(51);
  buffer["PITCH"] = Eigen::VectorXf::Random(51);
  buffer["CAN_THROTTLE_CMD"] = Eigen::VectorXf::Random(51);
  buffer["BRAKE_STATE"] = Eigen::VectorXf::Random(51);
  buffer["CAN_BRAKE_CMD"] = Eigen::VectorXf::Random(51);
  buffer["OMEGA_Z"] = Eigen::VectorXf::Random(51);

  EXPECT_TRUE(fileExists(mppi::tests::racer_dubins_elevation_uncertainty_test));
  if (!fileExists(mppi::tests::racer_dubins_elevation_uncertainty_test))
  {
    std::cerr << "Could not load neural net model at path: "
              << mppi::tests::racer_dubins_elevation_uncertainty_test.c_str();
    exit(-1);
  }
  cnpy::npz_t param_dict = cnpy::npz_load(mppi::tests::racer_dubins_elevation_uncertainty_test);

  float T = param_dict.at("T").data<double>()[0];
  float dt = param_dict.at("dt").data<double>()[0];
  float init_length = param_dict.at("init_length").data<double>()[0] + 1;
  int traj_length = T / dt;
  int num_points = static_cast<int>(param_dict.at("num_points").data<double>()[0]);
  int input_dim = static_cast<int>(param_dict.at("input_dim").data<double>()[0]);
  int state_dim = static_cast<int>(param_dict.at("state_dim").data<double>()[0]);
  int state_dim_raw = static_cast<int>(param_dict.at("state_dim_raw").data<double>()[0]);
  int output_dim = static_cast<int>(param_dict.at("output_dim").data<double>()[0]);
  int output_dim_raw = static_cast<int>(param_dict.at("output_dim_raw").data<double>()[0]);

  double* init_inputs = param_dict.at("init_input").data<double>();
  double* init_states = param_dict.at("init_state").data<double>();
  double* states = param_dict.at("state").data<double>();
  double* inputs = param_dict.at("input").data<double>();
  double* outputs = param_dict.at("output").data<double>();
  // double* inputs = param_dict.at("input").data<double>();
  // double* outputs = param_dict.at("output").data<double>();
  double* init_hidden = nullptr;  // = param_dict.at("init/hidden").data<double>();
  double* init_cell = nullptr;    // = param_dict.at("init/cell").data<double>();
  int hidden_dim = 0;
  // double* hidden = param_dict.at("hidden").data<double>();
  // double* cell = param_dict.at("cell").data<double>();

  for (int point = 0; point < num_points; point++)
  {
    // std::cout << "\n\n\n\n\n\n\n================= on point " << point << "=============" << std::endl;
    // sets up the buffer correctly
    for (int t = 0; t < init_length; t++)
    {
      int buffer_index = (51 - init_length) + t;
      int init_input_shift = point * init_length * input_dim + t * input_dim;
      buffer["CAN_THROTTLE_CMD"](buffer_index) = init_inputs[init_input_shift + 0];
      buffer["CAN_BRAKE_CMD"](buffer_index) = init_inputs[init_input_shift + 1];
      buffer["CAN_STEER_CMD"](buffer_index) = init_inputs[init_input_shift + 2];
      buffer["PITCH"](buffer_index) = init_inputs[init_input_shift + 3];
      buffer["ROLL"](buffer_index) = init_inputs[init_input_shift + 4];

      int init_state_shift = point * init_length * state_dim_raw + t * state_dim_raw;
      buffer["VEL_X"](buffer_index) = init_states[init_state_shift + 3];
      buffer["OMEGA_Z"](buffer_index) = init_states[init_state_shift + 5];
      buffer["BRAKE_STATE"](buffer_index) = init_states[init_state_shift + 6];
      buffer["STEER_ANGLE"](buffer_index) = init_states[init_state_shift + 7];
      buffer["STEER_ANGLE_RATE"](buffer_index) = init_states[init_state_shift + 8];
    }
    typename DYN::state_array state = DYN::state_array::Zero();
    typename DYN::state_array next_state = DYN::state_array::Zero();
    typename DYN::control_array control = DYN::control_array::Zero();
    typename DYN::state_array state_der = DYN::state_array::Zero();
    typename DYN::output_array output_array = DYN::output_array::Zero();

    int state_shift = point * traj_length * state_dim;
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, POS_X)) = states[state_shift + 0];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, POS_Y)) = states[state_shift + 1];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, YAW)) = states[state_shift + 2];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, VEL_X)) = states[state_shift + 3];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, OMEGA_Z)) = states[state_shift + 5];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE)) = states[state_shift + 6];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE)) = states[state_shift + 7];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE_RATE)) = states[state_shift + 8];
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_POS_X)) = 1.0e-5;
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_POS_Y)) = 1.0e-5;
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_YAW)) = 1.0e-5;
    state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_VEL_X)) = 1.0e-5;

    // std::cout << "got initial state " << state.transpose() << std::endl;

    assert(state(S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE)) >= 0.0f &&
           state(S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE)) <= 0.25f);
    assert(state(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE)) >= -5 &&
           state(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE)) <= 5);

    if (dynamics.checkRequiresBuffer())
    {
      dynamics.updateFromBuffer(buffer);
    }
    dynamics.initializeDynamics(state, control, output_array, 0, dt);

    // checks init values for hidden and cell states
    hidden_dim = dynamics.getHelper()->getLSTMModel()->getHiddenDim();
    init_hidden = param_dict.at("init/steering/hidden").data<double>();
    init_cell = param_dict.at("init/steering/cell").data<double>();
    for (int i = 0; i < hidden_dim; i++)
    {
      EXPECT_NEAR(dynamics.getHelper()->getLSTMModel()->getHiddenState()[i], init_hidden[point * hidden_dim + i],
                  1.0e-5);
      EXPECT_NEAR(dynamics.getHelper()->getLSTMModel()->getCellState()[i], init_cell[point * hidden_dim + i], 1.0e-5);
    }

    hidden_dim = dynamics.getMeanHelper()->getLSTMModel()->getHiddenDim();
    init_hidden = param_dict.at("init/terra/mean_network/hidden").data<double>();
    init_cell = param_dict.at("init/terra/mean_network/cell").data<double>();
    for (int i = 0; i < hidden_dim; i++)
    {
      EXPECT_NEAR(dynamics.getMeanHelper()->getLSTMModel()->getHiddenState()[i], init_hidden[point * hidden_dim + i],
                  1.0e-5);
      EXPECT_NEAR(dynamics.getMeanHelper()->getLSTMModel()->getCellState()[i], init_cell[point * hidden_dim + i],
                  1.0e-5);
    }

    hidden_dim = dynamics.getUncertaintyHelper()->getLSTMModel()->getHiddenDim();
    init_hidden = param_dict.at("init/terra/uncertainty_network/hidden").data<double>();
    init_cell = param_dict.at("init/terra/uncertainty_network/cell").data<double>();
    for (int i = 0; i < hidden_dim; i++)
    {
      EXPECT_NEAR(dynamics.getUncertaintyHelper()->getLSTMModel()->getHiddenState()[i],
                  init_hidden[point * hidden_dim + i], 1.0e-5);
      EXPECT_NEAR(dynamics.getUncertaintyHelper()->getLSTMModel()->getCellState()[i], init_cell[point * hidden_dim + i],
                  1.0e-5);
    }

    // TODO find a way to scale the tol as a function of number of computations
    // TODO scale tolerance by the number of timesteps

    float tol = 1.0e-5;

    // run init part of the networks and check hidden and cell states there
    for (int t = 1; t < 50; t++)
    {
      int input_shift = point * traj_length * input_dim + (t - 1) * input_dim;
      control(C_IND_CLASS(DYN::DYN_PARAMS_T, THROTTLE_BRAKE)) = inputs[input_shift + 0] - inputs[input_shift + 1];
      control(C_IND_CLASS(DYN::DYN_PARAMS_T, STEER_CMD)) = inputs[input_shift + 2];
      state(S_IND_CLASS(DYN::DYN_PARAMS_T, STATIC_PITCH)) = inputs[input_shift + 3];
      state(S_IND_CLASS(DYN::DYN_PARAMS_T, STATIC_ROLL)) = inputs[input_shift + 4];
      state(S_IND_CLASS(DYN::DYN_PARAMS_T, PITCH)) = inputs[input_shift + 3];
      state(S_IND_CLASS(DYN::DYN_PARAMS_T, ROLL)) = inputs[input_shift + 4];

      // std::cout << "at time t " << t - 1 << " got control " << control.transpose() << std::endl;
      // std::cout << "at time t " << t - 1 << " got state " << state.transpose() << std::endl;
      // std::cout << "at time t " << t - 1 << " got vx " << state(S_IND_CLASS(DYN::DYN_PARAMS_T, VEL_X)) << std::endl;

      dynamics.step(state, next_state, state_der, control, output_array, 0.0f, dt);

      int output_shift = point * traj_length * output_dim + (t - 1) * output_dim;
      EXPECT_NEAR(state_der(S_IND_CLASS(DYN::DYN_PARAMS_T, VEL_X)), outputs[output_shift + 0], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, VEL_X) << " at time " << t;
      EXPECT_NEAR(state_der(S_IND_CLASS(DYN::DYN_PARAMS_T, YAW)), outputs[output_shift + 2], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, YAW) << " at time " << t;
      EXPECT_NEAR(state_der(S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE)), outputs[output_shift + 3], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE) << " at time "
          << t;
      EXPECT_NEAR(state_der(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE_RATE)), outputs[output_shift + 4], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE_RATE)
          << " at time " << t;

      state_shift = point * traj_length * state_dim + t * state_dim;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, POS_X)), states[state_shift + 0], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, POS_X) << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, POS_Y)), states[state_shift + 1], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, POS_Y) << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, YAW)), angle_utils::normalizeAngle(states[state_shift + 2]),
                  tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, YAW) << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, VEL_X)), states[state_shift + 3], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, VEL_X) << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, OMEGA_Z)), states[state_shift + 5], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, OMEGA_Z) << " at time "
          << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE)), states[state_shift + 6], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, BRAKE_STATE) << " at time "
          << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE)), states[state_shift + 7], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE) << " at time "
          << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE_RATE)), states[state_shift + 8], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, STEER_ANGLE_RATE)
          << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_POS_X)), states[state_shift + state_dim_raw],
                  tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_POS_X)
          << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_POS_Y)),
                  states[state_shift + state_dim_raw + 5], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_POS_Y)
          << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_YAW)), states[state_shift + state_dim_raw + 10],
                  tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_YAW)
          << " at time " << t;
      EXPECT_NEAR(next_state(S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_VEL_X)),
                  states[state_shift + state_dim_raw + 15], tol)
          << "at sample " << point << ", next state dim: " << S_IND_CLASS(DYN::DYN_PARAMS_T, UNCERTAINTY_VEL_X)
          << " at time " << t;

      // checks init values for hidden and cell states
      hidden_dim = dynamics.getHelper()->getLSTMModel()->getHiddenDim();
      double* hidden = param_dict.at("pred/steering/hidden").data<double>();
      double* cell = param_dict.at("pred/steering/cell").data<double>();
      int hidden_shift = point * traj_length * hidden_dim + (t - 1) * hidden_dim;
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_NEAR(dynamics.getHelper()->getLSTMModel()->getHiddenState()[i], hidden[hidden_shift + i], tol)
            << "steering hidden lstm at point " << point << " dim " << i;
        EXPECT_NEAR(dynamics.getHelper()->getLSTMModel()->getCellState()[i], cell[hidden_shift + i], tol)
            << "steering cell lstm at point " << point << " dim " << i;
      }

      hidden_dim = dynamics.getMeanHelper()->getLSTMModel()->getHiddenDim();
      hidden = param_dict.at("pred/terra/mean_network/hidden").data<double>();
      cell = param_dict.at("pred/terra/mean_network/cell").data<double>();
      hidden_shift = point * traj_length * hidden_dim + (t - 1) * hidden_dim;
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_NEAR(dynamics.getMeanHelper()->getLSTMModel()->getHiddenState()[i], hidden[hidden_shift + i], tol)
            << "steering hidden lstm at point " << point << " dim " << i;
        EXPECT_NEAR(dynamics.getMeanHelper()->getLSTMModel()->getCellState()[i], cell[hidden_shift + i], tol)
            << "steering cell lstm at point " << point << " dim " << i;
      }

      hidden_dim = dynamics.getUncertaintyHelper()->getLSTMModel()->getHiddenDim();
      hidden = param_dict.at("pred/terra/uncertainty_network/hidden").data<double>();
      cell = param_dict.at("pred/terra/uncertainty_network/cell").data<double>();
      hidden_shift = point * traj_length * hidden_dim + (t - 1) * hidden_dim;
      for (int i = 0; i < hidden_dim; i++)
      {
        EXPECT_NEAR(dynamics.getUncertaintyHelper()->getLSTMModel()->getHiddenState()[i], hidden[hidden_shift + i], tol)
            << "steering hidden lstm at point " << point << " dim " << i;
        EXPECT_NEAR(dynamics.getUncertaintyHelper()->getLSTMModel()->getCellState()[i], cell[hidden_shift + i], tol)
            << "steering cell lstm at point " << point << " dim " << i;
      }

      state = next_state;

      tol += 5.0e-4 / traj_length;
    }
  }
}
