//
// Created by mgandhi3 on 3/4/20.
//
#include <gtest/gtest.h>
#include <mppi/dynamics/double_integrator/di_dynamics.cuh>
#include "di_dynamics_kernel_tests.cuh"
#include <memory>
#include <random>
#include <Eigen/Dense>

TEST(Miscellaneous, CompareModelSize)
{
  auto model = std::make_shared<DoubleIntegratorDynamics>();

  model->GPUSetup();

  long* model_size_GPU_d;

  HANDLE_ERROR(cudaMalloc((void**)&model_size_GPU_d, sizeof(long)));

  CheckModelSize<<<1, 1>>>(model->model_d_, model_size_GPU_d);
  CudaCheckError();

  long model_size_GPU;
  HANDLE_ERROR(cudaMemcpy(&model_size_GPU, model_size_GPU_d, sizeof(long), cudaMemcpyDeviceToHost));

  ASSERT_EQ(sizeof(*model), model_size_GPU);

  std::cout << "Size of the shared pointer to the model:" << sizeof(model)
            << std::endl;  // Should be the size of a pointer so 8 bytes for a 64 bit system

  std::cout << "Size of the model itself: " << sizeof(*model) << std::endl;  // Should be bigger?

  std::cout << "Size of the model on the GPU: " << model_size_GPU << std::endl;

  std::cout << "Size of the control ranges in the model: " << sizeof(model->control_rngs_)
            << std::endl;  // Should be 16 bytes ie. 4 floats!

  std::cout << "Size of the parameter structure of the model: " << sizeof(DoubleIntegratorParams) << std::endl;

  std::cout << "Size of the device pointer of the model: " << sizeof(model->model_d_) << std::endl;

  std::cout << "Size of the stream: " << sizeof(model->stream_) << std::endl;

  std::cout << "Size of GPU Memstatus: " << sizeof(model->GPUMemStatus_) << std::endl;
}

TEST(Miscellaneous, EigenNormalRandomVector)
{
  std::random_device rd;
  std::mt19937 gen(rd());  // here you could also set a seed
  std::normal_distribution<float> dis(1, 2);

  // generate a matrix expression
  Eigen::MatrixXd M = Eigen::MatrixXd::NullaryExpr(100, 100, [&]() { return dis(gen); });

  EXPECT_NEAR(M.mean(), 1, 1e-1);

  EXPECT_NEAR(sqrtf((M.array() * M.array()).mean() - M.mean() * M.mean()), 2, 1e-1);
}

TEST(Miscellaneous, CreateRandomStateArray)
{
  DoubleIntegratorDynamics::state_array X;
  DoubleIntegratorDynamics::state_array temp = DoubleIntegratorDynamics::state_array::Zero();

  std::random_device rd;
  std::mt19937 gen(rd());  // here you could also set a seed
  std::normal_distribution<float> dis(1, 2);

  // generate a matrix expression
  X = DoubleIntegratorDynamics::state_array::NullaryExpr([&]() { return dis(gen); });

  std::cout << temp + X * 0.01 << std::endl;
}

TEST(Miscellaneous, Smoothing)
{
  Eigen::Matrix<float, 1, 5> filter_coefficients;
  filter_coefficients << -3, 12, 17, 12, -3;
  filter_coefficients /= 35.0;

  Eigen::Matrix<float, 14, 3> control_buffer = Eigen::Matrix<float, 14, 3>::Ones();

  std::cout << "previous control buffer" << std::endl;
  std::cout << control_buffer << std::endl;

  Eigen::Matrix<float, 2, 3> control_history;
  control_history << 1, 2, 3, 4, 5, 6;

  control_buffer.topRows<2>() = control_history;

  Eigen::Matrix<float, 3, 10> nominal_control = 5 * Eigen::Matrix<float, 3, 10>::Ones();

  // Fill the last two timesteps with the end of the current nominal control trajectory
  nominal_control.col(9) << 10, 10, 10;
  control_buffer.middleRows(2, 10) = nominal_control.transpose();

  control_buffer.row(10 + 2) = nominal_control.transpose().row(10 - 1);
  control_buffer.row(10 + 3) = nominal_control.transpose().row(10 - 1);

  std::cout << "current control buffer" << std::endl;
  std::cout << control_buffer << std::endl;

  // Apply convolutional filter to each timestep
  for (int i = 0; i < 10; ++i)
  {
    nominal_control.col(i) = (filter_coefficients * control_buffer.middleRows(i, 5)).transpose();
  }

  std::cout << nominal_control << std::endl;
}