//
// Created by mgandhi3 on 7/22/21.
//

#include <gtest/gtest.h>
#include <mppi/utils/numerical_integration.h>
#include <mppi/dynamics/dynamics.cuh>

struct TestDynamicsParams : public DynamicsParams
{
};

using namespace MPPI_internal;

class TestDynamics : public Dynamics<TestDynamics, TestDynamicsParams>
{
public:
  explicit TestDynamics(cudaStream_t stream = nullptr) : Dynamics<TestDynamics, TestDynamicsParams>(stream){};

  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der)
  {
    state_der = -2 * state;
  }

  state_array stateFromMap(const std::map<std::string, float>& map)
  {
  }
};

TEST(Integration, RK4)
{
  using DYN = TestDynamics;
  DYN dynamics;
  DYN::state_array x_k, x_kp1;
  x_k << 3;
  DYN::control_array u_k;
  float dt = 0.05;
  int num_timesteps = 100;
  for (int i = 0; i < num_timesteps; ++i)
  {
    rk4integrate(&dynamics, dt, x_k, u_k, x_kp1);
    x_k = x_kp1;
  }

  ASSERT_NEAR(0.0, x_k[0], 1e-6);
}
