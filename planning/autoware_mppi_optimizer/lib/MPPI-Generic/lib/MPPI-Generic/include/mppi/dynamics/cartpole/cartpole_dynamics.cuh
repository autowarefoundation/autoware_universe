#ifndef CARTPOLE_CUH_
#define CARTPOLE_CUH_

#include <mppi/dynamics/dynamics.cuh>

struct CartpoleDynamicsParams : public DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    VEL_X,
    THETA,
    THETA_DOT,
    NUM_STATES
  };
  enum class ControlIndex : int
  {
    FORCE = 0,
    NUM_CONTROLS
  };
  enum class OutputIndex : int
  {
    POS_X = 0,
    VEL_X,
    THETA,
    THETA_DOT,
    NUM_OUTPUTS
  };
  float cart_mass = 1.0f;
  float pole_mass = 1.0f;
  float pole_length = 1.0f;

  CartpoleDynamicsParams() = default;
  CartpoleDynamicsParams(float cart_mass, float pole_mass, float pole_length)
    : cart_mass(cart_mass), pole_mass(pole_mass), pole_length(pole_length){};
};
using namespace MPPI_internal;

class CartpoleDynamics : public Dynamics<CartpoleDynamics, CartpoleDynamicsParams>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using PARENT_CLASS = Dynamics<CartpoleDynamics, CartpoleDynamicsParams>;
  CartpoleDynamics(float cart_mass = 1.0f, float pole_mass = 1.0f, float pole_length = 1.0f, cudaStream_t stream = 0);

  std::string getDynamicsModelName() const override
  {
    return "Cartpole Model";
  }

  /**
   * runs dynamics using state and control and sets it to state
   * derivative. Everything is Eigen Matrices, not Eigen Vectors!
   *
   * @param state     input of current state, passed by reference
   * @param control   input of currrent control, passed by reference
   * @param state_der output of new state derivative, passed by reference
   */
  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  /**
   * compute the Jacobians with respect to state and control
   *
   * @param state   input of current state, passed by reference
   * @param control input of currrent control, passed by reference

   */
  bool computeGrad(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                   Eigen::Ref<dfdx> A, Eigen::Ref<dfdu> B);

  __host__ __device__ float getCartMass()
  {
    return this->params_.cart_mass;
  };
  __host__ __device__ float getPoleMass()
  {
    return this->params_.pole_mass;
  };
  __host__ __device__ float getPoleLength()
  {
    return this->params_.pole_length;
  };
  __host__ __device__ float getGravity()
  {
    return gravity_;
  }

  void printState(const Eigen::Ref<const state_array>& state);
  void printState(float* state);
  void printParams();

  // void computeKinematics(const Eigen::Ref<const state_array>&,
  //                        Eigen::Ref<state_array>& state_der) {};

  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta = nullptr);

  state_array stateFromMap(const std::map<std::string, float>& map) override;

protected:
  const float gravity_ = 9.81;
};

#if __CUDACC__
#include "cartpole_dynamics.cu"
#endif

#endif  // CARTPOLE_CUH_
