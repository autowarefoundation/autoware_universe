#pragma once

#ifndef DYNAMICS_CUH_
#define DYNAMICS_CUH_

/*
Header file for dynamics
*/

#include <Eigen/Dense>
#include <mppi/utils/managed.cuh>
#include <mppi/utils/angle_utils.cuh>
#include <mppi/utils/math_utils.h>

#include <stdio.h>
#include <math.h>

#include <array>
#include <cfloat>
#include <type_traits>
#include <vector>
#include <map>
#include <string>

// helpful macros to use the enum setup
#ifndef E_INDEX
#define E_INDEX(ENUM, enum_val) static_cast<int>(ENUM::enum_val)
#endif
#ifndef S_INDEX
#define S_IND_CLASS(CLASS, enum_val) E_INDEX(CLASS::StateIndex, enum_val)
#define S_INDEX(enum_val) S_IND_CLASS(PARENT_CLASS::DYN_PARAMS_T, enum_val)
#endif

#ifndef C_INDEX
#define C_IND_CLASS(CLASS, enum_val) E_INDEX(CLASS::ControlIndex, enum_val)
#define C_INDEX(enum_val) C_IND_CLASS(PARENT_CLASS::DYN_PARAMS_T, enum_val)
#endif

#ifndef O_INDEX
#define O_IND_CLASS(CLASS, enum_val) E_INDEX(CLASS::OutputIndex, enum_val)
#define O_INDEX(enum_val) O_IND_CLASS(PARENT_CLASS::DYN_PARAMS_T, enum_val)
#endif

struct DynamicsParams
{
  enum class StateIndex : int
  {
    POS_X = 0,
    NUM_STATES
  };
  enum class ControlIndex : int
  {
    VEL_X = 0,
    NUM_CONTROLS
  };
  enum class OutputIndex : int
  {
    POS_X = 0,
    NUM_OUTPUTS
  };
};

template <typename T>
using paramsInheritsFrom = typename std::is_base_of<DynamicsParams, T>;

namespace MPPI_internal
{
template <class CLASS_T, class PARAMS_T>
class Dynamics : public Managed
{
  static_assert(paramsInheritsFrom<PARAMS_T>::value, "Dynamics PARAMS_T does not inherit from DynamicsParams");

public:
  //  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  static const int STATE_DIM = S_IND_CLASS(PARAMS_T, NUM_STATES);
  static const int CONTROL_DIM = C_IND_CLASS(PARAMS_T, NUM_CONTROLS);
  static const int OUTPUT_DIM = O_IND_CLASS(PARAMS_T, NUM_OUTPUTS);
  typedef CLASS_T DYN_T;
  typedef PARAMS_T DYN_PARAMS_T;

  /**
   * useful typedefs
   */
  typedef Eigen::Matrix<float, CONTROL_DIM, 1> control_array;                 // Control at a time t
  typedef Eigen::Matrix<float, STATE_DIM, 1> state_array;                     // State at a time t
  typedef Eigen::Matrix<float, OUTPUT_DIM, 1> output_array;                   // Output at a time t
  typedef Eigen::Matrix<float, STATE_DIM, STATE_DIM> dfdx;                    // Jacobian wrt x
  typedef Eigen::Matrix<float, STATE_DIM, CONTROL_DIM> dfdu;                  // Jacobian wrt u
  typedef Eigen::Matrix<float, CONTROL_DIM, STATE_DIM> feedback_matrix;       // Feedback matrix
  typedef Eigen::Matrix<float, STATE_DIM, STATE_DIM + CONTROL_DIM> Jacobian;  // Jacobian of x and u

  typedef std::map<std::string, Eigen::VectorXf> buffer_trajectory;

  // protected constructor prevent anyone from trying to construct a Dynamics
protected:
  /**
   * sets the default control ranges to -infinity and +infinity
   */
  Dynamics(cudaStream_t stream = 0) : Managed(stream)
  {
    setDefaultControlRanges();
  }

  /**
   * sets the control ranges to the passed in value
   * @param control_rngs
   * @param stream
   */
  Dynamics(std::array<float2, CONTROL_DIM>& control_rngs, cudaStream_t stream = 0) : Managed(stream)
  {
    setControlRanges(control_rngs);
  }

  Dynamics(PARAMS_T& params, std::array<float2, CONTROL_DIM>& control_rngs, cudaStream_t stream = 0) : Managed(stream)
  {
    setParams(params);
    setControlRanges(control_rngs);
  }

  Dynamics(PARAMS_T& params, cudaStream_t stream = 0) : Managed(stream)
  {
    setParams(params);
    setDefaultControlRanges();
  }

public:
  // This variable defines what the zero control is
  // For most systems, it should be zero but for things like a quadrotor,
  // it should be the command to hover
  control_array zero_control_ = control_array::Zero();

  /**
   * Destructor must be virtual so that children are properly
   * destroyed when called from a Dynamics reference
   */
  virtual ~Dynamics()
  {
    freeCudaMem();
  }

  virtual std::string getDynamicsModelName() const
  {
    return "Dynamics model name not set";
  }

  /**
   * Allocates all of the GPU memory
   */
  void GPUSetup();

  void setZeroControl(const Eigen::Ref<const control_array>& zero_control)
  {
    zero_control_ = zero_control;
  }
  control_array getZeroControl() const
  {
    return zero_control_;
  }

  std::array<float2, CONTROL_DIM> getControlRanges()
  {
    std::array<float2, CONTROL_DIM> result;
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      result[i] = control_rngs_[i];
    }
    return result;
  }
  __host__ __device__ float2* getControlRangesRaw()
  {
    return control_rngs_;
  }

  void setControlRanges(std::array<float2, CONTROL_DIM>& control_rngs, bool synchronize = true);

  void setControlDeadbands(std::array<float, CONTROL_DIM>& control_deadband, bool synchronize = true);

  void setParams(const PARAMS_T& params)
  {
    params_ = params;
    if (GPUMemStatus_)
    {
      CLASS_T& derived = static_cast<CLASS_T&>(*this);
      derived.paramsToDevice();
    }
  }

  __device__ __host__ PARAMS_T getParams()
  {
    return params_;
  }

  /*
   *
   */
  void freeCudaMem();

  /**
   *
   */
  void printState(float* state);

  /**
   *
   */
  // TODO should not assume it is going to cout, pass in stream
  void printParams();

  /**
   *
   */
  void paramsToDevice(bool synchronize = true);

  /**
   * loads the .npz at given path
   * @param model_path
   */
  void loadParams(const std::string& model_path);

  /**
   * updates the internals of the dynamics model
   * @param description
   * @param data
   */
  // TODO generalize
  void updateModel(std::vector<int> description, std::vector<float> data);

  /**
   * compute the Jacobians with respect to state and control
   *
   * @param state   input of current state, passed by reference
   * @param control input of currrent control, passed by reference
   * @param A       output Jacobian wrt state, passed by reference
   * @param B       output Jacobian wrt control, passed by reference
   */
  bool computeGrad(const Eigen::Ref<const state_array>& state = state_array(),
                   const Eigen::Ref<const control_array>& control = control_array(), Eigen::Ref<dfdx> A = dfdx(),
                   Eigen::Ref<dfdu> B = dfdu())
  {
    return false;
  }
  /**
   * enforces control constraints
   * @param state
   * @param control
   */
  void enforceConstraints(Eigen::Ref<state_array> state, Eigen::Ref<control_array> control)
  {
    for (int i = 0; i < CONTROL_DIM; i++)
    {
      if (fabsf(control[i]) < this->control_deadband_[i])
      {
        control[i] = this->zero_control_[i];
      }
      else
      {
        control[i] += this->control_deadband_[i] * -mppi::math::sign(control[i]);
      }
      control[i] = fminf(fmaxf(this->control_rngs_[i].x, control[i]), this->control_rngs_[i].y);
    }
  }

  /**
   * updates the current state using s_der
   * @param s state
   * @param s_der
   */
  DEPRECATED void updateState(Eigen::Ref<state_array> state, Eigen::Ref<state_array> state_der, const float dt)
  {
    CLASS_T* derived = static_cast<CLASS_T*>(this);
    derived->updateState(state, state, state_der, dt);
  }

  void updateState(const Eigen::Ref<const state_array> state, Eigen::Ref<state_array> next_state,
                   Eigen::Ref<state_array> state_der, const float dt)
  {
    next_state = state + state_der * dt;
  }

  void step(Eigen::Ref<state_array> state, Eigen::Ref<state_array> next_state, Eigen::Ref<state_array> state_der,
            const Eigen::Ref<const control_array>& control, Eigen::Ref<output_array> output, const float t,
            const float dt)
  {
    CLASS_T* derived = static_cast<CLASS_T*>(this);
    derived->computeStateDeriv(state, control, state_der);
    derived->updateState(state, next_state, state_der, dt);
    derived->stateToOutput(next_state, output);
  }

  void stateToOutput(const Eigen::Ref<const state_array>& state, Eigen::Ref<output_array> output)
  {
    // TODO this is a hack
    for (int i = 0; i < OUTPUT_DIM && i < STATE_DIM; i++)
    {
      output[i] = state[i];
    }
  }

  __host__ __device__ void stateToOutput(const float* __restrict__ state, float* __restrict__ output);

  __host__ __device__ void outputToState(const float* __restrict__ output, float* __restrict__ state);

  state_array getZeroState() const;

  __host__ __device__ void getZeroState(float* state) const;

  /**
   * does a linear interpolation of states
   * @param state_1
   * @param state_2
   * @param alpha
   * @return
   */
  state_array interpolateState(const Eigen::Ref<const state_array>& state_1,
                               const Eigen::Ref<const state_array>& state_2, const float alpha)
  {
    return (1 - alpha) * state_1 + alpha * state_2;
  }

  /**
   * computes a specific state error
   * @param pred_state
   * @param true_state
   * @return
   */
  state_array computeStateError(const Eigen::Ref<const state_array>& pred_state,
                                const Eigen::Ref<const state_array>& true_state)
  {
    return pred_state - true_state;
  }

  /**
   * computes the section of the state derivative that comes form the dyanmics
   * @param state
   * @param control
   * @param state_der
   */
  void computeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                       Eigen::Ref<state_array> state_der);

  /**
   * computes the parts of the state that are based off of kinematics
   * @param s state
   * @param s_der
   */
  void computeKinematics(const Eigen::Ref<const state_array>& state, Eigen::Ref<state_array> s_der){};

  /**
   * computes the full state derivative by calling computeKinematics then computeDynamics
   * @param state
   * @param control
   * @param state_der
   */
  void computeStateDeriv(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                         Eigen::Ref<state_array> state_der)
  {
    CLASS_T* derived = static_cast<CLASS_T*>(this);
    derived->computeKinematics(state, state_der);
    derived->computeDynamics(state, control, state_der);
  }

  /**
   * computes the section of the state derivative that comes form the dyanmics
   * @param state
   * @param control
   * @param state_der
   * @param theta_s shared memory that can be used when computation is computed across the same block
   */
  __device__ void computeDynamics(float* state, float* control, float* state_der, float* theta_s = nullptr);

  /**
   * computes the parts of the state that are based off of kinematics
   * parallelized on X only
   * @param state
   * @param state_der
   */
  __device__ void computeKinematics(float* state, float* state_der){};

  /**
   * @param state
   * @param control
   * @param state_der
   * @param theta_s shared memory that can be used when computation is computed across the same block
   */
  __device__ inline void computeStateDeriv(float* state, float* control, float* state_der, float* theta_s);

  __device__ inline void step(float* state, float* next_state, float* state_der, float* control, float* output,
                              float* theta_s, const float t, const float dt);

  /**
   * applies the state derivative
   * @param state
   * @param state_der
   * @param dt
   */
  DEPRECATED __device__ void updateState(float* state, float* state_der, const float dt)
  {
    CLASS_T* derived = static_cast<CLASS_T*>(this);
    derived->updateState(state, state, state_der, dt);
  }

  __device__ void updateState(float* state, float* next_state, float* state_der, const float dt);

  /**
   * enforces control constraints
   */
  __device__ void enforceConstraints(float* state, float* control);

  /**
   * Method to allow setup of dynamics on the GPU. This is needed for
   * initializing the memory of an LSTM for example
   */
  void initializeDynamics(const Eigen::Ref<const state_array>& state, const Eigen::Ref<const control_array>& control,
                          Eigen::Ref<output_array> output, float t_0, float dt)
  {
    for (int i = 0; i < OUTPUT_DIM && i < STATE_DIM; i++)
    {
      output[i] = state[i];
    }
  }

  /**
   * Method to allow setup of dynamics on the GPU. This is needed for
   * initializing the memory of an LSTM for example
   */
  __device__ void initializeDynamics(float* state, float* control, float* output, float* theta_s, float t_0, float dt)
  {
    for (int i = 0; i < OUTPUT_DIM && i < STATE_DIM; i++)
    {
      output[i] = state[i];
    }
  }

  /**
   * Method to compute an emergency stopping control
   */
  virtual void getStoppingControl(const Eigen::Ref<const state_array>& state, Eigen::Ref<control_array> u)
  {
    u.setZero();
  }

  /**
   * Method to enforce a leash on the initial state, which depends on type of dynamics.
   */
  virtual void enforceLeash(const Eigen::Ref<const state_array>& state_true,
                            const Eigen::Ref<const state_array>& state_nominal,
                            const Eigen::Ref<const state_array>& leash_values, Eigen::Ref<state_array> state_output)
  {
    for (int i = 0; i < DYN_T::STATE_DIM; i++)
    {
      float diff = fabsf(state_nominal[i] - state_true[i]);

      if (leash_values[i] < diff)
      {
        float leash_dir = fminf(fmaxf(state_nominal[i] - state_true[i], -leash_values[i]), leash_values[i]);
        state_output[i] = state_true[i] + leash_dir;
      }
      else
      {
        state_output[i] = state_nominal[i];
      }
    }
  }

  virtual bool checkRequiresBuffer()
  {
    return requires_buffer_;
  }

  bool updateFromBuffer(const buffer_trajectory& buffer)
  {
    return true;
  }

  bool checkIfKeysInBuffer(const buffer_trajectory& buffer, std::vector<std::string>& needed_keys)
  {
    bool missing_key = false;
    for (const auto& key : needed_keys)
    {
      if (buffer.find(key) == buffer.end())
      {  // Print out all missing keys
        this->logger_->warning("Could not find key %s in init buffer for %s\n", key.c_str(),
                               this->getDynamicsModelName().c_str());
        missing_key = true;
      }
    }
    return missing_key;
  }

  bool checkIfKeysInMap(const std::map<std::string, float>& map, std::vector<std::string>& needed_keys)
  {
    bool missing_key = false;
    for (const auto& key : needed_keys)
    {
      if (map.find(key) == map.end())
      {  // Print out all missing keys
        this->logger_->warning("Could not find key %s in state map for %s\n", key.c_str(),
                               this->getDynamicsModelName().c_str());
        missing_key = true;
      }
    }
    return missing_key;
  }

  virtual state_array stateFromMap(const std::map<std::string, float>& map) = 0;

  // control ranges [.x, .y]
  float2 control_rngs_[CONTROL_DIM];
  float control_deadband_[CONTROL_DIM] = { 0.0f };

  // device pointer, null on the device
  CLASS_T* model_d_ = nullptr;

protected:
  // generic parameter structure
  PARAMS_T params_;

  bool requires_buffer_ = false;

private:
  void setDefaultControlRanges();
};

#ifdef __CUDACC__
#include "dynamics.cu"
#endif

template <class CLASS_T, class PARAMS_T>
const int Dynamics<CLASS_T, PARAMS_T>::STATE_DIM;

template <class CLASS_T, class PARAMS_T>
const int Dynamics<CLASS_T, PARAMS_T>::CONTROL_DIM;

template <class CLASS_T, class PARAMS_T>
const int Dynamics<CLASS_T, PARAMS_T>::OUTPUT_DIM;
}  // namespace MPPI_internal
#endif  // DYNAMICS_CUH_
