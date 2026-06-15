// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPPI__DDP__DDP_MODEL_WRAPPER_H_
#define MPPI__DDP__DDP_MODEL_WRAPPER_H_

#include "ddp_dynamics.h"
#include <cstdint>
#include <type_traits>

template <typename T>
struct HasAnalyticGrad
{
  template <typename U>
  static char Test(decltype(&U::computeGrad));
  template <typename U>
  static int64_t Test(...);
  static const bool Has = sizeof(Test<T>(0)) == sizeof(char);
};

template <typename T>
using WrappedDynamics = DDP_structures::Dynamics<float, T::STATE_DIM, T::CONTROL_DIM>;

template <typename T>
bool getGrad(T* model, typename WrappedDynamics<T>::Jacobian& jac, typename WrappedDynamics<T>::State& x,
             typename WrappedDynamics<T>::Control& u, std::true_type)
{
  // T::dfdx A;
  // T::dfdu B;
  Eigen::Matrix<float, T::STATE_DIM, T::STATE_DIM> A = Eigen::Matrix<float, T::STATE_DIM, T::STATE_DIM>::Zero();
  Eigen::Matrix<float, T::STATE_DIM, T::CONTROL_DIM> B = Eigen::Matrix<float, T::STATE_DIM, T::CONTROL_DIM>::Zero();
  bool exists = model->computeGrad(x, u, A, B);
  jac.block(0, 0, T::STATE_DIM, T::STATE_DIM) = A;
  jac.block(0, T::STATE_DIM, T::STATE_DIM, T::CONTROL_DIM) = B;
  // for (int i = 0; i < T::STATE_DIM; i++){
  //     for (int j = 0; j < T::STATE_DIM; j++){
  //         jac(i,j) = A(i,j);
  //     }
  // }
  return exists;
}

template <typename T>
bool getGrad(T* model, typename WrappedDynamics<T>::Jacobian& jac, typename WrappedDynamics<T>::State& x,
             typename WrappedDynamics<T>::Control& u, std::false_type)
{
  return false;
}

template <class DYNAMICS_T>
struct ModelWrapperDDP : public WrappedDynamics<DYNAMICS_T>
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using Scalar = float;
  using State = typename WrappedDynamics<DYNAMICS_T>::State;
  using Control = typename WrappedDynamics<DYNAMICS_T>::Control;
  using Jacobian = typename WrappedDynamics<DYNAMICS_T>::Jacobian;
  using StateTrajectory = typename WrappedDynamics<DYNAMICS_T>::StateTrajectory;
  using ControlTrajectory = typename WrappedDynamics<DYNAMICS_T>::ControlTrajectory;

  State state;
  Control control;

  DYNAMICS_T* model_;

  explicit ModelWrapperDDP(DYNAMICS_T* model)
  {
    model_ = model;
  }

  State f(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u)
  {
    // This section is specific to the neural network implementation for the autorally
    state = x;
    control = u;

    // Compute the state derivative xDot
    State dx;
    State next_state;
    typename DYNAMICS_T::output_array output;
    // TODO(tier4): Pass timestep from configuration instead of hard-coded value.
    model_->step(state, next_state, dx, control, output, 0, 0.01);
    return dx;
  }

  Jacobian df(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u)
  {
    constexpr int jacobian_cols = DYNAMICS_T::STATE_DIM + DYNAMICS_T::CONTROL_DIM;
    Jacobian j_ = Jacobian::Zero(DYNAMICS_T::STATE_DIM, jacobian_cols);
    state = x;
    control = u;
    using HasGrad = std::integral_constant<bool, HasAnalyticGrad<DYNAMICS_T>::Has>;
    bool analyticGradComputed = getGrad(model_, j_, state, control, HasGrad{});
    if (!analyticGradComputed)
    {
      j_ = WrappedDynamics<DYNAMICS_T>::df(x, u);
    }
    return j_;
  }
};

#endif  // MPPI__DDP__DDP_MODEL_WRAPPER_H_
