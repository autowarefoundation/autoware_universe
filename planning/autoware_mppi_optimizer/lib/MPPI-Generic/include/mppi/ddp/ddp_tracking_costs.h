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

#ifndef MPPI__DDP__DDP_TRACKING_COSTS_H_
#define MPPI__DDP__DDP_TRACKING_COSTS_H_

#include "ddp_dynamics.h"
#include "ddp_costs.h"

template <typename DynamicsT>
struct TrackingCostDDP : public CostFunction<DynamicsT>
{
  using Dynamics = DynamicsT;
  using Scalar = typename Dynamics::Scalar;
  using State = typename Dynamics::State;
  using Control = typename Dynamics::Control;
  using Gradient = typename CostFunction<Dynamics>::Gradient;
  using Hessian = typename CostFunction<Dynamics>::Hessian;

  static const int N = Dynamics::StateSize;
  static const int M = Dynamics::ControlSize;
  using StateCostWeight = Eigen::Matrix<Scalar, N, N>;
  using ControlCostWeight = Eigen::Matrix<Scalar, M, M>;

public:
  Eigen::MatrixXf traj_target_x_;
  Eigen::MatrixXf traj_target_u_;

  TrackingCostDDP(const Eigen::Ref<const StateCostWeight>& Q, const Eigen::Ref<const ControlCostWeight>& R,
                  int num_timesteps)
    : Q_(Q), R_(R)
  {
    QR_.setZero();
    QR_.template topLeftCorner<N, N>() = Q;
    QR_.template bottomRightCorner<M, M>() = R;
    traj_target_x_ = Eigen::MatrixXf::Zero(Dynamics::StateSize, num_timesteps);
    traj_target_u_ = Eigen::MatrixXf::Zero(Dynamics::ControlSize, num_timesteps);
  }

  Scalar c(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u, int t)
  {
    // TODO(tier4): Use method inside of dynamics to compute this.
    const auto state_error = x - traj_target_x_.col(t);
    float state_cost = (state_error.transpose() * Q_ * state_error).value();
    const auto control_error = u - traj_target_u_.col(t);
    float control_cost = (control_error.transpose() * R_ * control_error).value();
    return state_cost + control_cost;
  }

  Gradient dc(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u, int t)
  {
    return (Gradient() << Q_ * (x - traj_target_x_.col(t)), R_ * (u - traj_target_u_.col(t))).finished();
  }

  Hessian d2c(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u, int t)
  {
    return QR_;
  }

  void setTargets(const float* traj_target, const float* control_target, int timesteps)
  {
    for (int t = 0; t < timesteps; t++)
    {
      for (int i = 0; i < Dynamics::StateSize; i++)
      {
        traj_target_x_(i, t) = traj_target[Dynamics::StateSize * t + i];
      }
    }

    for (int t = 0; t < timesteps; t++)
    {
      for (int i = 0; i < Dynamics::ControlSize; i++)
      {
        traj_target_u_(i, t) = control_target[Dynamics::ControlSize * t + i];
      }
    }
  }

  // Still specific for autorally
  //    void setStop(Eigen::MatrixXf state, int timesteps)
  //    {
  //        for (int t = 0; t < timesteps; t++){
  //            traj_target_x_.col(t) << state;
  //        }
  //        traj_target_u_ = Eigen::MatrixXf::Zero(Dynamics::ControlSize, timesteps);
  //        Q_.diagonal() << 10.0, 10.0, 25.0, 10.0, 10.0, 10.0, 10.0;
  //    }

private:
  StateCostWeight Q_;
  ControlCostWeight R_;
  Hessian QR_;
};

template <typename DynamicsT>
struct TrackingTerminalCost : public TerminalCostFunction<DynamicsT>
{
  using Dynamics = DynamicsT;
  using Scalar = typename Dynamics::Scalar;
  using State = typename Dynamics::State;
  using Gradient = typename TerminalCostFunction<Dynamics>::Gradient;
  using Hessian = typename TerminalCostFunction<Dynamics>::Hessian;

  static const int N = Dynamics::StateSize;
  using StateCostWeight = Eigen::Matrix<Scalar, N, N>;

public:
  explicit TrackingTerminalCost(const Eigen::Ref<const StateCostWeight>& Qf,
                                const Eigen::Ref<const State>& xf = State::Zero())
    : Qf_(Qf)
  {
    this->target() = xf;  // Initialize the target to zero
  }

  Scalar c(const Eigen::Ref<const State>& x)
  {
    return ((x - this->target()).transpose() * Qf_ * (x - this->target())).value();
  }

  Gradient dc(const Eigen::Ref<const State>& x)
  {
    return Qf_ * (x - this->target());
  }

  Hessian d2c(const Eigen::Ref<const State>& x)
  {
    return Qf_;
  }

private:
  StateCostWeight Qf_;
};

#endif  // MPPI__DDP__DDP_TRACKING_COSTS_H_
