#ifndef DDP_TRACKING_COSTS_
#define DDP_TRACKING_COSTS_

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
    // TODO use method inside of dynamics to compute this
    float state_cost = ((x - traj_target_x_.col(t)).transpose() * Q_ * (x - traj_target_x_.col(t))).value();
    float control_cost = ((u - traj_target_u_.col(t)).transpose() * R_ * (u - traj_target_u_.col(t))).value();
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
  TrackingTerminalCost(const Eigen::Ref<const StateCostWeight>& Qf, const Eigen::Ref<const State>& xf = State::Zero())
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

#endif /*DDP_TRACKING_COSTS_H_*/
