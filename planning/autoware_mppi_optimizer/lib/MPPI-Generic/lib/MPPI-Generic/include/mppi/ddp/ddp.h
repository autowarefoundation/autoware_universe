#ifndef TRAJOPT_DDP_HPP
#define TRAJOPT_DDP_HPP

#include "boxqp.h"
#include "result.h"
#include "util.h"
#include <Eigen/Dense>
#include <limits>
#include <memory>

template <class DynamicsT>
class DDP
{
  using Scalar = typename DynamicsT::Scalar;
  using State = typename DynamicsT::State;
  using StateTrajectory = typename DynamicsT::StateTrajectory;
  using Control = typename DynamicsT::Control;
  using ControlTrajectory = typename DynamicsT::ControlTrajectory;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DDP(Scalar time_step, int time_steps, int iterations, util::Logger* logger, bool verbose,
      util::BoxQPOptions<Scalar> qp_options = util::BoxQPOptions<Scalar>())
    : dt_(time_step)
    , H_(time_steps)
    , iter_(iterations)
    , logger_(logger)
    , verbose_(verbose)
    , Lk_(H_, Eigen::Matrix<Scalar, DynamicsT::ControlSize, DynamicsT::StateSize>::Zero())
    , df_(H_, Eigen::Matrix<Scalar, DynamicsT::StateSize, DynamicsT::StateSize + DynamicsT::ControlSize>::Zero())
    , d2L_(H_, Eigen::Matrix<Scalar, DynamicsT::StateSize + DynamicsT::ControlSize,
                             DynamicsT::StateSize + DynamicsT::ControlSize>::Zero())
    , Vxx_(H_, Eigen::Matrix<Scalar, DynamicsT::StateSize, DynamicsT::StateSize>::Zero())
    , VxxT_(H_, Eigen::Matrix<Scalar, DynamicsT::StateSize, DynamicsT::StateSize>::Zero())
    , boxqp_(qp_options, logger_)
  {
    x_.setZero(DynamicsT::StateSize, H_);
    u_.setZero(DynamicsT::ControlSize, H_);
    Is_.setIdentity();
    dL_.setZero(DynamicsT::StateSize + DynamicsT::ControlSize, H_);
    L_.setZero(H_);
    Vx_.setZero(DynamicsT::StateSize, H_);
    V_.setZero(H_);
    qx_.setZero();
    qu_.setZero();
    qux_.setZero();
    qxx_.setZero();
    quu_.setZero();
    lk_.setZero(DynamicsT::ControlSize, H_);
    cost_.setZero(iter_, H_);
  }

  template <typename CostFunction, typename TerminalCostFunction>
  OptimizerResult<DynamicsT>
  run(const Eigen::Ref<const State>& x0, const Eigen::Ref<const ControlTrajectory>& u, DynamicsT& dynamics,
      CostFunction& run_cost, TerminalCostFunction& term_cost,
      const Eigen::Ref<const Control>& u_min = -Control::Constant(std::numeric_limits<Scalar>::infinity()),
      const Eigen::Ref<const Control>& u_max = Control::Constant(std::numeric_limits<Scalar>::infinity()))
  {
    x_.col(0) = x0;
    u_ = u;
    for (int i = 1; i < H_; ++i)
    {
      if (i < H_ - 1)
      {
        // u_.col(i-1) += Lk_.at(i)*(x_.col(i-1) - x_.col(i));
        u_.col(i - 1) = u_.col(i - 1).cwiseMin(u_max).cwiseMax(u_min);
      }
      x_.col(i) = x_.col(i - 1) + dynamics.f(x_.col(i - 1), u_.col(i - 1)) * dt_;
    }

    int current_iteration = 0;
    while (current_iteration < iter_)
    {
      // Obtain Jacobians -- this can be parallelized but is pretty bad with OpenMP
      for (int k = 0; k < H_; ++k)
      {
        df_.at(k) = dynamics.df(x_.col(k), u_.col(k)) * dt_;
        df_.at(k).leftCols(DynamicsT::StateSize) += Is_;
        d2L_.at(k) = run_cost.d2c(x_.col(k), u_.col(k), k);
        // std::cout << run_cost.c(x_.col(k), u_.col(k), k) << std::endl;
        dL_.col(k) = run_cost.dc(x_.col(k), u_.col(k), k);
        L_(k) = run_cost.c(x_.col(k), u_.col(k), k);
      }

      // Evaluate boundary condition
      Vxx_.at(H_ - 1) = term_cost.d2c(x_.col(H_ - 1));
      VxxT_.at(H_ - 1) = Vxx_.at(H_ - 1).transpose();
      Vxx_.at(H_ - 1) = static_cast<Scalar>(0.5) * (Vxx_.at(H_ - 1) + VxxT_.at(H_ - 1));
      Vx_.col(H_ - 1) = term_cost.dc(x_.col(H_ - 1));
      V_(H_ - 1) = term_cost.c(x_.col(H_ - 1));

      // Perform backward pass
      for (int k = H_ - 2; k >= 0; --k)
      {
        const auto& Phi = df_.at(k).leftCols(DynamicsT::StateSize);
        const auto& B = df_.at(k).rightCols(DynamicsT::ControlSize);

        qx_ = dL_.col(k).head(DynamicsT::StateSize) * dt_ + Phi.transpose() * Vx_.col(k + 1);
        qu_ = dL_.col(k).tail(DynamicsT::ControlSize) * dt_ + B.transpose() * Vx_.col(k + 1);
        qux_ = d2L_.at(k).bottomLeftCorner(DynamicsT::ControlSize, DynamicsT::StateSize) * dt_ +
               B.transpose() * Vxx_.at(k + 1) * Phi;
        qxx_ = d2L_.at(k).topLeftCorner(DynamicsT::StateSize, DynamicsT::StateSize) * dt_ +
               Phi.transpose() * Vxx_.at(k + 1) * Phi;
        quu_ = d2L_.at(k).bottomRightCorner(DynamicsT::ControlSize, DynamicsT::ControlSize) * dt_ +
               B.transpose() * Vxx_.at(k + 1) * B;

        Eigen::LDLT<Eigen::Matrix<Scalar, DynamicsT::ControlSize, DynamicsT::ControlSize> > ldlt(quu_);
        if (ldlt.info() == Eigen::Success)
        {
          Lk_.at(k) = ldlt.solve(-qux_);
          lk_.col(k) = ldlt.solve(-qu_);
        }
        else
        {
          logger_->error("Failed -- DEAD\n");
          std::exit(-3);
        }

        // Backprop value function
        Vxx_.at(k) = qxx_ + qux_.transpose() * Lk_.at(k);
        VxxT_.at(k) = Vxx_.at(k).transpose();
        Vxx_.at(k) = static_cast<Scalar>(0.5) * (Vxx_.at(k) + VxxT_.at(k));
        Vx_.col(k) = qx_ + qux_.transpose() * lk_.col(k);
        V_(k) = static_cast<Scalar>(0.5) * (qu_.transpose() * lk_.col(k)).value();
      }

      bool accept = false;
      Scalar alpha = static_cast<Scalar>(1.0);
      StateTrajectory xnew = StateTrajectory::Zero(DynamicsT::StateSize, H_);
      ControlTrajectory unew = ControlTrajectory::Zero(DynamicsT::ControlSize, H_);
      while (!accept)
      {
        xnew.col(0) = x_.col(0);

        for (int k = 0; k < H_ - 1; ++k)
        {
          State dx = xnew.col(k) - x_.col(k);
          unew.col(k) = u_.col(k) + alpha * lk_.col(k) + Lk_.at(k) * dx;
          unew.col(k) = unew.col(k).cwiseMin(u_max).cwiseMax(u_min);
          xnew.col(k + 1) = xnew.col(k) + dynamics.f(xnew.col(k), unew.col(k)) * dt_;
          cost_(current_iteration, k) = run_cost.c(xnew.col(k), unew.col(k), k) * dt_;
        }
        cost_(current_iteration, H_ - 1) = V_(H_ - 1);

        if (current_iteration == 0 || alpha < static_cast<Scalar>(0.0001) ||
            cost_.row(current_iteration).sum() <= cost_.row(current_iteration - 1).sum())
        {
          u_ = unew;
          x_ = xnew;
          accept = true;
        }
        else
        {
          alpha *= static_cast<Scalar>(0.5);
          cost_.row(current_iteration).setZero();
          unew.setZero();
          xnew.setZero();
        }
      }
      ++current_iteration;
    }

    return OptimizerResult<DynamicsT>(current_iteration, H_, cost_.row(current_iteration - 1).sum(), cost_, x_, u_, Lk_,
                                      lk_);
  }

private:
  Scalar dt_;
  int H_;
  int iter_;
  util::Logger* logger_;
  bool verbose_;

  util::EigenAlignedVector<Scalar, DynamicsT::ControlSize, DynamicsT::StateSize> Lk_;
  util::EigenAlignedVector<Scalar, DynamicsT::StateSize, DynamicsT::StateSize + DynamicsT::ControlSize> df_;
  util::EigenAlignedVector<Scalar, DynamicsT::StateSize + DynamicsT::ControlSize,
                           DynamicsT::StateSize + DynamicsT::ControlSize>
      d2L_;
  util::EigenAlignedVector<Scalar, DynamicsT::StateSize, DynamicsT::StateSize> Vxx_;
  util::EigenAlignedVector<Scalar, DynamicsT::StateSize, DynamicsT::StateSize> VxxT_;
  util::BoxQP<Scalar, DynamicsT::ControlSize> boxqp_;
  StateTrajectory x_;
  ControlTrajectory u_;
  Eigen::Matrix<Scalar, DynamicsT::StateSize, DynamicsT::StateSize> Is_;
  Eigen::Matrix<Scalar, DynamicsT::StateSize + DynamicsT::ControlSize, Eigen::Dynamic> dL_;
  Eigen::Matrix<Scalar, 1, Eigen::Dynamic> L_;
  Eigen::Matrix<Scalar, DynamicsT::StateSize, Eigen::Dynamic> Vx_;
  Eigen::Matrix<Scalar, 1, Eigen::Dynamic> V_;
  State qx_;
  Control qu_;
  Eigen::Matrix<Scalar, DynamicsT::ControlSize, DynamicsT::StateSize> qux_;
  Eigen::Matrix<Scalar, DynamicsT::StateSize, DynamicsT::StateSize> qxx_;
  Eigen::Matrix<Scalar, DynamicsT::ControlSize, DynamicsT::ControlSize> quu_;
  Eigen::Matrix<Scalar, DynamicsT::ControlSize, Eigen::Dynamic> lk_;
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cost_;
};

#endif  // TRAJOPT_DDP_HPP
