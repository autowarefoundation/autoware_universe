#ifndef TRAJOPT_COSTS_HPP
#define TRAJOPT_COSTS_HPP

#include <Eigen/Dense>

template <class Dynamics>
struct CostFunction
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Scalar = typename Dynamics::Scalar;
  using State = typename Dynamics::State;
  using Control = typename Dynamics::Control;
  using Gradient = Eigen::Matrix<Scalar, Dynamics::StateSize + Dynamics::ControlSize, 1>;
  using Hessian =
      Eigen::Matrix<Scalar, Dynamics::StateSize + Dynamics::ControlSize, Dynamics::StateSize + Dynamics::ControlSize>;

  CostFunction() = default;
  CostFunction(const CostFunction& other) = default;
  CostFunction(CostFunction&& other) = default;
  CostFunction& operator=(const CostFunction& other) = default;
  CostFunction& operator=(CostFunction&& other) = default;
  virtual ~CostFunction() = default;

  CostFunction(const Eigen::Ref<const State>& target) : xf(target)
  {
  }

  const Eigen::Ref<const State>& target() const
  {
    return xf;
  }
  Eigen::Ref<State> target()
  {
    return xf;
  }
  const Scalar& target(int idx) const
  {
    return xf(idx);
  }
  Scalar& target(int idx)
  {
    return xf(idx);
  }

  virtual Scalar c(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u, int t) = 0;
  virtual Gradient dc(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u, int t) = 0;
  virtual Hessian d2c(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u, int t) = 0;

  State xf;
};

template <class Dynamics>
struct TerminalCostFunction
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Scalar = typename Dynamics::Scalar;
  using State = typename Dynamics::State;
  using Gradient = Eigen::Matrix<Scalar, Dynamics::StateSize, 1>;
  using Hessian = Eigen::Matrix<Scalar, Dynamics::StateSize, Dynamics::StateSize>;

  TerminalCostFunction() = default;
  TerminalCostFunction(const TerminalCostFunction& other) = default;
  TerminalCostFunction(TerminalCostFunction&& other) = default;
  TerminalCostFunction& operator=(const TerminalCostFunction& other) = default;
  TerminalCostFunction& operator=(TerminalCostFunction&& other) = default;
  virtual ~TerminalCostFunction() = default;

  TerminalCostFunction(const Eigen::Ref<const State>& target) : xf(target)
  {
  }

  const Eigen::Ref<const State>& target() const
  {
    return xf;
  }

  Eigen::Ref<State> target()
  {
    return xf;
  }

  const Scalar& target(int idx) const
  {
    return xf(idx);
  }

  Scalar& target(int idx)
  {
    return xf(idx);
  }

  virtual Scalar c(const Eigen::Ref<const State>& x) = 0;
  virtual Gradient dc(const Eigen::Ref<const State>& x) = 0;
  virtual Hessian d2c(const Eigen::Ref<const State>& x) = 0;

  State xf;
};

#endif  // TRAJOPT_COSTS_HPP
