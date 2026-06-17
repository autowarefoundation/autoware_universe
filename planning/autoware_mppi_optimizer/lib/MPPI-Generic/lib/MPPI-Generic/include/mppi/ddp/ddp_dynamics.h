#ifndef TRAJOPT_DYNAMICS_HPP
#define TRAJOPT_DYNAMICS_HPP

#include "util.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/NumericalDiff>
#include <functional>

namespace internal
{
template <class T, int S, int C>
struct Differentiable
{
  /*****************************************************************************/
  /*** Replicate Eigen's generic functor implementation to avoid inheritance ***/
  /*** We only use the fixed-size functionality ********************************/
  /*****************************************************************************/
  enum
  {
    InputsAtCompileTime = S + C,
    ValuesAtCompileTime = S
  };
  using Scalar = T;
  using InputType = Eigen::Matrix<T, InputsAtCompileTime, 1>;
  using ValueType = Eigen::Matrix<T, ValuesAtCompileTime, 1>;
  using JacobianType = Eigen::Matrix<T, ValuesAtCompileTime, InputsAtCompileTime>;
  int inputs() const
  {
    return InputsAtCompileTime;
  }
  int values() const
  {
    return ValuesAtCompileTime;
  }
  int operator()(const Eigen::Ref<const InputType>& xu, Eigen::Ref<ValueType> dx) const
  {
    dx = f_(xu.template head<S>(), xu.template tail<C>());
    return 0;
  }
  /*****************************************************************************/

  using DiffFunc = std::function<Eigen::Matrix<T, S, 1>(const Eigen::Ref<const Eigen::Matrix<T, S, 1>>&,
                                                        const Eigen::Ref<const Eigen::Matrix<T, C, 1>>&)>;
  Differentiable(const DiffFunc& f) : f_(f)
  {
  }

private:
  DiffFunc f_;
};

}  // namespace internal

namespace DDP_structures
{
template <class T, int S, int C>
struct Dynamics
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum
  {
    StateSize = S,
    ControlSize = C
  };
  using Scalar = T;
  using State = Eigen::Matrix<T, StateSize, 1>;
  using Control = Eigen::Matrix<T, ControlSize, 1>;
  using StateTrajectory = Eigen::Matrix<T, StateSize, Eigen::Dynamic>;
  using ControlTrajectory = Eigen::Matrix<T, ControlSize, Eigen::Dynamic>;
  using Jacobian = typename internal::Differentiable<T, StateSize, ControlSize>::JacobianType;
  using StateControl = typename internal::Differentiable<T, StateSize, ControlSize>::InputType;
  using FeedbackGain = Eigen::Matrix<T, ControlSize, StateSize>;
  using FeedforwardGain = Eigen::Matrix<T, ControlSize, 1>;
  using FeedbackGainTrajectory = util::EigenAlignedVector<T, C, S>;
  using FeedforwardGainTrajectory = Eigen::Matrix<T, ControlSize, Eigen::Dynamic>;

  Dynamics()
    : diff_([this](const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u) -> State {
      return this->f(x, u);
    })
    , num_diff_(diff_)
  {
  }

  Dynamics(const Dynamics& other) = default;

  Dynamics(Dynamics&& other) = default;

  Dynamics& operator=(const Dynamics& other) = default;

  Dynamics& operator=(Dynamics&& other) = default;

  virtual ~Dynamics() = default;

  // Implementation required
  virtual State f(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u) = 0;

  // Implementation optional
  virtual Jacobian df(const Eigen::Ref<const State>& x, const Eigen::Ref<const Control>& u)
  {
    num_diff_.df((typename internal::Differentiable<T, StateSize, ControlSize>::InputType() << x, u).finished(), j_);
    return j_;
  }

private:
  Jacobian j_;
  typename internal::Differentiable<T, StateSize, ControlSize> diff_;
  Eigen::NumericalDiff<typename internal::Differentiable<T, StateSize, ControlSize>, Eigen::Central> num_diff_;
};
}  // namespace DDP_structures

#endif  // TRAJOPT_DYNAMICS_HPP
