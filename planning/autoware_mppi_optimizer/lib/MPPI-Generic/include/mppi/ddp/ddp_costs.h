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

#ifndef MPPI__DDP__DDP_COSTS_H_
#define MPPI__DDP__DDP_COSTS_H_

#include <Eigen/Dense>

template <class Dynamics>
struct CostFunction
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Scalar = typename Dynamics::Scalar;
  using State = typename Dynamics::State;
  using Control = typename Dynamics::Control;
  using Gradient = Eigen::Matrix<Scalar, Dynamics::StateSize + Dynamics::ControlSize, 1>;
  static constexpr int kStateControlSize = Dynamics::StateSize + Dynamics::ControlSize;
  using Hessian = Eigen::Matrix<Scalar, kStateControlSize, kStateControlSize>;

  CostFunction() = default;
  CostFunction(const CostFunction& other) = default;
  CostFunction(CostFunction&& other) = default;
  CostFunction& operator=(const CostFunction& other) = default;
  CostFunction& operator=(CostFunction&& other) = default;
  virtual ~CostFunction() = default;

  explicit CostFunction(const Eigen::Ref<const State>& target) : xf(target)
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

  explicit TerminalCostFunction(const Eigen::Ref<const State>& target) : xf(target)
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

#endif  // MPPI__DDP__DDP_COSTS_H_
