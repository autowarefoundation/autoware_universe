// Copyright 2025 TIER IV, Inc.
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
#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__KALMAN_FILTER_TEMPLATE_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__KALMAN_FILTER_TEMPLATE_HPP_

#include <Eigen/Core>

namespace autoware::multi_object_tracker
{

template <int StateSize, int MeasurementSize, int ControlSize = 0, typename Scalar = double>
class KalmanFilterTemplate
{
public:
  using StateVec = Eigen::Matrix<Scalar, StateSize, 1>;
  using StateMat = Eigen::Matrix<Scalar, StateSize, StateSize>;
  using MeasVec = Eigen::Matrix<Scalar, MeasurementSize, 1>;
  using MeasMat = Eigen::Matrix<Scalar, MeasurementSize, MeasurementSize>;
  using ControlVec = Eigen::Matrix<Scalar, ControlSize, 1>;
  using ProcessMat = Eigen::Matrix<Scalar, StateSize, StateSize>;
  using ControlMat = Eigen::Matrix<Scalar, StateSize, ControlSize>;
  using MeasModelMat = Eigen::Matrix<Scalar, MeasurementSize, StateSize>;

  /**
   * @brief No initialization constructor.
   */
  KalmanFilterTemplate() = default;

  /**
   * @brief constructor with initialization
   * @param x initial state
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param C coefficient matrix of x for measurement model
   * @param Q covariance matrix for process model
   * @param R covariance matrix for measurement model
   * @param P initial covariance of estimated state
   */
  KalmanFilterTemplate(
    const StateVec & x, const ProcessMat & A, const ControlMat & B, const MeasModelMat & C,
    const StateMat & Q, const MeasMat & R, const StateMat & P)
  : x_(x), A_(A), B_(B), C_(C), Q_(Q), R_(R), P_(P)
  {
  }

  /**
   * @brief destructor
   */
  ~KalmanFilterTemplate() = default;

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param C coefficient matrix of x for measurement model
   * @param Q covariance matrix for process model
   * @param R covariance matrix for measurement model
   * @param P initial covariance of estimated state
   */
  void init(
    const StateVec & x, const ProcessMat & A, const ControlMat & B, const MeasModelMat & C,
    const StateMat & Q, const MeasMat & R, const StateMat & P)
  {
    x_ = x;
    A_ = A;
    B_ = B;
    C_ = C;
    Q_ = Q;
    R_ = R;
    P_ = P;
  }

  /**
   * @brief initialization of kalman filter
   * @param x initial state
   * @param P initial covariance of estimated state
   */
  void init(const StateVec & x, const StateMat & P)
  {
    x_ = x;
    P_ = P;
  }

  /**
   * @brief set A of process model
   * @param A coefficient matrix of x for process model
   */
  void setA(const ProcessMat & A) noexcept { A_ = A; }

  /**
   * @brief set B of control model
   * @param B coefficient matrix of u for process model
   */
  void setB(const ControlMat & B) noexcept
  {
    static_assert(ControlSize > 0, "Control matrix requires ControlSize > 0");
    B_ = B;
  }

  /**
   * @brief set C of measurement model
   * @param C coefficient matrix of x for measurement model
   */
  void setC(const MeasModelMat & C) noexcept { C_ = C; }

  /**
   * @brief set Q of process model
   * @param Q covariance matrix for process model
   */
  void setQ(const StateMat & Q) noexcept { Q_ = Q; }

  /**
   * @brief set R of measurement model
   * @param R covariance matrix for measurement model
   */
  void setR(const MeasMat & R) noexcept { R_ = R; }

  /**
   * @brief get current kalman filter state
   * @param x kalman filter state
   */
  void getX(StateVec & x) const noexcept { x = x_; }

  /**
   * @brief get current kalman filter covariance
   * @param P kalman filter covariance
   */
  void getP(StateMat & P) const noexcept { P = P_; }

  /**
   * @brief get element of x
   * @param i index of x
   * @return value of i's component of the kalman filter state x[i]
   */
  [[nodiscard]] Scalar getXelement(unsigned int i) const noexcept { return x_(i); }

  /**
   * @brief calculate kalman filter state and covariance by prediction model with A, B, Q matrix.
   * This is mainly for EKF with variable matrix.
   * @param u input for model
   * @param A coefficient matrix of x for process model
   * @param B coefficient matrix of u for process model
   * @param Q covariance matrix for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const ControlVec & u, const ProcessMat & A, const ControlMat & B, const StateMat & Q)
  {
    x_ = A * x_ + B * u;
    P_ = A * P_ * A.transpose() + Q;
    return true;
  }
  /**
   * @brief calculate kalman filter covariance with prediction model with x, A, Q matrix. This is
   * mainly for EKF with variable matrix.
   * @param x_next predicted state
   * @param A coefficient matrix of x for process model
   * @param Q covariance matrix for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const StateVec & x_next, const ProcessMat & A, const StateMat & Q)
  {
    x_ = x_next;
    P_ = A * P_ * A.transpose() + Q;
    return true;
  }

  /**
   * @brief calculate kalman filter covariance with prediction model with x, A, Q matrix. This is
   * mainly for EKF with variable matrix.
   * @param x_next predicted state
   * @param A coefficient matrix of x for process model
   * @return bool to check matrix operations are being performed properly
   */
  bool predict(const StateVec & x_next, const ProcessMat & A) { return predict(x_next, A, Q_); }

  /**
   * @brief predict
   * @param u control vector
   * @return true if prediction is successful
   */
  bool predict(const ControlVec & u) noexcept
  {
    static_assert(
      ControlSize > 0,
      "predict(u) called but ControlSize = 0. Use predict() without arguments instead.");
    return predict(u, A_, B_, Q_);
  }

  /**
   * @brief predict
   * @return true if prediction is successful
   */
  bool predict() noexcept
  {
    static_assert(ControlSize == 0, "This predict() can only be used when ControlSize = 0");
    x_ = A_ * x_;
    P_ = A_ * P_ * A_.transpose() + Q_;
    return true;
  }

  /**
   * @brief calculate kalman filter state by measurement model with y_pred, C and R matrix. This is
   * mainly for EKF with variable matrix.
   * @param y measured values
   * @param y_pred output values expected from measurement model
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @return bool to check matrix operations are being performed properly
   */
  bool update(const MeasVec & y, const MeasVec & y_pred, const MeasModelMat & C, const MeasMat & R)
  {
    const MeasModelMat PCT = P_ * C.transpose();
    const MeasMat S = C * PCT + R;
    const MeasModelMat K = PCT * S.inverse();

    if (!K.allFinite()) {
      return false;
    }
    x_ += K * (y - y_pred);
    P_ -= K * (C * P_);
    return true;
  }

  /**
   * @brief calculate kalman filter state by measurement model with C and R matrix. This is mainly
   * for EKF with variable matrix.
   * @param y measured values
   * @param C coefficient matrix of x for measurement model
   * @param R covariance matrix for measurement model
   * @return bool to check matrix operations are being performed properly
   */
  template <int DynamicMeasurementSize>
  bool update(
    const Eigen::Matrix<Scalar, DynamicMeasurementSize, 1> & y,
    const Eigen::Matrix<Scalar, DynamicMeasurementSize, StateSize> & C,
    const Eigen::Matrix<Scalar, DynamicMeasurementSize, DynamicMeasurementSize> & R)
  {
    const Eigen::Matrix<Scalar, DynamicMeasurementSize, 1> y_pred = C * x_;
    const Eigen::Matrix<Scalar, DynamicMeasurementSize, DynamicMeasurementSize> S =
      C * P_ * C.transpose() + R;
    const Eigen::Matrix<Scalar, StateSize, DynamicMeasurementSize> K =
      P_ * C.transpose() * S.inverse();

    if (!K.allFinite()) {
      return false;
    }
    x_ += K * (y - y_pred);
    P_ -= K * C * P_;
    return true;
  }

  /**
   * @brief calculate kalman filter state by measurement model with C and R being class member
   * variables.
   * @param y measured values
   * @return bool to check matrix operations are being performed properly
   */
  bool update(const MeasVec & y) noexcept { return update(y, C_, R_); }

protected:
  StateVec x_;      // State vector
  ProcessMat A_;    // State transition matrix
  ControlMat B_;    // Control matrix
  MeasModelMat C_;  // Measurement matrix
  StateMat Q_;      // Process noise covariance
  MeasMat R_;       // Measurement noise covariance
  StateMat P_;      // Estimate error covariance
};

}  // namespace autoware::multi_object_tracker
#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__KALMAN_FILTER_TEMPLATE_HPP_
