// Copyright 2026 The Autoware Foundation
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

#ifndef AUTOWARE__MPC_LATERAL_CONTROLLER__TAUBIN_CURVATURE_HPP_
#define AUTOWARE__MPC_LATERAL_CONTROLLER__TAUBIN_CURVATURE_HPP_

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

namespace autoware::motion::control::mpc_lateral_controller
{

constexpr double kDefaultStraightThreshold = 5e-3;
constexpr double kMinDiscriminant = 1e-14;
constexpr double kMinKappa = 1e-9;

struct TaubinResult
{
  double kappa{0.0};
  double radius{std::numeric_limits<double>::infinity()};
  std::optional<Eigen::Vector2d> center{std::nullopt};
  bool is_straight{false};
  double rms_residual{0.0};
  Eigen::Vector4d eigenvector{Eigen::Vector4d::Zero()};
  double discriminant{0.0};
};

namespace detail
{

[[nodiscard]] inline double collinearity_ratio(const Eigen::MatrixX2d & pts_centered)
{
  const Eigen::JacobiSVD<Eigen::MatrixX2d> svd(
    pts_centered, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const auto & sv = svd.singularValues();
  if (sv[0] < 1e-12) {
    return 0.0;
  }
  return sv[1] / sv[0];
}

[[nodiscard]] inline double curvature_sign(const Eigen::MatrixX2d & pts)
{
  const int N = static_cast<int>(pts.rows());
  double signed_area = 0.0;
  for (int i = 0; i < N - 1; ++i) {
    signed_area += pts(i, 0) * pts(i + 1, 1) - pts(i + 1, 0) * pts(i, 1);
  }
  return (signed_area >= 0.0) ? 1.0 : -1.0;
}

inline void build_taubin_matrices(
  const Eigen::VectorXd & x, const Eigen::VectorXd & y, Eigen::Matrix4d & M, Eigen::Matrix4d & C)
{
  const int N = static_cast<int>(x.size());
  const double inv_N = 1.0 / static_cast<double>(N);

  const Eigen::VectorXd z = x.array().square() + y.array().square();

  const double Mzz = z.dot(z) * inv_N;
  const double Mxz = x.dot(z) * inv_N;
  const double Myz = y.dot(z) * inv_N;
  const double Mxx = x.dot(x) * inv_N;
  const double Myy = y.dot(y) * inv_N;
  const double Mxy = x.dot(y) * inv_N;
  const double Mzo = z.mean();
  const double Mxo = x.mean();
  const double Myo = y.mean();

  // clang-format off
  M << Mzz, Mxz, Myz, Mzo,
       Mxz, Mxx, Mxy, Mxo,
       Myz, Mxy, Myy, Myo,
       Mzo, Mxo, Myo, 1.0;
  // clang-format on

  C.setZero();
  C(0, 3) = 2.0;
  C(1, 1) = 1.0;
  C(2, 2) = 1.0;
  C(3, 0) = 2.0;
}

inline void solve_and_extract_kappa(
  const Eigen::Matrix4d & M, const Eigen::Matrix4d & C, Eigen::Vector4d & eigenvector,
  double & kappa_abs, double & discriminant)
{
  const Eigen::Matrix4d CinvM = C.inverse() * M;
  const Eigen::EigenSolver<Eigen::Matrix4d> solver(CinvM);

  const Eigen::Vector4cd & evals = solver.eigenvalues();
  const Eigen::Matrix4cd & evecs = solver.eigenvectors();

  int best_idx = -1;
  double best_abs_eval = std::numeric_limits<double>::max();

  for (int i = 0; i < 4; ++i) {
    const double re = evals[i].real();
    const double im = std::abs(evals[i].imag());

    if (im > 1e-6 * std::abs(re)) {
      continue;
    }

    const Eigen::Vector4d v = evecs.col(i).real();
    const double v0 = v[0];
    const double v1 = v[1];
    const double v2 = v[2];
    const double v3 = v[3];
    const double disc = v1 * v1 + v2 * v2 - 4.0 * v0 * v3;

    if (disc <= kMinDiscriminant) {
      continue;
    }

    const double k_abs = 2.0 * std::abs(v0) / std::sqrt(disc);
    if (k_abs <= 0.0) {
      continue;
    }

    const double abs_eval = std::abs(re);
    if (abs_eval < best_abs_eval) {
      best_abs_eval = abs_eval;
      best_idx = i;
    }
  }

  if (best_idx < 0) {
    eigenvector = Eigen::Vector4d::Zero();
    kappa_abs = 0.0;
    discriminant = 0.0;
    return;
  }

  eigenvector = evecs.col(best_idx).real();

  const double v0 = eigenvector[0];
  const double v1 = eigenvector[1];
  const double v2 = eigenvector[2];
  const double v3 = eigenvector[3];

  discriminant = v1 * v1 + v2 * v2 - 4.0 * v0 * v3;
  kappa_abs = 2.0 * std::abs(v0) / std::sqrt(discriminant);
}

[[nodiscard]] inline double compute_rms_residual(
  const Eigen::MatrixX2d & pts_c, const Eigen::Vector4d & v)
{
  const double v0 = v[0];
  const double v1 = v[1];
  const double v2 = v[2];
  const double v3 = v[3];
  const auto & x = pts_c.col(0);
  const auto & y = pts_c.col(1);

  const Eigen::VectorXd z = x.array().square() + y.array().square();
  const Eigen::VectorXd F = v0 * z + v1 * x + v2 * y + Eigen::VectorXd::Constant(x.size(), v3);

  const Eigen::VectorXd gx = 2.0 * v0 * x.array() + v1;
  const Eigen::VectorXd gy = 2.0 * v0 * y.array() + v2;
  const Eigen::VectorXd g_norm = (gx.array().square() + gy.array().square()).sqrt();

  const Eigen::VectorXd safe_g = g_norm.array().max(1e-12);
  const Eigen::VectorXd sampson = F.array().abs() / safe_g.array();

  return std::sqrt(sampson.squaredNorm() / static_cast<double>(x.size()));
}

}  // namespace detail

[[nodiscard]] inline TaubinResult taubin_curvature(
  const Eigen::MatrixX2d & points, const double straight_threshold = kDefaultStraightThreshold)
{
  if (points.cols() != 2 || points.rows() < 3) {
    throw std::invalid_argument("taubin_curvature: input must be (N×2) with N ≥ 3");
  }

  TaubinResult result;

  const Eigen::Vector2d centroid = points.colwise().mean();
  const Eigen::MatrixX2d pts_c = points.rowwise() - centroid.transpose();

  const double col_ratio = detail::collinearity_ratio(pts_c);
  if (col_ratio < straight_threshold) {
    result.is_straight = true;
    result.kappa = 0.0;
    result.radius = std::numeric_limits<double>::infinity();
    result.rms_residual = 0.0;
    return result;
  }

  const Eigen::VectorXd x = pts_c.col(0);
  const Eigen::VectorXd y = pts_c.col(1);

  Eigen::Matrix4d M;
  Eigen::Matrix4d C;
  detail::build_taubin_matrices(x, y, M, C);

  double kappa_abs{0.0};
  detail::solve_and_extract_kappa(M, C, result.eigenvector, kappa_abs, result.discriminant);

  const double sign = detail::curvature_sign(pts_c);
  result.kappa = sign * kappa_abs;

  if (kappa_abs > kMinKappa) {
    result.radius = 1.0 / kappa_abs;

    const double v0 = result.eigenvector[0];
    const double v1 = result.eigenvector[1];
    const double v2 = result.eigenvector[2];
    const Eigen::Vector2d center_c{-v1 / (2.0 * v0), -v2 / (2.0 * v0)};
    result.center = center_c + centroid;
  }

  result.rms_residual = detail::compute_rms_residual(pts_c, result.eigenvector);

  return result;
}

}  // namespace autoware::motion::control::mpc_lateral_controller

#endif  // AUTOWARE__MPC_LATERAL_CONTROLLER__TAUBIN_CURVATURE_HPP_
