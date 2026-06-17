#ifndef TRAJOPT_BOXQP_HPP
#define TRAJOPT_BOXQP_HPP

#include "util.h"
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <cmath>
#include <tuple>

namespace util
{
template <typename T>
struct BoxQPOptions
{
  int max_iter;
  T min_grad_norm;
  T min_rel_improvement;
  T step_dec;
  T min_step;
  T armijo;
  bool verbose;

  BoxQPOptions(int max_iter, T min_grad_norm, T min_rel_improvement, T step_dec, T min_step, T armijo, bool verbose)
    : max_iter(max_iter)
    , min_grad_norm(min_grad_norm)
    , min_rel_improvement(min_rel_improvement)
    , step_dec(step_dec)
    , min_step(min_step)
    , armijo(armijo)
    , verbose(verbose)
  {
  }

  BoxQPOptions()
    : BoxQPOptions(100, static_cast<T>(1.0e-8), static_cast<T>(1.0e-8), static_cast<T>(0.6), static_cast<T>(1.0e-22),
                   static_cast<T>(0.1), false)
  {
  }
};

template <typename T, int N>
struct BoxQPResult
{
  Eigen::Matrix<T, N, 1> solution;
  Eigen::LLT<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> free_chol;
  Eigen::Matrix<bool, N, 1> free_indices;
  int result_code;

  BoxQPResult(const Eigen::Ref<const Eigen::Matrix<T, N, 1>>& solution,
              const Eigen::LLT<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>& free_chol,
              const Eigen::Ref<const Eigen::Matrix<bool, N, 1>>& free_indices, int result_code)
    : solution(solution), free_chol(free_chol), free_indices(free_indices), result_code(result_code)
  {
  }
};

template <typename T, int N>
class BoxQP
{
public:
  using Scalar = T;
  using Vector = Eigen::Matrix<Scalar, N, 1>;
  using SquareMatrix = Eigen::Matrix<Scalar, N, N>;
  using Hessian = SquareMatrix;
  using Gradient = Vector;
  using Solution = Vector;
  using FreeIndices = Eigen::Matrix<bool, N, 1>;
  using Result = BoxQPResult<T, N>;

  static const int HESSIAN_NOT_POSITIVE_DEFINITE = -1;
  static const int NO_DESCENT_DIRECTION = 0;
  static const int MAX_ITERATIONS_EXCEEDED = 1;
  static const int MAX_LINESEARCH_ITERATIONS_EXCEEDED = 2;
  static const int NO_BOUNDS = 3;
  static const int IMPROVEMENT_TOO_SMALL = 4;
  static const int GRADIENT_NORM_TOO_SMALL = 5;
  static const int ALL_DIMENSIONS_CLAMPED = 6;

  BoxQP(int max_iter, Scalar min_grad_norm, Scalar min_rel_improvement, Scalar step_dec, Scalar min_step, Scalar armijo,
        Logger* logger, bool verbose)
    : max_iter_(max_iter)
    , min_grad_(min_grad_norm)
    , min_rel_improve_(min_rel_improvement)
    , step_dec_(step_dec)
    , min_step_(min_step)
    , armijo_(armijo)
    , logger_(logger)
    , verbose_(verbose)
  {
  }

  BoxQP(const BoxQPOptions<Scalar>& options, Logger* logger)
    : BoxQP(options.max_iter, options.min_grad_norm, options.min_rel_improvement, options.step_dec, options.min_step,
            options.armijo, logger, options.verbose)
  {
  }

  BoxQP(Logger* logger) : BoxQP(BoxQPOptions<Scalar>(), logger)
  {
  }

  Result operator()(const Eigen::Ref<const Hessian>& H, const Eigen::Ref<const Gradient>& g,
                    const Eigen::Ref<const Vector>& lower_bound, const Eigen::Ref<const Vector>& upper_bound,
                    const Eigen::Ref<const Vector>& x0)
  {
    Eigen::Matrix<bool, N, 1> clamped = Eigen::Matrix<bool, N, 1>::Constant(false);
    Eigen::Matrix<bool, N, 1> free = Eigen::Matrix<bool, N, 1>::Constant(true);
    Vector x = x0.cwiseMin(upper_bound).cwiseMax(lower_bound);
    Scalar old_value = static_cast<Scalar>(0.0);
    int result = 0;
    Scalar gnorm = static_cast<Scalar>(0.0);
    int nfactor = 0;

    // Initial objective value
    Scalar value = x.transpose() * g + static_cast<Scalar>(0.5) * (x.transpose() * H * x).value();

    // Main loop
    int iter;
    for (iter = 0; iter < max_iter_; ++iter)
    {
      if (result != 0)
      {
        break;
      }

      // Check relative improvement
      if (iter > 0 && (old_value - value) < min_rel_improve_ * std::abs(old_value))
      {
        result = IMPROVEMENT_TOO_SMALL;
        if (verbose_)
          logger_->warning("BoxQP: Relative improvement too small - code %d", result);
        break;
      }
      old_value = value;

      // Get gradient
      Gradient grad = g + H * x;

      // Find clamped dimensions
      Eigen::Matrix<bool, N, 1> old_clamped = clamped;
      clamped = Eigen::Matrix<bool, N, 1>::Constant(false);
      for (int i = 0; i < N; ++i)
      {
        clamped(i) = (x(i) <= lower_bound(i) && grad(i) > static_cast<Scalar>(0.0)) ||
                     (x(i) >= upper_bound(i) && grad(i) < static_cast<Scalar>(0.0));
        free(i) = !clamped(i);
      }

      // Check for all clamped
      if (clamped.all())
      {
        result = ALL_DIMENSIONS_CLAMPED;
        if (verbose_)
          logger_->warning("BoxQP: All dimensions clamped - code %d", result);
        break;
      }

      // Factorize if clamped has changed
      if (iter == 0 || (clamped.template cast<int>() - old_clamped.template cast<int>()).template cast<bool>().any())
      {
        Eigen::Matrix<Scalar, Eigen::Dynamic, N> Hfr(free.count(), N);
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Hff(free.count(), free.count());
        int j = 0, k = 0;
        for (int i = 0; i < N; ++i)
        {
          if (free(i))
          {
            Hfr.row(j++) = H.row(i);
          }
        }
        for (int i = 0; i < N; ++i)
        {
          if (free(i))
          {
            Hff.col(k++) = Hfr.col(i);
          }
        }

        llt_.compute(Hff);
        if (llt_.info() != Eigen::Success)
        {
          result = HESSIAN_NOT_POSITIVE_DEFINITE;
          if (verbose_)
            logger_->warning("BoxQP: Hessian not positive definite - code %d", result);
          break;
        }

        ++nfactor;
      }

      // Check gradient norm
      gnorm = free.array().select(grad, Gradient::Zero()).norm();
      if (gnorm < min_grad_)
      {
        result = GRADIENT_NORM_TOO_SMALL;
        if (verbose_)
          logger_->warning("BoxQP: Gradient norm too small - code %d", result);
        break;
      }

      // Get search direction
      Gradient grad_clamped = g + H * (x.cwiseProduct(clamped.template cast<Scalar>()));
      Vector search = Vector::Zero();
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> grad_clamped_free(free.count());
      Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x_free(free.count());
      int j = 0;
      for (int i = 0; i < N; ++i)
      {
        if (free(i))
        {
          grad_clamped_free(j) = grad_clamped(i);
          x_free(j) = x(i);
          ++j;
        }
      }

      auto search_free = -llt_.solve(grad_clamped_free) - x_free;
      for (int i = N; i > 0; --i)
      {
        if (free(i - 1))
        {
          search(i - 1) = search_free(--j);
        }
      }

      // Check for descent direction
      Scalar sdotg = search.dot(grad);
      if (sdotg >= static_cast<Scalar>(0.0))
      {
        result = NO_DESCENT_DIRECTION;
        if (verbose_)
          logger_->warning("BoxQP: No descent direction - code %d", result);
        break;
      }

      // Armijo linesearch
      Scalar step = static_cast<Scalar>(1.0);
      int nstep = 0;
      Vector xc = (x + step * search).cwiseMin(upper_bound).cwiseMax(lower_bound);
      Scalar vc = xc.transpose() * g + static_cast<Scalar>(0.5) * (xc.transpose() * H * xc).value();
      while (((vc - old_value) / (step * sdotg)) < armijo_)
      {
        step *= step_dec_;
        ++nstep;
        xc = (x + step * search).cwiseMin(upper_bound).cwiseMax(lower_bound);
        vc = xc.transpose() * g + static_cast<Scalar>(0.5) * (xc.transpose() * H * xc).value();
        if (step < min_step_)
        {
          result = MAX_LINESEARCH_ITERATIONS_EXCEEDED;
          if (verbose_)
            logger_->warning("BoxQP: Max line search iterations exceeded - code %d", result);
          break;
        }
      }

      if (verbose_)
      {
        logger_->info("iter %-3d\tvalue %-9.5g\t|g| %-9.3g\treduction %-9.3g\tlinesearch %g^%-2d\tn_clamped %d\n", iter,
                      vc, gnorm, old_value - vc, step_dec_, nstep, clamped.count());
      }

      x = xc;
      value = vc;
    }

    if (iter >= max_iter_)
    {
      result = MAX_ITERATIONS_EXCEEDED;
      if (verbose_)
        logger_->info("BoxQP: All iterations completed - code %d", result);
    }

    if (verbose_)
    {
      logger_->info("Result code %d\titerations %d\tgradient %-12.6g\tfinal value %-12.6g\tfactorizations %d\n", result,
                    iter, gnorm, value, nfactor);
    }

    return { x, llt_, free, result };
  }

private:
  int max_iter_;
  Scalar min_grad_;
  Scalar min_rel_improve_;
  Scalar step_dec_;
  Scalar min_step_;
  Scalar armijo_;
  Logger* logger_;
  bool verbose_;
  Eigen::LLT<Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>> llt_;
};

}  // namespace util

#endif  // TRAJOPT_BOXQP_HPP
