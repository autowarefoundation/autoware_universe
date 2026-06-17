#ifndef TRAJOPT_UTIL_HPP
#define TRAJOPT_UTIL_HPP

#include <Eigen/StdVector>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <vector>

/**
 * @namespace util  Miscellaneous utilities, convenience items, and type definitions.
 */
namespace util
{
/**
 * @tparam T    Scalar type of the Eigen::Matrix being stored
 * @tparam R    Number of Matrix rows
 * @tparam C    Number of Matrix columns
 * @brief       Convenience typedef for Eigen-aligned std::vector storing fixed-size Eigen matrices.
 */
template <typename T, int R, int C>
using EigenAlignedVector = std::vector<Eigen::Matrix<T, R, C>, Eigen::aligned_allocator<Eigen::Matrix<T, R, C>>>;

/**
 * @tparam MatrixType   Typedef'd Eigen type
 * @brief               Convenience typedef for Eigen-aligned std::vector storing fixed-size Eigen matrices.
 */
template <typename MatrixType>
using NamedEigenAlignedVector = std::vector<MatrixType, Eigen::aligned_allocator<MatrixType>>;

template <typename Dynamics>
/**
 * @brief   Convenience function to combine State and Control into a single vector.  Note that a copy may be made if
 *          RVO is not applied.
 * @param x State vector
 * @param u Control vector
 * @return  Combined State and Control vector of size StateSize + ControlSize
 */
inline typename Dynamics::StateControl combine_xu(const Eigen::Ref<const typename Dynamics::State>& x,
                                                  const Eigen::Ref<const typename Dynamics::Control>& u)
{
  using StateControl = typename Dynamics::StateControl;
  return (StateControl() << x, u).finished();
}

/**
 * @brief       Return the number of time steps that fit in the time horizon.
 * @param tf    Time horizon
 * @param dt    Step size
 * @return      Ceiling of the number of dt's in tf
 */
inline int time_steps(double tf, double dt)
{
  return static_cast<int>(std::ceil(tf / dt));
}

/**
 * @brief   Abstract class for logging informational, error, and warning messages.
 */
class Logger
{
public:
  Logger() = default;
  Logger(const Logger& other) = default;
  Logger(Logger&& other) = default;
  virtual ~Logger() = default;
  Logger& operator=(const Logger& other) = default;
  Logger& operator=(Logger&& other) = default;

  /**
   * @brief       Pure virtual function to log informational messages.
   * @param fmt   Format string (if additional arguments are passed) or message to display
   */
  virtual void info(const char* fmt, ...) = 0;
  /**
   * @brief       Pure virtual function to log warnings.
   * @param fmt   Format string (if additional arguments are passed) or message to display
   */
  virtual void warning(const char* fmt, ...) = 0;
  /**
   * @brief       Pure virtual function to log errors.
   * @param fmt   Format string (if additional arguments are passed) or message to display
   */
  virtual void error(const char* fmt, ...) = 0;
};

/**
 * @brief   A logger that outputs to stdout for info messages and stderr for warnings and errors.
 */
class DefaultLogger : public Logger
{
public:
  /**
   * @brief       Log info messages to stdout.
   * @param fmt   Format string (if additional arguments are passed) or message to display
   */
  virtual void info(const char* fmt, ...)
  {
    std::va_list argptr;
    va_start(argptr, fmt);
    std::vprintf(fmt, argptr);
    va_end(argptr);
  }

  /**
   * @brief       Log warnings to stderr.
   * @param fmt   Format string (if additional arguments are passed) or message to display
   */
  virtual void warning(const char* fmt, ...)
  {
    std::va_list argptr;
    va_start(argptr, fmt);
    std::vfprintf(stderr, fmt, argptr);
    va_end(argptr);
  }

  /**
   * @brief       Log errors to stderr.
   * @param fmt   Format string (if additional arguments are passed) or message to display
   */
  virtual void error(const char* fmt, ...)
  {
    std::va_list argptr;
    va_start(argptr, fmt);
    std::vfprintf(stderr, fmt, argptr);
    va_end(argptr);
  }
};

}  // namespace util

#endif  // TRAJOPT_UTIL_HPP
