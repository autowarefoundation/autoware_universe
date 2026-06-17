#ifndef TRAJOPT_RESULT_HPP
#define TRAJOPT_RESULT_HPP

#include <Eigen/Dense>

/**
 * @tparam  Dynamics
 * @brief   Represent the return type of an optimization of subject to Dynamics.
 */
template <class Dynamics>
struct OptimizerResult
{
  using Scalar = typename Dynamics::Scalar;

  OptimizerResult() = default;
  OptimizerResult(const OptimizerResult& other) = default;
  OptimizerResult(OptimizerResult&& other) = default;
  ~OptimizerResult() = default;
  OptimizerResult& operator=(const OptimizerResult& other) = default;
  OptimizerResult& operator=(OptimizerResult&& other) = default;

  /**
   * @brief       Constructor to use if the optimizer does not produce feedback or feedforward gains.
   * @param iter  Number of iterations performed (I)
   * @param ts    Number of time steps (H)
   * @param tc    Total cost of the final trajectory returned by the optimizer
   * @param c     Cost for each time step in each iteration (I x H)
   * @param x     Optimized state trajectory
   * @param u     Optimized control trajectory
   */
  OptimizerResult(int iter, int ts, Scalar tc,
                  const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>& c,
                  const Eigen::Ref<const typename Dynamics::StateTrajectory>& x,
                  const Eigen::Ref<const typename Dynamics::ControlTrajectory>& u)
    : iterations(iter), timesteps(ts), total_cost(tc), cost(c), state_trajectory(x), control_trajectory(u)
  {
  }

  /**
   * @brief       Constructor to use if the optimizer produces feedback or feedforward gains.
   * @param iter  Number of iterations performed (I)
   * @param ts    Number of time steps (H)
   * @param tc    Total cost of the final trajectory returned by the optimizer
   * @param c     Cost for each time step in each iteration (I x H)
   * @param x     Optimized state trajectory
   * @param u     Optimized control trajectory
   * @param fb    Feedback gains for every time step in the optimized trajectory
   * @param ff    Feedforward gains for every time step in the optimized trajectory
   */
  OptimizerResult(int iter, int ts, Scalar tc,
                  const Eigen::Ref<const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>>& c,
                  const Eigen::Ref<const typename Dynamics::StateTrajectory>& x,
                  const Eigen::Ref<const typename Dynamics::ControlTrajectory>& u,
                  const typename Dynamics::FeedbackGainTrajectory& fb,
                  const Eigen::Ref<const typename Dynamics::FeedforwardGainTrajectory>& ff)
    : iterations(iter)
    , timesteps(ts)
    , total_cost(tc)
    , cost(c)
    , state_trajectory(x)
    , control_trajectory(u)
    , feedback_gain(fb)
    , feedforward_gain(ff)
  {
  }

  int iterations;     ///< # of optimizing iterations (I)
  int timesteps;      ///< # of timesteps (H)
  Scalar total_cost;  ///< Total cost over the trajectory returned by the optimizer after I iterations
  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cost;  ///< Cost for each time step cost per iteration (I x H)
  typename Dynamics::StateTrajectory state_trajectory;         ///< State trajectory created by optimizer
  typename Dynamics::ControlTrajectory control_trajectory;     ///< Control trajectory created by optimizer
  typename Dynamics::FeedbackGainTrajectory feedback_gain;     ///< Feedback gain at each time step
  typename Dynamics::FeedforwardGainTrajectory feedforward_gain;  ///< Feedforward gain at each time step
};

#endif  // TRAJOPT_RESULT_HPP
