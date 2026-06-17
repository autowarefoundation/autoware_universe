/**
 * Racer Dubins stadium path tracking — log one MPPI iteration (rollouts, costs, weights).
 *
 * Build: cmake --build build --target racer_dubins_stadium_mppi_rollout_analysis_example
 * Run:   ./build/examples/racer_dubins_stadium_mppi_rollout_analysis_example [output_prefix]
 * Plot:  python3 scripts/mppi/plot_mppi_rollout_analysis.py racer_dubins_stadium_mppi_rollout_analysis
 */
#include <mppi/utils/data_manager.hpp>

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/racer/racer_cost.cuh>
#include <mppi/cost_functions/racer/racer_cost_bridge.hpp>
#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
#include <mppi/feedback_controllers/path_tracker_feedback.cuh>
#include <mppi/path/path2d.hpp>
#include <mppi/path/path_reference_generator.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace
{
constexpr int kMppiHorizon = 80;
constexpr int kRefHorizon = kMppiHorizon;
constexpr float kDt = 0.1F;
constexpr int kNumRollouts = 32 * 1024;
constexpr float kTargetSpeed = 5.0F;
constexpr float kVMax = 5.0F;

constexpr float kStraightLength = 40.0F;
constexpr float kTurnRadius = 10.0F;
constexpr int kSamplesPerArc = 48;
constexpr float kInitArcLength = kStraightLength - 2.0F;
constexpr float kLambda = 1000.0F;

using DYN = RacerDubins;
using COST = RacerCost<kRefHorizon>;
using FB = PathTrackerFeedback<DYN, kMppiHorizon>;
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;
}  // namespace

int main(int argc, char** argv)
{
  std::string prefix = "racer_dubins_stadium_mppi_rollout_analysis";
  for (int a = 1; a < argc; ++a)
  {
    if (argv[a][0] != '-')
    {
      prefix = argv[a];
    }
  }

  const mppi::path::Path2D path = mppi::path::Path2D::stadium(kStraightLength, kTurnRadius, kSamplesPerArc);
  mppi::data::MppiDataManager<DYN> data_mgr;
  data_mgr.beginAnalysisRun(prefix, path);
  const mppi::data::RolloutOutputIndices kRolloutOutIdx(
      static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X),
      static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y),
      static_cast<int>(RacerDubinsParams::OutputIndex::YAW),
      static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_VEL_B_X));

  mppi::path::PathReferenceGenerator ref_gen(kDt);
  ref_gen.setSpeedCap(kVMax);
  ref_gen.setTargetSpeed(kTargetSpeed);

  DYN model;
  RacerDubinsParams dyn;
  dyn.wheel_base = 0.3f;
  model.setParams(dyn);
  std::array<float2, DYN::CONTROL_DIM> u_rng{};
  u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = { -1.0f, 1.0f };
  u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = { -1.0f, 1.0f };
  model.setControlRanges(u_rng);

  COST cost;
  cost.GPUSetup();

  RacerCostParams<kRefHorizon> cost_params;
  cost_params.desired_speed = kTargetSpeed;
  cost_params.wheel_base = dyn.wheel_base;
  cost_params.steer_angle_scale = dyn.steer_angle_scale;
  cost.setParams(cost_params);

  SAMPLER::SAMPLING_PARAMS_T sp{};
  sp.std_dev[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = 0.2F;
  sp.std_dev[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = 0.3F;
  sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
  SAMPLER sampler(sp);

  FB feedback(&model, kDt);
  Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
  Mppi controller(&model, &cost, &feedback, &sampler, kDt, 1, kLambda, 0.0F, kMppiHorizon, u_nom);
  {
    auto cp = controller.getParams();
    cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
    cp.cost_rollout_dim_ = dim3(32, 2, 1);
    cp.seed_ = 42U;
    controller.setParams(cp);
  }
  model.GPUSetup();

  DYN::state_array x = model.getZeroState();
  const mppi::path::Pose2D p0 = path.poseAt(kInitArcLength);
  x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)) = p0.x;
  x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)) = p0.y;
  x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)) = p0.yaw;
  x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)) = kTargetSpeed;

  const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(
      path, kInitArcLength, kRefHorizon, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)),
      x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)),
      x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)),
      x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)));
  mppi::cost::fillRacerCostFromPathReference<kRefHorizon>(cost, ref);
  const Mppi::state_trajectory goal_traj = mppi::feedback::goalTrajectoryFromPathReference<DYN, kMppiHorizon>(ref);
  feedback.updateReference(path, ref);
  feedback.applyFeedforwardToNominal(u_nom, x, goal_traj);
  controller.updateImportanceSampler(u_nom);

  std::cout << "Racer Dubins stadium MPPI rollout analysis  straight=" << kStraightLength << " m  R=" << kTurnRadius
            << " m  init_s=" << kInitArcLength << " m  v=" << kTargetSpeed << " m/s  rollouts=" << kNumRollouts
            << "  horizon=" << kMppiHorizon << "  lambda=" << kLambda << "\n";

  controller.computeControl(x, 1);
  cudaStreamSynchronize(controller.stream_);

  const auto& weights_eig = controller.getSampledCostSeq();
  const float baseline = static_cast<float>(controller.getBaselineCost());
  const float normalizer = static_cast<float>(controller.getNormalizerCost());

  const int num_logged = kNumRollouts;
  std::vector<float> raw_costs(num_logged, 0.0F);
  std::vector<float> normalized_weights(num_logged, 0.0F);
  for (int i = 0; i < num_logged; ++i)
  {
    const float w = weights_eig(i);
    normalized_weights[i] = (normalizer > 0.0F) ? w / normalizer : 0.0F;
    raw_costs[i] = (w > 0.0F) ? (baseline - kLambda * std::log(w)) : (baseline + 1.0e30F);
  }

  const Mppi::control_trajectory u_opt = controller.getControlSeq();
  data_mgr.dumpSingleIterationFromController(x, controller, model, sampler, kMppiHorizon, kLambda, kDt, u_opt,
                                             kRolloutOutIdx);

  const auto min_it = std::min_element(raw_costs.begin(), raw_costs.end());
  const auto max_it = std::max_element(raw_costs.begin(), raw_costs.end());
  const int best_idx = static_cast<int>(std::distance(raw_costs.begin(), min_it));
  float sum_w_sq = 0.0F;
  for (int i = 0; i < num_logged; ++i)
  {
    sum_w_sq += normalized_weights[i] * normalized_weights[i];
  }
  const float ess = (sum_w_sq > 0.0F) ? (1.0F / sum_w_sq) : 0.0F;

  std::cout << "One MPPI iteration done.\n";
  std::cout << "  baseline=" << baseline << "  normalizer=" << normalizer << "  ESS=" << ess << "/" << num_logged
            << " (" << (100.0F * ess / static_cast<float>(num_logged)) << "%)\n";
  std::cout << "  raw_cost spread [" << *min_it << ", " << *max_it << "]  delta=" << (*max_it - *min_it) << "\n";
  std::cout << "  best rollout index=" << best_idx << "  raw_cost=" << raw_costs[best_idx]
            << "  weight=" << normalized_weights[best_idx] << "\n";
  if (ess > 0.9F * num_logged)
  {
    std::cout << "  warning: ESS ~ N -> weights nearly uniform (lambda=" << kLambda
              << " may be too large relative to cost spread).\n";
  }
  if ((*max_it - *min_it) < 100.0F)
  {
    std::cout << "  warning: raw cost spread < 100 -> rollouts look similar in cost (check RacerCost track term).\n";
  }
  std::cout << "Wrote " << prefix << "_meta.csv, _costs.csv, _combined.csv, _rollouts_xy.csv (top "
            << mppi::data::kDefaultTopRollouts << " trajectories)\n";
  std::cout << "Plot: python3 scripts/mppi/plot_mppi_rollout_analysis.py " << prefix << "\n";
  std::cout << "Retune: python3 scripts/mppi/plot_mppi_lambda_retune.py " << prefix << "\n";
  cost.freeCudaMem();
  return 0;
}
