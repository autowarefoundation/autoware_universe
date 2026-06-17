/**
 * Dubins stadium path tracking - log one MPPI iteration at the corner-entry bend.
 *
 * Build: cmake --build build --target dubins_stadium_mppi_rollout_analysis_example
 * Run:   ./build/examples/dubins_stadium_mppi_rollout_analysis_example [output_prefix]
 * Plot:  python3 scripts/mppi/plot_mppi_rollout_analysis.py dubins_stadium_mppi_rollout_analysis
 *
 * Same single-iteration structure as dubins_circle_mppi_rollout_analysis_example, but on a
 * stadium track. The vehicle is placed 2 m before the right-turn corner entry (s = straight_length
 * is the bend) at a steady 2.5 m/s, so the 5-second MPPI horizon spans the full curvature step
 * and the rollouts visualize what MPPI actually sees crossing the bend.
 *
 * The standard MPPI pipeline runs once on the GPU. Per-rollout costs/weights come straight from
 * the controller's public API:
 *   weights[i] = controller.getSampledCostSeq()[i]    (= exp(-(c_i - base)/lambda) after the in-
 *                                                       place norm-exp kernel)
 *   baseline   = controller.getBaselineCost()
 *   normalizer = controller.getNormalizerCost()
 *   raw_cost[i] = baseline - lambda * log(weights[i])
 * No host-side cost replay (the GPU already did it correctly, including the IS likelihood term).
 * We do still pull noise samples and host-replay the dynamics for each rollout, only because
 * output_d_ is not reliably accessible from the host in this codebase and we need (x, y) for plot.
 */
#include <mppi/utils/data_manager.hpp>

#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/cost_functions/path_tracking/path_tracking_cost.cuh>
#include <mppi/dynamics/dubins_bicycle/dubins_bicycle.cuh>
#include <mppi/feedback_controllers/DDP/ddp.cuh>
#include <mppi/path/path_projection.hpp>
#include <mppi/path/path_reference_generator.hpp>
#include <mppi/path/path_tracking_bridge.hpp>
#include <mppi/path/path2d.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <iterator>
#include <numeric>
#include <string>
#include <vector>

namespace
{
constexpr int kMppiHorizon = 80;
constexpr int kRefHorizon = kMppiHorizon + 8;
constexpr float kDt = 0.1F;
constexpr int kNumRollouts = 4*1024;
constexpr float kTargetSpeed = 2.5F;
constexpr float kVMax = 3.0F;

constexpr float kStraightLength = 40.0F;
constexpr float kTurnRadius = 10.0F;
constexpr int kSamplesPerArc = 48;

// s = kStraightLength is exactly the right-turn corner entry of the stadium. Start 2 m before it
// so MPPI sees the curvature step around step 8 of the horizon - the bend lives in the middle of
// the prediction window where the rollouts have spread enough to be informative.
constexpr float kInitArcLength = kStraightLength - 2.0F;
constexpr float kInitLateralOffset = 0.1F;

// Match the closed-loop stadium tracking example so this analysis reflects the same controller.
constexpr float kNoiseStdAccel = 0.15F;
constexpr float kNoiseStdSteer = 0.12F;
constexpr float kNomLatSteerGain = 0.0F;
constexpr float kNomHeadingSteerGain = 0.0F;
constexpr float kLambda = 30.0F;

using DYN = DubinsBicycle;
using COST = PathTrackingCost<kRefHorizon>;
using FB = DDPFeedback<DYN, kMppiHorizon>;
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

}  // namespace

int main(int argc, char** argv)
{
  std::string prefix = "dubins_stadium_mppi_rollout_analysis";
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
      static_cast<int>(DubinsBicycleParams::OutputIndex::POS_X),
      static_cast<int>(DubinsBicycleParams::OutputIndex::POS_Y),
      static_cast<int>(DubinsBicycleParams::OutputIndex::YAW),
      static_cast<int>(DubinsBicycleParams::OutputIndex::VEL_X));

  mppi::path::PathReferenceGenerator ref_gen(kDt);
  ref_gen.setSpeedCap(kVMax);

  DYN model;
  DubinsBicycleParams dyn;
  model.setParams(dyn);
  std::array<float2, DYN::CONTROL_DIM> u_rng{};
  u_rng[static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL)] = { dyn.min_accel, dyn.max_accel };
  u_rng[static_cast<int>(DubinsBicycleParams::ControlIndex::STEER)] = { -dyn.max_steer_angle, dyn.max_steer_angle };
  model.setControlRanges(u_rng);

  COST cost;
  PathTrackingCostParams<kRefHorizon> cost_params;
  // Order: w_pos, w_heading_so2, w_vel, w_lat_accel, w_lat_jerk, w_steer_dot, w_accel, w_steer.
  // These match the closed-loop examples/dubins/dubins_stadium_path_tracking_example defaults.
  mppi::path::fillPathTrackingCostWeights<kRefHorizon>(cost_params, 2.0F, 1.0F, 5.0F, 5.0F, 20.0F, 50.0F, 5.0F, 0.5F);
  mppi::path::fillPathTrackingBicycleGeometry<kRefHorizon>(cost_params, dyn);
  cost.setParams(cost_params);

  SAMPLER::SAMPLING_PARAMS_T sp{};
  sp.std_dev[static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL)] = kNoiseStdAccel;
  sp.std_dev[static_cast<int>(DubinsBicycleParams::ControlIndex::STEER)] = kNoiseStdSteer;
  sp.control_cost_coeff[0] = cost_params.control_cost_coeff[0];
  sp.control_cost_coeff[1] = cost_params.control_cost_coeff[1];
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
  // PathTrackingCost has a large device params blob; the combined rollout kernel mis-aligns
  // shared memory (illegal memory access). Use split kernels for the optimization itself.
  controller.setKernelChoice(kernelType::USE_SPLIT_KERNELS);
  model.GPUSetup();
  cost.GPUSetup();

  DYN::state_array x = model.getZeroState();
  const mppi::path::Pose2D p0 = path.poseAt(kInitArcLength);
  float init_x = p0.x;
  float init_y = p0.y;
  mppi::path::applyInitialLateralOffset(path, kInitArcLength, kInitLateralOffset, init_x, init_y);
  x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_X)) = init_x;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_Y)) = init_y;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::YAW)) = p0.yaw;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::VEL_X)) = kTargetSpeed;

  // ===========================================================================
  // A sim-loop is:
  //   (1) find the current reference from the current state
  //   (2) find a control using MPPI
  //   (3) update the state
  //   (4) repeat
  //
  // This file is NOT a sim-loop. It contains a fragment of one would-be
  // iteration of (1) and (2), then stops. Step (3) and the loop in (4) are
  // absent on purpose: this file exists to dump a single MPPI optimization
  // for plotting/diagnostics, not to drive the vehicle along the path. The
  // closed-loop equivalent lives in
  // closed-loop equivalent lives in examples/dubins/dubins_stadium_path_tracking_example.cu.
  // ===========================================================================

  // (1) Reference from current state.
  //     NOTE: this is a degenerate version of "from the current state" - the
  //     arc-length anchor is the constant kInitArcLength rather than a
  //     projectPoseOntoPath(x) result. That's fine for a single iteration
  //     because x is itself constructed at kInitArcLength just above, but it
  //     would be wrong inside a real sim-loop where x has moved.
  const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(path, kInitArcLength, kRefHorizon);
  mppi::path::fillCostFromPathReference<kRefHorizon>(cost_params, ref, &path, &dyn);
  cost.setParams(cost_params);
  mppi::path::fillNominalControlFromReference(u_nom, x, ref, dyn, kDt, &path, kNomLatSteerGain, kNomHeadingSteerGain);
  controller.updateImportanceSampler(u_nom);

  std::cout << "Dubins stadium MPPI rollout analysis  straight=" << kStraightLength << " m  R=" << kTurnRadius
            << " m  init_s=" << kInitArcLength << " m  v=" << kTargetSpeed << " m/s  rollouts=" << kNumRollouts
            << "  horizon=" << kMppiHorizon << "  lambda=" << kLambda << "\n";

  // (2) Control from MPPI.
  //     Run a single MPPI optimization iteration. The split rollout kernel
  //     computes cost(rollout_i) on the GPU; the optimal control sequence
  //     ends up in controller.getControlSeq() (read much later, at line ~248).
  controller.computeControl(x, 1);

  // (3) State update: NOT PRESENT.
  //     A sim-loop iteration would do (matching the for-loop body in
  //     examples/dubins/dubins_stadium_path_tracking_example.cu):
  //       Mppi::control_trajectory u_opt = controller.getControlSeq();
  //       model.enforceConstraints(x, u_opt.col(0));
  //       model.step(x, x_next, xdot, u_opt.col(0), y, k, kDt);
  //       x = x_next;
  //       controller.slideControlSequence(1);
  //     None of that happens here.
  //
  // (4) Repeat: NOT PRESENT.
  //     There is no enclosing loop. Everything below is analysis of the one
  //     MPPI optimization above, not subsequent sim-loop iterations.
  // ===========================================================================

  // ---------- ANALYSIS-ONLY (none of this appears in a sim-loop) ----------
  // After computeControl, trajectory_costs_d_ has been transformed in place by the norm-exp
  // kernel, so getSampledCostSeq() returns the unnormalized weights w_i = exp(-(c_i-base)/lam).
  // Recover raw costs as c_i = base - lam * log(w_i). The GPU already included the IS likelihood
  // term and used the right cost weights, so this is the controller's true view of the rollouts.
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
  const int best_idx = static_cast<int>(std::distance(raw_costs.begin(), min_it));
  // ESS = 1 / sum(w_norm^2): how many rollouts effectively contribute. Healthy = 5-20% of N.
  float sum_w_sq = 0.0F;
  for (int i = 0; i < num_logged; ++i)
  {
    sum_w_sq += normalized_weights[i] * normalized_weights[i];
  }
  const float ess = (sum_w_sq > 0.0F) ? (1.0F / sum_w_sq) : 0.0F;

  std::cout << "One MPPI iteration done.\n";
  std::cout << "  baseline=" << baseline << "  normalizer=" << normalizer << "  ESS=" << ess << "/" << num_logged
            << " (" << (100.0F * ess / static_cast<float>(num_logged)) << "%)\n";
  std::cout << "  best rollout index=" << best_idx << "  raw_cost=" << raw_costs[best_idx]
            << "  weight=" << normalized_weights[best_idx] << "\n";
  if (ess > 0.9F * num_logged)
  {
    std::cout << "  warning: ESS ~ N -> weights nearly uniform -> MPPI is averaging out the noise\n"
                 "           (lambda=" << kLambda << " is too large relative to the cost spread). "
                 "Try --lambda smaller.\n";
  }
  std::cout << "Wrote " << prefix << "_meta.csv, _costs.csv, _combined.csv, _rollouts_xy.csv (top "
            << mppi::data::kDefaultTopRollouts << " trajectories)\n";
  std::cout << "Plot: python3 scripts/mppi/plot_mppi_rollout_analysis.py " << prefix << "\n";
  std::cout << "Retune: python3 scripts/mppi/plot_mppi_lambda_retune.py " << prefix << "\n";
  return 0;
}
