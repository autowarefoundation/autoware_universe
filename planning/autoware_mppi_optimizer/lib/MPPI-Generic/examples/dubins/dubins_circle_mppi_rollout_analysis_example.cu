/**
 * Dubins circle path tracking — log one MPPI iteration (rollouts, costs, weights).
 *
 * Build: cmake --build build --target dubins_circle_mppi_rollout_analysis_example
 * Run:   ./build/examples/dubins_circle_mppi_rollout_analysis_example [output_prefix]
 * Plot:  python3 scripts/mppi/plot_mppi_rollout_analysis.py dubins_circle_mppi_rollout_analysis
 *
 * Uses the standard MPPI pipeline (no custom kernels):
 *   computeControl  →  calculateSampledStateTrajectories  →  getSampledOutputTrajectories
 *                                                          → getSampledCostTrajectories
 * Per-rollout cost is the sum of the visualize-cost kernel's per-timestep cost,
 * and weights are exp(-(cost-baseline)/lambda) / normalizer.
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
#include <iterator>
#include <numeric>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr int kMppiHorizon = 80;
constexpr int kRefHorizon = kMppiHorizon + 8;
constexpr float kDt = 0.1F;
constexpr int kNumRollouts = 4096;
constexpr float kTargetSpeed = 2.5F;
constexpr float kVMax = 3.0F;

constexpr float kCircleRadius = 20.0F;
constexpr float kCircleCenterX = 0.0F;
constexpr float kCircleCenterY = 0.0F;
constexpr float kCircleTheta0 = 0.0F;
constexpr int kCirclePlotSamples = 512;

constexpr float kInitLateralOffset = 0.1F;

using DYN = DubinsBicycle;
using COST = PathTrackingCost<kRefHorizon>;
using FB = DDPFeedback<DYN, kMppiHorizon>;
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

}  // namespace

int main(int argc, char** argv)
{
  std::string prefix = "dubins_circle_mppi_rollout_analysis";
  for (int a = 1; a < argc; ++a)
  {
    if (argv[a][0] != '-')
    {
      prefix = argv[a];
    }
  }

  const mppi::path::Path2D path =
      mppi::path::Path2D::circle(kCircleCenterX, kCircleCenterY, kCircleRadius, kCircleTheta0, kCirclePlotSamples);
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
  mppi::path::fillPathTrackingCostWeights<kRefHorizon>(cost_params, 5.0F, 1.0F, 5.0F, 5.0F, 5.0F);
  mppi::path::fillPathTrackingBicycleGeometry<kRefHorizon>(cost_params, dyn);
  cost.setParams(cost_params);

  SAMPLER::SAMPLING_PARAMS_T sp{};
  sp.std_dev[static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL)] = 0.15F;
  sp.std_dev[static_cast<int>(DubinsBicycleParams::ControlIndex::STEER)] = 0.04F;
  sp.control_cost_coeff[0] = cost_params.control_cost_coeff[0];
  sp.control_cost_coeff[1] = cost_params.control_cost_coeff[1];
  sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
  SAMPLER sampler(sp);

  FB feedback(&model, kDt);
  Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
  const float kLambda = 15000.0F;
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
  const mppi::path::Pose2D p0 = path.poseAt(0.0F);
  float init_x = p0.x;
  float init_y = p0.y;
  mppi::path::applyInitialLateralOffset(path, 0.0F, kInitLateralOffset, init_x, init_y);
  x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_X)) = init_x;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_Y)) = init_y;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::YAW)) = p0.yaw;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::VEL_X)) = kTargetSpeed;

  const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(path, 0.0F, kRefHorizon);
  mppi::path::fillCostFromPathReference<kRefHorizon>(cost_params, ref, &path, &dyn);
  cost.setParams(cost_params);
  mppi::path::fillNominalControlFromReference(u_nom, x, ref, dyn, kDt, &path);
  controller.updateImportanceSampler(u_nom);

  std::cout << "Dubins circle MPPI rollout analysis  R=" << kCircleRadius << " m  v=" << kTargetSpeed
            << " m/s  rollouts=" << kNumRollouts << "  horizon=" << kMppiHorizon << "  lambda=" << kLambda << "\n";

  // Run a single MPPI optimization iteration (the standard public path). The split rollout kernel
  // computes cost(rollout_i) on the GPU; the optimal control sequence ends up in
  // controller.getControlSeq().
  controller.computeControl(x, 1);

  // We want to visualize the actual rollouts that the GPU sampled. The library's GPU output
  // buffer (output_d_) is not reliably accessible from the host in this codebase (the split
  // dynamics kernel writes to it but the contents do not survive the kernel boundary in any
  // way the public/visualization API can read). The reliably-accessible thing the GPU does
  // produce is the control noise samples, in the sampling distribution's device buffer. We pull
  // those samples off the GPU and replay each rollout through the dynamics model on the host
  // to recover the per-rollout (x, y, yaw, vel) trajectories.
  const int H = kMppiHorizon;
  const int kNumLogged = kNumRollouts;
  std::vector<float> host_controls(static_cast<size_t>(kNumLogged) * H * DYN::CONTROL_DIM);
  {
    float* device_controls = sampler.getControlSample(0, 0, 0);
    HANDLE_ERROR(cudaMemcpy(host_controls.data(), device_controls, host_controls.size() * sizeof(float),
                            cudaMemcpyDeviceToHost));
  }

  using OutputTraj = Mppi::output_trajectory;
  std::vector<OutputTraj> sampled_outputs(kNumLogged, OutputTraj::Zero());
  std::vector<float> raw_costs(kNumLogged, 0.0F);
  {
    typename DYN::state_array x_local;
    typename DYN::state_array x_next = model.getZeroState();
    typename DYN::state_array xdot = model.getZeroState();
    typename DYN::output_array y_t = DYN::output_array::Zero();
    typename DYN::control_array u_local = DYN::control_array::Zero();
    for (int i = 0; i < kNumLogged; ++i)
    {
      x_local = x;
      float cost_i = 0.0F;
      for (int t = 0; t < H; ++t)
      {
        const float* src = host_controls.data() + (i * H + t) * DYN::CONTROL_DIM;
        for (int d = 0; d < DYN::CONTROL_DIM; ++d)
        {
          u_local(d) = src[d];
        }
        model.enforceConstraints(x_local, u_local);
        model.step(x_local, x_next, xdot, u_local, y_t, static_cast<float>(t), kDt);
        sampled_outputs[i].col(t) = y_t;
        cost_i += cost.computeRunningCost(y_t, u_local, t, nullptr);
        x_local = x_next;
      }
      raw_costs[i] = cost_i;
    }
  }
  const int num_logged = kNumLogged;

  // Standard MPPI weighting: w_i = exp(-(c_i - baseline) / lambda); normalizer = sum_i w_i.
  const float baseline = *std::min_element(raw_costs.begin(), raw_costs.end());
  std::vector<float> unnormalized_importance(num_logged, 0.0F);
  for (int i = 0; i < num_logged; ++i)
  {
    unnormalized_importance[i] = std::exp(-(raw_costs[i] - baseline) / kLambda);
  }
  const float normalizer = std::accumulate(unnormalized_importance.begin(), unnormalized_importance.end(), 0.0F);
  std::vector<float> normalized_weights(num_logged, 0.0F);
  for (int i = 0; i < num_logged; ++i)
  {
    normalized_weights[i] = (normalizer > 0.0F) ? unnormalized_importance[i] / normalizer : 0.0F;
  }

  const Mppi::control_trajectory u_opt = controller.getControlSeq();
  data_mgr.template dumpSingleIterationHostReplay<Mppi::output_trajectory>(
      x, model, sampler, kMppiHorizon, kLambda, kDt, num_logged, u_opt, raw_costs, baseline, kRolloutOutIdx);

  const auto min_it = std::min_element(raw_costs.begin(), raw_costs.end());
  const int best_idx = static_cast<int>(std::distance(raw_costs.begin(), min_it));
  std::cout << "One MPPI iteration done.\n";
  std::cout << "  baseline=" << baseline << "  normalizer=" << normalizer << "\n";
  std::cout << "  best rollout index=" << best_idx << "  raw_cost=" << raw_costs[best_idx]
            << "  weight=" << normalized_weights[best_idx] << "\n";
  std::cout << "  (slot 0 is the optimal/combined trajectory; slots i>=1 are rollout i)\n";
  std::cout << "Wrote " << prefix << "_meta.csv, _costs.csv, _combined.csv, _rollouts_xy.csv (top "
            << mppi::data::kDefaultTopRollouts << " trajectories)\n";
  std::cout << "Plot: python3 scripts/mppi/plot_mppi_rollout_analysis.py " << prefix << "\n";
  std::cout << "Retune: python3 scripts/mppi/plot_mppi_lambda_retune.py " << prefix << "\n";
  return 0;
}
