/**
 * Dubins bicycle + MPPI path tracking on an open straight-line path.
 *
 *   Path2D  →  projectPoseOntoPath  →  PathReferenceGenerator (dt=0.1 s)  →  PathTrackingCost  →  MPPI
 *
 * Build: cmake --build build --target dubins_path_tracking_example
 * Run:   ./build/examples/dubins_path_tracking_example [log.csv]
 * Plot:  python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py dubins_path_tracking_log.csv
 *
 * For a circular track, use examples/dubins/dubins_circle_path_tracking_example.cu instead.
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
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

namespace
{
constexpr int kMppiHorizon = 80;
constexpr int kRefHorizon = kMppiHorizon + 8;
constexpr int kSimSteps = 800;
constexpr float kDt = 0.1F;
constexpr int kNumRollouts = 32*1024;
constexpr float kVMax = 3.0F;

using DYN = DubinsBicycle;
using COST = PathTrackingCost<kRefHorizon>;
using FB = DDPFeedback<DYN, kMppiHorizon>;
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

const mppi::data::RolloutOutputIndices kRolloutOutIdx{
    static_cast<int>(DubinsBicycleParams::OutputIndex::POS_X),
    static_cast<int>(DubinsBicycleParams::OutputIndex::POS_Y),
    static_cast<int>(DubinsBicycleParams::OutputIndex::YAW),
    static_cast<int>(DubinsBicycleParams::OutputIndex::VEL_X),
};

void clipControl(const DubinsBicycleParams& p, DYN::control_array& u)
{
  u(static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL)) =
      std::max(p.min_accel, std::min(u(static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL)), p.max_accel));
  u(static_cast<int>(DubinsBicycleParams::ControlIndex::STEER)) =
      std::max(-p.max_steer_angle, std::min(u(static_cast<int>(DubinsBicycleParams::ControlIndex::STEER)), p.max_steer_angle));
}

}  // namespace

int main(int argc, char** argv)
{
  std::string log_path = "dubins_path_tracking_log.csv";
  for (int a = 1; a < argc; ++a)
  {
    if (argv[a][0] != '-')
    {
      log_path = argv[a];
    }
  }

  const mppi::path::Path2D path = mppi::path::Path2D::straightLine(0.0F, 0.0F, 100.0F, 0.0F, 64);

  mppi::data::MppiDataManager<DYN> data_mgr;
  if (!data_mgr.beginRun(log_path, path, mppi::data::PathTrackingLogSchema::kRefV))
  {
    return 1;
  }
  data_mgr.setRoadBoundaryLimits(0.8F, 0.8F);

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
  mppi::path::fillPathTrackingCostWeights<kRefHorizon>(cost_params, 10.0F, 1.0F, 5.0F, 1.0F, 0.05F);
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
  // See examples/dubins/dubins_circle_path_tracking_example.cu for the lambda tuning rationale; the cost weights
  // and per-step magnitudes are the same here, so the same temperature lands in the healthy band.
  constexpr float kMppiLambda = 100.0F;
  Mppi controller(&model, &cost, &feedback, &sampler, kDt, 1, kMppiLambda, 0.0F, kMppiHorizon, u_nom);
  {
    auto cp = controller.getParams();
    cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
    cp.cost_rollout_dim_ = dim3(32, 2, 1);
    cp.seed_ = 42U;
    controller.setParams(cp);
  }
  controller.setKernelChoice(kernelType::USE_SPLIT_KERNELS);
  model.GPUSetup();
  cost.GPUSetup();

  const std::vector<mppi::path::PathReferenceSample> ref_init = ref_gen.generate(path, 0.0F, kRefHorizon);
  mppi::path::fillCostFromPathReference<kRefHorizon>(cost_params, ref_init, &path, &dyn);
  cost.setParams(cost_params);

  DYN::state_array x = model.getZeroState();
  const mppi::path::Pose2D p0 = path.poseAt(0.0F);
  float init_x = p0.x;
  float init_y = p0.y;
  mppi::path::applyInitialLateralOffset(path, 0.0F, 0.1F, init_x, init_y);
  x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_X)) = init_x;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_Y)) = init_y;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::YAW)) = p0.yaw;
  x(static_cast<int>(DubinsBicycleParams::StateIndex::VEL_X)) = 1.5F;

  mppi::path::fillNominalControlFromReference(u_nom, x, ref_init, dyn, kDt, &path);
  controller.updateImportanceSampler(u_nom);

  DYN::state_array x_next = model.getZeroState();
  DYN::state_array xdot = model.getZeroState();
  DYN::output_array y = DYN::output_array::Zero();

  float s_prog = 0.0F;
  float max_lat = 0.0F;

  std::cout << "Dubins straight path tracking  path_length=" << path.length() << " m\n";
  std::cout << "Logging to " << log_path << "\n";

  const auto t0 = std::chrono::steady_clock::now();
  for (int k = 0; k < kSimSteps; ++k)
  {
    const float sim_time = static_cast<float>(k) * kDt;

    const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(
        path, x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_X)),
        x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_Y)), s_prog);
    s_prog = proj.arc_length_s;
    max_lat = std::max(max_lat, std::fabs(proj.signed_lateral_error));

    if (s_prog >= path.length() - 0.5F &&
        x(static_cast<int>(DubinsBicycleParams::StateIndex::POS_X)) >= path.length() - 0.5F)
    {
      break;
    }

    const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(path, s_prog, kRefHorizon);
    mppi::path::fillCostFromPathReference<kRefHorizon>(cost_params, ref, &path, &dyn);
    cost.setParams(cost_params);

    mppi::path::fillNominalControlFromReference(u_nom, x, ref, dyn, kDt, &path);
    controller.updateImportanceSampler(u_nom);
    controller.computeControl(x, 1);
    cudaStreamSynchronize(controller.stream_);
    controller.calculateSampledStateTrajectories();
    const Mppi::control_trajectory u_opt_traj = controller.getControlSeq();
    data_mgr.dumpRolloutSnapshot(k, sim_time, x, controller, model, sampler, kMppiHorizon, kMppiLambda, kDt,
                                 u_opt_traj, kRolloutOutIdx);
    const DYN::control_array u_apply = u_opt_traj.col(0);
    DYN::control_array u = u_apply;
    clipControl(dyn, u);
    const float baseline = static_cast<float>(controller.getBaselineCost());
    model.enforceConstraints(x, u);
    model.step(x, x_next, xdot, u, y, static_cast<float>(k), kDt);

    const mppi::path::PathReferenceSample& r0 = ref.front();
    const float t_end = static_cast<float>(k + 1) * kDt;
    mppi::data::PathTrackingStepLog step_log{};
    step_log.t = t_end;
    step_log.pos_x = x_next(static_cast<int>(DubinsBicycleParams::StateIndex::POS_X));
    step_log.pos_y = x_next(static_cast<int>(DubinsBicycleParams::StateIndex::POS_Y));
    step_log.yaw = x_next(static_cast<int>(DubinsBicycleParams::StateIndex::YAW));
    step_log.vel_x = x_next(static_cast<int>(DubinsBicycleParams::StateIndex::VEL_X));
    step_log.steer_angle = x_next(static_cast<int>(DubinsBicycleParams::StateIndex::STEER_ANGLE));
    step_log.u_accel = u(static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL));
    step_log.u_steer = u(static_cast<int>(DubinsBicycleParams::ControlIndex::STEER));
    step_log.nom_u_accel = u_apply(static_cast<int>(DubinsBicycleParams::ControlIndex::ACCEL));
    step_log.nom_u_steer = u_apply(static_cast<int>(DubinsBicycleParams::ControlIndex::STEER));
    step_log.ref_x = r0.x;
    step_log.ref_y = r0.y;
    step_log.ref_yaw = r0.yaw;
    step_log.ref_v = r0.v;
    step_log.arc_s = s_prog;
    step_log.lat_err = proj.signed_lateral_error;
    step_log.baseline = baseline;
    data_mgr.logPathTrackingStep(step_log);

    x = x_next;
    controller.slideControlSequence(1);

    if (k % 50 == 0)
    {
      std::cout << "t=" << t_end << "  s=" << s_prog << "  lat=" << proj.signed_lateral_error
                << "  v=" << x(static_cast<int>(DubinsBicycleParams::StateIndex::VEL_X)) << "\n";
    }
  }

  data_mgr.close();
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::cout << "Done. max |lateral|=" << max_lat << " m  elapsed=" << elapsed << " s\n";
  std::cout << "Wrote " << log_path << " and rollout snapshots under " << data_mgr.rolloutDirectory() << "\n";
  std::cout << "Plot: python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py " << log_path << "\n";
  return 0;
}
