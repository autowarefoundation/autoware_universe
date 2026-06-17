/**
 * @file racer_dubins_stadium_path_tracking_example.cu
 * @brief Example of MPPI-based path tracking and obstacle avoidance for a Racer Dubins model on a stadium track.
 *
 * Build: cmake --build build --target racer_dubins_stadium_path_tracking_example
 * Run:   ./build/examples/racer/racer_dubins_stadium_path_tracking_example [seed] [log.csv]
 * Plot:  python3 scripts/mppi/plot_racer_dubins_temporal_mppi.py racer_dubins_stadium_path_tracking_log.csv
 */

#include <mppi/utils/data_manager.hpp>

#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
#include <mppi/cost_functions/racer/racer_cost.cuh>
#include <mppi/cost_functions/racer/racer_cost_bridge.hpp>
#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/feedback_controllers/zero_feedback.cuh>
#include <mppi/path/path_projection.hpp>
#include <mppi/path/path_reference_generator.hpp>
#include <mppi/path/path2d.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>

#include <mppi/path/drivable_area.hpp>
#include <mppi/viz/path_tracking_viewer.hpp>
#include <mppi/viz/rollout_viz_gpu.cuh>
#include <mppi/utils/step_timing.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <random>
#include <string>

namespace
{
  // --- Simulation Parameters ---
  constexpr int kMppiHorizon = 80;
  constexpr int kRefHorizon = kMppiHorizon;
  constexpr float kDt = 0.1F;
  constexpr int kNumRollouts = 32*1024;
  constexpr float kTargetSpeed = 3.0F;
  constexpr float kVMax = 5.0F;
  constexpr size_t kSimLaps = 5;
  
  constexpr float kStraightLength = 40.0F;
  constexpr float kTurnRadius = 10.0F;
  constexpr int kSamplesPerArc = 48;
  
  constexpr float kInitArcLength = kStraightLength - 2.0F;
  
  constexpr float kLambda = 1500.0F;
  constexpr float kVideoFps = 30.0F;

  // --- MPPI Controller Setup ---
  using DYN = RacerDubins;
  using COST = RacerCost<kRefHorizon>;
  using FB = ZeroFeedback<DYN, kMppiHorizon>;
  using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
  using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

  const mppi::data::RolloutOutputIndices kRolloutOutIdx{
      static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X),
      static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y),
      static_cast<int>(RacerDubinsParams::OutputIndex::YAW),
      static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_VEL_B_X),
  };

  /**
   * @brief Calculates simulation steps needed for a given number of laps.
   */
  int simStepsForLaps(const mppi::path::Path2D& path, const float laps)
  {
    const float lap_time = path.length() / kVMax;
    return static_cast<int>(std::ceil(laps * lap_time / kDt));
  }
}  // namespace

/**
 * @brief Main simulation loop for the Racer Dubins path tracking example.
 *
 * @param argc Argument count.
 * @param argv Argument vector (optional: seed for RNG).
 * @return int Exit code.
 */
int main(int argc, char** argv)
{
    // 1. Initialize random seed for reproducibility
    unsigned int seed = 42;
    if (argc > 1) {
        seed = std::stoul(argv[1]);
    }
    std::cout << "Using random seed: " << seed << std::endl;
    std::string log_path = "racer_dubins_stadium_path_tracking_log.csv";
    if (argc > 2) {
        log_path = argv[2];
    }

    /* Environment */
    // 2. Generate track path (stadium shape)
    const mppi::path::Path2D path = mppi::path::Path2D::stadium(kStraightLength, kTurnRadius, kSamplesPerArc);

    constexpr float kRoadHalfWidth = 0.8F;

    mppi::data::MppiDataManager<DYN> data_mgr;
    if (!data_mgr.beginRun(log_path, path))
    {
      return 1;
    }
    data_mgr.setRoadBoundaryLimits(kRoadHalfWidth, kRoadHalfWidth);

    // Parked cars along both shoulders (near road boundary), alternating L/R with gaps for weaving.
    const std::vector<mppi::cost::ParkedCarObstacle> parked_cars =
        mppi::cost::generateParkedCarsAlongRoad(path, kRoadHalfWidth, seed);
    std::cout << "Parked cars along track: " << parked_cars.size() << "\n";

    mppi::path::PathReferenceGenerator ref_gen(kDt);
    ref_gen.setSpeedCap(kVMax);
    ref_gen.setTargetSpeed(kTargetSpeed);

    const size_t num_sim_steps = simStepsForLaps(path, kSimLaps);

    float arcLength = kInitArcLength;

    /* Model parameters */
    // 5. Setup model and sampling distributions
    DYN model;
    RacerDubinsParams dyn;
    dyn.wheel_base = 0.3f;
    // Engine tuned for ~5.5 m/s steady state at full throttle (see RacerDubinsParams defaults).
    model.setParams(dyn);

    COST cost;
    cost.GPUSetup();

    RacerCostParams<kRefHorizon> cost_params;
    cost_params.desired_speed = kTargetSpeed;
    // Keep curvature/jerk comfort terms consistent with the active vehicle model.
    cost_params.wheel_base = dyn.wheel_base;
    cost_params.steer_angle_scale = dyn.steer_angle_scale;
    cost_params.boundary_threshold = kRoadHalfWidth;
    constexpr float kEgoLength = 0.55F * 1.5F;
    constexpr float kEgoWidth = 0.28F * 1.5F;
    mppi::cost::setRacerCostEgoFootprint(cost_params, dyn.wheel_base, kEgoLength, kEgoWidth);
    cost.setParams(cost_params);
    mppi::cost::fillRacerCostParkedCars<kRefHorizon>(cost, parked_cars);

    std::array<float2, DYN::CONTROL_DIM> u_rng{};
    u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = { -1.0f, 1.0f };
    u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = { -1.0f, 1.0f };
    model.setControlRanges(u_rng);

    SAMPLER::SAMPLING_PARAMS_T sp{};
    sp.std_dev[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = 0.2F;
    sp.std_dev[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = 0.3F;
    sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
    SAMPLER sampler(sp);

    // 6. Setup MPPI Controller
    FB feedback(&model, kDt);
    Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
    Mppi controller(&model, &cost, &feedback, &sampler, kDt, 1, kLambda, 0.0F, kMppiHorizon, u_nom);
    {
      auto cp = controller.getParams();
      cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
      cp.cost_rollout_dim_ = dim3(32, 2, 1);
      cp.seed_ = 1U;
      controller.setParams(cp);
      // 128 sampled rollouts (128 % 32 == 0); 0.01F yields 40 and breaks visualizeKernel
      controller.setPercentageSampledControlTrajectories(128.0F / static_cast<float>(kNumRollouts));
    }
    model.GPUSetup();

    // 7. Initialize simulation state
    DYN::state_array x = model.getZeroState();
    const mppi::path::Pose2D p0 = path.poseAt(kInitArcLength);
    x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)) = p0.x;
    x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)) = p0.y;
    x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)) = p0.yaw;
    x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)) = kTargetSpeed;

    const std::vector<mppi::path::PathReferenceSample> ref_init = ref_gen.generate(
        path, arcLength, kRefHorizon, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)),
        x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)),
        x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)),
        x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)));
    mppi::cost::fillRacerCostFromPathReference<kRefHorizon>(cost, ref_init);

    // 8. Prepare visualization
    mppi::viz::PathTrackingViewer viewer;
    mppi::viz::PathTrackingStaticScene scene{};
    scene.centerline = mppi::viz::polylineFromPath(path);
    scene.trajectory_corridor = mppi::path::symmetricPathCorridorPolygon(path, kRoadHalfWidth);
    scene.drivable = scene.trajectory_corridor;
    scene.static_obstacles = parked_cars;
    viewer.open("MPPI Stadium Tracking", scene, p0.x, p0.y, "racer_dubins_stadium_path_tracking.mp4", kVideoFps,
                kDt);

    mppi::viz::RunningTimeSeries signal_history;
    const mppi::viz::PathTrackingEgoFootprint ego_fp{ kEgoLength, kEgoWidth, cost_params.ego_axle_to_box_center };
    const mppi::viz::PathTrackingSignalLimits signal_limits{ kTargetSpeed, kVMax, -1.0F, 1.0F, dyn.max_steer_angle };

    mppi::timing::StepTimingCollector step_timing;
    step_timing.reserve(num_sim_steps);

    const int state_x_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_X);
    const int state_y_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_Y);
    const int output_x_idx = static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X);
    const int output_y_idx = static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y);

    // 9. Main simulation loop
    for (size_t k = 0; k < num_sim_steps; ++k) {
      step_timing.beginStep();

      const float sim_time = static_cast<float>(k) * kDt;

      const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(
          path, arcLength, kRefHorizon, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)),
          x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)),
          x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)),
          x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)));
      mppi::cost::fillRacerCostFromPathReference<kRefHorizon>(cost, ref);
      
      // Update importance sampling based on current nominal control
      controller.updateImportanceSampler(u_nom);
      const DYN::control_array u_nom_step = u_nom.col(0);

      // Compute control sequence
      controller.computeControl(x, 1);
      cudaStreamSynchronize(controller.stream_);
      controller.launchSampledVisTrajectories();

      step_timing.endMppi();

      Mppi::control_trajectory u_opt = controller.getControlSeq();

      data_mgr.dumpRolloutSnapshot(k, sim_time, x, controller, model, sampler, kMppiHorizon, kLambda, kDt, u_opt,
                                   kRolloutOutIdx);
      step_timing.endDump();

      mppi::viz::PathTrackingVizFrame viz_frame{};
      viz_frame.ref = ref;
      viz_frame.ego_x = x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X));
      viz_frame.ego_y = x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y));
      {
        const auto device_view = controller.sampledVisDeviceView();
        mppi::viz::RolloutVisDeviceView rollout_view{};
        rollout_view.outputs_d = device_view.outputs_d;
        rollout_view.costs_d = device_view.costs_d;
        rollout_view.num_rollouts = device_view.num_rollouts;
        rollout_view.num_timesteps = device_view.num_timesteps;
        rollout_view.output_dim = device_view.output_dim;
        rollout_view.stream = device_view.stream;
        mppi::viz::fillRolloutsFromDevice(viz_frame.rollouts, rollout_view, output_x_idx, output_y_idx, kMppiHorizon);
      }
      const auto state_trajectory = controller.getActualStateSeq();
      mppi::viz::extractPolyline(state_trajectory, state_x_idx, state_y_idx, kMppiHorizon, viz_frame.planned);
      viz_frame.ego_yaw = x(static_cast<int>(RacerDubinsParams::StateIndex::YAW));
      viz_frame.vel = x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X));
      viz_frame.signals = signal_history;
      viz_frame.sim_time = sim_time;
      viz_frame.step = static_cast<int>(k);
      viz_frame.total_steps = static_cast<int>(num_sim_steps);

      const mppi::path::PathProjection proj_pre = mppi::path::projectPoseOntoPath(
          path, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)),
          x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)), arcLength);
      viz_frame.lat_err = proj_pre.signed_lateral_error;
      viz_frame.baseline_cost = static_cast<float>(controller.getBaselineCost());

      if (!viewer.showFrame(viz_frame, ego_fp, signal_limits))
      {
        step_timing.endStepEarlyExit();
        break;
      }

      // Step simulation model
      DYN::state_array x_next = model.getZeroState();
      DYN::state_array xdot = model.getZeroState();
      DYN::output_array y = DYN::output_array::Zero();

      DYN::control_array u_apply = u_opt.col(0);
      model.enforceConstraints(x, u_apply);
      model.step(x, x_next, xdot, u_apply, y, static_cast<float>(k), kDt);

      // Shift nominal control trajectory for the next step
      u_nom.leftCols(kMppiHorizon - 1) = u_opt.rightCols(kMppiHorizon - 1);
      u_nom.rightCols(1) = u_opt.rightCols(1); 

      x = x_next;

      const float t_end = static_cast<float>(k + 1) * kDt;
      const float vel_x = x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X));
      const float throttle_cmd = u_apply(static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE));
      const float steer_cmd = u_apply(static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD));
      const float steer_state = x(static_cast<int>(RacerDubinsParams::StateIndex::STEER_ANGLE));
      signal_history.push(t_end, vel_x, throttle_cmd, steer_cmd, steer_state);
      step_timing.endViz();

      // Project state onto path to update progress
      const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(path, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)), x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)), arcLength);
      arcLength = proj.arc_length_s;

      const mppi::path::PathReferenceSample& r0 = ref.front();
      const float ref_v_target = ref_gen.speedAt(path, proj.arc_length_s);
      mppi::data::PathTrackingStepLog step_log{};
      step_log.t = t_end;
      step_log.pos_x = x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X));
      step_log.pos_y = x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y));
      step_log.yaw = x(static_cast<int>(RacerDubinsParams::StateIndex::YAW));
      step_log.vel_x = vel_x;
      step_log.steer_angle = steer_state;
      step_log.brake_state = x(static_cast<int>(RacerDubinsParams::StateIndex::BRAKE_STATE));
      step_log.u_accel = throttle_cmd;
      step_log.u_steer = steer_cmd;
      step_log.nom_u_accel = u_nom_step(static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE));
      step_log.nom_u_steer = u_nom_step(static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD));
      step_log.ref_x = r0.x;
      step_log.ref_y = r0.y;
      step_log.ref_yaw = r0.yaw;
      step_log.ref_v_pose = r0.v;
      step_log.ref_v_target = ref_v_target;
      step_log.arc_s = proj.arc_length_s;
      step_log.lat_err = proj.signed_lateral_error;
      step_log.baseline = static_cast<float>(controller.getBaselineCost());
      data_mgr.logPathTrackingStep(step_log);

      step_timing.endStep();
    }

    viewer.close();
    data_mgr.close();
    step_timing.printReport();
    cost.freeCudaMem();
    std::cout << "Wrote " << log_path << " and rollout snapshots under "
              << data_mgr.rolloutDirectory() << "\n";
    return 0;
}
