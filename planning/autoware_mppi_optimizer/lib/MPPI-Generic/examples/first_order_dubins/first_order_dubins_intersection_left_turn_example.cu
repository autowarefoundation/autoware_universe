/**
 * @file first_order_dubins_intersection_left_turn_example.cu
 * @brief Left turn at intersection with cross traffic from the left (time-varying OBB obstacles).
 *
 * Build: cmake --build build --target first_order_dubins_intersection_left_turn_example
 * Run:   ./build/examples/first_order_dubins_intersection_left_turn_example [seed] [log.csv]
 */

#include <mppi/utils/data_manager.hpp>

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost_bridge.hpp>
#include <mppi/cost_functions/moving_car_obstacles.hpp>
#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>
#include <mppi/feedback_controllers/zero_feedback.cuh>
#include <mppi/path/path_projection.hpp>
#include <mppi/path/path_reference_generator.hpp>
#include <mppi/path/path2d.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/sampling_distributions/diffusion/obstacle_context.hpp>

#include <mppi/path/drivable_area.hpp>
#include <mppi/viz/path_tracking_viewer.hpp>
#include <mppi/viz/rollout_viz_gpu.cuh>
#include <mppi/utils/step_timing.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace
{
  constexpr int kMppiHorizon = 80;
  constexpr int kRefHorizon = kMppiHorizon;
  constexpr float kDt = 0.1F;
  constexpr int kNumRollouts = 32 * 1024;
  constexpr float kTargetSpeed = 3.0F;
  constexpr float kVMax = 5.0F;
  constexpr float kLambda = 1500.0F;
  constexpr float kVideoFps = 30.0F;

  constexpr float kRoadHalfWidth = 0.8F;
  constexpr float kCrossStreetHalfWidth = 3.0F;
  constexpr float kInitArcLength = 2.0F;

  using DYN = FirstOrderDubinsBicycle;
  using COST = FirstOrderDubinsBicycleCost<kRefHorizon>;
  using FB = ZeroFeedback<DYN, kMppiHorizon>;
  using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
  using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

  const mppi::data::RolloutOutputIndices kRolloutOutIdx{
      static_cast<int>(FirstOrderDubinsBicycleParams::OutputIndex::BASELINK_POS_I_X),
      static_cast<int>(FirstOrderDubinsBicycleParams::OutputIndex::BASELINK_POS_I_Y),
      static_cast<int>(FirstOrderDubinsBicycleParams::OutputIndex::YAW),
      static_cast<int>(FirstOrderDubinsBicycleParams::OutputIndex::BASELINK_VEL_B_X),
  };

  mppi::path::Path2D makeLeftTurnIntersectionPath()
  {
    const std::vector<std::pair<float, float>> control = { { 0.0F, -28.0F }, { 0.0F, -10.0F }, { -6.0F, -2.0F },
                                                           { 0.0F, 8.0F },  { 25.0F, 12.0F } };
    return mppi::path::Path2D::catmullRom(control, false, 20);
  }

  int simulationSteps(const mppi::path::Path2D& path)
  {
    const float duration = path.length() / kTargetSpeed + 8.0F;
    return static_cast<int>(std::ceil(duration / kDt));
  }
}  // namespace

int main(int argc, char** argv)
{
  unsigned int seed = 42;
  if (argc > 1)
  {
    seed = std::stoul(argv[1]);
  }
  (void)seed;
  std::cout << "Using random seed: " << seed << std::endl;

  std::string log_path = "first_order_dubins_intersection_left_turn_log.csv";
  if (argc > 2)
  {
    log_path = argv[2];
  }

  const mppi::path::Path2D path = makeLeftTurnIntersectionPath();

  mppi::data::MppiDataManager<DYN> data_mgr;
  if (!data_mgr.beginRun(log_path, path))
  {
    return 1;
  }
  data_mgr.setRoadBoundaryLimits(kRoadHalfWidth, kRoadHalfWidth);

  std::vector<mppi::cost::MovingCarObstacle> cross_traffic = mppi::cost::defaultIntersectionCrossTraffic();
  std::cout << "Cross-traffic vehicles: " << cross_traffic.size() << "\n";

  mppi::path::PathReferenceGenerator ref_gen(kDt);
  ref_gen.setSpeedCap(kVMax);
  ref_gen.setTargetSpeed(kTargetSpeed);
  const int num_sim_steps = simulationSteps(path);
  float arcLength = kInitArcLength;

  DYN model;
  FirstOrderDubinsBicycleParams dyn;
  model.setParams(dyn);

  COST cost;
  cost.GPUSetup();

  FirstOrderDubinsBicycleCostParams<kRefHorizon> cost_params;
  cost_params.desired_speed = kTargetSpeed;
  cost_params.boundary_threshold = kRoadHalfWidth;
  mppi::cost::fillFirstOrderDubinsBicycleCostGeometry<kRefHorizon>(cost_params, dyn);
  constexpr float kEgoLength = 0.55F * 1.5F;
  constexpr float kEgoWidth = 0.28F * 1.5F;
  mppi::cost::setFirstOrderDubinsBicycleCostEgoFootprint<kRefHorizon>(cost_params, dyn.wheel_base, kEgoLength,
                                                                    kEgoWidth);
  cost.setParams(cost_params);

  const float kMaxSteer = dyn.max_steer_angle;
  std::array<float2, DYN::CONTROL_DIM> u_rng{};
  u_rng[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] = { dyn.min_accel,
                                                                                             dyn.max_accel };
  u_rng[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] = { -kMaxSteer, kMaxSteer };
  model.setControlRanges(u_rng);

  SAMPLER::SAMPLING_PARAMS_T sp{};
  sp.std_dev[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD)] = 0.35F;
  sp.std_dev[static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD)] = 0.06F;
  sp.sum_strides = std::max(32, (kNumRollouts + 1023) / 1024);
  SAMPLER sampler(sp);

  FB feedback(&model, kDt);
  Mppi::control_trajectory u_nom = Mppi::control_trajectory::Zero();
  Mppi::control_trajectory u_opt = u_nom;
  Mppi controller(&model, &cost, &feedback, &sampler, kDt, 1, kLambda, 0.0F, kMppiHorizon, u_nom);
  {
    auto cp = controller.getParams();
    cp.dynamics_rollout_dim_ = dim3(32, 2, 1);
    cp.cost_rollout_dim_ = dim3(32, 2, 1);
    cp.seed_ = 1U;
    controller.setParams(cp);
    controller.setPercentageSampledControlTrajectories(128.0F / static_cast<float>(kNumRollouts));
  }
  model.GPUSetup();

  DYN::state_array x = model.getZeroState();
  const mppi::path::Pose2D p0 = path.poseAt(kInitArcLength);
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)) = p0.x;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)) = p0.y;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)) = p0.yaw;
  x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)) = kTargetSpeed;

  std::vector<float> obs_traj_x;
  std::vector<float> obs_traj_y;
  std::vector<float> obs_traj_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;

  mppi::viz::PathTrackingViewer viewer;
  mppi::viz::PathTrackingStaticScene scene{};
  scene.centerline = mppi::viz::polylineFromPath(path);
  scene.trajectory_corridor = mppi::path::symmetricPathCorridorPolygon(path, kRoadHalfWidth);
  scene.extra_corridors.push_back(
      mppi::path::straightCorridorPolygon(-120.0F, -2.5F, 55.0F, -2.5F, kCrossStreetHalfWidth * 0.9F));
  scene.extra_corridors.push_back(
      mppi::path::straightCorridorPolygon(-120.0F, -3.8F, 55.0F, -3.8F, kCrossStreetHalfWidth));
  scene.extra_corridors.push_back(
      mppi::path::straightCorridorPolygon(-120.0F, -5.0F, 55.0F, -5.0F, kCrossStreetHalfWidth * 0.92F));
  scene.extra_corridors.push_back(
      mppi::path::straightCorridorPolygon(-120.0F, -6.2F, 55.0F, -6.2F, kCrossStreetHalfWidth * 0.85F));
  viewer.open("MPPI Intersection", scene, p0.x, p0.y, "first_order_dubins_intersection_left_turn.mp4", kVideoFps,
              kDt);

  mppi::viz::RunningTimeSeries signal_history;
  const mppi::viz::PathTrackingEgoFootprint ego_fp{ kEgoLength, kEgoWidth, cost_params.ego_axle_to_box_center };
  const mppi::viz::PathTrackingSignalLimits signal_limits{ kTargetSpeed, kVMax, dyn.min_accel, dyn.max_accel, kMaxSteer };

  mppi::timing::StepTimingCollector step_timing;
  step_timing.reserve(static_cast<size_t>(num_sim_steps));

  const int state_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X);
  const int state_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y);
  const int output_x_idx = static_cast<int>(FirstOrderDubinsBicycleParams::OutputIndex::BASELINK_POS_I_X);
  const int output_y_idx = static_cast<int>(FirstOrderDubinsBicycleParams::OutputIndex::BASELINK_POS_I_Y);

  for (int k = 0; k < num_sim_steps; ++k)
  {
    step_timing.beginStep();

    const float sim_time = static_cast<float>(k) * kDt;

    const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(
        path, arcLength, kRefHorizon, x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)),
        x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)),
        x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)),
        x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)));
    mppi::cost::fillFirstOrderDubinsBicycleCostFromPathReference<kRefHorizon>(cost, ref);

    mppi::cost::buildObstacleTrajectoryBuffers(cross_traffic, sim_time, kDt, kRefHorizon, obs_traj_x, obs_traj_y,
                                               obs_traj_yaw, obs_half_length, obs_half_width);
    if (!obs_traj_x.empty())
    {
      mppi::cost::fillFirstOrderDubinsBicycleCostObstacleTrajectories<kRefHorizon>(
          cost, obs_traj_x.data(), obs_traj_y.data(), obs_traj_yaw.data(), obs_half_length.data(),
          obs_half_width.data(), static_cast<int>(cross_traffic.size()), kRefHorizon);
    }
    else
    {
      cost.clearObstacles();
    }

    if (k > 0)
    {
      u_nom.leftCols(kMppiHorizon - 1) = u_opt.rightCols(kMppiHorizon - 1);
      u_nom.rightCols(1) = u_opt.rightCols(1);
    }

    controller.updateImportanceSampler(u_nom);
    const DYN::control_array u_nom_step = u_nom.col(0);

    controller.computeControl(x, 1);
    cudaStreamSynchronize(controller.stream_);
    controller.launchSampledVisTrajectories();

    step_timing.endMppi();

    const Mppi::control_trajectory u_opt_traj = controller.getControlSeq();
    u_opt = u_opt_traj;

    const std::vector<float> diffusion_context =
        mppi::sampling_distributions::diffusion::encodeDiffusionObstacleContextFromMovingCars(
            x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)),
            x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)),
            x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW)),
            x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X)), kRoadHalfWidth, kRoadHalfWidth,
            cross_traffic, sim_time);

    data_mgr.dumpRolloutSnapshot(k, sim_time, x, controller, model, sampler, kMppiHorizon, kLambda, kDt, u_opt_traj,
                                 kRolloutOutIdx, mppi::data::kDefaultTopRollouts, diffusion_context);
    step_timing.endDump();

    mppi::viz::PathTrackingVizFrame viz_frame{};
    viz_frame.ref = ref;
    viz_frame.ego_x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
    viz_frame.ego_y = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
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
    viz_frame.ego_yaw = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
    viz_frame.obstacles = mppi::cost::movingCarPosesAt(cross_traffic, sim_time);
    viz_frame.vel = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
    viz_frame.signals = signal_history;
    viz_frame.sim_time = sim_time;
    viz_frame.step = k;
    viz_frame.total_steps = num_sim_steps;

    const mppi::path::PathProjection proj_pre = mppi::path::projectPoseOntoPath(
        path, x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)),
        x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)), arcLength);
    viz_frame.lat_err = proj_pre.signed_lateral_error;
    viz_frame.baseline_cost = static_cast<float>(controller.getBaselineCost());

    if (!viewer.showFrame(viz_frame, ego_fp, signal_limits))
    {
      step_timing.endStepEarlyExit();
      break;
    }

    DYN::state_array x_next = model.getZeroState();
    DYN::state_array xdot = model.getZeroState();
    DYN::output_array y = DYN::output_array::Zero();

    DYN::control_array u_apply = u_opt_traj.col(0);
    model.enforceConstraints(x, u_apply);
    model.step(x, x_next, xdot, u_apply, y, static_cast<float>(k), kDt);

    x = x_next;

    const float t_end = static_cast<float>(k + 1) * kDt;
    const float accel_cmd = u_apply(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD));
    const float steer_cmd = u_apply(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD));
    const float vel_x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::VEL_X));
    const float steer_state = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::STEER_ANGLE));
    signal_history.push(t_end, vel_x, accel_cmd, steer_cmd, steer_state);
    step_timing.endViz();

    const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(
        path, x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X)),
        x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y)), arcLength);
    arcLength = proj.arc_length_s;

    const mppi::path::PathReferenceSample& r0 = ref.front();
    const float ref_v_target = ref_gen.speedAt(path, proj.arc_length_s);
    mppi::data::PathTrackingStepLog step_log{};
    step_log.t = t_end;
    step_log.pos_x = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_X));
    step_log.pos_y = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::POS_Y));
    step_log.yaw = x(static_cast<int>(FirstOrderDubinsBicycleParams::StateIndex::YAW));
    step_log.vel_x = vel_x;
    step_log.steer_angle = steer_state;
    step_log.u_accel = accel_cmd;
    step_log.u_steer = steer_cmd;
    step_log.nom_u_accel = u_nom_step(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::ACCELERATION_CMD));
    step_log.nom_u_steer = u_nom_step(static_cast<int>(FirstOrderDubinsBicycleParams::ControlIndex::STEER_CMD));
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
  std::cout << "Wrote " << log_path << " and rollout snapshots under " << data_mgr.rolloutDirectory() << "\n";
  return 0;
}
