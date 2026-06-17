/**
 * @file racer_dubins_stadium_path_tracking_cost_map_example.cu
 * @brief MPPI path tracking with GPU costmap for a Racer Dubins model on a stadium track.
 *
 * Build: cmake --build build --target racer_dubins_stadium_path_tracking_cost_map_example
 * Run:   ./build/examples/racer/racer_dubins_stadium_path_tracking_cost_map_example [seed]
 */

#include <mppi/dynamics/racer_dubins/racer_dubins.cuh>
#include <mppi/cost_functions/racer/racer_cost_map.cuh>
#include <mppi/cost_functions/racer/racer_costmap_builder.hpp>
#include <mppi/controllers/MPPI/mppi_controller.cuh>
#include <mppi/feedback_controllers/path_tracker_feedback.cuh>
#include <mppi/path/path_projection.hpp>
#include <mppi/path/path_reference_generator.hpp>
#include <mppi/path/path2d.hpp>
#include <mppi/path/drivable_area.hpp>
#include <mppi/sampling_distributions/gaussian/gaussian.cuh>
#include <mppi/viz/path_tracking_viewer.hpp>
#include <mppi/viz/rollout_viz_gpu.cuh>

#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace
{
constexpr int kMppiHorizon = 80;
constexpr int kRefHorizon = kMppiHorizon;
constexpr float kDt = 0.1F;
constexpr int kNumRollouts = 4 * 1024;
constexpr float kTargetSpeed = 2.5F;
constexpr float kVMax = 3.0F;
constexpr size_t kSimLaps = 1;

constexpr float kStraightLength = 40.0F;
constexpr float kTurnRadius = 10.0F;
constexpr int kSamplesPerArc = 48;

constexpr float kInitArcLength = kStraightLength - 2.0F;
constexpr float kRoadHalfWidth = 0.8F;
constexpr float kLambda = 100.0F;
constexpr float kVideoFps = 30.0F;

using DYN = RacerDubins;
using COST = RacerCostMap;
using FB = PathTrackerFeedback<DYN, kMppiHorizon>;
using SAMPLER = mppi::sampling_distributions::GaussianDistribution<DYN::DYN_PARAMS_T>;
using Mppi = VanillaMPPIController<DYN, COST, FB, kMppiHorizon, kNumRollouts, SAMPLER>;

int simStepsForLaps(const mppi::path::Path2D& path, const float laps)
{
  const float lap_time = path.length() / kVMax;
  return static_cast<int>(std::ceil(laps * lap_time / kDt));
}

std::vector<mppi::cost::ParkedCarObstacle> costmapObstaclesToViz(
    const std::vector<mppi::cost::RacerCostmapObstacle>& obstacles)
{
  std::vector<mppi::cost::ParkedCarObstacle> viz_obstacles;
  viz_obstacles.reserve(obstacles.size());
  for (const mppi::cost::RacerCostmapObstacle& obs : obstacles)
  {
    mppi::cost::ParkedCarObstacle car;
    car.ox = obs.ox;
    car.oy = obs.oy;
    car.yaw = 0.0F;
    car.length = 2.0F * obs.r;
    car.width = 2.0F * obs.r;
    viz_obstacles.push_back(car);
  }
  return viz_obstacles;
}
}  // namespace

int main(int argc, char** argv)
{
  unsigned int seed = 42;
  if (argc > 1)
  {
    seed = static_cast<unsigned int>(std::stoul(argv[1]));
  }
  std::cout << "Using random seed: " << seed << std::endl;

  const mppi::path::Path2D path = mppi::path::Path2D::stadium(kStraightLength, kTurnRadius, kSamplesPerArc);

  const float x_min = -40.0F;
  const float x_max = 60.0F;
  const float y_min = -30.0F;
  const float y_max = 30.0F;
  const float ppm = 10.0F;
  const int width = static_cast<int>((x_max - x_min) * ppm);
  const int height = static_cast<int>((y_max - y_min) * ppm);

  cv::Mat costmap_img;

  std::vector<mppi::cost::RacerCostmapObstacle> obstacles;
  std::mt19937 gen(seed);
  std::uniform_real_distribution<float> dist_s(0.0F, path.length());
  std::uniform_real_distribution<float> dist_side(-1.0F, 1.0F);
  std::uniform_real_distribution<float> dist_r(2.0F, 4.5F);

  for (int i = 0; i < 15; ++i)
  {
    const float s = dist_s(gen);
    const float side = (dist_side(gen) > 0.0F ? 1.0F : -1.0F) * 2.5F;
    const float r = dist_r(gen);
    const mppi::path::Pose2D p = path.poseAt(s);
    float tx = 0.0F;
    float ty = 0.0F;
    path.tangentAt(s, tx, ty);
    obstacles.emplace_back(p.x - side * ty, p.y + side * tx, r);
  }

  mppi::path::PathReferenceGenerator ref_gen(kDt);
  ref_gen.setSpeedCap(kVMax);

  const size_t num_sim_steps = static_cast<size_t>(simStepsForLaps(path, static_cast<float>(kSimLaps)));

  float arcLength = kInitArcLength;
  const std::vector<mppi::path::PathReferenceSample> ref_init = ref_gen.generate(path, arcLength, kRefHorizon);
  std::vector<float4> cost_map_gpu_data;
  mppi::cost::updateRacerCostmap(costmap_img, ref_init, obstacles, width, height, ppm, x_min, y_min,
                                 cost_map_gpu_data);

  COST cost;
  cost.GPUSetup();
  cost.costmapToTexture(width, height, cost_map_gpu_data.data());
  cost.setCpuCostmap(costmap_img);

  RacerCostMapParams cost_params;
  cost_params.desired_speed = kTargetSpeed;
  cost.setParams(cost_params);
  cost.setWorldToCostmapBounds(x_min, x_max, y_min, y_max);

  DYN model;
  RacerDubinsParams dyn;
  dyn.wheel_base = 0.3F;
  model.setParams(dyn);
  std::array<float2, DYN::CONTROL_DIM> u_rng{};
  u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE)] = { -1.0F, 1.0F };
  u_rng[static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD)] = { -1.0F, 1.0F };
  model.setControlRanges(u_rng);

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
    cp.seed_ = 1U;
    controller.setParams(cp);
    controller.setPercentageSampledControlTrajectories(128.0F / static_cast<float>(kNumRollouts));
  }
  model.GPUSetup();

  DYN::state_array x = model.getZeroState();
  const mppi::path::Pose2D p0 = path.poseAt(kInitArcLength);
  x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)) = p0.x;
  x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)) = p0.y;
  x(static_cast<int>(RacerDubinsParams::StateIndex::YAW)) = p0.yaw;
  x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X)) = kTargetSpeed;

  constexpr float kEgoLength = 0.55F * 1.5F;
  constexpr float kEgoWidth = 0.28F * 1.5F;
  const float ego_axle_to_center = dyn.wheel_base * 0.5F;

  mppi::viz::PathTrackingViewer viewer;
  mppi::viz::PathTrackingStaticScene scene{};
  scene.centerline = mppi::viz::polylineFromPath(path);
  scene.trajectory_corridor = mppi::path::symmetricPathCorridorPolygon(path, kRoadHalfWidth);
  scene.drivable = scene.trajectory_corridor;
  scene.static_obstacles = costmapObstaclesToViz(obstacles);
  viewer.open("MPPI Stadium Costmap", scene, p0.x, p0.y, "racer_dubins_stadium_path_tracking_cost_map.mp4",
              kVideoFps, kDt);

  mppi::viz::RunningTimeSeries signal_history;
  const mppi::viz::PathTrackingEgoFootprint ego_fp{ kEgoLength, kEgoWidth, ego_axle_to_center };
  const mppi::viz::PathTrackingSignalLimits signal_limits{ kTargetSpeed, kVMax, -1.0F, 1.0F, dyn.max_steer_angle };

  const int state_x_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_X);
  const int state_y_idx = static_cast<int>(RacerDubinsParams::StateIndex::POS_Y);
  const int output_x_idx = static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_X);
  const int output_y_idx = static_cast<int>(RacerDubinsParams::OutputIndex::BASELINK_POS_I_Y);

  for (size_t k = 0; k < num_sim_steps; ++k)
  {
    const std::vector<mppi::path::PathReferenceSample> ref = ref_gen.generate(path, arcLength, kRefHorizon);

    mppi::cost::updateRacerCostmap(costmap_img, ref, obstacles, width, height, ppm, x_min, y_min, cost_map_gpu_data);
    cost.updateCostmapTexture(cost_map_gpu_data.data());
    cost.setCpuCostmap(costmap_img);

    const Mppi::state_trajectory goal_traj = mppi::feedback::goalTrajectoryFromPathReference<DYN, kMppiHorizon>(ref);
    feedback.updateReference(path, ref);
    feedback.applyFeedforwardToNominal(u_nom, x, goal_traj);

    controller.updateImportanceSampler(u_nom);

    controller.computeControl(x, 1);
    cudaStreamSynchronize(controller.stream_);
    controller.launchSampledVisTrajectories();

    const Mppi::control_trajectory u_opt = controller.getControlSeq();

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
    viz_frame.sim_time = static_cast<float>(k) * kDt;
    viz_frame.step = static_cast<int>(k);
    viz_frame.total_steps = static_cast<int>(num_sim_steps);

    const mppi::path::PathProjection proj_pre = mppi::path::projectPoseOntoPath(
        path, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)),
        x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)), arcLength);
    viz_frame.lat_err = proj_pre.signed_lateral_error;
    viz_frame.baseline_cost = static_cast<float>(controller.getBaselineCost());

    if (!viewer.showFrame(viz_frame, ego_fp, signal_limits))
    {
      break;
    }

    DYN::state_array x_next = model.getZeroState();
    DYN::state_array xdot = model.getZeroState();
    DYN::output_array y = DYN::output_array::Zero();

    DYN::control_array u_apply = u_opt.col(0);
    model.enforceConstraints(x, u_apply);
    model.step(x, x_next, xdot, u_apply, y, static_cast<float>(k), kDt);

    u_nom.leftCols(kMppiHorizon - 1) = u_opt.rightCols(kMppiHorizon - 1);
    u_nom.rightCols(1) = u_opt.rightCols(1);

    x = x_next;

    const float t_end = static_cast<float>(k + 1) * kDt;
    const float vel_x = x(static_cast<int>(RacerDubinsParams::StateIndex::VEL_X));
    const float throttle_cmd = u_apply(static_cast<int>(RacerDubinsParams::ControlIndex::THROTTLE_BRAKE));
    const float steer_cmd = u_apply(static_cast<int>(RacerDubinsParams::ControlIndex::STEER_CMD));
    const float steer_state = x(static_cast<int>(RacerDubinsParams::StateIndex::STEER_ANGLE));
    signal_history.push(t_end, vel_x, throttle_cmd, steer_cmd, steer_state);

    const mppi::path::PathProjection proj = mppi::path::projectPoseOntoPath(
        path, x(static_cast<int>(RacerDubinsParams::StateIndex::POS_X)),
        x(static_cast<int>(RacerDubinsParams::StateIndex::POS_Y)), arcLength);
    arcLength = proj.arc_length_s;
  }

  viewer.close();
  cost.freeCudaMem();
  return 0;
}
