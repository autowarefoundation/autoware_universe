/**
 * @file collision_check_gpu_test.cu
 * @brief GPU unit test for ego-vs-obstacle OBB collision (FirstOrderDubinsBicycleCost).
 *
 * Runs egoIntersectsObstacleAtStep on the device cost object, compares to host,
 * and optionally opens an ImGui + OpenGL viewer for interactive inspection.
 *
 * Build: cmake --build build --target collision_check_gpu_test
 * Run:   ./build/tests/collision_check_gpu_test [--ui]
 */

#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost.cuh>
#include <mppi/cost_functions/dubins/first_order_dubins_bicycle_cost_bridge.hpp>
#include <mppi/cost_functions/moving_car_obstacles.hpp>
#include <mppi/dynamics/dubins/first_order_dubins_bicycle.cuh>
#include <mppi/path/drivable_area.hpp>
#include <mppi/viz/collision_viewer.hpp>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace
{
constexpr int kHorizon = 8;
constexpr float kDt = 0.1F;
constexpr float kRoadYaw = mppi::cost::TwoLaneRoadLayout::kRoadYaw;
constexpr float kEgoLength = 0.55F * 1.5F;
constexpr float kEgoWidth = 0.28F * 1.5F;

using BicycleCost = FirstOrderDubinsBicycleCost<kHorizon>;

struct PoseCase
{
  const char* name;
  float x;
  float y;
  float yaw;
  bool expect_hit;
};

__global__ void collisionCheckKernel(const BicycleCost* cost_d, const float* ego_x, const float* ego_y,
                                     const float* ego_yaw, int* hits, const int count, const int timestep)
{
  const int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count)
  {
    return;
  }
  hits[i] = cost_d->egoIntersectsObstacleAtStep(ego_x[i], ego_y[i], ego_yaw[i], timestep) ? 1 : 0;
}

bool runGpuCollisionBatch(BicycleCost& cost, const std::vector<float>& ego_x, const std::vector<float>& ego_y,
                          const std::vector<float>& ego_yaw, std::vector<int>& gpu_hits, const int timestep = 0)
{
  const int n = static_cast<int>(ego_x.size());
  if (n == 0)
  {
    return false;
  }

  float* d_ego_x = nullptr;
  float* d_ego_y = nullptr;
  float* d_ego_yaw = nullptr;
  int* d_hits = nullptr;

  HANDLE_ERROR(cudaMalloc(&d_ego_x, sizeof(float) * static_cast<size_t>(n)));
  HANDLE_ERROR(cudaMalloc(&d_ego_y, sizeof(float) * static_cast<size_t>(n)));
  HANDLE_ERROR(cudaMalloc(&d_ego_yaw, sizeof(float) * static_cast<size_t>(n)));
  HANDLE_ERROR(cudaMalloc(&d_hits, sizeof(int) * static_cast<size_t>(n)));

  HANDLE_ERROR(cudaMemcpy(d_ego_x, ego_x.data(), sizeof(float) * static_cast<size_t>(n), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(d_ego_y, ego_y.data(), sizeof(float) * static_cast<size_t>(n), cudaMemcpyHostToDevice));
  HANDLE_ERROR(cudaMemcpy(d_ego_yaw, ego_yaw.data(), sizeof(float) * static_cast<size_t>(n), cudaMemcpyHostToDevice));

  constexpr int kBlock = 256;
  const int grid = (n + kBlock - 1) / kBlock;
  collisionCheckKernel<<<grid, kBlock, 0, cost.stream_>>>(cost.cost_d_, d_ego_x, d_ego_y, d_ego_yaw, d_hits, n,
                                                          timestep);
  HANDLE_ERROR(cudaGetLastError());
  HANDLE_ERROR(cudaStreamSynchronize(cost.stream_));

  gpu_hits.resize(static_cast<size_t>(n));
  HANDLE_ERROR(cudaMemcpy(gpu_hits.data(), d_hits, sizeof(int) * static_cast<size_t>(n), cudaMemcpyDeviceToHost));

  cudaFree(d_ego_x);
  cudaFree(d_ego_y);
  cudaFree(d_ego_yaw);
  cudaFree(d_hits);
  return true;
}

void setupDoubleParkCost(BicycleCost& cost, FirstOrderDubinsBicycleCostParams<kHorizon>& cost_params)
{
  FirstOrderDubinsBicycleParams dyn;
  cost_params.desired_speed = 3.0F;
  mppi::cost::fillFirstOrderDubinsBicycleCostGeometry<kHorizon>(cost_params, dyn);
  mppi::cost::setFirstOrderDubinsBicycleCostEgoFootprint<kHorizon>(cost_params, dyn.wheel_base, kEgoLength, kEgoWidth);

  cost.GPUSetup();
  cost.setParams(cost_params);

  const std::vector<mppi::cost::MovingCarObstacle> obstacles = mppi::cost::twoLaneDoubleParkAndRearApproach();
  std::vector<float> obs_traj_x;
  std::vector<float> obs_traj_y;
  std::vector<float> obs_traj_yaw;
  std::vector<float> obs_half_length;
  std::vector<float> obs_half_width;
  mppi::cost::buildObstacleTrajectoryBuffers(obstacles, 0.0F, kDt, kHorizon, obs_traj_x, obs_traj_y, obs_traj_yaw,
                                             obs_half_length, obs_half_width);
  mppi::cost::fillFirstOrderDubinsBicycleCostObstacleTrajectories<kHorizon>(
      cost, obs_traj_x.data(), obs_traj_y.data(), obs_traj_yaw.data(), obs_half_length.data(), obs_half_width.data(),
      static_cast<int>(obstacles.size()), kHorizon);
}
}  // namespace

int main(int argc, char** argv)
{
  bool show_ui = false;
  for (int i = 1; i < argc; ++i)
  {
    if (std::strcmp(argv[i], "--ui") == 0 || std::strcmp(argv[i], "--show") == 0)
    {
      show_ui = true;
    }
  }

  BicycleCost cost;
  FirstOrderDubinsBicycleCostParams<kHorizon> cost_params;
  setupDoubleParkCost(cost, cost_params);

  const std::vector<mppi::cost::MovingCarObstacle> obstacles = mppi::cost::twoLaneDoubleParkAndRearApproach();
  const std::vector<mppi::cost::ParkedCarObstacle> cars_viz = mppi::cost::movingCarPosesAt(obstacles, 0.0F);
  const mppi::path::Polygon2D drivable_poly = mppi::path::twoLaneRoadPolygon();

  const PoseCase cases[] = {
    { "left_lane_at_stopped_car", mppi::cost::TwoLaneRoadLayout::kLeftLaneX, 11.0F, kRoadYaw, true },
    { "left_lane_well_before", mppi::cost::TwoLaneRoadLayout::kLeftLaneX, 5.0F, kRoadYaw, false },
    { "right_lane_beside_obstacle", mppi::cost::TwoLaneRoadLayout::kRightLaneX, 11.0F, kRoadYaw, false },
    { "weave_pose_partial_right", 0.35F, 10.8F, kRoadYaw, false },
    { "left_lane_just_before", mppi::cost::TwoLaneRoadLayout::kLeftLaneX, 8.5F, kRoadYaw, false },
  };

  int failures = 0;
  std::vector<mppi::viz::CollisionScenarioView> scenario_views;
  scenario_views.reserve(sizeof(cases) / sizeof(cases[0]));

  for (const PoseCase& pose : cases)
  {
    const bool host_hit = cost.egoIntersectsObstacleAtStep(pose.x, pose.y, pose.yaw, 0);
    std::vector<int> gpu_hits;
    runGpuCollisionBatch(cost, { pose.x }, { pose.y }, { pose.yaw }, gpu_hits, 0);
    const bool gpu_hit = gpu_hits[0] != 0;

    if (gpu_hit != pose.expect_hit || host_hit != pose.expect_hit || gpu_hit != host_hit)
    {
      ++failures;
      std::cerr << "FAIL " << pose.name << " expect=" << pose.expect_hit << " gpu=" << gpu_hit
                << " host=" << host_hit << "\n";
    }
    else
    {
      std::cout << "PASS " << pose.name << "\n";
    }

    mppi::viz::CollisionScenarioView view{};
    view.name = pose.name;
    view.ego_x = pose.x;
    view.ego_y = pose.y;
    view.ego_yaw = pose.yaw;
    view.expect_hit = pose.expect_hit;
    view.gpu_hit = gpu_hit;
    view.host_hit = host_hit;
    scenario_views.push_back(view);
  }

  constexpr float kXMin = -1.4F;
  constexpr float kXMax = 1.4F;
  constexpr float kYMin = 9.0F;
  constexpr float kYMax = 12.5F;
  constexpr float kStep = 0.08F;
  mppi::viz::CollisionSweepView sweep{};
  sweep.x_min = kXMin;
  sweep.x_max = kXMax;
  sweep.y_min = kYMin;
  sweep.y_max = kYMax;
  for (float y = kYMin; y <= kYMax; y += kStep)
  {
    for (float x = kXMin; x <= kXMax; x += kStep)
    {
      sweep.x.push_back(x);
      sweep.y.push_back(y);
    }
  }
  std::vector<float> sweep_yaw(sweep.x.size(), kRoadYaw);

  std::vector<int> sweep_hits;
  runGpuCollisionBatch(cost, sweep.x, sweep.y, sweep_yaw, sweep_hits, 0);
  sweep.hits = sweep_hits;

  bool sweep_host_mismatch = false;
  for (size_t i = 0; i < sweep.x.size(); ++i)
  {
    const bool host_hit = cost.egoIntersectsObstacleAtStep(sweep.x[i], sweep.y[i], kRoadYaw, 0);
    if (static_cast<bool>(sweep.hits[i]) != host_hit)
    {
      sweep_host_mismatch = true;
      std::cerr << "GPU/host mismatch at (" << sweep.x[i] << ", " << sweep.y[i] << "): gpu=" << sweep.hits[i]
                << " host=" << host_hit << "\n";
      break;
    }
  }

  if (sweep_host_mismatch)
  {
    ++failures;
    std::cerr << "FAIL gpu_host_sweep_mismatch\n";
  }
  else
  {
    std::cout << "PASS gpu_host_sweep (" << sweep.x.size() << " poses)\n";
  }

  if (show_ui)
  {
    if (!mppi::viz::runCollisionTestViewer(scenario_views, sweep, drivable_poly, cars_viz, kEgoLength, kEgoWidth,
                                           cost_params.ego_axle_to_box_center, failures))
    {
      std::cerr << "Could not open collision test UI.\n";
      cost.freeCudaMem();
      return 1;
    }
  }

  cost.freeCudaMem();

  if (failures > 0)
  {
    std::cerr << failures << " failure(s).\n";
    return 1;
  }

  std::cout << "All collision checks passed.\n";
  return 0;
}
