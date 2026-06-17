/**
 * Interactive OpenGL + ImGui viewer for collision-check unit tests.
 */
#pragma once

#include <mppi/cost_functions/parked_car_obstacles.hpp>
#include <mppi/path/drivable_area.hpp>

#include <string>
#include <vector>

namespace mppi
{
namespace viz
{

struct CollisionScenarioView
{
  std::string name;
  float ego_x = 0.0F;
  float ego_y = 0.0F;
  float ego_yaw = 0.0F;
  bool expect_hit = false;
  bool gpu_hit = false;
  bool host_hit = false;
};

struct CollisionSweepView
{
  std::vector<float> x;
  std::vector<float> y;
  std::vector<int> hits;
  float x_min = 0.0F;
  float x_max = 0.0F;
  float y_min = 0.0F;
  float y_max = 0.0F;
};

/** Blocking UI loop; returns false if the window failed to open. */
bool runCollisionTestViewer(const std::vector<CollisionScenarioView>& scenarios, const CollisionSweepView& sweep,
                            const path::Polygon2D& drivable_poly,
                            const std::vector<cost::ParkedCarObstacle>& obstacles, float ego_length, float ego_width,
                            float ego_axle_to_box_center, int failure_count);

}  // namespace viz
}  // namespace mppi
