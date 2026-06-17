#include <mppi/viz/collision_viewer.hpp>

#include <mppi/viz/imgui_scene_draw.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GLFW/glfw3.h>

#include <cmath>
#include <string>
#include <vector>

namespace
{
using mppi::viz::detail::Rgb8;
using mppi::viz::detail::ViewCamera;
using mppi::viz::detail::drawEgoAtRearAxle;
using mppi::viz::detail::drawFilledPolygon;
using mppi::viz::detail::drawOrientedBox;
using mppi::viz::detail::drawPolylineLoop;

enum class ViewMode
{
  SCENARIO = 0,
  SWEEP = 1
};

void drawParkedCars(ImDrawList* draw_list, const std::vector<mppi::cost::ParkedCarObstacle>& cars,
                    const ViewCamera& camera, const ImVec2& origin, const ImVec2& size, const Rgb8& fill,
                    const Rgb8& outline)
{
  for (const mppi::cost::ParkedCarObstacle& car : cars)
  {
    drawOrientedBox(draw_list, car.ox, car.oy, car.yaw, car.length, car.width, camera, origin, size, fill, outline,
                    1.5F);
  }
}

void renderScene(ImDrawList* draw_list, const mppi::viz::CollisionScenarioView* scenario,
                 const mppi::viz::CollisionSweepView* sweep, const ViewMode mode,
                 const mppi::path::Polygon2D& drivable_poly, const std::vector<mppi::cost::ParkedCarObstacle>& obstacles,
                 const float ego_length, const float ego_width, const float ego_axle_to_center,
                 const ViewCamera& camera, const ImVec2& origin, const ImVec2& size)
{
  drawFilledPolygon(draw_list, drivable_poly.x, drivable_poly.y, camera, origin, size, Rgb8{ 235, 245, 235, 255 });
  drawPolylineLoop(draw_list, drivable_poly.x, drivable_poly.y, camera, origin, size, Rgb8{ 120, 180, 120, 255 },
                   1.5F);
  drawParkedCars(draw_list, obstacles, camera, origin, size, Rgb8{ 70, 70, 70, 230 }, Rgb8{ 30, 30, 30, 255 });

  if (mode == ViewMode::SWEEP && sweep != nullptr)
  {
    for (size_t i = 0; i < sweep->x.size(); ++i)
    {
      const ImVec2 p = mppi::viz::detail::worldToScreen(sweep->x[i], sweep->y[i], camera, origin, size);
      const Rgb8 color = sweep->hits[i] != 0 ? Rgb8{ 80, 80, 220, 255 } : Rgb8{ 180, 230, 180, 255 };
      draw_list->AddCircleFilled(p, 3.0F, color.toImU32());
    }

    const float box_x[4] = { sweep->x_min, sweep->x_max, sweep->x_max, sweep->x_min };
    const float box_y[4] = { sweep->y_min, sweep->y_min, sweep->y_max, sweep->y_max };
    drawPolylineLoop(draw_list, std::vector<float>(box_x, box_x + 4), std::vector<float>(box_y, box_y + 4), camera,
                     origin, size, Rgb8{ 120, 120, 120, 255 }, 1.0F);
  }
  else if (scenario != nullptr)
  {
    drawEgoAtRearAxle(draw_list, scenario->ego_x, scenario->ego_y, scenario->ego_yaw, ego_length, ego_width,
                      ego_axle_to_center, camera, origin, size);
  }
}
}  // namespace

namespace mppi
{
namespace viz
{

bool runCollisionTestViewer(const std::vector<CollisionScenarioView>& scenarios, const CollisionSweepView& sweep,
                            const path::Polygon2D& drivable_poly,
                            const std::vector<cost::ParkedCarObstacle>& obstacles, const float ego_length,
                            const float ego_width, const float ego_axle_to_box_center, const int failure_count)
{
  if (!glfwInit())
  {
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow* window = glfwCreateWindow(1280, 900, "Collision Check GPU Test", nullptr, nullptr);
  if (window == nullptr)
  {
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  ViewCamera camera;
  ViewMode view_mode = ViewMode::SCENARIO;
  int selected_scenario = 0;

  while (!glfwWindowShouldClose(window))
  {
    glfwPollEvents();

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowPos(ImVec2(0.0F, 0.0F), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(320.0F, io.DisplaySize.y), ImGuiCond_Always);
    ImGui::Begin("Collision Test", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    ImGui::Text("Failures: %d", failure_count);
    ImGui::Separator();

    if (ImGui::RadioButton("Scenario", view_mode == ViewMode::SCENARIO))
    {
      view_mode = ViewMode::SCENARIO;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("GPU sweep", view_mode == ViewMode::SWEEP))
    {
      view_mode = ViewMode::SWEEP;
    }

    if (view_mode == ViewMode::SCENARIO)
    {
      ImGui::Separator();
      for (int i = 0; i < static_cast<int>(scenarios.size()); ++i)
      {
        const CollisionScenarioView& s = scenarios[static_cast<size_t>(i)];
        const bool pass = (s.gpu_hit == s.expect_hit) && (s.host_hit == s.expect_hit) && (s.gpu_hit == s.host_hit);
        const ImVec4 color = pass ? ImVec4(0.4F, 0.9F, 0.4F, 1.0F) : ImVec4(1.0F, 0.35F, 0.35F, 1.0F);
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        if (ImGui::Selectable(s.name.c_str(), selected_scenario == i))
        {
          selected_scenario = i;
        }
        ImGui::PopStyleColor();
      }

      if (!scenarios.empty() && selected_scenario >= 0 &&
          selected_scenario < static_cast<int>(scenarios.size()))
      {
        const CollisionScenarioView& s = scenarios[static_cast<size_t>(selected_scenario)];
        ImGui::Separator();
        ImGui::Text("expect: %s", s.expect_hit ? "HIT" : "MISS");
        ImGui::Text("gpu:    %s", s.gpu_hit ? "HIT" : "MISS");
        ImGui::Text("host:   %s", s.host_hit ? "HIT" : "MISS");
        ImGui::Text("pose: (%.2f, %.2f)", s.ego_x, s.ego_y);
      }
    }
    else
    {
      ImGui::Separator();
      ImGui::Text("Samples: %zu", sweep.x.size());
      ImGui::Text("Red = collision");
      ImGui::Text("Green = clear");
    }

    ImGui::Separator();
    ImGui::Text("View");
    ImGui::SliderFloat("zoom", &camera.pixels_per_meter, 8.0F, 48.0F, "%.1f px/m");
    ImGui::DragFloat("pan X", &camera.center_x, 0.05F, -4.0F, 4.0F);
    ImGui::DragFloat("pan Y", &camera.center_y, 0.05F, -5.0F, 30.0F);
    if (ImGui::Button("Reset view"))
    {
      camera.center_x = 0.0F;
      camera.center_y = 8.0F;
      camera.pixels_per_meter = 22.0F;
    }

    ImGui::End();

    const float canvas_x = 330.0F;
    const float canvas_w = io.DisplaySize.x - canvas_x;
    const float canvas_h = io.DisplaySize.y;

    ImGui::SetNextWindowPos(ImVec2(canvas_x, 0.0F), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(canvas_w, canvas_h), ImGuiCond_Always);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0F, 0.0F));
    ImGui::Begin("Scene", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse | ImGuiWindowFlags_NoTitleBar);
    ImGui::PopStyleVar();

    const ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    const ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    ImGui::InvisibleButton("canvas", canvas_size,
                           ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    const bool hovered = ImGui::IsItemHovered();
    if (hovered && ImGui::IsMouseDragging(ImGuiMouseButton_Right))
    {
      const ImVec2 drag = ImGui::GetIO().MouseDelta;
      camera.center_x -= drag.x / camera.pixels_per_meter;
      camera.center_y += drag.y / camera.pixels_per_meter;
    }
    if (hovered)
    {
      camera.pixels_per_meter *= (1.0F + 0.1F * io.MouseWheel);
      camera.pixels_per_meter = std::max(8.0F, std::min(48.0F, camera.pixels_per_meter));
    }

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(canvas_pos,
                             ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                             IM_COL32(250, 250, 250, 255));

    const CollisionScenarioView* scenario_ptr = nullptr;
    if (view_mode == ViewMode::SCENARIO && !scenarios.empty() && selected_scenario >= 0 &&
        selected_scenario < static_cast<int>(scenarios.size()))
    {
      scenario_ptr = &scenarios[static_cast<size_t>(selected_scenario)];
    }

    renderScene(draw_list, scenario_ptr, view_mode == ViewMode::SWEEP ? &sweep : nullptr, view_mode, drivable_poly,
                obstacles, ego_length, ego_width, ego_axle_to_box_center, camera, canvas_pos, canvas_size);

    ImGui::End();

    ImGui::Render();
    const int display_w = static_cast<int>(io.DisplaySize.y);
    const int display_h = static_cast<int>(io.DisplaySize.y);
    (void)display_w;
    glViewport(0, 0, static_cast<int>(io.DisplaySize.x), display_h);
    glClearColor(0.1F, 0.1F, 0.12F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return true;
}

}  // namespace viz
}  // namespace mppi
