#include <mppi/viz/path_tracking_viewer.hpp>

#include <mppi/viz/framebuffer_recorder.hpp>
#include <mppi/viz/imgui_scene_draw.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

namespace
{
using mppi::viz::detail::Rgb8;
using mppi::viz::detail::ViewCamera;
using mppi::viz::detail::drawEgoAtRearAxle;
using mppi::viz::detail::drawFilledPolygon;
using mppi::viz::detail::drawOrientedBox;
using mppi::viz::detail::drawPolyline;
using mppi::viz::detail::drawPolylineLoop;
using mppi::viz::detail::rolloutCostColor;

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

void drawReferencePath(ImDrawList* draw_list, const std::vector<mppi::path::PathReferenceSample>& ref,
                       const ViewCamera& camera, const ImVec2& origin, const ImVec2& size)
{
  if (ref.size() < 2U)
  {
    return;
  }

  std::vector<float> x(ref.size());
  std::vector<float> y(ref.size());
  for (size_t i = 0; i < ref.size(); ++i)
  {
    x[i] = ref[i].x;
    y[i] = ref[i].y;
  }
  drawPolyline(draw_list, x, y, camera, origin, size, Rgb8{ 0, 165, 255, 220 }, 2.0F);
}

void drawRollouts(ImDrawList* draw_list, const std::vector<mppi::viz::VizRollout>& rollouts, const ViewCamera& camera,
                  const ImVec2& origin, const ImVec2& size)
{
  if (rollouts.empty())
  {
    return;
  }

  float min_cost = rollouts.front().cost;
  float max_cost = rollouts.front().cost;
  for (const mppi::viz::VizRollout& rollout : rollouts)
  {
    min_cost = std::min(min_cost, rollout.cost);
    max_cost = std::max(max_cost, rollout.cost);
  }

  for (const mppi::viz::VizRollout& rollout : rollouts)
  {
    const ImU32 color = rolloutCostColor(rollout.cost, min_cost, max_cost);
    for (size_t i = 0; i + 1 < rollout.x.size(); ++i)
    {
      const ImVec2 p0 = mppi::viz::detail::worldToScreen(rollout.x[i], rollout.y[i], camera, origin, size);
      const ImVec2 p1 = mppi::viz::detail::worldToScreen(rollout.x[i + 1], rollout.y[i + 1], camera, origin, size);
      draw_list->AddLine(p0, p1, color, 1.0F);
    }
  }
}

void drawStaticScene(ImDrawList* draw_list, const mppi::viz::PathTrackingStaticScene& scene, const ViewCamera& camera,
                     const ImVec2& origin, const ImVec2& size)
{
  if (!scene.drivable.empty())
  {
    drawFilledPolygon(draw_list, scene.drivable.x, scene.drivable.y, camera, origin, size,
                      Rgb8{ 235, 245, 235, 255 });
  }

  for (const mppi::path::Polygon2D& corridor : scene.extra_corridors)
  {
    if (!corridor.empty())
    {
      drawFilledPolygon(draw_list, corridor.x, corridor.y, camera, origin, size, Rgb8{ 235, 245, 235, 200 });
    }
  }

  if (!scene.trajectory_corridor.empty())
  {
    drawFilledPolygon(draw_list, scene.trajectory_corridor.x, scene.trajectory_corridor.y, camera, origin, size,
                      Rgb8{ 245, 250, 255, 180 });
    drawPolylineLoop(draw_list, scene.trajectory_corridor.x, scene.trajectory_corridor.y, camera, origin, size,
                     Rgb8{ 40, 90, 200, 255 }, 1.5F);
  }

  if (!scene.centerline.x.empty())
  {
    drawPolyline(draw_list, scene.centerline.x, scene.centerline.y, camera, origin, size, Rgb8{ 200, 200, 200, 255 },
                 2.0F);
  }

  drawParkedCars(draw_list, scene.static_obstacles, camera, origin, size, Rgb8{ 70, 70, 70, 230 },
                 Rgb8{ 30, 30, 30, 255 });
}

void drawDynamicScene(ImDrawList* draw_list, const mppi::viz::PathTrackingVizFrame& frame, const ViewCamera& camera,
                      const ImVec2& origin, const ImVec2& size, const mppi::viz::PathTrackingEgoFootprint& ego)
{
  drawReferencePath(draw_list, frame.ref, camera, origin, size);
  drawRollouts(draw_list, frame.rollouts, camera, origin, size);

  if (frame.planned.x.size() >= 2U)
  {
    drawPolyline(draw_list, frame.planned.x, frame.planned.y, camera, origin, size, Rgb8{ 255, 64, 0, 255 }, 3.0F);
  }

  if (!frame.obstacles.empty())
  {
    drawParkedCars(draw_list, frame.obstacles, camera, origin, size, Rgb8{ 40, 80, 200, 230 },
                   Rgb8{ 20, 40, 120, 255 });
  }

  drawEgoAtRearAxle(draw_list, frame.ego_x, frame.ego_y, frame.ego_yaw, ego.length, ego.width, ego.axle_to_center,
                    camera, origin, size);
}

void expandBounds(float& min_x, float& min_y, float& max_x, float& max_y, const float x, const float y)
{
  min_x = std::min(min_x, x);
  min_y = std::min(min_y, y);
  max_x = std::max(max_x, x);
  max_y = std::max(max_y, y);
}

void expandBounds(float& min_x, float& min_y, float& max_x, float& max_y, const std::vector<float>& x,
                  const std::vector<float>& y)
{
  const size_t n = std::min(x.size(), y.size());
  for (size_t i = 0; i < n; ++i)
  {
    expandBounds(min_x, min_y, max_x, max_y, x[i], y[i]);
  }
}

bool computeSceneBounds(const mppi::viz::PathTrackingStaticScene& scene, float& min_x, float& min_y, float& max_x,
                        float& max_y)
{
  min_x = min_y = std::numeric_limits<float>::max();
  max_x = max_y = std::numeric_limits<float>::lowest();

  expandBounds(min_x, min_y, max_x, max_y, scene.centerline.x, scene.centerline.y);
  expandBounds(min_x, min_y, max_x, max_y, scene.drivable.x, scene.drivable.y);
  expandBounds(min_x, min_y, max_x, max_y, scene.trajectory_corridor.x, scene.trajectory_corridor.y);
  for (const mppi::path::Polygon2D& corridor : scene.extra_corridors)
  {
    expandBounds(min_x, min_y, max_x, max_y, corridor.x, corridor.y);
  }
  for (const mppi::cost::ParkedCarObstacle& car : scene.static_obstacles)
  {
    expandBounds(min_x, min_y, max_x, max_y, car.ox, car.oy);
  }

  return min_x <= max_x && min_y <= max_y;
}

void fitCameraToCanvas(ViewCamera& camera, const float min_x, const float min_y, const float max_x, const float max_y,
                       const float canvas_w, const float canvas_h, const float padding = 0.88F)
{
  const float span_x = std::max(1.0F, max_x - min_x);
  const float span_y = std::max(1.0F, max_y - min_y);
  camera.center_x = 0.5F * (min_x + max_x);
  camera.center_y = 0.5F * (min_y + max_y);

  const float ppm_x = (canvas_w * padding) / span_x;
  const float ppm_y = (canvas_h * padding) / span_y;
  camera.pixels_per_meter = std::max(8.0F, std::min(48.0F, std::min(ppm_x, ppm_y)));
}

void setFollowZoom(ViewCamera& camera, const float canvas_w, const float canvas_h, const float view_width_m = 34.0F,
                   const float view_height_m = 24.0F)
{
  const float ppm_x = (canvas_w * 0.92F) / view_width_m;
  const float ppm_y = (canvas_h * 0.92F) / view_height_m;
  camera.pixels_per_meter = std::max(8.0F, std::min(48.0F, std::min(ppm_x, ppm_y)));
}

float sidebarWidth(const float display_w)
{
  return std::max(300.0F, std::min(420.0F, display_w * 0.24F));
}
}  // namespace

namespace mppi
{
namespace viz
{

struct PathTrackingViewer::Impl
{
  GLFWwindow* window = nullptr;
  bool open = false;
  PathTrackingStaticScene static_scene;
  ViewCamera camera;
  bool follow_ego = true;
  bool initial_zoom_set = false;
  std::string title;
  AsyncFramebufferRecorder recorder;
  int video_frames_per_capture = 1;
  std::array<std::vector<std::uint8_t>, 2> capture_buffers{};
  int capture_index = 0;
};

PathTrackingViewer::~PathTrackingViewer()
{
  close();
}

bool PathTrackingViewer::open(const char* title, const PathTrackingStaticScene& scene, const float camera_x,
                              const float camera_y, const char* video_path, const float video_fps, const float sim_dt)
{
  close();
  impl_ = new Impl();
  impl_->static_scene = scene;
  impl_->camera.center_x = camera_x;
  impl_->camera.center_y = camera_y;
  impl_->title = title != nullptr ? title : "MPPI Path Tracking";
  impl_->video_frames_per_capture = 1;
  if (sim_dt > 0.0F && video_fps > 0.0F)
  {
    impl_->video_frames_per_capture =
        std::max(1, static_cast<int>(std::lround(static_cast<double>(video_fps) * static_cast<double>(sim_dt))));
  }

  if (!glfwInit())
  {
    close();
    return false;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  impl_->window = glfwCreateWindow(1440, 900, impl_->title.c_str(), nullptr, nullptr);
  if (impl_->window == nullptr)
  {
    glfwTerminate();
    close();
    return false;
  }

  glfwMakeContextCurrent(impl_->window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(impl_->window, true);
  ImGui_ImplOpenGL3_Init("#version 330");

  impl_->open = true;
  if (video_path != nullptr && video_path[0] != '\0')
  {
    beginRecording(video_path, video_fps);
  }
  return true;
}

bool PathTrackingViewer::beginRecording(const char* output_path, const float fps)
{
  if (impl_ == nullptr || output_path == nullptr || output_path[0] == '\0')
  {
    return false;
  }
  return impl_->recorder.start(output_path, fps);
}

void PathTrackingViewer::stopRecording()
{
  if (impl_ != nullptr)
  {
    impl_->recorder.stop();
  }
}

bool PathTrackingViewer::isRecording() const
{
  return impl_ != nullptr && impl_->recorder.isRecording();
}

bool PathTrackingViewer::showFrame(const PathTrackingVizFrame& frame, const PathTrackingEgoFootprint& ego,
                                   const PathTrackingSignalLimits& limits)
{
  if (impl_ == nullptr || !impl_->open || impl_->window == nullptr)
  {
    return false;
  }

  if (impl_->follow_ego)
  {
    impl_->camera.center_x = frame.ego_x;
    impl_->camera.center_y = frame.ego_y;
  }

  glfwPollEvents();
  if (glfwWindowShouldClose(impl_->window))
  {
    return false;
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::SetNextWindowPos(ImVec2(0.0F, 0.0F), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(360.0F, ImGui::GetIO().DisplaySize.y), ImGuiCond_Always);
  ImGui::Begin("Simulation", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

  ImGui::Text("%s", impl_->title.c_str());
  ImGui::Separator();
  ImGui::Text("step %d / %d", frame.step, frame.total_steps);
  ImGui::Text("t = %.2f s", frame.sim_time);
  ImGui::Text("vel = %.2f m/s", frame.vel);
  ImGui::Text("lat err = %.3f m", frame.lat_err);
  ImGui::Text("baseline = %.1f", frame.baseline_cost);

  ImGui::Separator();
  ImGui::Checkbox("Follow ego", &impl_->follow_ego);
  ImGui::SliderFloat("zoom", &impl_->camera.pixels_per_meter, 8.0F, 48.0F, "%.1f px/m");
  if (!impl_->follow_ego)
  {
    ImGui::DragFloat("pan X", &impl_->camera.center_x, 0.05F);
    ImGui::DragFloat("pan Y", &impl_->camera.center_y, 0.05F);
  }
  if (ImGui::Button("Reset view"))
  {
    impl_->camera.center_x = frame.ego_x;
    impl_->camera.center_y = frame.ego_y;
    impl_->camera.pixels_per_meter = 22.0F;
    impl_->follow_ego = true;
  }

  ImGui::Separator();
  if (!frame.signals.vel.empty())
  {
    ImGui::PlotLines("velocity", frame.signals.vel.data(), static_cast<int>(frame.signals.vel.size()), 0, nullptr,
                     0.0F, limits.v_max, ImVec2(-1.0F, 70.0F));
    ImGui::PlotLines("accel cmd", frame.signals.accel.data(), static_cast<int>(frame.signals.accel.size()), 0,
                     nullptr, limits.accel_min, limits.accel_max, ImVec2(-1.0F, 70.0F));
    ImGui::PlotLines("steer cmd", frame.signals.steer_cmd.data(), static_cast<int>(frame.signals.steer_cmd.size()), 0,
                     nullptr, -limits.steer_max, limits.steer_max, ImVec2(-1.0F, 70.0F));
    ImGui::PlotLines("steer state", frame.signals.steer_state.data(),
                     static_cast<int>(frame.signals.steer_state.size()), 0, nullptr, -limits.steer_max,
                     limits.steer_max, ImVec2(-1.0F, 70.0F));
  }

  ImGui::End();

  const float canvas_x = 370.0F;
  const float canvas_w = ImGui::GetIO().DisplaySize.x - canvas_x;
  const float canvas_h = ImGui::GetIO().DisplaySize.y;

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
    impl_->follow_ego = false;
    const ImVec2 drag = ImGui::GetIO().MouseDelta;
    impl_->camera.center_x -= drag.x / impl_->camera.pixels_per_meter;
    impl_->camera.center_y += drag.y / impl_->camera.pixels_per_meter;
  }
  if (hovered)
  {
    impl_->camera.pixels_per_meter *= (1.0F + 0.1F * ImGui::GetIO().MouseWheel);
    impl_->camera.pixels_per_meter = std::max(8.0F, std::min(48.0F, impl_->camera.pixels_per_meter));
  }

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(canvas_pos,
                           ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                           IM_COL32(252, 252, 252, 255));
  drawStaticScene(draw_list, impl_->static_scene, impl_->camera, canvas_pos, canvas_size);
  drawDynamicScene(draw_list, frame, impl_->camera, canvas_pos, canvas_size, ego);

  ImGui::End();

  ImGui::Render();
  int fb_w = 0;
  int fb_h = 0;
  glfwGetFramebufferSize(impl_->window, &fb_w, &fb_h);
  glViewport(0, 0, fb_w, fb_h);
  glClearColor(0.1F, 0.1F, 0.12F, 1.0F);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  if (impl_->recorder.isRecording())
  {
    const std::size_t nbytes = static_cast<std::size_t>(fb_w) * static_cast<std::size_t>(fb_h) * 4U;
    std::vector<std::uint8_t>& read_buf = impl_->capture_buffers[static_cast<std::size_t>(impl_->capture_index)];
    if (read_buf.size() != nbytes)
    {
      read_buf.resize(nbytes);
    }
    glReadPixels(0, 0, fb_w, fb_h, GL_RGBA, GL_UNSIGNED_BYTE, read_buf.data());
    impl_->recorder.enqueueRgbaFrame(std::move(read_buf), fb_w, fb_h, impl_->video_frames_per_capture);
    impl_->capture_index ^= 1;
  }

  glfwSwapBuffers(impl_->window);

  if (ImGui::IsKeyPressed(ImGuiKey_Escape))
  {
    return false;
  }

  return true;
}

void PathTrackingViewer::close()
{
  if (impl_ == nullptr)
  {
    return;
  }

  impl_->recorder.stop();

  if (impl_->open)
  {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    if (impl_->window != nullptr)
    {
      glfwDestroyWindow(impl_->window);
    }
    glfwTerminate();
  }

  delete impl_;
  impl_ = nullptr;
}

}  // namespace viz
}  // namespace mppi
