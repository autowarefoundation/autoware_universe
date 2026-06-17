include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_Declare(glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.6
)
FetchContent_MakeAvailable(imgui)

find_package(OpenGL REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)

add_library(mppi_viz_gl STATIC
  collision_viewer.cpp
  framebuffer_recorder.cpp
  path_tracking_viewer.cpp
  rollout_viz_gpu.cu
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)

target_include_directories(mppi_viz_gl PUBLIC
  ${CMAKE_SOURCE_DIR}/include
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)

target_link_libraries(mppi_viz_gl PUBLIC
  mppi_path_tracking
  MPPI::MPPI
  OpenGL::GL
  glfw
  ${OpenCV_LIBS}
)

set_target_properties(mppi_viz_gl PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_include_directories(mppi_viz_gl PUBLIC ${OpenCV_INCLUDE_DIRS})

set_target_properties(mppi_viz_gl PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
