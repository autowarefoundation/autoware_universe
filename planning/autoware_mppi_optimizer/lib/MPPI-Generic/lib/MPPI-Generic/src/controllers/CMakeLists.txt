# Needed for methods to be built into libraries in Release mode
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler=-fno-guess-branch-probability")

add_subdirectory(cartpole)
add_subdirectory(autorally)
add_subdirectory(double_integrator)
add_subdirectory(quadrotor)