cmake_minimum_required(VERSION 3.25)
project(agent_steering_mcu_host LANGUAGES CXX)

# Host build of the MCU (float) simulation core, to prove it still has zero failures before flashing.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)
include_directories(../sim)

add_executable(suite suite.cpp)
# -Wdouble-promotion is clean in the per-frame paths; world generation stays double on purpose (exact rng).

add_executable(bench bench.cpp)
add_executable(suite_double suite.cpp)
target_compile_definitions(suite_double PRIVATE SIM_REAL_DOUBLE)
