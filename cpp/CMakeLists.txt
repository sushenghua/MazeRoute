cmake_minimum_required(VERSION 3.25)
project(agent_steering_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

# Headless simulation core (header-only) + two drivers that mirror tests/run.js and tests/bench.js.
add_executable(suite src/suite.cpp)
add_executable(bench src/bench.cpp)
