cmake_minimum_required(VERSION 3.16)
project(minicollide CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(MC_BUILD_DEMOS "Build raylib demos (fetches raylib from GitHub)" ON)

find_package(Threads REQUIRED)

add_library(minicollide STATIC
  src/collision/collide.cpp
  src/collision/gjk.cpp
  src/solver/contact_solver.cpp
  src/solver/joints.cpp
  src/core/world.cpp
)
target_include_directories(minicollide PUBLIC src)
target_compile_options(minicollide PRIVATE -Wall -Wextra)
target_link_libraries(minicollide PUBLIC Threads::Threads)

enable_testing()
add_executable(mc_tests tests/test_main.cpp)
target_link_libraries(mc_tests PRIVATE minicollide)
target_compile_options(mc_tests PRIVATE -Wall -Wextra)
add_test(NAME unit COMMAND mc_tests)

# Benchmarks: compiled with AVX2+FMA; the kernel checks CPU support at runtime.
add_executable(mc_bench bench/bench_main.cpp)
target_link_libraries(mc_bench PRIVATE minicollide)
target_compile_options(mc_bench PRIVATE -Wall -Wextra -mavx2 -mfma)

# Networked demo. The server is always headless; the client gets a raylib
# viewer when demos are enabled and still supports headless --probe otherwise.
add_executable(net_server net/net_server.cpp)
target_link_libraries(net_server PRIVATE minicollide)
target_include_directories(net_server PRIVATE net)

add_executable(net_client net/net_client.cpp)
target_link_libraries(net_client PRIVATE minicollide)
target_include_directories(net_client PRIVATE net)

if(MC_BUILD_DEMOS)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(raylib)

  foreach(demo demo_stack demo_chain demo_pile demo_fort demo_molecules)
    add_executable(${demo} demos/${demo}.cpp)
    target_link_libraries(${demo} PRIVATE minicollide raylib)
    target_include_directories(${demo} PRIVATE demos)
  endforeach()

  target_link_libraries(net_client PRIVATE raylib)
  target_include_directories(net_client PRIVATE demos)
  target_compile_definitions(net_client PRIVATE MC_HAVE_RAYLIB)
endif()
