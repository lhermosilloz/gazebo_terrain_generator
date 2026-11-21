# GroundFriction system plugin for Gazebo Harmonic (gz-sim 8).
#
# Build and install into ~/.gz/sim/plugins, which gz-sim searches by default:
#   cmake -S gazebo_plugins/ground_friction -B build/ground_friction
#   cmake --build build/ground_friction
#   cmake --install build/ground_friction
#
# Or skip the install and point gz-sim at the build directory instead:
#   export GZ_SIM_SYSTEM_PLUGIN_PATH=$PWD/build/ground_friction
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(GroundFriction LANGUAGES CXX)

find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-physics7 REQUIRED)

add_library(GroundFriction SHARED GroundFriction.cc)
target_compile_features(GroundFriction PRIVATE cxx_std_17)
target_link_libraries(GroundFriction
  PRIVATE
    gz-plugin2::register
    gz-sim8::gz-sim8
    gz-physics7::gz-physics7)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.gz/sim" CACHE PATH "Install prefix" FORCE)
endif()
install(TARGETS GroundFriction LIBRARY DESTINATION plugins)
