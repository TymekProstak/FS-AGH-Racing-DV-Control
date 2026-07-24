cmake_minimum_required(VERSION 3.13)
project(dv_acc_launch_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  visualization_msgs
  dv_interfaces
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp roslib std_msgs geometry_msgs nav_msgs tf2
    visualization_msgs dv_interfaces
)

add_executable(dv_acc_launch_control_node
  src/main.cpp
  src/acc_types.cpp
  src/math_utils.cpp
  src/wrapper.cpp
  src/ltv_mpc_unbounded.cpp
  src/longitudal_control/longitudal_utils.cpp
)

add_dependencies(
  dv_acc_launch_control_node
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(dv_acc_launch_control_node PRIVATE
  include
  include/longitudal_control
  ../common/include
  ${catkin_INCLUDE_DIRS}
)

target_compile_options(dv_acc_launch_control_node PRIVATE
  -Wall
  -Wextra
  $<$<CONFIG:Release>:-O3>
)

target_link_libraries(dv_acc_launch_control_node PRIVATE
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

install(
  TARGETS dv_acc_launch_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/longitudal_control/launch_brake_maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include/longitudal_control
)
