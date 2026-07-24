cmake_minimum_required(VERSION 3.13)
project(dv_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(
  DV_CONTROL_USE_TUSTIN_DISCRETIZATION
  "Use Tustin instead of exact zero-order-hold discretization in LTV MPC"
  OFF
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  visualization_msgs
  dv_interfaces
  roslib
)

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros
    visualization_msgs dv_interfaces roslib
)

add_executable(dv_control_node
  src/main.cpp
  src/wrapper.cpp
  src/math_utilis.cpp
  src/longitudal_utils.cpp
  src/mpc_ltv_unbouded.cpp
  src/control_types.cpp
)

add_dependencies(
  dv_control_node
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(dv_control_node PRIVATE
  include
  ../common/include
  ${catkin_INCLUDE_DIRS}
)

target_compile_options(dv_control_node PRIVATE
  -Wall
  -Wextra
  $<$<CONFIG:Release>:-O3>
)

if(DV_CONTROL_USE_TUSTIN_DISCRETIZATION)
  target_compile_definitions(
    dv_control_node
    PRIVATE DV_CONTROL_USE_TUSTIN_DISCRETIZATION=1
  )
endif()

target_link_libraries(dv_control_node PRIVATE
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  nlohmann_json::nlohmann_json
)

install(
  TARGETS dv_control_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
