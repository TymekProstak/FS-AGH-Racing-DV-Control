cmake_minimum_required(VERSION 3.0.2)
project(dv_interfaces)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_message_files(
  FILES
    AccDebug_lat.msg
    AccDebug_long.msg
    Acc_handling_info.msg
    Control.msg
    ControlDebug_lat.msg
    ControlDebug_long.msg
    Controlhandling_info.msg
    DV_board.msg
    EmergencyAutoX.msg
    EmergencyInfo.msg
    Imu.msg
    OdomDebug.msg
    Pos.msg
    SkidpadDebug_lat.msg
    SkidpadDebug_long.msg
    Skidpad_handling_info.msg
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    std_msgs
)
