cmake_minimum_required(VERSION 3.3)
project(choreonoid_rosplugin)

if(NOT CMAKE_BUILD_TYPE)
  set(
    CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  image_transport
  choreonoid
  realtime_tools
  control_toolbox
  controller_manager
  pr2_controller_manager
  hardware_interface
  transmission_interface
  joint_limits_interface
  joint_trajectory_controller
  joint_state_controller
  angles
  pluginlib
  urdf
  )

add_definitions(${CHOREONOID_DEFINITIONS})

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${CHOREONOID_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  )

add_subdirectory(src)
