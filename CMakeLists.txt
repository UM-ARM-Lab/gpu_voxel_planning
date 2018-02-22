cmake_minimum_required(VERSION 2.8.3)
project(gpu_voxel_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CATKIN_PACKAGES
  roscpp
  victor_hardware_interface
  or_victor
  trajectory_msgs
  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)
FIND_PACKAGE(icl_core REQUIRED )
FIND_PACKAGE(gpu_voxels REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(ompl REQUIRED)

FIND_PACKAGE(orocos_kdl REQUIRED)


# This is a quirk and should be removed in upcoming versions
# If you built GPU Voxels without ROS support, remove this.
# FIND_PACKAGE(orocos_kdl REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")


# Also we have to inherit some Environment definitions required for our base libs:
add_definitions(
  ${icl_core_DEFINITIONS}
  ${gpu_voxels_DEFINITIONS}
  )


add_service_files(
  FILES
  PlanPath.srv
  )

generate_messages(
  DEPENDENCIES
  trajectory_msgs
  victor_hardware_interface
)




###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gpu_voxel_planning
 CATKIN_DEPENDS ${CATKIN_PACKAGES}
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${gpu_voxels_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS} # this should be removed in upcoming versions.

)

add_library(victor_validator src/victor_validator.cpp)

add_library(victor_planning src/victor_planning.cpp)
target_link_libraries(victor_planning
  victor_validator
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${icl_core_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  ${OMPL_LIBRARIES}
  )

add_executable(victor_planning_exec src/victor_planning.cpp)
target_link_libraries(victor_planning_exec
  victor_validator
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${icl_core_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  ${OMPL_LIBRARIES}
  )


add_executable(collision_test src/collision_test.cpp)
target_link_libraries (collision_test
  victor_planning
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${icl_core_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  )

add_executable(validator_timing src/validator_timing.cpp)
target_link_libraries (validator_timing
  victor_planning
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${icl_core_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  )


add_executable(urdf_test src/URDF_Loader_ROS_listener.cpp)
target_link_libraries (urdf_test
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${icl_core_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  )
