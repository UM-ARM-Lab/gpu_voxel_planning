cmake_minimum_required(VERSION 2.8.3)
project(gpu_voxel_planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CATKIN_PACKAGES
  roscpp
  victor_hardware_interface
  or_victor
  trajectory_msgs
  arc_utilities
  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation gazebo_victor)
FIND_PACKAGE(icl_core REQUIRED )
FIND_PACKAGE(gpu_voxels REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system REQUIRED)
FIND_PACKAGE(CUDA REQUIRED)
FIND_PACKAGE(ompl REQUIRED)
FIND_PACKAGE(orocos_kdl REQUIRED)

find_package(GTest)




# This is a quirk and should be removed in upcoming versions
# If you built GPU Voxels without ROS support, remove this.
# FIND_PACKAGE(orocos_kdl REQUIRED)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -g")


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
 LIBRARIES gpu_voxel_planning
 CATKIN_DEPENDS ${CATKIN_PACKAGES}
 DEPENDS arc_utilities
)

###########
## Build ##
###########


set(EXTERN_LIBS
  ${catkin_LIBRARIES}
  ${Boost_SYSTEM_LIBRARY}
  ${icl_core_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  ${OMPL_LIBRARIES}
  )


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


add_library(lazyrrtf src/lazyrrt_fullpath.cpp)
target_link_libraries(lazyrrtf
  ${OMPL_LIBRARIES}
  )

add_library(custom_rrtstar src/custom_rrtstar.cpp)
target_link_libraries(custom_rrtstar
  ${OMPL_LIBRARIES}
  )

add_library(flrrtc src/fullpath_lrrtconnect.cpp)
target_link_libraries(flrrtc
  ${OMPL_LIBRARIES}
  )

add_library(crrtc src/custom_rrtconnect.cpp)
target_link_libraries(crrtc
  ${OMPL_LIBRARIES}
  )




add_executable(box_world src/box_world.cpp)
target_link_libraries(box_world
  custom_rrtstar
  flrrtc
  ${EXTERN_LIBS}
  lazyrrtf

  )

add_library(box_world_lib src/box_world.cpp)
target_link_libraries(box_world_lib
  custom_rrtstar
  ${EXTERN_LIBS}
  lazyrrtf

  )

add_executable(box_world_recorder src/box_world_recorder.cpp)
target_link_libraries(box_world_recorder
  ${EXTERN_LIBS}
  box_world_lib
  )

add_library(victor_model src/gpu_voxels_victor.cpp)
target_link_libraries(victor_model
  ${catkin_LIBRARIES}
  ${gpu_voxels_LIBRARIES}
  )



# add_executable(victor_sim_world src/victor_sim_world.cpp)
# target_link_libraries(victor_sim_world
#   ${catkin_LIBRARIES}
#   ${Boost_SYSTEM_LIBRARY}
#   ${icl_core_LIBRARIES}
#   ${gpu_voxels_LIBRARIES}
#   ${OMPL_LIBRARIES}
#   lazyrrtf
#   victor_model
#   )


add_library(victor_validator src/victor_validator.cpp)
target_link_libraries(victor_validator
  victor_model)

add_library(victor_planning src/victor_planning.cpp)
target_link_libraries(victor_planning
  victor_validator
  victor_model
  ${EXTERN_LIBS}
  lazyrrtf
  crrtc
  )

add_executable(wip_victor_sim src/wip_victor_sim.cpp)
target_link_libraries(wip_victor_sim
  ${EXTERN_LIBS}
  lazyrrtf
  victor_model
  victor_planning
  victor_validator
  )

add_executable(victor_sim_timings src/victor_sim_timings.cpp)
target_link_libraries(victor_sim_timings
  ${EXTERN_LIBS}
  lazyrrtf
  victor_model
  victor_planning
  victor_validator
  )



# add_library(wip_optimization_objective src/wip_optimization_objective.cpp)
# target_link_libraries(wip_optimization_objective
#   victor_model
#   # victor_planning_base
#   )



# add_executable(ros_planning_node src/ros_planner.cpp)
# target_link_libraries (ros_planning_node
#   victor_lbkpiece
#   victor_trrt
#   victor_rrtstar
#   victor_model
#   ${catkin_LIBRARIES}
#   ${Boost_SYSTEM_LIBRARY}
#   ${icl_core_LIBRARIES}
#   ${gpu_voxels_LIBRARIES}
#   )

# add_executable(validator_timing src/validator_timing.cpp)
# target_link_libraries (validator_timing
#   victor_planning_lbkpiece
#   ${catkin_LIBRARIES}
#   ${Boost_SYSTEM_LIBRARY}
#   ${icl_core_LIBRARIES}
#   ${gpu_voxels_LIBRARIES}
#   )


add_executable(urdf_test src/URDF_Loader_ROS_listener.cpp)
target_link_libraries (urdf_test
  ${EXTERN_LIBS}
  )



###########
## Tests ##
###########

# To build tests, install google's testing framework
# https://github.com/google/googletest/

IF(GTEST_FOUND)
  MESSAGE("-----------------------")
  MESSAGE("${ARC_UTILITIES_LIBRARIES}")
  MESSAGE("-----------------------")
  include_directories(${GTEST_INCLUDE_DIRS})
  add_executable(test_gpu_voxels_victor tests/gpu_voxels_victor_test.cpp)
  target_link_libraries(test_gpu_voxels_victor
    victor_model
    ${EXTERN_LIBS}
    ${GTEST_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    )

  add_executable(test_validators tests/validators_test.cpp)
  target_link_libraries(test_validators
    victor_model
    victor_validator
    ${EXTERN_LIBS}
    ${GTEST_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}

    )
ENDIF(GTEST_FOUND)
