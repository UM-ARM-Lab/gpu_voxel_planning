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
  arm_pointcloud_utilities
  graph_planner
  moveit_core
  moveit_visual_tools
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


add_message_files(DIRECTORY msg FILES
  CollisionInformation.msg
  )

add_service_files(
  FILES
  PlanPath.srv
  AttemptPath.srv
  AttemptPathStart.srv
  AttemptPathResult.srv
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
  include/${PROJECT_NAME}/old
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${gpu_voxels_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS} # this should be removed in upcoming versions.
  )


add_library(path_smoothing src/ompl_modifications/path_smoothing.cpp
  ${OMPL_LIBRARIES}
  )

add_library(lazyrrtf src/ompl_modifications/lazyrrt_fullpath.cpp)
target_link_libraries(lazyrrtf
  ${OMPL_LIBRARIES}
  )

add_library(custom_rrtstar src/ompl_modifications/custom_rrtstar.cpp)
target_link_libraries(custom_rrtstar
  ${OMPL_LIBRARIES}
  )

add_library(flrrtc src/ompl_modifications/fullpath_lrrtconnect.cpp)
target_link_libraries(flrrtc
  ${OMPL_LIBRARIES}
  )

add_library(crrtc src/ompl_modifications/custom_rrtconnect.cpp)
target_link_libraries(crrtc
  ${OMPL_LIBRARIES}
  )

add_library(costrrtc src/ompl_modifications/cost_rrtconnect.cpp)
target_link_libraries(costrrtc
  ${OMPL_LIBRARIES}
  path_smoothing
  )




# add_executable(box_world src/box_world.cpp)
# target_link_libraries(box_world
#   custom_rrtstar
#   flrrtc
#   ${EXTERN_LIBS}
#   lazyrrtf

#   )

# add_library(box_world_lib src/box_world.cpp)
# target_link_libraries(box_world_lib
#   custom_rrtstar
#   ${EXTERN_LIBS}
#   lazyrrtf

#   )

# add_executable(box_world_recorder src/box_world_recorder.cpp)
# target_link_libraries(box_world_recorder
#   ${EXTERN_LIBS}
#   box_world_lib
#   )


add_library(victor_model
  src/old/gpu_voxels_victor.cpp
  src/old/worlds.cpp
  src/path_utils.cpp)
add_dependencies(victor_model gpu_voxel_planning_generate_messages_cpp)
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


add_library(victor_validator src/old/victor_validator.cpp)
target_link_libraries(victor_validator
  victor_model)

add_library(victor_planning src/old/victor_planning.cpp)
target_link_libraries(victor_planning
  victor_validator
  victor_model
  ${EXTERN_LIBS}
  lazyrrtf
  crrtc
  costrrtc
  custom_rrtstar
  )


add_library(victor_local_controller src/old/victor_local_controller.cpp)
target_link_libraries(victor_local_controller
  # victor_validator
  victor_model                 
  ${EXTERN_LIBS}
  )

add_executable(wip_victor_sim src/old/wip_victor_sim.cpp)
target_link_libraries(wip_victor_sim
  ${EXTERN_LIBS}
  lazyrrtf
  victor_model
  victor_planning
  victor_local_controller
  victor_validator
  )


add_library(victor_halton_roadmap
  src/victor_halton_roadmap.cpp)



set(Planning_SOURCES
  src/scenario_tester.cpp
  src/graph_search_strategies.cpp
  src/path_utils.cpp
  )

add_library(planning_library
  ${Planning_SOURCES}
  )
# add_dependencies(planning_library
#   ${EXTERN_LIBS}
#   ${catkin_EXPORTED_TARGETS}
#   )
target_link_libraries(planning_library
  ${EXTERN_LIBS}
  )


add_executable(wip_prob_map
  src/wip/wip_prob_map.cpp
  )
# add_dependencies(wip_prob_map
#   planning_library
#   ${EXTERN_LIBS}
#   ${catkin_EXPORTED_TARGETS}
#   )
target_link_libraries(wip_prob_map
  ${EXTERN_LIBS}
  planning_library
  victor_halton_roadmap
  )

add_executable(simulation_timings
  src/simulation_timings.cpp
  )
target_link_libraries(simulation_timings
  ${EXTERN_LIBS}
  planning_library
  victor_halton_roadmap
  )


add_executable(wip_roadmap
  src/wip_roadmap.cpp)

target_link_libraries(wip_roadmap
  ${EXTERN_LIBS}
  planning_library
  victor_halton_roadmap
  )

# add_executable(victor_sim_timings src/old/victor_sim_timings.cpp)
# target_link_libraries(victor_sim_timings
#   ${EXTERN_LIBS}
#   lazyrrtf
#   victor_model
#   victor_planning
#   victor_validator
#   )



# add_library(wip_optimization_objective src/old/wip_optimization_objective.cpp)
# target_link_libraries(wip_optimization_objective
#   victor_model
#   # victor_planning_base
#   )



# add_executable(ros_planning_node src/old/ros_planner.cpp)
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

# add_executable(validator_timing src/old/validator_timing.cpp)
# target_link_libraries (validator_timing
#   victor_planning_lbkpiece
#   ${catkin_LIBRARIES}
#   ${Boost_SYSTEM_LIBRARY}
#   ${icl_core_LIBRARIES}
#   ${gpu_voxels_LIBRARIES}
#   )


# add_executable(urdf_test src/URDF_Loader_ROS_listener.cpp)
# target_link_libraries (urdf_test
#   ${EXTERN_LIBS}
#   )


# add_executable(collision_link_node src/wip/collision_link_node.cpp)
# target_link_libraries(collision_link_node
#   ${catkin_LIBRARIES}
#   )

# add_executable(wip_victor_real src/old/wip_victor_real.cpp)
# target_link_libraries(wip_victor_real
#   ${EXTERN_LIBS}
#   lazyrrtf
#   victor_model
#   victor_planning
#   victor_validator
#   )

# add_executable(victor_real_timings src/old/victor_real_timings.cpp)
# target_link_libraries(victor_real_timings
#   ${EXTERN_LIBS}
#   lazyrrtf
#   victor_model
#   victor_planning
#   victor_validator
#   )



###########
## Tests ##
###########

# To build tests, install google's testing framework
# https://github.com/google/googletest/


# catkin_add_gtest(test_gpu_voxels_victor tests/gpu_voxels_victor_test.cpp)
# target_link_libraries(test_gpu_voxels_victor
#   victor_model
#   ${EXTERN_LIBS}
#   ${GTEST_LIBRARIES}
#   )

# catkin_add_gtest(test_validators tests/validators_test.cpp)
# target_link_libraries(test_validators
#   victor_model
#   victor_validator
#   ${EXTERN_LIBS}
#   ${GTEST_LIBRARIES}
#   )

# catkin_add_gtest(test_planners tests/planners_test.cpp)
# target_link_libraries(test_planners
#   victor_model
#   victor_validator
#   victor_planning
#   ${EXTERN_LIBS}
#   ${GTEST_LIBRARIES}
#   )

catkin_add_gtest(test_path_utils tests/test_path_utils.cpp)
target_link_libraries(test_path_utils
  victor_model
  victor_validator
  victor_planning
  ${EXTERN_LIBS}
  ${GTEST_LIBRARIES}
  )


catkin_add_gtest(test_GVP
  tests/GVP_test.cpp
  src/path_utils.cpp)
target_link_libraries(test_GVP
  ${EXTERN_LIBS}
)
