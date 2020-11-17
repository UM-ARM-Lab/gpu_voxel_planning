cmake_minimum_required(VERSION 2.8.3)
project(gpu_voxel_planning_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

set(CATKIN_PACKAGES
  victor_hardware_interface_msgs
  trajectory_msgs
  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)




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
  victor_hardware_interface_msgs
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
 CATKIN_DEPENDS ${CATKIN_PACKAGES}
)

###########
## Build ##
###########


