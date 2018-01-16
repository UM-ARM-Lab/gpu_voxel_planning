// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// This file is part of the GPU Voxels Software Library.
//
// This program is free software licensed under the CDDL
// (COMMON DEVELOPMENT AND DISTRIBUTION LICENSE Version 1.0).
// You can find a copy of this license in LICENSE.txt in the top
// directory of the source code.
//
// © Copyright 2014 FZI Forschungszentrum Informatik, Karlsruhe, Germany
//
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Andreas Hermann
 * \date    2015-05-27
 *
 * This little example program shows how to load and animate your ROS Robot.
 * Use "binvox" (external Software from Patrick Min)
 * to voxelize your meshes beforehand!
 *
 */
//----------------------------------------------------------------------
#include "collision_detection.hpp"

#include "victor_planning.hpp"

#include <cstdlib>
#include <signal.h>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>


using namespace gpu_voxels;
namespace bfs = boost::filesystem;

std::vector<std::string> left_arm_names{"victor_left_arm_joint_1",  "victor_left_arm_joint_2",  "victor_left_arm_joint_3",  "victor_left_arm_joint_4",  "victor_left_arm_joint_5",  "victor_left_arm_joint_6",  "victor_left_arm_joint_7"};

std::vector<std::string> right_arm_names{"victor_right_arm_joint_1", "victor_right_arm_joint_2", "victor_right_arm_joint_3", "victor_right_arm_joint_4", "victor_right_arm_joint_5", "victor_right_arm_joint_6", "victor_right_arm_joint_7"};


std::shared_ptr<gpu_voxels_planner::VictorPlanner> vpln;


void ctrlchandler(int)
{
    vpln->vv_ptr->~VictorValidator();
    ros::shutdown();
}
void killhandler(int)
{
    ros::shutdown();
}


void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

  robot::JointValueMap joint_positions;
  for(size_t i = 0; i < msg->name.size(); i++)
  {
    joint_positions[msg->name[i]] = msg->position[i];
  }
  std::cout << "Callback\n";
  vpln->vv_ptr->setVictorPosition(joint_positions);
  
}

void checkCollisionCallback(victor_hardware_interface::MotionStatus::ConstPtr motion_msg)
{
    CollisionInformation c = checkCollision(motion_msg);
    if(c.collision)
    {
        vpln->vv_ptr->addCollisionPoints(c);
    }
}


int main(int argc, char* argv[])
{
  // signal(SIGINT, ctrlchandler);
  // signal(SIGTERM, killhandler);

  ros::init(argc, argv, "gpu_voxels");
  

  icl_core::logging::initialize(argc, argv);

  vpln = std::make_shared<gpu_voxels_planner::VictorPlanner>();



  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("joint_states", 1, jointStateCallback);
  ros::Subscriber collision_sub = n.subscribe("right_arm/motion_status", 1, checkCollisionCallback);
  
  

  while(ros::ok())
  {
    ros::spinOnce();
    vpln->vv_ptr->doVis();

    usleep(30000);
  }

  std::cout << "Exiting Collision Test\n";
  vpln->vv_ptr->~VictorValidator();
  exit(EXIT_SUCCESS);
}
