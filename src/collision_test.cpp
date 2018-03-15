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

#include "victor_hardware_interface/victor_utils.hpp"

#include <cstdlib>
#include <signal.h>

#include <gpu_voxels/GpuVoxels.h>
#include <gpu_voxels/helpers/MetaPointCloud.h>
#include <gpu_voxels/robot/urdf_robot/urdf_robot.h>
#include <gpu_voxels/logging/logging_gpu_voxels.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <gazebo_victor/GetContactLinks.h>

#include "gpu_voxel_planning/PlanPath.h"


using namespace gpu_voxels;
namespace bfs = boost::filesystem;
namespace vu = victor_utils;

std::vector<std::string> left_arm_names{"victor_left_arm_joint_1",  "victor_left_arm_joint_2",  "victor_left_arm_joint_3",  "victor_left_arm_joint_4",  "victor_left_arm_joint_5",  "victor_left_arm_joint_6",  "victor_left_arm_joint_7"};

std::vector<std::string> right_arm_names{"victor_right_arm_joint_1", "victor_right_arm_joint_2", "victor_right_arm_joint_3", "victor_right_arm_joint_4", "victor_right_arm_joint_5", "victor_right_arm_joint_6", "victor_right_arm_joint_7"};


std::shared_ptr<gpu_voxels_planner::VictorPlanner> vpln;

ros::ServiceClient* col_links_client;

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
  // std::cout << "Callback\n";
  vpln->vv_ptr->setVictorPosition(joint_positions);
  
}

trajectory_msgs::JointTrajectory toTrajectoryMessage(ompl::geometric::PathGeometric* sol)
{
    trajectory_msgs::JointTrajectory path;
    size_t num_points = sol->getStateCount();
    path.points.resize(num_points);
    for(size_t step = 0; step < num_points; step++)
    {
        const double *values = sol->getState(step)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        std::vector<double> angles(7);
        for(size_t i = 0; i < angles.size(); i++)
        {
            angles[i] = values[i];
        }
        path.points[step].positions = angles;
    }

    return path;
}

void checkCollisionCallback(victor_hardware_interface::MotionStatus::ConstPtr motion_msg)
{
    CollisionInformation c = checkCollision(motion_msg);
    if(c.collision)
    {
        gazebo_victor::GetContactLinks srv_msg;
        col_links_client->call(srv_msg);
        c.links_in_contact = srv_msg.response.links_in_contact;
        vpln->vv_ptr->addCollisionPoints(c);
    }
}

bool planPath(gpu_voxel_planning::PlanPath::Request &req,
              gpu_voxel_planning::PlanPath::Response &res)
{
    ompl::base::PathPtr path = vpln->planPath(vu::jvqToVector(req.start),
                                              vu::jvqToVector(req.goal));
    ompl::geometric::PathGeometric* sol = path->as<ompl::geometric::PathGeometric>();
    res.path = toTrajectoryMessage(sol);
    return true;
}




int main(int argc, char* argv[])
{
  // signal(SIGINT, ctrlchandler);
  // signal(SIGTERM, killhandler);

  ros::init(argc, argv, "gpu_voxels");
  

  icl_core::logging::initialize(argc, argv);

  vpln = std::make_shared<gpu_voxels_planner::VictorPlanner>();
  // vpln->vv_ptr->moveObstacle();


  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("joint_states", 1, jointStateCallback);
  ros::Subscriber collision_sub = n.subscribe("right_arm/motion_status", 1, checkCollisionCallback);
  ros::ServiceServer planing_srv = n.advertiseService("plan_path", planPath);

  //ONly valid in simulation
  ros::ServiceClient tmp = n.serviceClient<gazebo_victor::GetContactLinks>("simulation/right_arm/get_last_contact_links");
  col_links_client = &tmp;

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
