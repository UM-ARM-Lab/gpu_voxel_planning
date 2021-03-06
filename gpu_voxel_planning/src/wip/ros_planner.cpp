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

#include "gpu_voxels_victor.hpp"

#include "victor_lbkpiece.hpp"
#include "victor_trrt.hpp"
#include "victor_rrtstar.hpp"

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
std::shared_ptr<GpuVoxelsVictor> victor_model;

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
  victor_model->updateVictorPosition(joint_positions);
  
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
        victor_model->addCollisionPoints(c);
    }
}

bool planPath(gpu_voxel_planning::PlanPath::Request &req,
              gpu_voxel_planning::PlanPath::Response &res)
{
    victor_model->hideSolution();
    
    victor_model->updateVictorPosition(
        victor_model->toRightJointValueMap(vu::jvqToVector(req.start).data()));
    
    ompl::base::PathPtr path = vpln->planPath(vu::jvqToVector(req.start),
                                              vu::jvqToVector(req.goal));

    ompl::geometric::PathGeometric* sol = path->as<ompl::geometric::PathGeometric>();

    
    sol->interpolate();
    std::vector<robot::JointValueMap> joint_maps;
    for(size_t step = 0; step < sol->getStateCount(); ++step)
    {
        const double *values = sol->getState(step)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        robot::JointValueMap state_joint_values = victor_model->toRightJointValueMap(values);
        joint_maps.push_back(state_joint_values);
    }
    victor_model->visualizeSolution(joint_maps);

        
    
    res.path = toTrajectoryMessage(sol);

    return true;
}




int main(int argc, char* argv[])
{
  // signal(SIGINT, ctrlchandler);
  // signal(SIGTERM, killhandler);

  ros::init(argc, argv, "gpu_voxels_ros_planner");
  
  ros::NodeHandle n;
  ros::NodeHandle private_n("~");

  icl_core::logging::initialize(argc, argv);



  std::string arg;
  private_n.getParam("planner", arg);



  victor_model = std::make_shared<GpuVoxelsVictor>();
  
  if(arg.length() == 0)
  {
      ROS_INFO("Using Default LBKPiece Planner");
      vpln = std::make_shared<gpu_voxels_planner::VictorLBKPiece>(victor_model);
  }
  if(arg == "lbkpiece")
  {
      ROS_INFO("Using LBKPiece Planner");
      vpln = std::make_shared<gpu_voxels_planner::VictorLBKPiece>(victor_model);
  }
  if(arg == "trrt")
  {
      ROS_INFO("Using TRRT Planner");
      vpln = std::make_shared<gpu_voxels_planner::VictorTrrt>(victor_model);
  }
  if(arg == "rrtstar")
  {
      ROS_INFO("Using RRTstar Planner");
      vpln = std::make_shared<gpu_voxels_planner::VictorRrtStar>(victor_model);
  }

  



  ros::Subscriber sub1 = n.subscribe("joint_states", 1, jointStateCallback);
  ros::Subscriber collision_sub = n.subscribe("right_arm/motion_status", 1, checkCollisionCallback);
  ros::ServiceServer planing_srv = n.advertiseService("plan_path", planPath);

  //ONly valid in simulation
  ros::ServiceClient tmp = n.serviceClient<gazebo_victor::GetContactLinks>("simulation/right_arm/get_last_contact_links");
  col_links_client = &tmp;

  victor_model->hideSolution();

  while(ros::ok())
  {
    ros::spinOnce();
    victor_model->doVis();

    usleep(30000);
  }

  std::cout << "Exiting Collision Test\n";
  vpln->vv_ptr->~VictorValidator();
  exit(EXIT_SUCCESS);
}
