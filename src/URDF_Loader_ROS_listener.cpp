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
#include "collision_detection.hpp"

using namespace gpu_voxels;
namespace bfs = boost::filesystem;

std::vector<std::string> left_arm_names{"victor_left_arm_joint_1",  "victor_left_arm_joint_2",  "victor_left_arm_joint_3",  "victor_left_arm_joint_4",  "victor_left_arm_joint_5",  "victor_left_arm_joint_6",  "victor_left_arm_joint_7"};

std::vector<std::string> right_arm_names{"victor_right_arm_joint_1", "victor_right_arm_joint_2", "victor_right_arm_joint_3", "victor_right_arm_joint_4", "victor_right_arm_joint_5", "victor_right_arm_joint_6", "victor_right_arm_joint_7"};


GpuVoxelsSharedPtr gvl;

void ctrlchandler(int)
{
  ros::shutdown();
}
void killhandler(int)
{
  ros::shutdown();
}

robot::JointValueMap myRobotJointValues;
Vector3f object_position(0.1, 0.15, 0.15);

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  //std::cout << "Got JointStateMessage" << std::endl;
  gvl->clearMap("victorCurrentVoxels");

  for(size_t i = 0; i < msg->name.size(); i++)
  {
    myRobotJointValues[msg->name[i]] = msg->position[i];
  }
  // update the robot joints:
  gvl->setRobotConfiguration("victor", myRobotJointValues);
  // insert the robot into the map:
  // gvl->insertRobotIntoMap("victor", "victorCurrentVoxels", eBVM_OCCUPIED);
  gvl->insertRobotIntoMap("victor", "victorCurrentVoxels", gpu_voxels::eBVM_COLLISION);


  //Equivalent? ways to insert into swept volume map
  gvl->insertRobotIntoMap("victor", "victorSweptVoxels", eBVM_SWEPT_VOLUME_START);
  // BitVoxelMeaning swept = eBVM_SWEPT_VOLUME_START;
  // gvl->getMap("victorSweptVoxels")->merge(gvl->getMap("victorCurrentVoxels"), Vector3f(), &swept);
  
  gpu_voxels::GpuVoxelsMapSharedPtr obstacles_ptr = gvl->getMap("possibleObstacles");
  voxellist::BitVectorVoxelList* obstacles = obstacles_ptr->as<voxellist::BitVectorVoxelList>();
  
  obstacles->subtract(gvl->getMap("victorSweptVoxels")->as<voxellist::BitVectorVoxelList>(), Vector3f());
  
}

void checkCollisionCallback(victor_hardware_interface::MotionStatus::ConstPtr motion_msg)
{
    CollisionInformation c = checkCollision(motion_msg);
    if(c.collision)
    {
        robot::JointValueMap robot_joints;
        for(size_t i = 0; i < c.joints.size(); i++)
        {
            myRobotJointValues[right_arm_names[i]] = c.joints[i] + 0.05 * c.dirs[i];
            // std::cout << 0.05 * c.dirs[i] << ", ";
            std::cout << c.joints[i] << ", ";
        }
        std::cout << "\n";
        
        voxellist::BitVectorVoxelList* obstacles = gvl->getMap("possibleObstacles")->
            as<voxellist::BitVectorVoxelList>();
        gvl->setRobotConfiguration("victor", myRobotJointValues);
        gvl->insertRobotIntoMap("victor", "possibleObstacles", eBVM_OCCUPIED);
        obstacles->subtract(gvl->getMap("victorSweptVoxels")->as<voxellist::BitVectorVoxelList>(), Vector3f());

    }
}

void obstaclePoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
  std::cout << "Got PoseMessage" << std::endl;
  gvl->clearMap("myObjectVoxelmap");

  object_position.x = msg->position.x;
  object_position.y = msg->position.y;
  object_position.z = msg->position.z;

  gvl->insertPointCloudFromFile("myObjectVoxelmap", "hals_vereinfacht.binvox", true,
                                eBVM_OCCUPIED, false, object_position, 0.3);
}

int main(int argc, char* argv[])
{
  signal(SIGINT, ctrlchandler);
  signal(SIGTERM, killhandler);

  ros::init(argc, argv, "gpu_voxels");
  

  icl_core::logging::initialize(argc, argv);

  /*
   * First, we generate an API class, which defines the
   * volume of our space and the resolution.
   * Be careful here! The size is limited by the memory
   * of your GPU. Even if an empty Octree is small, a
   * Voxelmap will always require the full memory.
   */
  gvl = GpuVoxels::getInstance();
  gvl->initialize(300, 200, 200, 0.01); // ==> 200 Voxels, each one is 1 mm in size so the map represents 20x20x20 centimeter

  // Add a map:
  // gvl->addMap(MT_PROBAB_VOXELMAP, "myObjectVoxelmap");
  
  gvl->addMap(MT_BITVECTOR_VOXELLIST, "victorCurrentVoxels");
  gvl->addMap(MT_BITVECTOR_VOXELLIST, "victorSweptVoxels");
  // gvl->addMap(MT_PROBAB_VOXELMAP, "possibleObstacles");
    // 3D-Array of probabilistic Voxels (identified by their Voxelmap-like Pointer adress)
    //that hold a Probability
  gvl->addMap(MT_BITVECTOR_VOXELLIST, "possibleObstacles");
  gvl->addMap(MT_BITVECTOR_VOXELLIST, "testMap");

  
  /*
   * At this point we can add geometries to the maps in different ways.
   *
   * We can add a simple box.
   */
  gpu_voxels::Vector3f center_box1_min(2.0,1.9,1.2);
  gpu_voxels::Vector3f center_box1_max(2.1,2.0,1.4);
  gvl->insertBoxIntoMap(center_box1_min,center_box1_max, "possibleObstacles", gpu_voxels::eBVM_OCCUPIED);
  // gvl->insertBoxIntoMap(center_box1_min,center_box1_max, "possibleObstacles", gpu_voxels::eBVM_UNKNOWN);
  // gpu_voxels::GpuVoxelsMapSharedPtr map = gvl->getMap("possibleObstacles");


  
  // And a robot, generated from a ROS URDF file:
  gvl->addRobot("victor", "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor.urdf", false);  

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("joint_states", 1, jointStateCallback);
  ros::Subscriber collision_sub = n.subscribe("right_arm/motion_status", 1, checkCollisionCallback);
  


  // update the robot joints:
  gvl->setRobotConfiguration("victor", myRobotJointValues);
  // insert the robot into the map:
  gvl->insertRobotIntoMap("victor", "victorCurrentVoxels", eBVM_OCCUPIED);
  /*
   * Now we start the main loop, that will read ROS messages and update the Robot.
   */
  BitVectorVoxel bits_in_collision;
  size_t num_colls;
  while(ros::ok())
  {
    ros::spinOnce();

    // num_colls = gvl->getMap("victorCurrentVoxels")->as<voxellist::BitVectorVoxelList>()->collideWithTypes(gvl->getMap("myObjectVoxelmap")->as<voxelmap::ProbVoxelMap>(), bits_in_collision);

    // std::cout << "Detected " << num_colls << " collisiosn " << std::endl;
    //std::cout << "with bits \n" << bits_in_collision << std::endl;

    // tell the visualier that the map has changed:
    gvl->visualizeMap("victorCurrentVoxels");
    gvl->visualizeMap("victorSweptVoxels");
    gvl->visualizeMap("possibleObstacles");
    // gvl->visualizeMap("testMap");
    
    // gvl->visualizeMap("myObjectVoxelmap");

    usleep(30000);
  }

  gvl.reset();
  exit(EXIT_SUCCESS);
}
