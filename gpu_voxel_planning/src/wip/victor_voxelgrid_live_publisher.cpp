//
// Created by bsaund on 3/8/21.
//

#include <ros/init.h>
#include "gpu_voxel_planning/robot/robot_model.hpp"
#include "gpu_voxel_planning/ros_interface/ros_interface.hpp"

using namespace GVP;

int main(int argc, char* argv[]) {
//    icl_core::logging::initialize(argc, argv);
    ros::init(argc, argv, "live_republisher");
    ros::NodeHandle n;
    VictorRightArm victor;
    RosInterface ri(n);

}