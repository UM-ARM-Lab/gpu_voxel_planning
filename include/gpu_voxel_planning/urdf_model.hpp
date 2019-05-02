#ifndef GPU_VOXEL_URDF_MODEL_HPP
#define GPU_VOXEL_URDF_MODEL_HPP

#include "robot_model.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/ros.h>



class UrdfArm
{
public:
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
};

class VictorKinematics : UrdfArm
{
public:
    VictorKinematics(bool mock) // Constructor used to mock for tests
    {
    }
    
    VictorKinematics()
    {
        robot_model_loader::RobotModelLoader robot_model_load("robot_description");
        kinematic_model = robot_model_load.getModel();
        kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
        joint_model_group = kinematic_model->getJointModelGroup("right_arm");
    }

    Eigen::Vector3d getEEPosition(const std::vector<double> &joint_values,
                                  std::string ee_group_name="victor_right_gripper_palm_surface")
    {
        kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
        const Eigen::Affine3d &end_effector_tf =
            kinematic_state->getGlobalLinkTransform(ee_group_name);
        return end_effector_tf.translation();
    }
};




#endif
