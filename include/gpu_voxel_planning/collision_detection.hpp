#ifndef COLLISION_DETECTION_HPP
#define COLLISION_DETECTION_HPP

#include <victor_hardware_interface/MotionStatus.h>



double torque_collision_limit = 4;



struct CollisionInformation
{
    bool collision;
    std::vector<double> torques;
    std::vector<int> dirs;
    std::vector<double> joints;
};

std::vector<double> jvqToVector(victor_hardware_interface::JointValueQuantity jvq)
{
    std::vector<double> v{jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4,
                          jvq.joint_5, jvq.joint_6, jvq.joint_7};
    return v;
};


CollisionInformation checkCollision(victor_hardware_interface::MotionStatus::ConstPtr motion_msg)
{
    CollisionInformation c;

    c.collision = false;
    for(auto ext_torque: jvqToVector(motion_msg->estimated_external_torque))
    {

        if(ext_torque > torque_collision_limit)
        {
            c.collision = true;
        }
    }

    if(c.collision)
    {
        std::cout << "Collision!\n";
        c.torques = jvqToVector(motion_msg->estimated_external_torque);
        c.joints = jvqToVector(motion_msg->measured_joint_position);
        c.dirs.resize(c.joints.size());
        for(size_t i=0; i<c.torques.size(); i++)
        {
            double tqr = c.torques[i];
            c.dirs[i] = (0 < -tqr) - (-tqr < 0); //sgn
        }

    }

    
    return c;
    
};
#endif
