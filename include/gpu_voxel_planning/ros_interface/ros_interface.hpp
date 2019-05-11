#ifndef GVP_ROS_INTERFACE_HPP
#define GVP_ROS_INTERFACE_HPP

#include <ros/ros.h>
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "robot/robot_model.hpp"
#include <gpu_voxel_planning/AttemptPathStart.h>
#include <gpu_voxel_planning/AttemptPathResult.h>
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>


namespace GVP{
    class RosInterface
    {
    public:
        GpuVoxelRvizVisualizer viz;
        ros::ServiceClient attempt_path_client;
        ros::Publisher right_arm_pub;
        ros::Publisher left_arm_pub;
        ros::Publisher right_gripper_pub;

        RosInterface(ros::NodeHandle &n) :
            viz(n)
        {
            attempt_path_client = n.serviceClient<gpu_voxel_planning::AttemptPathStart>("attempt_path_on_victor");
            using namespace victor_hardware_interface;
            right_arm_pub = n.advertise<MotionCommand>("right_arm/motion_command", 10);
            left_arm_pub = n.advertise<MotionCommand>("left_arm/motion_command", 10);
            right_gripper_pub = n.advertise<Robotiq3FingerCommand>("right_arm/gripper_command", 10);
        }

        victor_hardware_interface::MotionCommand vectorToMotionCommand(const std::vector<double>& c)
        {
            using namespace victor_hardware_interface;
            MotionCommand command;
            command.control_mode.mode = 2;
            command.joint_position.joint_1 = c[0];
            command.joint_position.joint_2 = c[1];
            command.joint_position.joint_3 = c[2];
            command.joint_position.joint_4 = c[3];
            command.joint_position.joint_5 = c[4];
            command.joint_position.joint_6 = c[5];
            command.joint_position.joint_7 = c[6];
            
            return command;
        }
                

        void setRightArm(const VictorRightArmConfig &c)
        {
            right_arm_pub.publish(vectorToMotionCommand(c.asVector()));
        }

        void setLeftArm(const VictorLeftArmConfig &c)
        {
            left_arm_pub.publish(vectorToMotionCommand(c.asVector()));
        }

        void setRightGripper(double val)
        {
            using namespace victor_hardware_interface;
            Robotiq3FingerCommand command;
            command.finger_a_command.position = val;
            command.finger_b_command.position = val;
            command.finger_c_command.position = val;
            right_gripper_pub.publish(command);
        }


        void moveRightArm(const VictorRightArmConfig &c)
        {
            gpu_voxel_planning::AttemptPathStart srv;
            srv.request.path.points.resize(1);
            srv.request.path.points[0].positions = c.asVector();

            if(!attempt_path_client.call(srv))
            {
                std::cout << "Failed to send command to ros robot\n";
            }
        }
    };
}


#endif //GVP_ROS_INTERFACE_HPP
