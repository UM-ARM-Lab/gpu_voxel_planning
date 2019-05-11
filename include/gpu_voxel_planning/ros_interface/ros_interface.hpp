#ifndef GVP_ROS_INTERFACE_HPP
#define GVP_ROS_INTERFACE_HPP

#include <ros/ros.h>
#include "ros_interface/gpu_voxel_rviz_visualization.hpp"
#include "robot_model.hpp"
#include <gpu_voxel_planning/AttemptPathStart.h>
#include <gpu_voxel_planning/AttemptPathResult.h>


namespace GVP{
    class RosInterface
    {
    public:
        GpuVoxelRvizVisualizer viz;
        ros::ServiceClient attempt_path_client;

        RosInterface(ros::NodeHandle &n) :
            viz(n)
        {
            attempt_path_client = n.serviceClient<gpu_voxel_planning::AttemptPathStart>("attempt_path_on_victor");
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
