#ifndef GPU_PLANNING_STATE_HPP
#define GPU_PLANNING_STATE_HPP

#include "robot_model.hpp"


namespace GVP
{
    class State
    {
    public:
        Robot &active_robot;
        Robot &passive_robot;
        ProbGrid known_obstacles;
        std::vector<ProbGrid> chs;

        State(Robot &active, Robot &passive) : active_robot(active),
                                               passive_robot(passive)
        {
        };
    };
}

// passive_robot("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_left_arm_and_body.urdf"),
//     active_robot("/home/bradsaund/catkin_ws/src/gpu_voxel_planning/urdf/victor_right_arm_only.urdf")

#endif
