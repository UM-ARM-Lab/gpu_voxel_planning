#ifndef GPU_PLANNING_STATE_HPP
#define GPU_PLANNING_STATE_HPP

#include "robot_model.hpp"


namespace GVP
{
    class State
    {
    public:
        Robot &robot;
        ProbGrid robot_self_collide_obstacles;
        ProbGrid known_obstacles;
        std::vector<ProbGrid> chs;
        GVP::VictorRightArmConfig current_config;
        VictorRightArmConfig goal_config;

        State(Robot &robot) : robot(robot)
        {
        };

        
    };
}


#endif
