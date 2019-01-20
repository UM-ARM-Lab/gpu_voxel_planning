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
        ProbGrid known_free;
        std::vector<ProbGrid> chs;
        robot::JointValueMap current_config;

        State(Robot &robot) : robot(robot)
        {
        };

        VictorRightArmConfig getCurConfig() const
        {
            return VictorRightArmConfig(current_config);
        }


        bool isPossiblyValid(const VictorRightArmConfig &c)
        {
            robot.set(c.asMap());
            if(robot.occupied_space.overlapsWith(&robot_self_collide_obstacles))
            {
                return false;
            }

            if(robot.occupied_space.overlapsWith(&known_obstacles))
            {
                return false;
            }
            return true;
        }

        bool move(const VictorRightArmConfig &c, const ProbGrid &true_world)
        {
            robot.set(c.asMap());
            if(!robot.occupied_space.overlapsWith(&true_world))
            {
                updateConfig(c.asMap());
                known_free.add(&robot.occupied_space);
                return true;
            }

            robot.set(current_config);
            return false;
        }


        void updateConfig(const robot::JointValueMap &jvm)
        {
            for(const auto& kv: jvm)
            {
                current_config[kv.first] = kv.second;
            }
        }
    };
}


#endif
