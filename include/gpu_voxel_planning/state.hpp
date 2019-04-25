#ifndef GPU_PLANNING_STATE_HPP
#define GPU_PLANNING_STATE_HPP

#include "robot_model.hpp"
#include <stdexcept>
#include <arc_utilities/timing.hpp>
#include "beliefs/beliefs.hpp"

namespace GVP
{
    class State
    {
    public:
        Robot &robot;
        DenseGrid robot_self_collide_obstacles;
        DenseGrid known_obstacles;
        std::unique_ptr<Belief> bel;

        robot::JointValueMap current_config;

        State(Robot &robot) : robot(robot)
        {
            bel = std::make_unique<ChsBelief>();
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

        double calcProbFree(const DenseGrid &volume)
        {
            PROFILE_START("CalcProbFreeKnown");
            if(robot_self_collide_obstacles.overlapsWith(&volume))
            {
                PROFILE_RECORD("CalcProbFreeKnown");
                return 0.0;
            }

            if(known_obstacles.overlapsWith(&volume))
            {
                PROFILE_RECORD("CalcProbFreeKnown");
                return 0.0;
            }
            PROFILE_RECORD("CalcProbFreeKnown");

            return bel->calcProbFree(volume);
        }

        double calcProbFree(const VictorRightArmConfig &c)
        {
            robot.set(c.asMap());
            return calcProbFree(robot.occupied_space);
        }

        void updateFreeSpace(const DenseGrid &new_free)
        {
            bel->updateFreeSpace(new_free);
        }

        void updateConfig(const robot::JointValueMap &jvm)
        {
            for(const auto& kv: jvm)
            {
                current_config[kv.first] = kv.second;
            }
        }
    };





    
    /*****************************************
     *      State for Simulation Trials
     ***************************************/
    class SimulationState : public State
    {
    public:
        SimulationState(Robot &robot) : State(robot)
        {
        };
        
        bool move(const VictorRightArmConfig &c, const DenseGrid &true_world)
        {
            robot.set(c.asMap());
            if(robot.occupied_space.overlapsWith(&true_world))
            {
                bel->updateCollisionSpace(robot, true_world);
                // addChs(true_world);
                robot.set(current_config);
                return false;
            }

            updateConfig(c.asMap());
            updateFreeSpace(robot.occupied_space);
            return true;
        }
    };
}


#endif
