#ifndef GPU_PLANNING_STATE_HPP
#define GPU_PLANNING_STATE_HPP

#include "robot_model.hpp"
#include <stdexcept>
#include <arc_utilities/timing.hpp>


namespace GVP
{
    class State
    {
    public:
        Robot &robot;
        DenseGrid robot_self_collide_obstacles;
        DenseGrid known_obstacles;
        DenseGrid known_free;
        std::vector<DenseGrid> chs;
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

        double calcProbFree(const DenseGrid &volume)
        {
            PROFILE_START("CalcProbFree");
            if(robot_self_collide_obstacles.overlapsWith(&volume))
            {
                PROFILE_RECORD("CalcProbFree");
                return 0.0;
            }

            if(known_obstacles.overlapsWith(&volume))
            {
                PROFILE_RECORD("CalcProbFree");
                return 0.0;
            }

            double p_free = 1.0;
            for(auto &c: chs)
            {
                p_free *= 1 - ((double)c.collideWith(&volume)/c.countOccupied());
            }
            PROFILE_RECORD("CalcProbFree");
            return p_free;

        }

        double calcProbFree(const VictorRightArmConfig &c)
        {
            robot.set(c.asMap());
            return calcProbFree(robot.occupied_space);
        }

        void updateFreeSpace(const DenseGrid &new_free)
        {
            known_free.add(&new_free);
            for(auto &c: chs)
            {
                c.subtract(&new_free);
            }
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
                addChs(true_world);
                robot.set(current_config);
                return false;

            }

            updateConfig(c.asMap());
            updateFreeSpace(robot.occupied_space);
            return true;

            
        }


        void addChs(const DenseGrid &true_world)
        {
            auto link_occupancies = robot.getLinkOccupancies();
            size_t first_link_in_collision = 0;
            for(auto &link:link_occupancies)
            {
                if(link.overlapsWith(&true_world))
                {
                    break;
                }
                first_link_in_collision++;
            }

            if(first_link_in_collision >= link_occupancies.size())
            {
                throw std::logic_error("Trying to add CHS, but no link collided");
            }

            DenseGrid new_chs;

            for(size_t i=first_link_in_collision; i<link_occupancies.size(); i++)
            {
                new_chs.add(&link_occupancies[i]);
            }
            new_chs.subtract(&known_free);
            chs.push_back(new_chs);
        }

    };
}


#endif
